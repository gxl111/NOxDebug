/*
 * modbus_slave.c - Modbus RTU slave. Implements 01H/03H/05H/06H/10H, register map (VAR_T),
 * coil/register Var_* accessors. RX timeout via TIM2 (3.5 char). Flash: see modbus_flash.h.
 */

#include "modbus_slave.h"
#include "nox_channel.h"
#include "app_config.h"
#include "main.h"
#include <string.h>
#include "usart.h"
#include "tim.h"
// #include "oled.h"  /* OLED 已禁用 */
#include "semphr.h"
/*
*********************************************************************************************************
*  Modbus RTU slave - global state
*********************************************************************************************************
*/


// Slave address (can override via SD config)
uint8_t SADDR485=1;
// Baud rate
uint32_t SBAUD485=115200;



/*
*********************************************************************************************************
*  RX timeout / callbacks
*********************************************************************************************************
*/
void (*s_TIM_CallBack1)(void);

static void MODS_SendWithCRC(uint8_t *_pBuf, uint8_t _ucLen);
static void MODS_SendAckOk(void);
static void MODS_SendAckErr(uint8_t _ucErrCode);

static void MODS_AnalyzeApp(void);

static void MODS_RxTimeOut(void);
static void MODS_RestartRxGapTimer(void);

static void MODS_01H(void);
static void MODS_03H(void);
static void MODS_05H(void);
static void MODS_06H(void);
static void MODS_10H(void);

static uint8_t MODS_ReadRegValue(uint16_t reg_addr, uint8_t *reg_value);
static uint8_t MODS_WriteRegValue(uint16_t reg_addr, uint8_t* reg_value);

void MODS_ReciveNew(uint8_t _byte);

static float RegistersToFloat_BE(uint16_t reg1, uint16_t reg2);





/*
 * Modbus slave globals and helpers
 */



MODS_T g_tModS = {0};
VAR_T g_tVar = { .S1 = { .nox_cal_trig = 0xFFFFu, .o2_cal_trig = 0xFFFFu },
                 .S2 = { .nox_cal_trig = 0xFFFFu, .o2_cal_trig = 0xFFFFu },
                 .S3 = { .nox_cal_trig = 0xFFFFu, .o2_cal_trig = 0xFFFFu } };

/* Mutex-protected read/write macros for single g_tVar fields; avoid repeating LOCK_VAR/UNLOCK_VAR. */
#define VAR_READ_U16(member, ret)   do { LOCK_VAR(); (ret) = g_tVar.member; UNLOCK_VAR(); } while(0)
#define VAR_READ_FLOAT(member, ret) do { LOCK_VAR(); (ret) = g_tVar.member; UNLOCK_VAR(); } while(0)
#define VAR_WRITE_U16(member, val)  do { LOCK_VAR(); g_tVar.member = (val); UNLOCK_VAR(); } while(0)
#define VAR_WRITE_FLOAT(member, val) do { LOCK_VAR(); g_tVar.member = (val); UNLOCK_VAR(); } while(0)


// RX semaphore (signalled on 3.5 char timeout)
QueueHandle_t MODRx_SemaphoreHandle;


/* 线圈 D01-D04 已废弃，阀门改由保持寄存器 40053-40055/40095-40097/40137-40139 控制 J1-J9 */


/*
 * Start UART RX interrupt for next byte
 */

void Start_Receive(void){
     HAL_UART_Receive_IT(&MDSUARTx,&rx_data, 1);
}


/*
*********************************************************************************************************
*  Function: MODS_Poll
*  Description: Check RX semaphore, validate CRC and address, then call MODS_AnalyzeApp under lock.
*********************************************************************************************************
*/
void MODS_Poll(void)
{
    uint16_t addr;
    uint16_t crc1;
    uint8_t pending_len = 0;
    uint8_t pending_save[S_RX_BUF_SIZE];

    /* After 3.5 char timeout callback gives semaphore; we take it here. */
    
    if(pdTRUE==xSemaphoreTake(MODRx_SemaphoreHandle,portMAX_DELAY)){

        NVIC_DisableIRQ(USART1_IRQn);
        pending_len = g_tModS.RxCount;
        if (pending_len > S_RX_BUF_SIZE)
            pending_len = S_RX_BUF_SIZE;
        if (pending_len > 0U)
            (void)memcpy(pending_save, g_tModS.RxBuf, (size_t)pending_len);

        /* 已完成帧只在 RxFrameSnap；RxBuf 留给 ISR 继续收下一帧（pending 已拷到栈上） */
        NVIC_EnableIRQ(USART1_IRQn);

        if (g_tModS.RxFrameLen < 4)				/* need at least 4 bytes: addr+func+reg */
        {
            goto err_ret;
        }

        /* CRC16 over frame; result 0 means valid（长度以快照 RxFrameLen 为准） */
        crc1 = CRC16_Modbus(g_tModS.RxFrameSnap, g_tModS.RxFrameLen);
        if (crc1 != 0)
        {
            goto err_ret;
        }

        /* slave address check */
        addr = g_tModS.RxFrameSnap[0];				/* byte 1: address */
        if (addr != SADDR485)		 			/* not for us */
        {
            goto err_ret;
        }

        /* Parse function code and build response (same lock order as NOxDefault task) */
				MODS_AnalyzeApp();
    
    }
    

err_ret:
    NVIC_DisableIRQ(USART1_IRQn);
    g_tModS.RxFrameLen = 0;
    if (pending_len > 0U) {
        if (pending_len > S_RX_BUF_SIZE)
            pending_len = S_RX_BUF_SIZE;
        (void)memcpy(g_tModS.RxBuf, pending_save, (size_t)pending_len);
        g_tModS.RxCount = pending_len;
        MODS_RestartRxGapTimer();
    } else {
        g_tModS.RxCount = 0;
    }
    NVIC_EnableIRQ(USART1_IRQn);
}


// TIM2 used for 3.5 character timeout
/*
*********************************************************************************************************
*  Function: StartHardTimer
*  Description: Start hardware timer for 3.5 character timeout; callback on overflow.
*  Parameters: _uiTimeOut, _pCallBack
*********************************************************************************************************
*/
void StartHardTimer(uint32_t _uiTimeOut, void * _pCallBack) {


    __HAL_TIM_SET_AUTORELOAD(&RX_TIMER, _uiTimeOut);
    s_TIM_CallBack1 = (void (*)(void)) _pCallBack;
    // Reset counter to 0
    __HAL_TIM_SET_COUNTER(&RX_TIMER, 0);

    // Clear update flag and start IT
    __HAL_TIM_CLEAR_FLAG(&RX_TIMER, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&RX_TIMER);

}


/*
 * HAL_TIM_OC_DelayElapsedCallback (legacy): TIM2 CC1 used for 3.5 char timeout; optional.
 */

//void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
//    if (htim->Instance == TIM2) {

//        if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC1) != RESET) {
//            __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC1);

//          __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);
//            if (s_TIM_CallBack1 != NULL) {
//                s_TIM_CallBack1();
//            }
//        }
//    }
//}

/*
*********************************************************************************************************
*  Function: MODS_ReciveNew
*  Description: Called from UART RX ISR on each byte. Restart 3.5 char timer; store byte.
*  RTU inter-frame delay depends on baud; see ModbusBaudRate table at top of file.
*********************************************************************************************************
*/
static void MODS_RestartRxGapTimer(void)
{
    uint8_t i;

    for (i = 0; i < MODBUS_BAUD_RATE_LEN; i++) {
        if (SBAUD485 == ModbusBaudRate[i].Bps)
            break;
    }
    if (i >= MODBUS_BAUD_RATE_LEN)
        i = MODBUS_BAUD_RATE_LEN - 1;

    StartHardTimer(ModbusBaudRate[i].usTimeOut, (void *)MODS_RxTimeOut);
}

void MODS_ReciveNew(uint8_t _byte)
{
    MODS_RestartRxGapTimer();

    if (g_tModS.RxCount < S_RX_BUF_SIZE)
    {
        g_tModS.RxBuf[g_tModS.RxCount++] = _byte;
    }
}


/*
*********************************************************************************************************
*  Function: MODS_RxTimeOut
*  Description: Called after 3.5 char time with no RX. Gives semaphore so MODS_Poll can process frame.
*********************************************************************************************************
*/
static void MODS_RxTimeOut(void)
{
    uint8_t n;

    NVIC_DisableIRQ(USART1_IRQn);
    n = g_tModS.RxCount;
    if (n > S_RX_BUF_SIZE)
        n = S_RX_BUF_SIZE;
    if (n > 0U)
        (void)memcpy(g_tModS.RxFrameSnap, g_tModS.RxBuf, (size_t)n);
    g_tModS.RxFrameLen = n;
    g_tModS.RxCount = 0;
    NVIC_EnableIRQ(USART1_IRQn);

    (void)xSemaphoreGiveFromISR(MODRx_SemaphoreHandle, NULL);
}


void RS485_Send_Data_IT(uint8_t *pData, uint16_t Size) {
    if (rs485_state.isSending) {
        return;
    }
    rs485_state.isSending = 1;
    RS485_Enable_TX(RS485_EN_PORT,RS485_EN_PIN);
    HAL_UART_Transmit_IT(&MDSUARTx, pData, Size);
}
/*
*********************************************************************************************************
*  Function: MODS_SendWithCRC
*  Description: Append 2-byte CRC to buffer and send via RS485.
*  Parameters: _pBuf, _ucLen (length before CRC)
*********************************************************************************************************
*/
static void MODS_SendWithCRC(uint8_t *_pBuf, uint8_t _ucLen)
{
    uint16_t crc;
    uint8_t buf[S_TX_BUF_SIZE];

    memcpy(buf, _pBuf, _ucLen);
    crc = CRC16_Modbus(_pBuf, _ucLen);
    buf[_ucLen++] = crc >> 8;
    buf[_ucLen++] = crc;

    RS485_Send_Data_IT(buf, _ucLen);
}

/*
*********************************************************************************************************
*  Function: MODS_SendAckErr
*  Description: Send exception response.
*  Parameters: _ucErrCode = exception code
*********************************************************************************************************
*/
static void MODS_SendAckErr(uint8_t _ucErrCode)
{
    uint8_t txbuf[3];

    if (g_tModS.RxFrameLen < 2)
        return;

    txbuf[0] = g_tModS.RxFrameSnap[0];					/* echo slave addr */
    txbuf[1] = g_tModS.RxFrameSnap[1] | 0x80;				/* set MSB for error */
    txbuf[2] = _ucErrCode;							/* exception code (01,02,03,04) */

    MODS_SendWithCRC(txbuf, 3);
}

/*
*********************************************************************************************************
*  Function: MODS_SendAckOk
*  Description: Echo request frame as success response.
*********************************************************************************************************
*/
static void MODS_SendAckOk(void)
{
    uint8_t txbuf[6];
    uint8_t i;

    if (g_tModS.RxFrameLen < 6)
        return;

    for (i = 0; i < 6; i++)
    {
        txbuf[i] = g_tModS.RxFrameSnap[i];
    }
    
    MODS_SendWithCRC(txbuf, 6);
}

/*
*********************************************************************************************************
*  Function: MODS_AnalyzeApp
*  Description: Dispatch by function code.
*********************************************************************************************************
*/
static void MODS_AnalyzeApp(void)
{
    //LCD_ShowString(30,400,210,24,24,"modbus_analyzing");
    switch (g_tModS.RxFrameSnap[1])				/* byte 2: function code */
    {
    case 0x01:							/* read coils */
        MODS_01H();
        break;

    case 0x03:							/* read holding registers (g_tVar) */
        MODS_03H();
        break;


    case 0x05:							/* write single coil */
        MODS_05H();
        break;

    case 0x06:							/* write single register to g_tVar */
        MODS_06H();
        break;

    case 0x10:							/* write multiple registers to g_tVar */
        MODS_10H();

        break;

    default:
        g_tModS.RspCode = RSP_ERR_CMD;
        MODS_SendAckErr(g_tModS.RspCode);	/* unsupported function */
        break;
    }
}

/*
*********************************************************************************************************
*  Function: MODS_01H
*  Description: Read coil status (D01/D02/D03/D04).
*********************************************************************************************************
*/
/* Note: LED/relay state read back from GPIO */
static void MODS_01H(void)
{
    /* 01H request: addr, 01, start coil, qty; response: addr, 01, byte count, coil data, CRC.
     * Coil map: D01-D04 map to relay GPIO readback. */
    uint16_t reg;
    uint16_t num;
    uint16_t i;
    uint16_t m;
    uint8_t status[10];

    g_tModS.RspCode = RSP_OK;

    /* Step 1: fixed 8-byte request for 01H */
    if (g_tModS.RxFrameLen != 8)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;
        return;
    }

    /* Step 2: parse start address and quantity */
    reg = BEBufToUint16(&g_tModS.RxFrameSnap[2]);
    num = BEBufToUint16(&g_tModS.RxFrameSnap[4]);

    m = (num + 7) / 8;   /* response data bytes */

    /* Step 3: validate coil range REG_D01..REG_DXX */
    if ( (num > 0) && (reg + num+ REG_D01 <= REG_DXX + 1))
    {
        for (i = 0; i < m; i++)
        {
            status[i] = 0;
        }

        LOCK_VAR();
        for (i = 0; i < num; i++)
        {
            uint8_t state = 0;
            switch (i) {
                case 0: state = g_tVar.coil_d01 ? 1 : 0; break;
                case 1: state = g_tVar.coil_d02 ? 1 : 0; break;
                case 2: state = g_tVar.coil_d03 ? 1 : 0; break;
                case 3: state = g_tVar.coil_d04 ? 1 : 0; break;
            }
            status[i / 8] |= (state << (i % 8));
        }
        UNLOCK_VAR();
    }
    else
    {
        g_tModS.RspCode = RSP_ERR_REG_ADDR;				
    }
    
    /* Step 4: build response */
    if (g_tModS.RspCode == RSP_OK)
    {
        g_tModS.TxCount = 0;
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxFrameSnap[0];
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxFrameSnap[1];
        g_tModS.TxBuf[g_tModS.TxCount++] = m;

        for (i = 0; i < m; i++)
            g_tModS.TxBuf[g_tModS.TxCount++] = status[i];
        MODS_SendWithCRC(g_tModS.TxBuf, g_tModS.TxCount);
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);
    }
}




/*
*********************************************************************************************************
*  Function: MODS_03H
*  Description: Read holding registers (float or uint16_t), map to g_tVar.
*********************************************************************************************************
*/
static void MODS_03H(void)
{
    /* 03H: read holding registers; max MODBUS_FC03_MAX_REGS words per Modbus PDU. */
    uint16_t reg;
    uint16_t num;
    uint16_t i;
    uint8_t reg_value[MODBUS_FC03_MAX_REGS * 2u];

    g_tModS.RspCode = RSP_OK;

    if (g_tModS.RxFrameLen != 8)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;
        goto err_ret;
    }

    /* Step 2: parse request */
    
    reg = BEBufToUint16(&g_tModS.RxFrameSnap[2]); 				
    num = BEBufToUint16(&g_tModS.RxFrameSnap[4]);					

    
    //
//    if(reg+SLAVE_REG_START+num-1>SLAVE_REG_END){

//        goto err_ret;    
//    }
    if (num == 0u || num > MODBUS_FC03_MAX_REGS)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;
        goto err_ret;
    }
    
    
    for (i = 0; i < num; )
    {
         uint8_t read_state = MODS_ReadRegValue(reg, &reg_value[2 * i]);
         if (read_state == 0) {
            g_tModS.RspCode = RSP_ERR_REG_ADDR;
            break;
         }
         if (read_state == 2) {
            /* Float: 2 registers = 4 bytes; advance by 2 slots to avoid overwrite */
            i += 2;
            reg += 2;
         } else {
            /* Single 16-bit register */
            i++;
            reg++;
         }
    }
    
    /* Step 3: send response or exception */
err_ret:
    if (g_tModS.RspCode == RSP_OK)							 
    {
        g_tModS.TxCount = 0;
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxFrameSnap[0]; 
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxFrameSnap[1]; 

        g_tModS.TxBuf[g_tModS.TxCount++] =num * 2;			 
      
        

        for (i = 0; i < num; i++)                            
        {
            g_tModS.TxBuf[g_tModS.TxCount++] = reg_value[2*i];
            g_tModS.TxBuf[g_tModS.TxCount++] = reg_value[2*i+1];
        }

        MODS_SendWithCRC(g_tModS.TxBuf, g_tModS.TxCount);	
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);					
    }
}


/*
*********************************************************************************************************
*  Function: MODS_05H
*  Description: Write single coil (D01/D02/D03/D04).
*********************************************************************************************************
*/
static void MODS_05H(void)
{
    /* 05H: write single coil; value FF00 = ON, 0000 = OFF. Maps to D01-D04 relays. */
    uint16_t reg;
    uint16_t value;

    g_tModS.RspCode = RSP_OK;

    /* Step 1: validate frame length */
    
    if (g_tModS.RxFrameLen != 8)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;		
        goto err_ret;
    }

    /* Step 2: parse request */
    
    reg = BEBufToUint16(&g_tModS.RxFrameSnap[2]); 	
    value = BEBufToUint16(&g_tModS.RxFrameSnap[4]);	

    if (value != 0x0000 && value != 0xFF00)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;		
        goto err_ret;
    }
    if(value == 0xFF00)value =1;
    
    
    
    LOCK_VAR();
    if (reg + REG_D01 == REG_D01)
        g_tVar.coil_d01 = value;
    else if (reg + REG_D01 == REG_D02)
        g_tVar.coil_d02 = value;
    else if (reg + REG_D01 == REG_D03)
        g_tVar.coil_d03 = value;
    else if (reg + REG_D01 == REG_D04)
        g_tVar.coil_d04 = value;
    else
        g_tModS.RspCode = RSP_ERR_REG_ADDR;
    UNLOCK_VAR();
    /* 线圈不再驱动 GPIO，阀门由保持寄存器 40053 等控制 */
    
    /* Step 3: send response or exception */
err_ret:
    if (g_tModS.RspCode == RSP_OK)				
    {
        MODS_SendAckOk();
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);		
    }
}

/*
*********************************************************************************************************
*  Function: MODS_06H
*  Description: Write single register.
*********************************************************************************************************
*/
static void MODS_06H(void)
{
    /* 06H: write single holding register; 8-byte request + CRC; echo request on success. */

    uint16_t reg;
    uint8_t write_state;

    g_tModS.RspCode = RSP_OK;

    /* Step 1: validate frame length */
    
    if (g_tModS.RxFrameLen != 8)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;		
        goto err_ret;
    }

    /* Step 2: parse request */
    
    reg = BEBufToUint16(&g_tModS.RxFrameSnap[2]); 	

    
    write_state = MODS_WriteRegValue(reg, &g_tModS.RxFrameSnap[4]);
    
    if (write_state!=0)	
    {
        ;
    }
    else
    {
        g_tModS.RspCode = RSP_ERR_REG_ADDR;		
         
    }

    /* Step 3: send response or exception */
err_ret:
    if (g_tModS.RspCode == RSP_OK)				
    {
        MODS_SendAckOk();
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);		
    }
}

/*
*********************************************************************************************************
*  Function: MODS_10H
*  Description: Write multiple registers to g_tVar.
*********************************************************************************************************
*/
static void MODS_10H(void)
{
    /* 10H: write multiple registers; byte count must equal 2 * reg count; MODS_WriteRegValue per word. */
    uint16_t reg_addr;
    uint16_t reg_num;
    uint16_t byte_num;
    uint8_t i;
//    uint16_t value;


    g_tModS.RspCode = RSP_OK;

    /* Step 1: 固定头 7 字节 + 数据 byte_num + CRC 2 字节；先保证能读到 ByteCount */
    if (g_tModS.RxFrameLen < 7)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;			
        goto err_ret;
    }

    /* Step 2: parse request */
    
    reg_addr = BEBufToUint16(&g_tModS.RxFrameSnap[2]); 	
    reg_num = BEBufToUint16(&g_tModS.RxFrameSnap[4]);		
    byte_num = g_tModS.RxFrameSnap[6];					

    
    if (byte_num != 2 * reg_num)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;			
        goto err_ret;
    }

    {
        uint16_t expect_len = (uint16_t)(9u + (uint16_t)byte_num);
        if ((uint16_t)g_tModS.RxFrameLen != expect_len)
        {
            g_tModS.RspCode = RSP_ERR_VALUE;
            goto err_ret;
        }
    }
    
    
    for (i = 0; i < reg_num; i++)
    {
        

        
        uint8_t write_state=MODS_WriteRegValue(reg_addr, &g_tModS.RxFrameSnap[7 + 2 * i]);
        if (write_state == 2)
        {
            ++i;
            ++reg_addr;
        }
        else if(write_state == 0)
        {
            g_tModS.RspCode = RSP_ERR_REG_ADDR;		
           
            break;
        }
        ++reg_addr;
    }
    
    /* Step 3: send response or exception */
err_ret:
    if (g_tModS.RspCode == RSP_OK)					
    {
        MODS_SendAckOk();
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);			
    }
}

/*
*********************************************************************************************************
*  MODS_ReadRegValue: map (reg_addr + SLAVE_REG_START) to g_tVar. Returns 1=u16, 2=float, 0=error
*********************************************************************************************************
*/
union { float f; uint32_t u; } converter;

static float RegistersToFloat_BE(uint16_t reg1, uint16_t reg2);

static uint8_t MODS_ReadRegValue(uint16_t reg_addr, uint8_t *reg_value)
{
    uint16_t addr = reg_addr + SLAVE_REG_START;
    uint16_t value = 0;
    float f_value = 0.0f;
    uint8_t f_flag = 0;
    SensorRegs_t *s;

    if (addr <= COMMON_REG_END) {
        LOCK_VAR();
        switch (addr) {
            case SLAVE_REG_NOX_OUTPUT:      f_value = g_tVar.nox_output; f_flag = 1; break;
            case SLAVE_REG_O2_OUTPUT:       f_value = g_tVar.o2_output; f_flag = 1; break;
            case SLAVE_REG_OUTPUT_CH_STATUS: value = g_tVar.output_ch_status; break;
            case SLAVE_REG_ALARM_NOX_HI:    f_value = g_tVar.alarm_nox_hi; f_flag = 1; break;
            case SLAVE_REG_ALARM_O2_LO:     f_value = g_tVar.alarm_o2_lo; f_flag = 1; break;
            case SLAVE_REG_MA_NOX:          value = g_tVar.ma_nox; break;
            case SLAVE_REG_MA_O2:           value = g_tVar.ma_o2; break;
            case SLAVE_REG_WORK_MODE:
                /* Read: work mode only (0/1/2). Single-channel index and runtime source are in P35. */
                value = (uint16_t)(g_tVar.work_mode & 0xFFu);
                break;
            case SLAVE_REG_OUTPUT_SENSOR:
                /* P35 R-only: 0b01=S0 0b10=S1 0b11=fusion 0b100=S2(CAN2) 0b00=fault */
                value = (uint16_t)NoxChannel_GetOutputSensorReg();
                break;
            default: UNLOCK_VAR(); return 0;
        }
        UNLOCK_VAR();
    } else if (addr >= SENSOR_BASE_1 && addr <= SLAVE_REG_S1_VALVE_CAL) {
        LOCK_VAR();
        s = &g_tVar.S1;
        goto sensor_read;
    } else if (addr >= SENSOR_BASE_2 && addr <= SLAVE_REG_S2_VALVE_CAL) {
        LOCK_VAR();
        s = &g_tVar.S2;
        goto sensor_read;
    } else if (addr >= SENSOR_BASE_3 && addr <= SLAVE_REG_S3_VALVE_CAL) {
        LOCK_VAR();
        s = &g_tVar.S3;
        goto sensor_read;
    } else {
        return 0;
    }
    goto output;

sensor_read:
    switch (addr) {
        /* S1 block 40014-40052 */
        case 40014: value = s->power_on; break;
        case 40015: case 40016: f_value = s->live_nox; f_flag = 1; break;
        case 40017: case 40018: f_value = s->live_o2;  f_flag = 1; break;
        case 40019: value = s->status; break;
        case 40020: case 40021: f_value = s->seg1_nox_a; f_flag = 1; break;
        case 40022: case 40023: f_value = s->seg1_nox_b; f_flag = 1; break;
        case 40024: case 40025: f_value = s->seg1_o2_a;  f_flag = 1; break;
        case 40026: case 40027: f_value = s->seg1_o2_b;  f_flag = 1; break;
        case 40028: case 40029: f_value = s->seg2_nox_a; f_flag = 1; break;
        case 40030: case 40031: f_value = s->seg2_nox_b; f_flag = 1; break;
        case 40032: case 40033: f_value = s->seg2_o2_a;  f_flag = 1; break;
        case 40034: case 40035: f_value = s->seg2_o2_b;  f_flag = 1; break;
        case 40036: case 40037: f_value = s->p2_nox; f_flag = 1; break;
        case 40038: case 40039: f_value = s->p2_o2;  f_flag = 1; break;
        case 40040: case 40041: f_value = s->p3_nox; f_flag = 1; break;
        case 40042: case 40043: f_value = s->p3_o2;  f_flag = 1; break;
        case 40044: value = s->nox_pt_sel; break;
        case 40045: value = s->nox_cal_trig; break;
        case 40046: value = s->o2_cal_trig; break;
        case 40047: value = s->o2_pt_sel; break;
        case 40048: value = s->blow_interval; break;
        case 40049: value = s->blow_duration; break;
        case 40050: value = s->blow_status; break;
        case 40051: value = s->blow_countdown; break;
        case 40052: value = s->blow_cmd; break;
        case 40053: value = s->valve_normal; break;
        case 40054: value = s->valve_blow; break;
        case 40055: value = s->valve_cal; break;
        /* S2 block 40056-40097 */
        case 40056: value = s->power_on; break;
        case 40057: case 40058: f_value = s->live_nox; f_flag = 1; break;
        case 40059: case 40060: f_value = s->live_o2;  f_flag = 1; break;
        case 40061: value = s->status; break;
        case 40062: case 40063: f_value = s->seg1_nox_a; f_flag = 1; break;
        case 40064: case 40065: f_value = s->seg1_nox_b; f_flag = 1; break;
        case 40066: case 40067: f_value = s->seg1_o2_a;  f_flag = 1; break;
        case 40068: case 40069: f_value = s->seg1_o2_b;  f_flag = 1; break;
        case 40070: case 40071: f_value = s->seg2_nox_a; f_flag = 1; break;
        case 40072: case 40073: f_value = s->seg2_nox_b; f_flag = 1; break;
        case 40074: case 40075: f_value = s->seg2_o2_a;  f_flag = 1; break;
        case 40076: case 40077: f_value = s->seg2_o2_b;  f_flag = 1; break;
        case 40078: case 40079: f_value = s->p2_nox; f_flag = 1; break;
        case 40080: case 40081: f_value = s->p2_o2;  f_flag = 1; break;
        case 40082: case 40083: f_value = s->p3_nox; f_flag = 1; break;
        case 40084: case 40085: f_value = s->p3_o2;  f_flag = 1; break;
        case 40086: value = s->nox_pt_sel; break;
        case 40087: value = s->nox_cal_trig; break;
        case 40088: value = s->o2_cal_trig; break;
        case 40089: value = s->o2_pt_sel; break;
        case 40090: value = s->blow_interval; break;
        case 40091: value = s->blow_duration; break;
        case 40092: value = s->blow_status; break;
        case 40093: value = s->blow_countdown; break;
        case 40094: value = s->blow_cmd; break;
        case 40095: value = s->valve_normal; break;
        case 40096: value = s->valve_blow; break;
        case 40097: value = s->valve_cal; break;
        /* S3 block 40098-40139 */
        case 40098: value = s->power_on; break;
        case 40099: case 40100: f_value = s->live_nox; f_flag = 1; break;
        case 40101: case 40102: f_value = s->live_o2;  f_flag = 1; break;
        case 40103: value = s->status; break;
        case 40104: case 40105: f_value = s->seg1_nox_a; f_flag = 1; break;
        case 40106: case 40107: f_value = s->seg1_nox_b; f_flag = 1; break;
        case 40108: case 40109: f_value = s->seg1_o2_a;  f_flag = 1; break;
        case 40110: case 40111: f_value = s->seg1_o2_b;  f_flag = 1; break;
        case 40112: case 40113: f_value = s->seg2_nox_a; f_flag = 1; break;
        case 40114: case 40115: f_value = s->seg2_nox_b; f_flag = 1; break;
        case 40116: case 40117: f_value = s->seg2_o2_a;  f_flag = 1; break;
        case 40118: case 40119: f_value = s->seg2_o2_b;  f_flag = 1; break;
        case 40120: case 40121: f_value = s->p2_nox; f_flag = 1; break;
        case 40122: case 40123: f_value = s->p2_o2;  f_flag = 1; break;
        case 40124: case 40125: f_value = s->p3_nox; f_flag = 1; break;
        case 40126: case 40127: f_value = s->p3_o2;  f_flag = 1; break;
        case 40128: value = s->nox_pt_sel; break;
        case 40129: value = s->nox_cal_trig; break;
        case 40130: value = s->o2_cal_trig; break;
        case 40131: value = s->o2_pt_sel; break;
        case 40132: value = s->blow_interval; break;
        case 40133: value = s->blow_duration; break;
        case 40134: value = s->blow_status; break;
        case 40135: value = s->blow_countdown; break;
        case 40136: value = s->blow_cmd; break;
        case 40137: value = s->valve_normal; break;
        case 40138: value = s->valve_blow; break;
        case 40139: value = s->valve_cal; break;
        default: UNLOCK_VAR(); return 0;
    }
    UNLOCK_VAR();

output:
    if (f_flag) {
        converter.f = f_value;
        reg_value[0] = (converter.u >> 24) & 0xFF;
        reg_value[1] = (converter.u >> 16) & 0xFF;
        reg_value[2] = (converter.u >> 8) & 0xFF;
        reg_value[3] = converter.u & 0xFF;
        return 2;
    }
    reg_value[0] = value >> 8;
    reg_value[1] = value & 0xFF;
    return 1;
}

/*
*********************************************************************************************************
*  MODS_WriteRegValue: map (reg_addr + SLAVE_REG_START) to g_tVar. Returns 1=u16, 2=float, 0=error
*********************************************************************************************************
*/
static uint8_t MODS_WriteRegValue(uint16_t reg_addr, uint8_t* reg_value)
{
    uint16_t addr = reg_addr + SLAVE_REG_START;
    uint16_t value = BEBufToUint16(reg_value);
    uint16_t value1;
    uint8_t f_flag = 0;
    SensorRegs_t *s;

    if (addr <= COMMON_REG_END) {
        LOCK_VAR();
        switch (addr) {
            case SLAVE_REG_NOX_OUTPUT:      value1 = BEBufToUint16(reg_value + 2); g_tVar.nox_output = RegistersToFloat_BE(value, value1); f_flag = 1; break;
            case SLAVE_REG_O2_OUTPUT:       value1 = BEBufToUint16(reg_value + 2); g_tVar.o2_output = RegistersToFloat_BE(value, value1); f_flag = 1; break;
            case SLAVE_REG_ALARM_NOX_HI:    value1 = BEBufToUint16(reg_value + 2); g_tVar.alarm_nox_hi = RegistersToFloat_BE(value, value1); f_flag = 1; break;
            case SLAVE_REG_ALARM_O2_LO:     value1 = BEBufToUint16(reg_value + 2); g_tVar.alarm_o2_lo = RegistersToFloat_BE(value, value1); f_flag = 1; break;
            case SLAVE_REG_MA_NOX:          g_tVar.ma_nox = value; break;
            case SLAVE_REG_MA_O2:           g_tVar.ma_o2 = value; break;
            case SLAVE_REG_WORK_MODE: {
                /* Mode 0: 0/256/512=ch0/ch1/ch2. Mode 1: 1/257/513=ch0/ch1/ch2 主. Mode 2: 2 only. */
                uint8_t mode = (uint8_t)(value & 0xFFu);
                if (mode == 0u || mode == 1u)
                    g_tVar.work_mode = (value & 0xFFu) | (value & 0x0300u);
                else
                    g_tVar.work_mode = mode;
                break;
            }
            default: UNLOCK_VAR(); return 0;
        }
        UNLOCK_VAR();
        return f_flag ? 2 : 1;
    }

    if (addr >= SENSOR_BASE_1 && addr <= SLAVE_REG_S1_VALVE_CAL)
        s = &g_tVar.S1;
    else if (addr >= SENSOR_BASE_2 && addr <= SLAVE_REG_S2_VALVE_CAL)
        s = &g_tVar.S2;
    else if (addr >= SENSOR_BASE_3 && addr <= SLAVE_REG_S3_VALVE_CAL)
        s = &g_tVar.S3;
    else
        return 0;

    LOCK_VAR();
    switch (addr) {
        /* S1 block 40014-40055 */
        case 40014:
            s->power_on = (value != 0u) ? 1u : 0u;
#if SENSOR_POWER_GPIO_ENABLE
            HAL_GPIO_WritePin(SENSOR_POWER0_GPIO_Port, SENSOR_POWER0_Pin, (s->power_on != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
            UNLOCK_VAR(); return 1;
        case 40015: case 40016: value1 = BEBufToUint16(reg_value + 2); s->live_nox = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40017: case 40018: value1 = BEBufToUint16(reg_value + 2); s->live_o2  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40020: case 40021: value1 = BEBufToUint16(reg_value + 2); s->seg1_nox_a = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40022: case 40023: value1 = BEBufToUint16(reg_value + 2); s->seg1_nox_b = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40024: case 40025: value1 = BEBufToUint16(reg_value + 2); s->seg1_o2_a  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40026: case 40027: value1 = BEBufToUint16(reg_value + 2); s->seg1_o2_b  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40028: case 40029: value1 = BEBufToUint16(reg_value + 2); s->seg2_nox_a = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40030: case 40031: value1 = BEBufToUint16(reg_value + 2); s->seg2_nox_b = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40032: case 40033: value1 = BEBufToUint16(reg_value + 2); s->seg2_o2_a  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40034: case 40035: value1 = BEBufToUint16(reg_value + 2); s->seg2_o2_b  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40036: case 40037: value1 = BEBufToUint16(reg_value + 2); s->p2_nox = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40038: case 40039: value1 = BEBufToUint16(reg_value + 2); s->p2_o2  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40040: case 40041: value1 = BEBufToUint16(reg_value + 2); s->p3_nox = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40042: case 40043: value1 = BEBufToUint16(reg_value + 2); s->p3_o2  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40044: s->nox_pt_sel = value; UNLOCK_VAR(); return 1;
        case 40045: s->nox_cal_trig = value; UNLOCK_VAR(); return 1;
        case 40046: s->o2_cal_trig = value; UNLOCK_VAR(); return 1;
        case 40047: s->o2_pt_sel = value; UNLOCK_VAR(); return 1;
        case 40048: s->blow_interval = value; UNLOCK_VAR(); return 1;
        case 40049: s->blow_duration = value; UNLOCK_VAR(); return 1;
        case 40052: s->blow_cmd = value; UNLOCK_VAR(); return 1;
        case 40053: s->valve_normal = (value != 0u) ? 1u : 0u; UNLOCK_VAR(); Var_Write_SensorValve(0, 0, s->valve_normal); return 1;
        case 40054: s->valve_blow   = (value != 0u) ? 1u : 0u; UNLOCK_VAR(); Var_Write_SensorValve(0, 1, s->valve_blow);   return 1;
        case 40055: s->valve_cal    = (value != 0u) ? 1u : 0u; UNLOCK_VAR(); Var_Write_SensorValve(0, 2, s->valve_cal);    return 1;
        /* S2 block 40056-40097 */
        case 40056:
            s->power_on = (value != 0u) ? 1u : 0u;
#if SENSOR_POWER_GPIO_ENABLE
            HAL_GPIO_WritePin(SENSOR_POWER1_GPIO_Port, SENSOR_POWER1_Pin, (s->power_on != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
            UNLOCK_VAR(); return 1;
        case 40057: case 40058: value1 = BEBufToUint16(reg_value + 2); s->live_nox = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40059: case 40060: value1 = BEBufToUint16(reg_value + 2); s->live_o2  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40062: case 40063: value1 = BEBufToUint16(reg_value + 2); s->seg1_nox_a = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40064: case 40065: value1 = BEBufToUint16(reg_value + 2); s->seg1_nox_b = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40066: case 40067: value1 = BEBufToUint16(reg_value + 2); s->seg1_o2_a  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40068: case 40069: value1 = BEBufToUint16(reg_value + 2); s->seg1_o2_b  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40070: case 40071: value1 = BEBufToUint16(reg_value + 2); s->seg2_nox_a = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40072: case 40073: value1 = BEBufToUint16(reg_value + 2); s->seg2_nox_b = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40074: case 40075: value1 = BEBufToUint16(reg_value + 2); s->seg2_o2_a  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40076: case 40077: value1 = BEBufToUint16(reg_value + 2); s->seg2_o2_b  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40078: case 40079: value1 = BEBufToUint16(reg_value + 2); s->p2_nox = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40080: case 40081: value1 = BEBufToUint16(reg_value + 2); s->p2_o2  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40082: case 40083: value1 = BEBufToUint16(reg_value + 2); s->p3_nox = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40084: case 40085: value1 = BEBufToUint16(reg_value + 2); s->p3_o2  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40086: s->nox_pt_sel = value; UNLOCK_VAR(); return 1;
        case 40087: s->nox_cal_trig = value; UNLOCK_VAR(); return 1;
        case 40088: s->o2_cal_trig = value; UNLOCK_VAR(); return 1;
        case 40089: s->o2_pt_sel = value; UNLOCK_VAR(); return 1;
        case 40090: s->blow_interval = value; UNLOCK_VAR(); return 1;
        case 40091: s->blow_duration = value; UNLOCK_VAR(); return 1;
        case 40094: s->blow_cmd = value; UNLOCK_VAR(); return 1;
        case 40095: s->valve_normal = (value != 0u) ? 1u : 0u; UNLOCK_VAR(); Var_Write_SensorValve(1, 0, s->valve_normal); return 1;
        case 40096: s->valve_blow   = (value != 0u) ? 1u : 0u; UNLOCK_VAR(); Var_Write_SensorValve(1, 1, s->valve_blow);   return 1;
        case 40097: s->valve_cal    = (value != 0u) ? 1u : 0u; UNLOCK_VAR(); Var_Write_SensorValve(1, 2, s->valve_cal);    return 1;
        /* S3 block 40098-40139 */
        case 40098:
            s->power_on = (value != 0u) ? 1u : 0u;
#if SENSOR_POWER_GPIO_ENABLE
            HAL_GPIO_WritePin(SENSOR_POWER2_GPIO_Port, SENSOR_POWER2_Pin, (s->power_on != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
            UNLOCK_VAR(); return 1;
        case 40099: case 40100: value1 = BEBufToUint16(reg_value + 2); s->live_nox = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40101: case 40102: value1 = BEBufToUint16(reg_value + 2); s->live_o2  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40104: case 40105: value1 = BEBufToUint16(reg_value + 2); s->seg1_nox_a = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40106: case 40107: value1 = BEBufToUint16(reg_value + 2); s->seg1_nox_b = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40108: case 40109: value1 = BEBufToUint16(reg_value + 2); s->seg1_o2_a  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40110: case 40111: value1 = BEBufToUint16(reg_value + 2); s->seg1_o2_b  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40112: case 40113: value1 = BEBufToUint16(reg_value + 2); s->seg2_nox_a = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40114: case 40115: value1 = BEBufToUint16(reg_value + 2); s->seg2_nox_b = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40116: case 40117: value1 = BEBufToUint16(reg_value + 2); s->seg2_o2_a  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40118: case 40119: value1 = BEBufToUint16(reg_value + 2); s->seg2_o2_b  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40120: case 40121: value1 = BEBufToUint16(reg_value + 2); s->p2_nox = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40122: case 40123: value1 = BEBufToUint16(reg_value + 2); s->p2_o2  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40124: case 40125: value1 = BEBufToUint16(reg_value + 2); s->p3_nox = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40126: case 40127: value1 = BEBufToUint16(reg_value + 2); s->p3_o2  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40128: s->nox_pt_sel = value; UNLOCK_VAR(); return 1;
        case 40129: s->nox_cal_trig = value; UNLOCK_VAR(); return 1;
        case 40130: s->o2_cal_trig = value; UNLOCK_VAR(); return 1;
        case 40131: s->o2_pt_sel = value; UNLOCK_VAR(); return 1;
        case 40132: s->blow_interval = value; UNLOCK_VAR(); return 1;
        case 40133: s->blow_duration = value; UNLOCK_VAR(); return 1;
        case 40136: s->blow_cmd = value; UNLOCK_VAR(); return 1;
        case 40137: s->valve_normal = (value != 0u) ? 1u : 0u; UNLOCK_VAR(); Var_Write_SensorValve(2, 0, s->valve_normal); return 1;
        case 40138: s->valve_blow   = (value != 0u) ? 1u : 0u; UNLOCK_VAR(); Var_Write_SensorValve(2, 1, s->valve_blow);   return 1;
        case 40139: s->valve_cal    = (value != 0u) ? 1u : 0u; UNLOCK_VAR(); Var_Write_SensorValve(2, 2, s->valve_cal);    return 1;
        default: UNLOCK_VAR(); return 0;
    }
}

// Convert two BE registers to float
float RegistersToFloat_BE(uint16_t reg1, uint16_t reg2) {
    converter.u = ((uint32_t)reg1 << 16) | reg2;
    return converter.f;
}

// ========================== Coil D01-D04 ==========================
void Var_Write_D01(uint16_t value) { VAR_WRITE_U16(coil_d01, value); }
uint16_t Var_Read_D01(void) { uint16_t r; VAR_READ_U16(coil_d01, r); return r; }

void Var_Write_D02(uint16_t value) { VAR_WRITE_U16(coil_d02, value); }
uint16_t Var_Read_D02(void) { uint16_t r; VAR_READ_U16(coil_d02, r); return r; }

void Var_Write_D03(uint16_t value) { VAR_WRITE_U16(coil_d03, value); }
uint16_t Var_Read_D03(void) { uint16_t r; VAR_READ_U16(coil_d03, r); return r; }

void Var_Write_D04(uint16_t value) { VAR_WRITE_U16(coil_d04, value); }
uint16_t Var_Read_D04(void) { uint16_t r; VAR_READ_U16(coil_d04, r); return r; }

/* ========================== Common: nox/o2 output, output_ch_status, alarm, ma, work_mode ========================== */
float Var_Read_NoxOutput(void) { float r; VAR_READ_FLOAT(nox_output, r); return r; }
float Var_Read_O2Output(void) { float r; VAR_READ_FLOAT(o2_output, r); return r; }
uint16_t Var_Read_OutputChStatus(void) { uint16_t r; VAR_READ_U16(output_ch_status, r); return r; }
void Var_Write_NoxOutput(float value) { VAR_WRITE_FLOAT(nox_output, value); }
void Var_Write_O2Output(float value) { VAR_WRITE_FLOAT(o2_output, value); }
void Var_Write_OutputChStatus(uint16_t value) { VAR_WRITE_U16(output_ch_status, value); }
void Var_Write_AlarmNoxHi(float value) { VAR_WRITE_FLOAT(alarm_nox_hi, value); }
float Var_Read_AlarmNoxHi(void) { float r; VAR_READ_FLOAT(alarm_nox_hi, r); return r; }
void Var_Write_AlarmO2Lo(float value) { VAR_WRITE_FLOAT(alarm_o2_lo, value); }
float Var_Read_AlarmO2Lo(void) { float r; VAR_READ_FLOAT(alarm_o2_lo, r); return r; }
void Var_Write_MaNox(uint16_t value) { VAR_WRITE_U16(ma_nox, value); }
uint16_t Var_Read_MaNox(void) { uint16_t r; VAR_READ_U16(ma_nox, r); return r; }
void Var_Write_MaO2(uint16_t value) { VAR_WRITE_U16(ma_o2, value); }
uint16_t Var_Read_MaO2(void) { uint16_t r; VAR_READ_U16(ma_o2, r); return r; }
void Var_Write_WorkMode(uint16_t value) { VAR_WRITE_U16(work_mode, value); }
uint16_t Var_Read_WorkMode(void) { uint16_t r; VAR_READ_U16(work_mode, r); return (uint16_t)(r & 0x03u); }
uint8_t Var_Read_SingleChannelIndex(void) { uint16_t r; VAR_READ_U16(work_mode, r); return (uint8_t)((r >> 8) & 3u); }
uint8_t Var_Read_OutputSensorReg(void) { return NoxChannel_GetOutputSensorReg(); }

// ========================== Sensor accessors by channel (ch=0, 1, or 2 for S1/S2/S3) ==========================
#define S(ch) ((ch) == 0 ? &g_tVar.S1 : ((ch) == 1 ? &g_tVar.S2 : &g_tVar.S3))

uint16_t Var_Read_SensorPowerOn(uint8_t ch) { uint16_t r; LOCK_VAR(); r = (ch <= 2u) ? S(ch)->power_on : 0u; UNLOCK_VAR(); return r; }
void Var_Write_SensorPowerOn(uint8_t ch, uint16_t v) {
    LOCK_VAR();
    if (ch == 0u) { g_tVar.S1.power_on = (v != 0u) ? 1u : 0u; }
    else if (ch == 1u) { g_tVar.S2.power_on = (v != 0u) ? 1u : 0u; }
    else if (ch == 2u) { g_tVar.S3.power_on = (v != 0u) ? 1u : 0u; }
    UNLOCK_VAR();
#if SENSOR_POWER_GPIO_ENABLE
    if (ch == 0u) HAL_GPIO_WritePin(SENSOR_POWER0_GPIO_Port, SENSOR_POWER0_Pin, (v != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    else if (ch == 1u) HAL_GPIO_WritePin(SENSOR_POWER1_GPIO_Port, SENSOR_POWER1_Pin, (v != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    else if (ch == 2u) HAL_GPIO_WritePin(SENSOR_POWER2_GPIO_Port, SENSOR_POWER2_Pin, (v != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}
float Var_Read_SensorLiveNox(uint8_t ch) { float r; LOCK_VAR(); r = S(ch)->live_nox; UNLOCK_VAR(); return r; }
float Var_Read_SensorLiveO2(uint8_t ch)  { float r; LOCK_VAR(); r = S(ch)->live_o2;  UNLOCK_VAR(); return r; }
uint16_t Var_Read_SensorStatus(uint8_t ch) { uint16_t r; LOCK_VAR(); r = S(ch)->status; UNLOCK_VAR(); return r; }
void Var_Write_SensorLiveNox(uint8_t ch, float v) { LOCK_VAR(); S(ch)->live_nox = v; UNLOCK_VAR(); }
void Var_Write_SensorLiveO2(uint8_t ch, float v)  { LOCK_VAR(); S(ch)->live_o2 = v;  UNLOCK_VAR(); }
void Var_Write_SensorStatus(uint8_t ch, uint16_t v) { LOCK_VAR(); S(ch)->status = v; UNLOCK_VAR(); }

float Var_Read_SensorSeg1NoxA(uint8_t ch) { float r; LOCK_VAR(); r = S(ch)->seg1_nox_a; UNLOCK_VAR(); return r; }
float Var_Read_SensorSeg1NoxB(uint8_t ch) { float r; LOCK_VAR(); r = S(ch)->seg1_nox_b; UNLOCK_VAR(); return r; }
float Var_Read_SensorSeg1O2A(uint8_t ch)  { float r; LOCK_VAR(); r = S(ch)->seg1_o2_a;  UNLOCK_VAR(); return r; }
float Var_Read_SensorSeg1O2B(uint8_t ch)  { float r; LOCK_VAR(); r = S(ch)->seg1_o2_b;  UNLOCK_VAR(); return r; }
void Var_Write_SensorSeg1NoxA(uint8_t ch, float v) { LOCK_VAR(); S(ch)->seg1_nox_a = v; UNLOCK_VAR(); }
void Var_Write_SensorSeg1NoxB(uint8_t ch, float v) { LOCK_VAR(); S(ch)->seg1_nox_b = v; UNLOCK_VAR(); }
void Var_Write_SensorSeg1O2A(uint8_t ch, float v)  { LOCK_VAR(); S(ch)->seg1_o2_a = v;  UNLOCK_VAR(); }
void Var_Write_SensorSeg1O2B(uint8_t ch, float v)  { LOCK_VAR(); S(ch)->seg1_o2_b = v;  UNLOCK_VAR(); }

float Var_Read_SensorSeg2NoxA(uint8_t ch) { float r; LOCK_VAR(); r = S(ch)->seg2_nox_a; UNLOCK_VAR(); return r; }
float Var_Read_SensorSeg2NoxB(uint8_t ch) { float r; LOCK_VAR(); r = S(ch)->seg2_nox_b; UNLOCK_VAR(); return r; }
float Var_Read_SensorSeg2O2A(uint8_t ch)  { float r; LOCK_VAR(); r = S(ch)->seg2_o2_a;  UNLOCK_VAR(); return r; }
float Var_Read_SensorSeg2O2B(uint8_t ch)  { float r; LOCK_VAR(); r = S(ch)->seg2_o2_b;  UNLOCK_VAR(); return r; }
void Var_Write_SensorSeg2NoxA(uint8_t ch, float v) { LOCK_VAR(); S(ch)->seg2_nox_a = v; UNLOCK_VAR(); }
void Var_Write_SensorSeg2NoxB(uint8_t ch, float v) { LOCK_VAR(); S(ch)->seg2_nox_b = v; UNLOCK_VAR(); }
void Var_Write_SensorSeg2O2A(uint8_t ch, float v)  { LOCK_VAR(); S(ch)->seg2_o2_a = v;  UNLOCK_VAR(); }
void Var_Write_SensorSeg2O2B(uint8_t ch, float v)  { LOCK_VAR(); S(ch)->seg2_o2_b = v;  UNLOCK_VAR(); }

float Var_Read_SensorP2Nox(uint8_t ch) { float r; LOCK_VAR(); r = S(ch)->p2_nox; UNLOCK_VAR(); return r; }
float Var_Read_SensorP2O2(uint8_t ch)  { float r; LOCK_VAR(); r = S(ch)->p2_o2;  UNLOCK_VAR(); return r; }
float Var_Read_SensorP3Nox(uint8_t ch) { float r; LOCK_VAR(); r = S(ch)->p3_nox; UNLOCK_VAR(); return r; }
float Var_Read_SensorP3O2(uint8_t ch)  { float r; LOCK_VAR(); r = S(ch)->p3_o2;  UNLOCK_VAR(); return r; }
void Var_Write_SensorP2Nox(uint8_t ch, float v) { LOCK_VAR(); S(ch)->p2_nox = v; UNLOCK_VAR(); }
void Var_Write_SensorP2O2(uint8_t ch, float v)  { LOCK_VAR(); S(ch)->p2_o2 = v;  UNLOCK_VAR(); }
void Var_Write_SensorP3Nox(uint8_t ch, float v) { LOCK_VAR(); S(ch)->p3_nox = v; UNLOCK_VAR(); }
void Var_Write_SensorP3O2(uint8_t ch, float v)  { LOCK_VAR(); S(ch)->p3_o2 = v;  UNLOCK_VAR(); }

uint16_t Var_Read_SensorNoxCalTrig(uint8_t ch) { uint16_t r; LOCK_VAR(); r = S(ch)->nox_cal_trig; UNLOCK_VAR(); return r; }
uint16_t Var_Read_SensorNoxPtSel(uint8_t ch)   { uint16_t r; LOCK_VAR(); r = S(ch)->nox_pt_sel;   UNLOCK_VAR(); return r; }
uint16_t Var_Read_SensorO2CalTrig(uint8_t ch)  { uint16_t r; LOCK_VAR(); r = S(ch)->o2_cal_trig;  UNLOCK_VAR(); return r; }
uint16_t Var_Read_SensorO2PtSel(uint8_t ch)    { uint16_t r; LOCK_VAR(); r = S(ch)->o2_pt_sel;    UNLOCK_VAR(); return r; }
void Var_Write_SensorNoxCalTrig(uint8_t ch, uint16_t v) { LOCK_VAR(); S(ch)->nox_cal_trig = v; UNLOCK_VAR(); }
void Var_Write_SensorNoxPtSel(uint8_t ch, uint16_t v)   { LOCK_VAR(); S(ch)->nox_pt_sel = v;   UNLOCK_VAR(); }
void Var_Write_SensorO2CalTrig(uint8_t ch, uint16_t v)  { LOCK_VAR(); S(ch)->o2_cal_trig = v;  UNLOCK_VAR(); }
void Var_Write_SensorO2PtSel(uint8_t ch, uint16_t v)    { LOCK_VAR(); S(ch)->o2_pt_sel = v;    UNLOCK_VAR(); }

uint16_t Var_Read_SensorBlowInterval(uint8_t ch)   { uint16_t r; LOCK_VAR(); r = S(ch)->blow_interval;   UNLOCK_VAR(); return r; }
uint16_t Var_Read_SensorBlowDuration(uint8_t ch)   { uint16_t r; LOCK_VAR(); r = S(ch)->blow_duration;   UNLOCK_VAR(); return r; }
uint16_t Var_Read_SensorBlowStatus(uint8_t ch)    { uint16_t r; LOCK_VAR(); r = S(ch)->blow_status;    UNLOCK_VAR(); return r; }
uint16_t Var_Read_SensorBlowCountdown(uint8_t ch) { uint16_t r; LOCK_VAR(); r = S(ch)->blow_countdown; UNLOCK_VAR(); return r; }
uint16_t Var_Read_SensorBlowCmd(uint8_t ch)       { uint16_t r; LOCK_VAR(); r = S(ch)->blow_cmd;       UNLOCK_VAR(); return r; }
void Var_Write_SensorBlowInterval(uint8_t ch, uint16_t v)   { LOCK_VAR(); S(ch)->blow_interval = v;   UNLOCK_VAR(); }
void Var_Write_SensorBlowDuration(uint8_t ch, uint16_t v)   { LOCK_VAR(); S(ch)->blow_duration = v;   UNLOCK_VAR(); }
void Var_Write_SensorBlowCmd(uint8_t ch, uint16_t v)       { LOCK_VAR(); S(ch)->blow_cmd = v;       UNLOCK_VAR(); }

/* 阀门 J1-J9：ch=0,1,2  v=0 正常 1 反吹 2 校准；写时驱动对应 GPIO */
uint16_t Var_Read_SensorValve(uint8_t ch, uint8_t v) {
    uint16_t r = 0;
    if (ch > 2u || v > 2u) return 0;
    LOCK_VAR();
    if (v == 0u) r = S(ch)->valve_normal;
    else if (v == 1u) r = S(ch)->valve_blow;
    else r = S(ch)->valve_cal;
    UNLOCK_VAR();
    return r;
}

static void Valve_WriteGPIO(uint8_t ch, uint8_t v, uint8_t on) {
    GPIO_PinState st = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
    if (ch == 0u) {
        if (v == 0u) HAL_GPIO_WritePin(J1_IN_GPIO_Port, J1_IN_Pin, st);
        else if (v == 1u) HAL_GPIO_WritePin(J2_IN_GPIO_Port, J2_IN_Pin, st);
        else HAL_GPIO_WritePin(J3_IN_GPIO_Port, J3_IN_Pin, st);
    } else if (ch == 1u) {
        if (v == 0u) HAL_GPIO_WritePin(J4_IN_GPIO_Port, J4_IN_Pin, st);
        else if (v == 1u) HAL_GPIO_WritePin(J5_IN_GPIO_Port, J5_IN_Pin, st);
        else HAL_GPIO_WritePin(J6_IN_GPIO_Port, J6_IN_Pin, st);
    } else {
        if (v == 0u) HAL_GPIO_WritePin(J7_IN_GPIO_Port, J7_IN_Pin, st);
        else if (v == 1u) HAL_GPIO_WritePin(J8_IN_GPIO_Port, J8_IN_Pin, st);
        else HAL_GPIO_WritePin(J9_IN_GPIO_Port, J9_IN_Pin, st);
    }
}

void Var_Write_SensorValve(uint8_t ch, uint8_t v, uint16_t value) {
    if (ch > 2u || v > 2u) return;
    uint8_t on = (value != 0u) ? 1u : 0u;
    LOCK_VAR();
    if (v == 0u) S(ch)->valve_normal = (uint16_t)on;
    else if (v == 1u) S(ch)->valve_blow = (uint16_t)on;
    else S(ch)->valve_cal = (uint16_t)on;
    UNLOCK_VAR();
    Valve_WriteGPIO(ch, v, on);
}

void Modbus_ApplySensorValveBlowToGPIO(uint8_t ch) {
    if (ch > 2u) return;
    uint16_t v;
    LOCK_VAR();
    v = (ch == 0u) ? g_tVar.S1.valve_blow : ((ch == 1u) ? g_tVar.S2.valve_blow : g_tVar.S3.valve_blow);
    UNLOCK_VAR();
    Valve_WriteGPIO(ch, 1, (v != 0u) ? 1u : 0u);
}

#undef S

void Var_Read_BlowbackCfg(uint16_t *p24, uint16_t *p25) {
    LOCK_VAR();
    if (p24) *p24 = g_tVar.S1.blow_interval;
    if (p25) *p25 = g_tVar.S1.blow_duration;
    UNLOCK_VAR();
}

void Var_Read_AlarmCfg(float *p12, float *p13) {
    LOCK_VAR();
    if (p12) *p12 = g_tVar.alarm_nox_hi;
    if (p13) *p13 = g_tVar.alarm_o2_lo;
    UNLOCK_VAR();
}

// ========================== Current output -> P01/P02/P07 (nox_output, o2_output, output_ch_status) ==========================
void Var_Update_SensorCore(float nox, float o2, uint16_t state) {
    LOCK_VAR();
    g_tVar.nox_output = nox;
    g_tVar.o2_output = o2;
    g_tVar.output_ch_status = state;
    UNLOCK_VAR();
}

void Var_Update_CalibPoint(float p18, float p19, float p20, float p21) {
    (void)p18; (void)p19; (void)p20; (void)p21;
}

/*****************************  (END OF FILE) *********************************/
