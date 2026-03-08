/*
 * modbus_slave.c - Modbus RTU slave. Implements 01H/03H/05H/06H/10H, register map (VAR_T),
 * coil/register Var_* accessors. RX timeout via TIM2 (3.5 char). Flash: see modbus_flash.h.
 */

#include "modbus_slave.h"
#include "main.h"
#include <string.h>
#include "usart.h"
#include "tim.h"
#include "oled.h"
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
*********************************************************************************************************
*	                                   ????
*********************************************************************************************************
*/



MODS_T g_tModS = {0};
VAR_T g_tVar = { .S1 = { .nox_cal_trig = 0xFFFFu, .o2_cal_trig = 0xFFFFu },
                 .S2 = { .nox_cal_trig = 0xFFFFu, .o2_cal_trig = 0xFFFFu } };

/* Mutex-protected read/write macros for single g_tVar fields; avoid repeating LOCK_VAR/UNLOCK_VAR. */
#define VAR_READ_U16(member, ret)   do { LOCK_VAR(); (ret) = g_tVar.member; UNLOCK_VAR(); } while(0)
#define VAR_READ_FLOAT(member, ret) do { LOCK_VAR(); (ret) = g_tVar.member; UNLOCK_VAR(); } while(0)
#define VAR_WRITE_U16(member, val)  do { LOCK_VAR(); g_tVar.member = (val); UNLOCK_VAR(); } while(0)
#define VAR_WRITE_FLOAT(member, val) do { LOCK_VAR(); g_tVar.member = (val); UNLOCK_VAR(); } while(0)


// RX semaphore (signalled on 3.5 char timeout)
QueueHandle_t MODRx_SemaphoreHandle;


// Relay GPIO table (D01-D04)
// Relay pins: Relay0/1 blowback, Relay2/3 calibration etc.
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} GPIOPin_t;

// Relay port/pin array for coil readback
const GPIOPin_t relayPins[] = {
    {Relay0_GPIO_Port, Relay0_Pin},
    {Relay1_GPIO_Port, Relay1_Pin},
    {Relay2_GPIO_Port, Relay2_Pin},
    {Relay3_GPIO_Port, Relay3_Pin}
};


/*
*********************************************************************************************************
*	                                  ????????
*********************************************************************************************************
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

    /* After 3.5 char timeout callback gives semaphore; we take it here. */
    
    if(pdTRUE==xSemaphoreTake(MODRx_SemaphoreHandle,portMAX_DELAY)){

        if (g_tModS.RxCount < 4)				/* need at least 4 bytes: addr+func+reg */
        {
            goto err_ret;
        }

        /* CRC16 over frame; result 0 means valid */
        crc1 = CRC16_Modbus(g_tModS.RxBuf, g_tModS.RxCount);
        if (crc1 != 0)
        {
            goto err_ret;
        }

        /* slave address check */
        addr = g_tModS.RxBuf[0];				/* byte 1: address */
        if (addr != SADDR485)		 			/* not for us */
        {
            goto err_ret;
        }

        /* dispatch: MODS_AnalyzeApp ?????????????????????????? NOxDefault ????? list ?????? */
				MODS_AnalyzeApp();
    
    }
    

err_ret:
    g_tModS.RxCount = 0;					/* clear for next frame */
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
*********************************************************************************************************
*	?? ?? ??: HAL_TIM_OC_DelayElapsedCallback
*	???????: tim2?????????????????3.5????????????????
*	??    ??: ??
*	?? ?? ??: ??
*********************************************************************************************************
*/

//void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
//    if (htim->Instance == TIM2) {
//        // ???????????1????????
//        if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC1) != RESET) {
//            __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC1);
//          //???????
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
void MODS_ReciveNew(uint8_t _byte)
{
    uint8_t i;

    /* Find baud in table to get timeout (us); timer 1 = slave, timer 2 = host */
    for(i = 0; i < MODBUS_BAUD_RATE_LEN; i++)
    {
        if(SBAUD485 == ModbusBaudRate[i].Bps)
        {

            break;
        }
    }

    /* Start timer (us); TIM1 for slave, TIM2 for host */
    StartHardTimer(ModbusBaudRate[i].usTimeOut, (void *)MODS_RxTimeOut);

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
    xSemaphoreGiveFromISR(MODRx_SemaphoreHandle, NULL);
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

    txbuf[0] = g_tModS.RxBuf[0];					/* echo slave addr */
    txbuf[1] = g_tModS.RxBuf[1] | 0x80;				/* set MSB for error */
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

    for (i = 0; i < 6; i++)
    {
        txbuf[i] = g_tModS.RxBuf[i];
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
    switch (g_tModS.RxBuf[1])				/* byte 2: function code */
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
    /*
     ??????
    	????????:
    		01 ??????
    		01 ??????
    		00 ?????????????????
    		13 ?????????????????
    		00 ??????????????
    		25 ??????????????
    		0E CRC?????????
    		84 CRC?????????

    	??????: 	1????ON??0????OFF?????????????????8????????????????????????????0????. BIT0?????1??
    		01 ??????
    		01 ??????
    		05 ?????????
    		CD ????1(???0013H-???001AH)
    		6B ????2(???001BH-???0022H)
    		B2 ????3(???0023H-???002AH)
    		0E ????4(???0032H-???002BH)
    		1B ????5(???0037H-???0033H)
    		45 CRC?????????
    		E6 CRC?????????

    	????:
    		01 01 00 01 00 03   xx xx	--- ?????D01?????3??????????
    		01 01 00 03 00 01   xx xx   --- ?????D03??????????
    */
    uint16_t reg;
    uint16_t num;
    uint16_t i;
    uint16_t m;
    uint8_t status[10];

    g_tModS.RspCode = RSP_OK;

    /** ??1???? ????????????????? ===============================================================*/
    /*  ?????????????????????
    	?????8bit??+?????8bit??+???????????????????16bit??+???????????16bit??+ CRC16
    */
    if (g_tModS.RxCount != 8)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;				/* ??????????? */
        return;
    }

    /** ??2???? ??????? ===========================================================================*/
    /* ???????????????????????? 0???????? */
    reg = BEBufToUint16(&g_tModS.RxBuf[2]); 			/* ??????? */
    num = BEBufToUint16(&g_tModS.RxBuf[4]);				/* ????????? */

    /* ??????????????????? */
    m = (num + 7) / 8;
    
    /* ?????????????????????? */
    if ( (num > 0) && (reg + num+ REG_D01 <= REG_DXX + 1))
    {
        for (i = 0; i < m; i++)
        {
            status[i] = 0;
        }

        LOCK_VAR();
        for (i = 0; i < num; i++)
        {
            //???????????
            GPIO_PinState state = HAL_GPIO_ReadPin(relayPins[i].port, relayPins[i].pin);
            switch(i) {
                case 0: g_tVar.D01 = state; break;
                case 1: g_tVar.D02 = state; break;
                case 2: g_tVar.D03 = state; break;
                case 3: g_tVar.D04 = state; break;
            }
            status[i / 8]|=(state<< (i % 8));
        }
        UNLOCK_VAR();
    }
    else
    {
        g_tModS.RspCode = RSP_ERR_REG_ADDR;				/* ????????????? */
    }
    
    /** ??3???? ??????? =========================================================================*/
    if (g_tModS.RspCode == RSP_OK)						/* ?????? */
    {
        g_tModS.TxCount = 0;
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[0]; /* ????????? */
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[1]; /* ????????? */
        g_tModS.TxBuf[g_tModS.TxCount++] = m;				 /* ????????? */

        for (i = 0; i < m; i++)
        {
            g_tModS.TxBuf[g_tModS.TxCount++] = status[i];	/* ??????????? */
        }
        MODS_SendWithCRC(g_tModS.TxBuf, g_tModS.TxCount);
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);				/* ???????????????? */
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
    /*
    	????????11H????????????????????006BH???????????006DH??????????????????3????????????

    	????????:
    		11 ??????
    		03 ??????
    		00 ??????????????
    		6B ??????????????
    		00 ??????????????
    		03 ??????????????
    		76 CRC??????
    		87 CRC??????

    	??????: 	???????????????2????????????????????????????????????????????????????
    				?????????????????????????????????????????????????????????????????
    		11 ??????
    		03 ??????
    		06 ?????
    		00 ????1??????(006BH)
    		6B ????1??????(006BH)
    		00 ????2??????(006CH)
    		13 ????2 ??????(006CH)
    		00 ????3??????(006DH)
    		00 ????3??????(006DH)
    		38 CRC??????
    		B9 CRC??????

    	????:
    		01 03 30 06 00 01  6B0B      ---- ?? 3006H, ????????
    		01 03 4000 0010 51C6         ---- ?? 4000H ??????1????????? 32???
    		01 03 4001 0010 0006         ---- ?? 4001H ??????1????????? 32???

    		01 03 F000 0008 770C         ---- ?? F000H ??????1????????? 16???
    		01 03 F001 0008 26CC         ---- ?? F001H ??????2????????? 16???

    		01 03 7000 0020 5ED2         ---- ?? 7000H ??????1????????????1?? 64???
    		01 03 7001 0020 0F12         ---- ?? 7001H ??????1????????????2?? 64???

    		01 03 7040 0020 5F06         ---- ?? 7040H ??????2????????????1?? 64???
    */
    uint16_t reg;
    uint16_t num;
    uint16_t i;
    uint8_t reg_value[128];

    g_tModS.RspCode = RSP_OK;

    /** ??1???? ????????????????? ===============================================================*/
    /* ?????8bit??+?????8bit??+???????????????????16bit??+???????????16bit??+ CRC16 */
    if (g_tModS.RxCount != 8)								/* 03H?????????8????? */
    {
        g_tModS.RspCode = RSP_ERR_VALUE;					/* ??????????? */
        goto err_ret;
    }

    /** ??2???? ??????? ===========================================================================*/
    /* ???????????????????????? */
    reg = BEBufToUint16(&g_tModS.RxBuf[2]); 				/* ??????? */
    num = BEBufToUint16(&g_tModS.RxBuf[4]);					/* ????????? */

    /* ??????????????????????? */
    //
//    if(reg+SLAVE_REG_START+num-1>SLAVE_REG_END){
//        g_tModS.RspCode = RSP_ERR_VALUE;					/* ??????????? */
//        goto err_ret;    
//    }
    if (num > sizeof(reg_value) / 2)
	{
		g_tModS.RspCode = RSP_ERR_VALUE;					/* ??????????? */
		goto err_ret;
	}
    
    /* ?????????????reg_value???? */
    for (i = 0; i < num; i++)
    {

         uint8_t read_state=MODS_ReadRegValue(reg, &reg_value[2 * i]);
         if ( read_state== 0)	/* ??????????????reg_value?????????????????????????????? */
        {
            g_tModS.RspCode = RSP_ERR_REG_ADDR;				/* ????????????? */
            break;
        }else if(read_state==2){
            //??????
            ++i;
            ++reg;
        }

         ++reg;

    }
    
    /** ??3???? ??????? =========================================================================*/
err_ret:
    if (g_tModS.RspCode == RSP_OK)							 /* ?????? */
    {
        g_tModS.TxCount = 0;
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[0]; /* ????????? */
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[1]; /* ????????? */

        g_tModS.TxBuf[g_tModS.TxCount++] =num * 2;			 /* ????????? */
      
        

        for (i = 0; i < num; i++)                            /* ????????*/
        {
            g_tModS.TxBuf[g_tModS.TxCount++] = reg_value[2*i];
            g_tModS.TxBuf[g_tModS.TxCount++] = reg_value[2*i+1];
        }

        MODS_SendWithCRC(g_tModS.TxBuf, g_tModS.TxCount);	/* ??????????? */
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);					/* ?????????? */
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
    /*
    	????????: ????????????????FF00H?????????????ON????0000H?????????????OFF???
    	??05H???????????????????15H???????????????????????
    		11 ??????
    		05 ??????
    		00 ??????????????
    		AC ??????????????
    		FF ????1??????
    		00 ????2??????
    		4E CRC?????????
    		8B CRC?????????

    	??????:
    		11 ??????
    		05 ??????
    		00 ??????????????
    		AC ??????????????
    		FF ?????1??????
    		00 ?????1??????
    		4E CRC?????????
    		8B CRC?????????

    	????:
    	01 05 10 01 FF 00   D93A   -- D01??
    	01 05 10 01 00 00   98CA   -- D01???

    	01 05 10 02 FF 00   293A   -- D02??
    	01 05 10 02 00 00   68CA   -- D02???

    	01 05 10 03 FF 00   78FA   -- D03??
    	01 05 10 03 00 00   390A   -- D03???
    */
    uint16_t reg;
    uint16_t value;

    g_tModS.RspCode = RSP_OK;

    /** ??1???? ????????????????? ===============================================================*/
    /* ?????8bit??+?????8bit??+???????????????????16bit??+???????????16bit??+ CRC16 */
    if (g_tModS.RxCount != 8)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;		/* ??????????? */
        goto err_ret;
    }

    /** ??2???? ??????? ===========================================================================*/
    /* ???????????????????????? */
    reg = BEBufToUint16(&g_tModS.RxBuf[2]); 	/* ??????? */
    value = BEBufToUint16(&g_tModS.RxBuf[4]);	/* ???? */

    if (value != 0x0000 && value != 0xFF00)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;		/* ??????????? */
        goto err_ret;
    }
    if(value == 0xFF00)value =1;
    
    
    /* ???????? ????????????FF00H?????????????ON????0000H?????????????OFF???*/
    LOCK_VAR();
    if (reg+REG_D01 == REG_D01)
    {
        
        g_tVar.D01 = value;
        HAL_GPIO_WritePin(relayPins[0].port, relayPins[0].pin,(GPIO_PinState)value);
    }
    else if (reg+REG_D01 == REG_D02)
    {
        g_tVar.D02 = value;
        HAL_GPIO_WritePin(relayPins[1].port, relayPins[1].pin,(GPIO_PinState)value);
    }
    else if (reg+REG_D01 == REG_D03)
    {
        g_tVar.D03 = value;
        HAL_GPIO_WritePin(relayPins[2].port, relayPins[2].pin,(GPIO_PinState)value);
    }
    else if (reg+REG_D01 == REG_D04)
    {
        g_tVar.D04 = value;
        HAL_GPIO_WritePin(relayPins[3].port, relayPins[3].pin,(GPIO_PinState)value);
    }
    else
    {
        g_tModS.RspCode = RSP_ERR_REG_ADDR;		/* ????????????? */
    }
    UNLOCK_VAR();
    
    /** ??3???? ??????? =========================================================================*/
err_ret:
    if (g_tModS.RspCode == RSP_OK)				/* ?????? */
    {
        MODS_SendAckOk();
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);		/* ???????????????? */
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

    /*
    	???????????????06????????????????????????16??????????????????????????????

    	????????:
    		11 ??????
    		06 ??????
    		00 ??????????????
    		01 ??????????????
    		00 ????1??????
    		01 ????1??????
    		9A CRC?????????
    		9B CRC?????????

    	??????:
    		11 ??????
    		06 ??????
    		00 ??????????????
    		01 ??????????????
    		00 ????1??????
    		01 ????1??????
    		1B CRC?????????
    		5A	CRC?????????

    	????:
    		01 06 30 06 00 25  A710    ---- ?????????????? 2.5
    		01 06 30 06 00 10  6707    ---- ?????????????? 1.0


    		01 06 30 1B 00 00  F6CD    ---- SMA ?????? = 0 ??????
    		01 06 30 1B 00 01  370D    ---- SMA ?????? = 1
    		01 06 30 1B 00 02  770C    ---- SMA ?????? = 2
    		01 06 30 1B 00 05  36CE    ---- SMA ?????? = 5

    		01 06 30 07 00 01  F6CB    ---- ??????????? T1
    		01 06 30 07 00 02  B6CA    ---- ??????????? T2

    		01 06 31 00 00 00  8736    ---- ?????????????
    		01 06 31 01 00 00  D6F6    ---- ???????????????

    */

    uint16_t reg;
    uint8_t write_state;

    g_tModS.RspCode = RSP_OK;

    /** ??1???? ????????????????? ===============================================================*/
    /* ?????8bit??+????8bit??+???????????????????16bit??+????????16bit??+ CRC16 */
    if (g_tModS.RxCount != 8)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;		/* ??????????? */
        goto err_ret;
    }

    /** ??2???? ??????? ===========================================================================*/
    /* ???????????????????????? */
    reg = BEBufToUint16(&g_tModS.RxBuf[2]); 	/* ??????? */
    //value = BEBufToUint16(&g_tModS.RxBuf[4]);	/* ??????? */
    
    write_state = MODS_WriteRegValue(reg, &g_tModS.RxBuf[4]);
    
    if (write_state!=0)	/* ??????????????????????? */
    {
        ;
    }
    else
    {
        g_tModS.RspCode = RSP_ERR_REG_ADDR;		/* ????????????? */
         
    }

    /** ??3???? ??????? =========================================================================*/
err_ret:
    if (g_tModS.RspCode == RSP_OK)				/* ?????? */
    {
        MODS_SendAckOk();
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);		/* ???????????????? */
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
    /*
    	????????11H????????????????????0001H?????????????????0002H?????????2?????????
    	????????0001H???????000AH??????????0002H???????0102H??

    	????????:
    		11 ??????
    		10 ??????
    		00 ?????????????????
    		01 ?????????????????
    		00 ??????????????
    		02 ??????????????
    		04 ?????
    		00 ????1??????
    		0A ????1??????
    		01 ????2??????
    		02 ????2??????
    		C6 CRC?????????
    		F0 CRC?????????

    	??????:
    		11 ??????
    		06 ??????
    		00 ??????????????
    		01 ??????????????
    		00 ????1??????
    		01 ????1??????
    		1B CRC?????????
    		5A	CRC?????????

    	????:
    		01 10 30 00 00 06 0C  07 DE  00 0A  00 01  00 08  00 0C  00 00     389A    ---- ????? 2014-10-01 08:12:00
    		01 10 30 00 00 06 0C  07 DF  00 01  00 1F  00 17  00 3B  00 39     5549    ---- ????? 2015-01-31 23:59:57

    */
    uint16_t reg_addr;
    uint16_t reg_num;
    uint16_t byte_num;
    uint8_t i;
//    uint16_t value;


    g_tModS.RspCode = RSP_OK;

    /** ??1???? ????????????????? ===============================================================*/
    /* ?????8bit??+?????8bit??+???????????????????16bit??+???????????16bit??+ ???????8bit??+ ???????????16bit??+ CRC16 */
    if (g_tModS.RxCount < 11)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;			/* ??????????? */
        goto err_ret;
    }

    /** ??2???? ??????? ===========================================================================*/
    /* ???????????????????????? */
    reg_addr = BEBufToUint16(&g_tModS.RxBuf[2]); 	/* ??????? */
    reg_num = BEBufToUint16(&g_tModS.RxBuf[4]);		/* ????????? */
    byte_num = g_tModS.RxBuf[6];					/* ???????????????? */

    /* ???????????????????????????????? */
    if (byte_num != 2 * reg_num)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;			/* ??????????? */
        goto err_ret;
    }
    
    
    /* ???????? */
    for (i = 0; i < reg_num; i++)
    {
        
        //value = BEBufToUint16(&g_tModS.RxBuf[7 + 2 * i]);	/* ??????? */
        
        uint8_t write_state=MODS_WriteRegValue(reg_addr, &g_tModS.RxBuf[7 + 2 * i]);
        if (write_state == 2)
        {
            ++i;
            ++reg_addr;
        }
        else if(write_state == 0)
        {
            g_tModS.RspCode = RSP_ERR_REG_ADDR;		/* ????????????? */
           
            break;
        }
        ++reg_addr;
    }
    
    /** ??3???? ??????? =========================================================================*/
err_ret:
    if (g_tModS.RspCode == RSP_OK)					/* ?????? */
    {
        MODS_SendAckOk();
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);			/* ???????????????? */
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
            case SLAVE_REG_NOX_OUTPUT:      f_value = g_tVar.P01; f_flag = 1; break;
            case SLAVE_REG_O2_OUTPUT:       f_value = g_tVar.P02; f_flag = 1; break;
            case SLAVE_REG_OUTPUT_CH_STATUS: value = g_tVar.P07; break;
            case SLAVE_REG_ALARM_NOX_HI:    f_value = g_tVar.P12; f_flag = 1; break;
            case SLAVE_REG_ALARM_O2_LO:     f_value = g_tVar.P13; f_flag = 1; break;
            case SLAVE_REG_MA_NOX:          value = g_tVar.P22; break;
            case SLAVE_REG_MA_O2:           value = g_tVar.P23; break;
            case SLAVE_REG_WORK_MODE:       value = g_tVar.P34; break;
            default: UNLOCK_VAR(); return 0;
        }
        UNLOCK_VAR();
    } else if (addr >= SENSOR_BASE_1 && addr <= SLAVE_REG_S1_BLOW_CMD) {
        LOCK_VAR();
        s = &g_tVar.S1;
        goto sensor_read;
    } else if (addr >= SENSOR_BASE_2 && addr <= SLAVE_REG_S2_BLOW_CMD) {
        LOCK_VAR();
        s = &g_tVar.S2;
        goto sensor_read;
    } else {
        return 0;
    }
    goto output;

sensor_read:
    switch (addr) {
        case 40013: case 40014: f_value = s->live_nox; f_flag = 1; break;
        case 40015: case 40016: f_value = s->live_o2;  f_flag = 1; break;
        case 40017: value = s->status; break;
        case 40018: case 40019: f_value = s->seg1_nox_a; f_flag = 1; break;
        case 40020: case 40021: f_value = s->seg1_nox_b; f_flag = 1; break;
        case 40022: case 40023: f_value = s->seg1_o2_a;  f_flag = 1; break;
        case 40024: case 40025: f_value = s->seg1_o2_b;  f_flag = 1; break;
        case 40026: case 40027: f_value = s->seg2_nox_a; f_flag = 1; break;
        case 40028: case 40029: f_value = s->seg2_nox_b; f_flag = 1; break;
        case 40030: case 40031: f_value = s->seg2_o2_a;  f_flag = 1; break;
        case 40032: case 40033: f_value = s->seg2_o2_b;  f_flag = 1; break;
        case 40034: case 40035: f_value = s->p2_nox; f_flag = 1; break;
        case 40036: case 40037: f_value = s->p2_o2;  f_flag = 1; break;
        case 40038: case 40039: f_value = s->p3_nox; f_flag = 1; break;
        case 40040: case 40041: f_value = s->p3_o2;  f_flag = 1; break;
        case 40042: value = s->nox_cal_trig; break;
        case 40043: value = s->nox_pt_sel; break;
        case 40044: value = s->o2_cal_trig; break;
        case 40045: value = s->o2_pt_sel; break;
        case 40046: value = s->blow_interval; break;
        case 40047: value = s->blow_duration; break;
        case 40048: value = s->blow_status; break;
        case 40049: value = s->blow_countdown; break;
        case 40050: value = s->blow_cmd; break;
        case 40056: case 40057: f_value = s->seg1_nox_a; f_flag = 1; break;
        case 40058: case 40059: f_value = s->seg1_nox_b; f_flag = 1; break;
        case 40060: case 40061: f_value = s->seg1_o2_a;  f_flag = 1; break;
        case 40062: case 40063: f_value = s->seg1_o2_b;  f_flag = 1; break;
        case 40064: case 40065: f_value = s->seg2_nox_a; f_flag = 1; break;
        case 40066: case 40067: f_value = s->seg2_nox_b; f_flag = 1; break;
        case 40068: case 40069: f_value = s->seg2_o2_a;  f_flag = 1; break;
        case 40070: case 40071: f_value = s->seg2_o2_b;  f_flag = 1; break;
        case 40072: case 40073: f_value = s->p2_nox; f_flag = 1; break;
        case 40074: case 40075: f_value = s->p2_o2;  f_flag = 1; break;
        case 40076: case 40077: f_value = s->p3_nox; f_flag = 1; break;
        case 40078: case 40079: f_value = s->p3_o2;  f_flag = 1; break;
        case 40080: value = s->nox_cal_trig; break;
        case 40081: value = s->nox_pt_sel; break;
        case 40082: value = s->o2_cal_trig; break;
        case 40083: value = s->o2_pt_sel; break;
        case 40084: value = s->blow_interval; break;
        case 40085: value = s->blow_duration; break;
        case 40086: value = s->blow_status; break;
        case 40087: value = s->blow_countdown; break;
        case 40088: value = s->blow_cmd; break;
        case 40051: case 40052: f_value = s->live_nox; f_flag = 1; break;
        case 40053: case 40054: f_value = s->live_o2;  f_flag = 1; break;
        case 40055: value = s->status; break;
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
            case SLAVE_REG_NOX_OUTPUT:      value1 = BEBufToUint16(reg_value + 2); g_tVar.P01 = RegistersToFloat_BE(value, value1); f_flag = 1; break;
            case SLAVE_REG_O2_OUTPUT:       value1 = BEBufToUint16(reg_value + 2); g_tVar.P02 = RegistersToFloat_BE(value, value1); f_flag = 1; break;
            case SLAVE_REG_ALARM_NOX_HI:    value1 = BEBufToUint16(reg_value + 2); g_tVar.P12 = RegistersToFloat_BE(value, value1); f_flag = 1; break;
            case SLAVE_REG_ALARM_O2_LO:     value1 = BEBufToUint16(reg_value + 2); g_tVar.P13 = RegistersToFloat_BE(value, value1); f_flag = 1; break;
            case SLAVE_REG_MA_NOX:          g_tVar.P22 = value; break;
            case SLAVE_REG_MA_O2:           g_tVar.P23 = value; break;
            case SLAVE_REG_WORK_MODE:       g_tVar.P34 = value; break;
            default: UNLOCK_VAR(); return 0;
        }
        UNLOCK_VAR();
        return f_flag ? 2 : 1;
    }

    if (addr >= SENSOR_BASE_1 && addr <= SLAVE_REG_S1_BLOW_CMD)
        s = &g_tVar.S1;
    else if (addr >= SENSOR_BASE_2 && addr <= SLAVE_REG_S2_BLOW_CMD)
        s = &g_tVar.S2;
    else
        return 0;

    LOCK_VAR();
    switch (addr) {
        case 40013: case 40014: value1 = BEBufToUint16(reg_value + 2); s->live_nox = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40015: case 40016: value1 = BEBufToUint16(reg_value + 2); s->live_o2  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40018: case 40019: value1 = BEBufToUint16(reg_value + 2); s->seg1_nox_a = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40020: case 40021: value1 = BEBufToUint16(reg_value + 2); s->seg1_nox_b = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40022: case 40023: value1 = BEBufToUint16(reg_value + 2); s->seg1_o2_a  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40024: case 40025: value1 = BEBufToUint16(reg_value + 2); s->seg1_o2_b  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40026: case 40027: value1 = BEBufToUint16(reg_value + 2); s->seg2_nox_a = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40028: case 40029: value1 = BEBufToUint16(reg_value + 2); s->seg2_nox_b = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40030: case 40031: value1 = BEBufToUint16(reg_value + 2); s->seg2_o2_a  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40032: case 40033: value1 = BEBufToUint16(reg_value + 2); s->seg2_o2_b  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40034: case 40035: value1 = BEBufToUint16(reg_value + 2); s->p2_nox = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40036: case 40037: value1 = BEBufToUint16(reg_value + 2); s->p2_o2  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40038: case 40039: value1 = BEBufToUint16(reg_value + 2); s->p3_nox = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40040: case 40041: value1 = BEBufToUint16(reg_value + 2); s->p3_o2  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40042: s->nox_cal_trig = value; UNLOCK_VAR(); return 1;
        case 40043: s->nox_pt_sel = value; UNLOCK_VAR(); return 1;
        case 40044: s->o2_cal_trig = value; UNLOCK_VAR(); return 1;
        case 40045: s->o2_pt_sel = value; UNLOCK_VAR(); return 1;
        case 40046: s->blow_interval = value; UNLOCK_VAR(); return 1;
        case 40047: s->blow_duration = value; UNLOCK_VAR(); return 1;
        case 40050: s->blow_cmd = value; UNLOCK_VAR(); return 1;
        case 40051: case 40052: value1 = BEBufToUint16(reg_value + 2); s->live_nox = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40053: case 40054: value1 = BEBufToUint16(reg_value + 2); s->live_o2  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40056: case 40057: value1 = BEBufToUint16(reg_value + 2); s->seg1_nox_a = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40058: case 40059: value1 = BEBufToUint16(reg_value + 2); s->seg1_nox_b = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40060: case 40061: value1 = BEBufToUint16(reg_value + 2); s->seg1_o2_a  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40062: case 40063: value1 = BEBufToUint16(reg_value + 2); s->seg1_o2_b  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40064: case 40065: value1 = BEBufToUint16(reg_value + 2); s->seg2_nox_a = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40066: case 40067: value1 = BEBufToUint16(reg_value + 2); s->seg2_nox_b = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40068: case 40069: value1 = BEBufToUint16(reg_value + 2); s->seg2_o2_a  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40070: case 40071: value1 = BEBufToUint16(reg_value + 2); s->seg2_o2_b  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40072: case 40073: value1 = BEBufToUint16(reg_value + 2); s->p2_nox = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40074: case 40075: value1 = BEBufToUint16(reg_value + 2); s->p2_o2  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40076: case 40077: value1 = BEBufToUint16(reg_value + 2); s->p3_nox = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40078: case 40079: value1 = BEBufToUint16(reg_value + 2); s->p3_o2  = RegistersToFloat_BE(value, value1); UNLOCK_VAR(); return 2;
        case 40080: s->nox_cal_trig = value; UNLOCK_VAR(); return 1;
        case 40081: s->nox_pt_sel = value; UNLOCK_VAR(); return 1;
        case 40082: s->o2_cal_trig = value; UNLOCK_VAR(); return 1;
        case 40083: s->o2_pt_sel = value; UNLOCK_VAR(); return 1;
        case 40084: s->blow_interval = value; UNLOCK_VAR(); return 1;
        case 40085: s->blow_duration = value; UNLOCK_VAR(); return 1;
        case 40088: s->blow_cmd = value; UNLOCK_VAR(); return 1;
        default: UNLOCK_VAR(); return 0;
    }
}

// Convert two BE registers to float
float RegistersToFloat_BE(uint16_t reg1, uint16_t reg2) {
    converter.u = ((uint32_t)reg1 << 16) | reg2;
    return converter.f;
}

// ========================== Coil D01-D04 ==========================
void Var_Write_D01(uint16_t value) { VAR_WRITE_U16(D01, value); }
uint16_t Var_Read_D01(void) { uint16_t r; VAR_READ_U16(D01, r); return r; }

void Var_Write_D02(uint16_t value) { VAR_WRITE_U16(D02, value); }
uint16_t Var_Read_D02(void) { uint16_t r; VAR_READ_U16(D02, r); return r; }

void Var_Write_D03(uint16_t value) { VAR_WRITE_U16(D03, value); }
uint16_t Var_Read_D03(void) { uint16_t r; VAR_READ_U16(D03, r); return r; }

void Var_Write_D04(uint16_t value) { VAR_WRITE_U16(D04, value); }
uint16_t Var_Read_D04(void) { uint16_t r; VAR_READ_U16(D04, r); return r; }

// ========================== Common P01, P02, P07, P12, P13, P22, P23, P34 ==========================
float Var_Read_NoxOutput(void) { float r; VAR_READ_FLOAT(P01, r); return r; }
float Var_Read_O2Output(void) { float r; VAR_READ_FLOAT(P02, r); return r; }
uint16_t Var_Read_OutputChStatus(void) { uint16_t r; VAR_READ_U16(P07, r); return r; }
void Var_Write_NoxOutput(float value) { VAR_WRITE_FLOAT(P01, value); }
void Var_Write_O2Output(float value) { VAR_WRITE_FLOAT(P02, value); }
void Var_Write_OutputChStatus(uint16_t value) { VAR_WRITE_U16(P07, value); }
void Var_Write_AlarmNoxHi(float value) { VAR_WRITE_FLOAT(P12, value); }
float Var_Read_AlarmNoxHi(void) { float r; VAR_READ_FLOAT(P12, r); return r; }
void Var_Write_AlarmO2Lo(float value) { VAR_WRITE_FLOAT(P13, value); }
float Var_Read_AlarmO2Lo(void) { float r; VAR_READ_FLOAT(P13, r); return r; }
void Var_Write_MaNox(uint16_t value) { VAR_WRITE_U16(P22, value); }
uint16_t Var_Read_MaNox(void) { uint16_t r; VAR_READ_U16(P22, r); return r; }
void Var_Write_MaO2(uint16_t value) { VAR_WRITE_U16(P23, value); }
uint16_t Var_Read_MaO2(void) { uint16_t r; VAR_READ_U16(P23, r); return r; }
void Var_Write_WorkMode(uint16_t value) { VAR_WRITE_U16(P34, value); }
uint16_t Var_Read_WorkMode(void) { uint16_t r; VAR_READ_U16(P34, r); return r; }

// ========================== Sensor accessors by channel (ch=0 or 1) ==========================
#define S(ch) ((ch) == 0 ? &g_tVar.S1 : &g_tVar.S2)

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

#undef S

void Var_Read_BlowbackCfg(uint16_t *p24, uint16_t *p25) {
    LOCK_VAR();
    if (p24) *p24 = g_tVar.S1.blow_interval;
    if (p25) *p25 = g_tVar.S1.blow_duration;
    UNLOCK_VAR();
}

void Var_Read_AlarmCfg(float *p12, float *p13) {
    LOCK_VAR();
    if (p12) *p12 = g_tVar.P12;
    if (p13) *p13 = g_tVar.P13;
    UNLOCK_VAR();
}

// ========================== ???/??????? P01/P02=NOx/O?, P07=?? ==========================
void Var_Update_SensorCore(float nox, float o2, uint16_t state) {
    LOCK_VAR();
    g_tVar.P01 = nox;
    g_tVar.P02 = o2;
    g_tVar.P07 = state;
    UNLOCK_VAR();
}

void Var_Update_CalibPoint(float p18, float p19, float p20, float p21) {
    (void)p18; (void)p19; (void)p20; (void)p21;
}

/*****************************  (END OF FILE) *********************************/
