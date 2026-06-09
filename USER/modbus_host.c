/**
 * @file    modbus_host.c
 * @brief   Modbus RTU master: writes 4-20 mA output to external AO modules.
 *          Optional readback is controlled by AO_MODULE_READBACK_ENABLE.
 *          Uses UART5/RS485 and software timer for reply timeout.
 */
#include "main.h"
#include <string.h>
#include "usart.h"
#include "tim.h"
#include "modbus_host.h"
#include "modbus_slave.h"

uint8_t SlaveAddr = AO_MODULE_SLAVE_1_ADDR;
uint32_t HBAUD485 = MODBUS_HOST_DEFAULT_BAUD;

void (*s_TIM_CallBack2)(void);

MODH_T g_tModH = {0};
uint8_t g_modh_timeout = 0;
VAR_T_H g_tVar_h = {0};
volatile int32_t time1 = 0;
volatile uint8_t g_ao_addr_config_status = AO_ADDR_CONFIG_STATUS_IDLE;

QueueHandle_t RS485send_SemaphoreHandle;
QueueHandle_t MODHx_SemaphoreHandle;
TimerHandle_t MODH_Timer;

static uint16_t s_write06_reg = 0u;
static uint16_t s_write06_value = 0u;

/* ----- Forward declarations ----- */

static void MODH_SendWithCRC(void);
static void MODH_AnalyzeApp(void);
static void MODH_Read_03H(void);
static void MODH_RxTimeOut(void);
void Start_Receive_H(void);
void MODH_Poll(void);

static void MODH_Read_06H(void);
static void MODH_Read_10H(void);
static uint8_t MODH_ReadParam_03H_Addr(uint8_t _addr, uint16_t _reg, uint16_t _num);
static uint8_t MODH_WriteParam_06H_Addr(uint8_t _addr, uint16_t _reg, uint16_t _value);
static uint8_t MODH_WriteParam_10H_Addr(uint8_t _addr, uint16_t _reg, uint8_t _num, uint8_t *_buf);
static void MODH_UpdateReadback(uint16_t reg, uint16_t value);

void Start_Receive_H(void) {
    HAL_UART_Receive_IT(&MDSUARTxH, &rx_data_h, 1);
}

/* RS485 host transmit with direction pin; non-blocking, skip if already sending */
void RS485_Send_Data_IT_H(uint8_t *pData, uint16_t Size) {
    if (rs485_state_h.isSending) {
        return;
    }
    rs485_state_h.isSending = 1;
    rs485_state_h.txStartTick = HAL_GetTick();
    RS485_Enable_TX(RS485_EN_PORT_H, RS485_EN_PIN_H);
    if (HAL_UART_Transmit_IT(&MDSUARTxH, pData, Size) != HAL_OK) {
        rs485_state_h.isSending = 0;
        rs485_state_h.txStartTick = 0;
        RS485_Enable_RX(RS485_EN_PORT_H, RS485_EN_PIN_H);
    }
}
/*
 * MODH_SendWithCRC: Append 2-byte CRC to TxBuf and send via RS485.
 * Uses g_tModH.TxBuf and g_tModH.TxCount.
 */

static void MODH_SendWithCRC(void)
{
	uint16_t crc;
	
	crc = CRC16_Modbus(g_tModH.TxBuf, g_tModH.TxCount);
	g_tModH.TxBuf[g_tModH.TxCount++] = crc >> 8;
	g_tModH.TxBuf[g_tModH.TxCount++] = crc;	
	
	
	RS485_Send_Data_IT_H(g_tModH.TxBuf, g_tModH.TxCount);

}


/*
 * MODH_Send03H — build and send Modbus 03H (read holding registers), one request.
 * _addr: slave address; _reg: start register; _num: number of registers.
 */

void MODH_Send03H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* slave address */
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x03;		/* function code 03 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* start reg high */
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* start reg low */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* quantity high */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* quantity low */

	MODH_SendWithCRC();		/* append CRC and transmit */
	g_tModH.fAck03H = 0;		/* clear ack until response parsed */
	g_tModH.RegNum = _num;
	g_tModH.Reg03H = _reg;		/* save for response handler */
}

/*
 * MODH_Send06H — build and send 06H (write single register).
 * _value: 16-bit value to write (big-endian on wire).
 */
void MODH_Send06H(uint8_t _addr, uint16_t _reg, uint16_t _value)
{
    g_tModH.fAck06H = 0;		/* clear until response matches */
    s_write06_reg = _reg;
    s_write06_value = _value;

	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x06;
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;
	g_tModH.TxBuf[g_tModH.TxCount++] = _value >> 8;
	g_tModH.TxBuf[g_tModH.TxCount++] = _value;

	MODH_SendWithCRC();
}
/*
 * MODH_Send10H — build and send 10H (write multiple registers).
 * _num: register count; _buf: 2*_num bytes payload (big-endian per register).
 */
void MODH_Send10H(uint8_t _addr, uint16_t _reg, uint8_t _num, uint8_t *_buf)
{
	uint16_t i;

	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x10;
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;
	g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;
	g_tModH.TxBuf[g_tModH.TxCount++] = _num;
	g_tModH.TxBuf[g_tModH.TxCount++] = 2 * _num;	/* byte count */

	for (i = 0; i < 2 * _num; i++)
	{
		if (g_tModH.TxCount > H_RX_BUF_SIZE - 3)
			return;		/* frame too large */
		g_tModH.TxBuf[g_tModH.TxCount++] = _buf[i];
	}

	g_tModH.fAck10H = 0;		/* clear until response matches */
	MODH_SendWithCRC();
}

/*
*********************************************************************************************************
*********************************************************************************************************
*/
//void StartHardTimer(uint32_t _uiTimeOut, void * _pCallBack) {


//    __HAL_TIM_SET_AUTORELOAD(&RX_TIMER, _uiTimeOut);
//    s_TIM_CallBack1 = (void (*)(void)) _pCallBack;

//    __HAL_TIM_SET_COUNTER(&RX_TIMER, 0);


//    __HAL_TIM_CLEAR_FLAG(&RX_TIMER, TIM_FLAG_UPDATE);
//    HAL_TIM_Base_Start_IT(&RX_TIMER);

//}
/*
*********************************************************************************************************
*********************************************************************************************************
*/
void StartHardTimer_H(uint32_t _uiTimeOut, void * _pCallBack) {


    __HAL_TIM_SET_AUTORELOAD(&RX_TIMER_H, _uiTimeOut);
    s_TIM_CallBack2 = (void (*)(void)) _pCallBack;

    __HAL_TIM_SET_COUNTER(&RX_TIMER_H, 0);


    __HAL_TIM_CLEAR_FLAG(&RX_TIMER_H, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&RX_TIMER_H);

}
/*
*********************************************************************************************************
*********************************************************************************************************
*/
void MODH_ReciveNew(uint8_t _data)
{
	/* 3.5 char timeout from baud table; restart timer on each byte (Modbus RTU). */
	uint8_t i;
	
	
	for (i = 0; i < MODBUS_BAUD_RATE_LEN; i++) {
		if (HBAUD485 == ModbusBaudRate[i].Bps)
			break;
	}
	if (i >= MODBUS_BAUD_RATE_LEN)
		i = MODBUS_BAUD_RATE_LEN - 1;  /* default if baud not in table */

	/* Timer 1 = slave, timer 2 = host RX timeout */
//	OLED_PrintASCIIString(10, 40,"2", &afont12x6, OLED_COLOR_NORMAL);
	StartHardTimer_H(ModbusBaudRate[i].usTimeOut, (void *)MODH_RxTimeOut);

	if (g_tModH.RxCount < H_RX_BUF_SIZE)
	{
		g_tModH.RxBuf[g_tModH.RxCount++] = _data;
	}
}

/*
*********************************************************************************************************
*********************************************************************************************************
*/
static void MODH_RxTimeOut(void)
{	
	//xSemaphoreGiveFromISR(MODHx_SemaphoreHandle,NULL);
	g_modh_timeout = 1;
}

/*
*********************************************************************************************************
*  Function: MODH_Poll
*  Description: Process received frame. Called from task (~1 ms response).
*  Returns: 0 = no data, 1 = valid frame processed
*********************************************************************************************************
*/
void MODH_Poll(void)
{	
	uint16_t crc1;
	if(g_modh_timeout==1)
	{
		g_modh_timeout = 0;
	
		/* Need at least 4 bytes: addr(8bit) + func(8bit) + reg(16bit) */
		if (g_tModH.RxCount < 4)
		{
			goto err_ret;
		}

		/* CRC16 over frame including CRC; result 0 means valid */
		crc1 = CRC16_Modbus(g_tModH.RxBuf, g_tModH.RxCount);
		if (crc1 != 0)
		{
			goto err_ret;
		}
	
		/* Parse application layer */
		MODH_AnalyzeApp();

	err_ret:
		g_tModH.RxCount = 0;	/* clear for next frame */
	}
}
/*
*********************************************************************************************************
*********************************************************************************************************
*/
static void MODH_AnalyzeApp(void)
{	
	switch (g_tModH.RxBuf[1])			
	{

//			MODH_Read_01H();
//			break;


//			MODH_Read_02H();
//			break;

		case 0x03:	
			MODH_Read_03H();
			break;


//			MODH_Read_04H();
//		
//			break;


//			MODH_Read_05H();
//			break;

		case 0x06:	
			MODH_Read_06H();
			break;		

		case 0x10:	
			MODH_Read_10H();
			break;
		default:
			break;
	}
}

/*
*********************************************************************************************************
*********************************************************************************************************
*/
static void MODH_Read_03H(void)
{
	uint8_t bytes;
	uint8_t *p;
	
	if (g_tModH.RxCount > 0)
	{
        if (g_tModH.RxBuf[0] != SlaveAddr)
            return;
		bytes = g_tModH.RxBuf[2];	
        if (bytes != (uint8_t)(g_tModH.RegNum * 2u))
            return;
        p = &g_tModH.RxBuf[3];
        for (int i=0;i<bytes/2;i++){
            uint16_t reg = (uint16_t)(g_tModH.Reg03H + i);
            uint16_t value;
            
            switch (reg)
            {
                case REG_P01:

                    value = BEBufToUint16(p); p += 2;
                    if (SlaveAddr == AO_MODULE_SLAVE_1_ADDR)
                        g_tVar_h.P01 = value;

                    MODH_UpdateReadback(reg, value);
                
                    g_tModH.fAck03H = 1;
                
                    break;
                case REG_P02:
 
                    value = BEBufToUint16(p); p += 2;
                    if (SlaveAddr == AO_MODULE_SLAVE_1_ADDR)
                        g_tVar_h.P02 = value;

                    MODH_UpdateReadback(reg, value);
                
                    g_tModH.fAck03H = 1;
                    break;
                case 0x000C:
                    value = BEBufToUint16(p); p += 2;
                    MODH_UpdateReadback(reg, value);
                    g_tModH.fAck03H = 1;
                    break;
                case 0x000D:
                    value = BEBufToUint16(p); p += 2;
                    MODH_UpdateReadback(reg, value);
                    g_tModH.fAck03H = 1;
                    break;
                case 0x000E:
                    value = BEBufToUint16(p); p += 2;
                    MODH_UpdateReadback(reg, value);
                    g_tModH.fAck03H = 1;
                    break;
                case REG_P06:
                    value = BEBufToUint16(p); p += 2;
                    MODH_UpdateReadback(reg, value);
                    g_tModH.fAck03H = 1;
                    break;
                case REG_P03:         
                    g_tVar_h.P03 = BEBufToUint16(p); p += 2;		
            
                    g_tModH.fAck03H = 1;

                    break;
                case REG_P04:
                    g_tVar_h.P04 = BEBufToUint16(p); p += 2;		
            
                    g_tModH.fAck03H = 1;

                    break;
                case REG_P05:
                    g_tVar_h.P05 = BEBufToUint16(p); p += 2;		
            
                    g_tModH.fAck03H = 1;
                    break;                
                default:
                    return ;
            }        
        
        }

		
	}
}
/*
*********************************************************************************************************
*********************************************************************************************************
*/
static void MODH_Read_06H(void)
{
	if (g_tModH.RxCount > 0)
	{
        if (g_tModH.RxCount != 8u)
            return;
		if (g_tModH.RxBuf[0] != SlaveAddr)
            return;
        if (g_tModH.RxBuf[1] != 0x06u)
            return;
        if (BEBufToUint16(&g_tModH.RxBuf[2]) != s_write06_reg)
            return;
        if (BEBufToUint16(&g_tModH.RxBuf[4]) != s_write06_value)
            return;

        g_tModH.fAck06H = 1;
	}
}

/*
*********************************************************************************************************
*********************************************************************************************************
*/
static void MODH_Read_10H(void)
{
	/* 10H response: echo addr, func, start reg, qty; set fAck10H if slave matches */
	if (g_tModH.RxCount > 0)
	{
		if (g_tModH.RxBuf[0] == SlaveAddr)		
		{
			g_tModH.fAck10H = 1;		
		}
	}
}

static void MODH_UpdateReadback(uint16_t reg, uint16_t value)
{
    if (SlaveAddr == AO_MODULE_SLAVE_1_ADDR) {
        if (reg == REG_P01)
            Var_Write_MaNox(value);
        else if (reg == REG_P02)
            Var_Write_MaO2(value);
        return;
    }

    if (SlaveAddr != AO_MODULE_SLAVE_2_ADDR)
        return;

    switch (reg) {
        case REG_P01: Var_Write_MaSensorNox(0u, value); break;
        case REG_P02: Var_Write_MaSensorO2(0u, value);  break;
        case 0x000C:  Var_Write_MaSensorNox(1u, value); break;
        case 0x000D:  Var_Write_MaSensorO2(1u, value);  break;
        case 0x000E:  Var_Write_MaSensorNox(2u, value); break;
        case REG_P06: Var_Write_MaSensorO2(2u, value);  break;
        default: break;
    }
}


/*
*********************************************************************************************************
*********************************************************************************************************
*/
uint8_t MODH_ReadParam_03H(uint16_t _reg, uint16_t _num)
{
    return MODH_ReadParam_03H_Addr(SlaveAddr, _reg, _num);
}

static uint8_t MODH_ReadParam_03H_Addr(uint8_t _addr, uint16_t _reg, uint16_t _num)
{;
	uint8_t i;
	
	for (i = 0; i < MODBUS_HOST_RETRY_NUM; i++)
	{
        SlaveAddr = _addr;
		MODH_Send03H (SlaveAddr, _reg, _num);
		xTimerStart( MODH_Timer, portMAX_DELAY);
				
		while (1)
		{
			MODH_Poll();
		
			if (time1 ==1)		
			{
				break;
			}
			
			if (g_tModH.fAck03H > 0)
			{
				break;
			}
			vTaskDelay(pdMS_TO_TICKS(1));
		}
		
		if (g_tModH.fAck03H > 0)
		{
            
			break;
		}
	}
	time1 =0;
	if (g_tModH.fAck03H == 0)
	{
		return 0;	/* communication timeout */
	}
	else 
	{
		return 1;	/* 03H read success */
	}
}

/*
*********************************************************************************************************
*********************************************************************************************************
*/
uint8_t MODH_WriteParam_06H(uint16_t _reg, uint16_t _value)
{
    return MODH_WriteParam_06H_Addr(SlaveAddr, _reg, _value);
}

static uint8_t MODH_WriteParam_06H_Addr(uint8_t _addr, uint16_t _reg, uint16_t _value)
{

	uint8_t i;
	
	for (i = 0; i < MODBUS_HOST_RETRY_NUM; i++)
	{	
        SlaveAddr = _addr;
		MODH_Send06H (SlaveAddr, _reg, _value);
		xTimerStart( MODH_Timer, portMAX_DELAY);
				
		while (1)
		{
			MODH_Poll();
		
			if (time1 ==1)		
			{
				break;
			}
			
			if (g_tModH.fAck06H > 0)
			{
				break;
			}
			vTaskDelay(pdMS_TO_TICKS(1));
		}
		
		if (g_tModH.fAck06H > 0)
		{
            
			break;
		}
	}
	time1 =0;
	if (g_tModH.fAck06H == 0)
	{
		return 0;	/* communication timeout */
	}
	else
	{
		return 1;	/* 06H write success */
	}
}
 /*
*********************************************************************************************************
*********************************************************************************************************
*/
uint8_t MODH_WriteParam_10H(uint16_t _reg, uint8_t _num, uint8_t *_buf)
{
    return MODH_WriteParam_10H_Addr(SlaveAddr, _reg, _num, _buf);
}

static uint8_t MODH_WriteParam_10H_Addr(uint8_t _addr, uint16_t _reg, uint8_t _num, uint8_t *_buf)
{

	uint8_t i;
	
	for (i = 0; i < MODBUS_HOST_RETRY_NUM; i++)
	{	
        SlaveAddr = _addr;
		MODH_Send10H(SlaveAddr, _reg, _num, _buf);
		xTimerStart( MODH_Timer, portMAX_DELAY);
				
		while (1)
		{
			MODH_Poll();
		
			if (time1 ==1)		
			{
				break;
			}
			
			if (g_tModH.fAck10H > 0)
			{
				break;
			}
			vTaskDelay(pdMS_TO_TICKS(1));
		}
		
		if (g_tModH.fAck10H > 0)
		{
			break;
		}
	}
	time1=0;
	if (g_tModH.fAck10H == 0)
	{
		return 0;	/* communication timeout */
	}
	else
	{
		return 1;	/* 10H write success */
	}
}


void Time_Out_Fun(TimerHandle_t xTimer) {
     time1=1;
}

uint8_t electricity_data_buf[4]={0};
uint8_t electricity_data_buf_6ch[12]={0};
void ModBusHost(void *argument)
{

    

    // Create semaphores (unused)
    MODHx_SemaphoreHandle=xSemaphoreCreateBinary( );
    // Send semaphore (unused)
    RS485send_SemaphoreHandle =xSemaphoreCreateBinary( );
    xSemaphoreGive(RS485send_SemaphoreHandle);
    
    
    // Software timer for slave reply timeout
    MODH_Timer = xTimerCreate( "rxtimer", pdMS_TO_TICKS(MODBUS_HOST_REPLY_TIMEOUT_MS), pdFALSE, NULL, Time_Out_Fun);
    
    // UART baud rate init
    MDSUARTxH.Init.BaudRate = HBAUD485;
    HAL_UART_Init(&MDSUARTxH);
    // Start RX interrupt
    Start_Receive_H();

#if AO_MODULE_SLAVE_2_ADDR_CONFIG_ENABLE
    g_ao_addr_config_status = MODH_WriteParam_06H_Addr(AO_MODULE_SLAVE_2_ADDR_CONFIG_FROM,
                                                       AO_MODULE_ADDR_REG,
                                                       AO_MODULE_SLAVE_2_ADDR_CONFIG_TO)
                              ? AO_ADDR_CONFIG_STATUS_SUCCESS
                              : AO_ADDR_CONFIG_STATUS_FAILED;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#else
    
    for(;;)
    {
        
        // Change address / device / parity if needed
        
        // Write register: set output current
        MODH_WriteParam_10H_Addr(AO_MODULE_SLAVE_1_ADDR,
                                 AO_MODULE_OUTPUT_REG_START,
                                 AO_MODULE_LEGACY_REG_COUNT,
                                 electricity_data_buf);
        vTaskDelay(pdMS_TO_TICKS(AO_MODULE_POLL_GAP_MS));
        
#if AO_MODULE_READBACK_ENABLE
        // Read back to verify actual current output
        MODH_ReadParam_03H_Addr(AO_MODULE_SLAVE_1_ADDR,
                                AO_MODULE_OUTPUT_REG_START,
                                AO_MODULE_LEGACY_REG_COUNT);
        vTaskDelay(pdMS_TO_TICKS(AO_MODULE_POLL_GAP_MS));
#endif

        MODH_WriteParam_10H_Addr(AO_MODULE_SLAVE_2_ADDR,
                                 AO_MODULE_OUTPUT_REG_START,
                                 AO_MODULE_6CH_REG_COUNT,
                                 electricity_data_buf_6ch);
        vTaskDelay(pdMS_TO_TICKS(AO_MODULE_POLL_GAP_MS));

#if AO_MODULE_READBACK_ENABLE
        MODH_ReadParam_03H_Addr(AO_MODULE_SLAVE_2_ADDR,
                                AO_MODULE_OUTPUT_REG_START,
                                AO_MODULE_6CH_REG_COUNT);
        vTaskDelay(pdMS_TO_TICKS(AO_MODULE_POLL_GAP_MS));
#endif

    }
#endif

}
