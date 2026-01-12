/*
*********************************************************************************************************
*
*	模块名称 : MODSBUS通信程序，主机模式【原创】
*	文件名称 : modbus_host.c
*	版    本 : V1.4
*	说    明 : MODBUS协议
*
*
*********************************************************************************************************
*/
#include "main.h"
#include <string.h>
#include "usart.h"
#include "tim.h"
#include "oled.h"
#include "modbus_host.h"
#include "modbus_slave.h"

uint8_t SlaveAddr=0x01;			
uint32_t HBAUD485=9600;

#define TIMEOUT		1000		/* 接收命令超时时间, 单位ms */
#define NUM			1			/* 循环发送次数 */



void (*s_TIM_CallBack2)(void);

MODH_T g_tModH = {0};
uint8_t g_modh_timeout = 0;

VAR_T_H g_tVar_h={0};

volatile int32_t time1=0;


QueueHandle_t RS485send_SemaphoreHandle;
QueueHandle_t MODHx_SemaphoreHandle;
TimerHandle_t MODH_Timer;

/*
*********************************************************************************************************
*	                                   函数声明
*********************************************************************************************************
*/

static void MODH_SendWithCRC(void);
static void MODH_AnalyzeApp(void);
static void MODH_Read_03H(void);
static void MODH_RxTimeOut(void);
void Start_Receive_H(void);
void MODH_Poll(void);

static void MODH_Read_06H(void);
static void MODH_Read_10H(void);

void Start_Receive_H(void){
     HAL_UART_Receive_IT(&MDSUARTxH,&rx_data_h, 1);
}


/*
*********************************************************************************************************
*	函 数 名: RS485_Send_Data_IT
*	功能说明: 中断方式发送数据，并设置方向引脚.  
*	形    参: 无。
*	返 回 值: 无
*********************************************************************************************************
*/

void RS485_Send_Data_IT_H(uint8_t *pData, uint16_t Size) {
    if (rs485_state_h.isSending) {
        return;
    }
    rs485_state_h.isSending = 1;
    RS485_Enable_TX(RS485_EN_PORT_H,RS485_EN_PIN_H);
    HAL_UART_Transmit_IT(&MDSUARTxH, pData, Size);
}
/*
*********************************************************************************************************
*	函 数 名: MODS_SendWithCRC
*	功能说明: 发送一串数据, 自动追加2字节CRC
*	形    参: _pBuf 数据；
*			  _ucLen 数据长度（不带CRC）
*	返 回 值: 无
*********************************************************************************************************
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
*********************************************************************************************************
*	函 数 名: MODH_Send03H
*	功能说明: 发送03H指令，查询1个或多个保持寄存器
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/

void MODH_Send03H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 */
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x03;		/* 功能码 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* 寄存器编号 低字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* 寄存器个数 低字节 */
	
	
	MODH_SendWithCRC();		/* 发送数据，自动加CRC */
	g_tModH.fAck03H = 0;		/* 清接收标志 */
	g_tModH.RegNum = _num;		/* 寄存器个数 */
	g_tModH.Reg03H = _reg;		/* 保存03H指令中的寄存器地址，方便对应答数据进行分类 */	
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send06H
*	功能说明: 发送06H指令，写1个保持寄存器
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _value : 寄存器值,2字节
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send06H(uint8_t _addr, uint16_t _reg, uint16_t _value)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;			/* 从站地址 */
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x06;			/* 功能码 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;		/* 寄存器编号 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;			/* 寄存器编号 低字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _value >> 8;		/* 寄存器值 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _value;			/* 寄存器值 低字节 */
	
	MODH_SendWithCRC();		/* 发送数据，自动加CRC */
	
	g_tModH.fAck06H = 0;		/* 如果收到从机的应答，则这个标志会设为1 */

}
/*
*********************************************************************************************************
*	函 数 名: MODH_Send10H
*	功能说明: 发送10H指令，连续写多个保持寄存器. 最多一次支持23个寄存器。
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数n (每个寄存器2个字节) 值域
*			  _buf : n个寄存器的数据。长度 = 2 * n
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send10H(uint8_t _addr, uint16_t _reg, uint8_t _num, uint8_t *_buf)
{
	uint16_t i;
	
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 */
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x10;		/* 从站地址 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* 寄存器编号 低字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* 寄存器个数 低字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = 2 * _num;	/* 数据字节数 */
	
	for (i = 0; i < 2 * _num; i++)
	{
		if (g_tModH.TxCount > H_RX_BUF_SIZE - 3)
		{
			return;		/* 数据超过缓冲区超度，直接丢弃不发送 */
		}
		g_tModH.TxBuf[g_tModH.TxCount++] = _buf[i];		/* 后面的数据长度 */
	}
	
	MODH_SendWithCRC();	/* 发送数据，自动加CRC */
}

/*
*********************************************************************************************************
*	函 数 名: StartHardTimer
*	功能说明: 开启定时器计时3.5个字符时间 此处使用定时器2
*	形    参: _uiTimeOut：超时时间
*            _pCallBack：超时回调函数函数指针
*	返 回 值: 无
*********************************************************************************************************
*/
//void StartHardTimer(uint32_t _uiTimeOut, void * _pCallBack) {


//    __HAL_TIM_SET_AUTORELOAD(&RX_TIMER, _uiTimeOut);
//    s_TIM_CallBack1 = (void (*)(void)) _pCallBack;
//    // 重置计数器为0
//    __HAL_TIM_SET_COUNTER(&RX_TIMER, 0);

//    // 清除中断标志
//    __HAL_TIM_CLEAR_FLAG(&RX_TIMER, TIM_FLAG_UPDATE);
//    HAL_TIM_Base_Start_IT(&RX_TIMER);

//}
/*
*********************************************************************************************************
*	函 数 名: StartHardTimer
*	功能说明: 开启定时器计时3.5个字符时间 此处使用定时器2
*	形    参: _uiTimeOut：超时时间
*            _pCallBack：超时回调函数函数指针
*	返 回 值: 无
*********************************************************************************************************
*/
void StartHardTimer_H(uint32_t _uiTimeOut, void * _pCallBack) {


    __HAL_TIM_SET_AUTORELOAD(&RX_TIMER_H, _uiTimeOut);
    s_TIM_CallBack2 = (void (*)(void)) _pCallBack;
    // 重置计数器为0
    __HAL_TIM_SET_COUNTER(&RX_TIMER_H, 0);

    // 清除中断标志
    __HAL_TIM_CLEAR_FLAG(&RX_TIMER_H, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&RX_TIMER_H);

}
/*
*********************************************************************************************************
*	函 数 名: MODH_ReciveNew
*	功能说明: 串口接收中断服务程序会调用本函数。当收到一个字节时，执行一次本函数。
*	形    参: 接收数据
*	返 回 值: 1 表示有数据
*********************************************************************************************************
*/
void MODH_ReciveNew(uint8_t _data)
{
	/*
		3.5个字符的时间间隔，只是用在RTU模式下面，因为RTU模式没有开始符和结束符，
		两个数据包之间只能靠时间间隔来区分，Modbus定义在不同的波特率下，间隔时间是不一样的，
		详情看此C文件开头
	*/
	uint8_t i;
	
	/* 根据波特率，获取需要延迟的时间 */
	for(i = 0; i < MODBUS_BAUD_RATE_LEN; i++)
	{
		if(HBAUD485 == ModbusBaudRate[i].Bps)
		{
			break;
		}	
	}

	/* 硬件定时中断，硬件定时器1用于MODBUS从机, 定时器2用于MODBUS主机
	*/
//	OLED_PrintASCIIString(10, 40,"2", &afont12x6, OLED_COLOR_NORMAL);
	StartHardTimer_H(ModbusBaudRate[i].usTimeOut, (void *)MODH_RxTimeOut);

	if (g_tModH.RxCount < H_RX_BUF_SIZE)
	{
		g_tModH.RxBuf[g_tModH.RxCount++] = _data;
	}
}

/*
*********************************************************************************************************
*	函 数 名: MODH_RxTimeOut
*	功能说明: 超过3.5个字符时间后执行本函数。 设置全局变量 g_rtu_timeout = 1; 通知主程序开始解码。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_RxTimeOut(void)
{	
	//xSemaphoreGiveFromISR(MODHx_SemaphoreHandle,NULL);
	g_modh_timeout = 1;
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Poll
*	功能说明: 接收控制器指令. 1ms 响应时间。
*	形    参: 无
*	返 回 值: 0 表示无数据 1表示收到正确命令
*********************************************************************************************************
*/
void MODH_Poll(void)
{	
	uint16_t crc1;
	if(g_modh_timeout==1)
	//if(pdTRUE==xSemaphoreTake(MODHx_SemaphoreHandle,portMAX_DELAY))
	{
//			if (g_modh_timeout == 0)	/* 超过3.5个字符时间后执行MODH_RxTimeOut()函数。全局变量 g_rtu_timeout = 1 */
//		{
//			/* 没有超时，继续接收。不要清零 g_tModH.RxCount */
//			return ;
//		}

		/* 收到命令
			05 06 00 88 04 57 3B70 (8 字节)
				05    :  数码管屏的号站，
				06    :  指令
				00 88 :  数码管屏的显示寄存器
				04 57 :  数据,,,转换成 10 进制是 1111.高位在前,
				3B70  :  二个字节 CRC 码	从05到 57的校验
		*/
		g_modh_timeout = 0;
	
		/* 接收到的数据小于4个字节就认为错误，地址（8bit）+指令（8bit）+操作寄存器（16bit） */
		if (g_tModH.RxCount < 4)
		{
			goto err_ret;
		}

		/* 计算CRC校验和，这里是将接收到的数据包含CRC16值一起做CRC16，结果是0，表示正确接收 */
		crc1 = CRC16_Modbus(g_tModH.RxBuf, g_tModH.RxCount);
		if (crc1 != 0)
		{
			goto err_ret;
		}
	
		/* 分析应用层协议 */
		MODH_AnalyzeApp();

	err_ret:
		g_tModH.RxCount = 0;	/* 必须清零计数器，方便下次帧同步 */
	}

}
/*
*********************************************************************************************************
*	函 数 名: MODH_AnalyzeApp
*	功能说明: 分析应用层协议。处理应答。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_AnalyzeApp(void)
{	
	switch (g_tModH.RxBuf[1])			/* 第2个字节 功能码 */
	{
//		case 0x01:	/* 读取线圈状态 */
//			MODH_Read_01H();
//			break;

//		case 0x02:	/* 读取输入状态 */
//			MODH_Read_02H();
//			break;

		case 0x03:	/* 读取保持寄存器 在一个或多个保持寄存器中取得当前的二进制值 */
			MODH_Read_03H();
			break;

//		case 0x04:	/* 读取输入寄存器 */
//			MODH_Read_04H();
//		
//			break;

//		case 0x05:	/* 强制单线圈 */
//			MODH_Read_05H();
//			break;

		case 0x06:	/* 写单个寄存器 */
			MODH_Read_06H();
			break;		

		case 0x10:	/* 写多个寄存器 */
			MODH_Read_10H();
			break;
		default:
			break;
	}
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Read_03H
*	功能说明: 分析03H指令的应答数据，读取保持寄存器，16bit访问
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_Read_03H(void)
{
	uint8_t bytes;
	uint8_t *p;
	
	if (g_tModH.RxCount > 0)
	{
		bytes = g_tModH.RxBuf[2];	/* 数据长度 字节数 */
        p = &g_tModH.RxBuf[3];
        for (int i=0;i<bytes/2;i++){
            
            switch (g_tModH.Reg03H+i)
            {
                case REG_P01:

                    g_tVar_h.P01 = BEBufToUint16(p); p += 2;	/* 寄存器 */	
                    //使从机寄存器的电流值更新
                    g_tVar.P22=g_tVar_h.P01;
                
                    g_tModH.fAck03H = 1;
                
                    break;
                case REG_P02:
 
                    g_tVar_h.P02 = BEBufToUint16(p); p += 2;	/* 寄存器 */	
                    //使从机寄存器的电流值更新
                    g_tVar.P23=g_tVar_h.P02;
                
                    g_tModH.fAck03H = 1;
                    break;
                case REG_P03:         
                    g_tVar_h.P03 = BEBufToUint16(p); p += 2;	/* 寄存器 */	
            
                    g_tModH.fAck03H = 1;

                    break;
                case REG_P04:
                    g_tVar_h.P04 = BEBufToUint16(p); p += 2;	/* 寄存器 */	
            
                    g_tModH.fAck03H = 1;

                    break;
                case REG_P05:
                    g_tVar_h.P05 = BEBufToUint16(p); p += 2;	/* 寄存器 */	
            
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
*	函 数 名: MODH_Read_06H
*	功能说明: 分析06H指令的应答数据，写单个保存寄存器，16bit访问
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_Read_06H(void)
{
	if (g_tModH.RxCount > 0)
	{
		if (g_tModH.RxBuf[0] == SlaveAddr)	//SlaveAddr不能是04，因为传感器通讯失败会返回04	
		{
			g_tModH.fAck06H = 1;		/* 接收到应答 */
			
			
			
		}
	}
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Read_10H
*	功能说明: 分析10H指令的应答数据，写多个保存寄存器，16bit访问
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_Read_10H(void)
{
	/*
		10H指令的应答:
			从机地址                11
			功能码                  10
			寄存器起始地址高字节	00
			寄存器起始地址低字节    01
			寄存器数量高字节        00
			寄存器数量低字节        02
			CRC校验高字节           12
			CRC校验低字节           98
	*/
	if (g_tModH.RxCount > 0)
	{
		if (g_tModH.RxBuf[0] == SlaveAddr)		
		{
			g_tModH.fAck10H = 1;		/* 接收到应答 */
		}
	}
}


/*
*********************************************************************************************************
*	函 数 名: MODH_ReadParam_03H
*	功能说明: 单个参数. 通过发送03H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_ReadParam_03H(uint16_t _reg, uint16_t _num)
{;
	uint8_t i;
	
	for (i = 0; i < NUM; i++)
	{
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
		}
		
		if (g_tModH.fAck03H > 0)
		{
            
			break;
		}
	}
	time1 =0;
	if (g_tModH.fAck03H == 0)
	{
		return 0;	/* 通信超时了 */
	}
	else 
	{
		return 1;	/* 写入03H参数成功 */
	}
}

/*
*********************************************************************************************************
*	函 数 名: MODH_WriteParam_06H
*	功能说明: 单个参数. 通过发送06H指令实现，发送之后，等待从机应答。循环NUM次写命令
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_WriteParam_06H(uint16_t _reg, uint16_t _value)
{

	uint8_t i;
	
	for (i = 0; i < NUM; i++)
	{	
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
		}
		
		if (g_tModH.fAck06H > 0)
		{
            
			break;
		}
	}
	time1 =0;
	if (g_tModH.fAck06H == 0)
	{
		return 0;	/* 通信超时了 */
	}
	else
	{
		return 1;	/* 写入06H参数成功 */
	}
}
 /*
*********************************************************************************************************
*	函 数 名: MODH_WriteParam_10H
*	功能说明: 单个参数. 通过发送10H指令实现，发送之后，等待从机应答。循环NUM次写命令
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_WriteParam_10H(uint16_t _reg, uint8_t _num, uint8_t *_buf)
{

	uint8_t i;
	
	for (i = 0; i < NUM; i++)
	{	
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
		}
		
		if (g_tModH.fAck10H > 0)
		{
			break;
		}
	}
	time1=0;
	if (g_tModH.fAck10H == 0)
	{
		return 0;	/* 通信超时了 */
	}
	else
	{
		return 1;	/* 写入10H参数成功 */
	}
}


void Time_Out_Fun(TimerHandle_t xTimer) {
     time1=1;
}

uint8_t electricity_data_buf[4]={0};
void ModBusHost(void *argument)
{

    

    //创建信号量 没有使用
    MODHx_SemaphoreHandle=xSemaphoreCreateBinary( );
    //发送信号量，没有使用
    RS485send_SemaphoreHandle =xSemaphoreCreateBinary( );
    xSemaphoreGive(RS485send_SemaphoreHandle);
    
    
    //软件定时器判断主机有无应答
    MODH_Timer = xTimerCreate( "rxtimer",pdMS_TO_TICKS(TIMEOUT),pdFALSE,NULL,Time_Out_Fun);  
    
    //串口波特率初始化
    MDSUARTxH.Init.BaudRate = HBAUD485;
    HAL_UART_Init(&MDSUARTxH);
    //开启接受中断
    Start_Receive_H();
    
    for(;;)
    {
        
        //改地址
        //改设备号
        //改奇偶校验
        
        //向寄存器写数据，设置输出电流值
        MODH_WriteParam_10H(REG_P01,2,electricity_data_buf);
        vTaskDelay(pdMS_TO_TICKS(500));
        
        //读寄存器的值，看实际电流输出为多少
        MODH_ReadParam_03H(REG_P01, 2);
        vTaskDelay(pdMS_TO_TICKS(500));

    }

}
