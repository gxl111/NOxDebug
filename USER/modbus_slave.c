/*
*********************************************************************************************************
*
*	模块名称 : MODS通信模块. 从站模式【原创】
*	文件名称 : modbus_slave.c
*	版    本 : V1.5
*	说    明 : 头文件
*
*
*********************************************************************************************************
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
*	                                  参数
*********************************************************************************************************
*/


//可通过sd卡修改
//modbus地址
uint8_t SADDR485=1;
//串口波特率
uint32_t SBAUD485=115200;



/*
*********************************************************************************************************
*	                                   函数声明
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
*	                                   变量
*********************************************************************************************************
*/




static uint8_t g_mods_timeout = 0;



MODS_T g_tModS = {0};
//寄存器数据
VAR_T g_tVar={ .P08=0xffff,.P10=0xffff};


//接受信号量
QueueHandle_t MODRx_SemaphoreHandle;


//继电器相关
// 定义一个结构体来表示GPIO端口和引脚
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} GPIOPin_t;

// 创建一个包含所有继电器GPIO信息的数组
const GPIOPin_t relayPins[] = {
    {Relay0_GPIO_Port, Relay0_Pin},
    {Relay1_GPIO_Port, Relay1_Pin},
    {Relay2_GPIO_Port, Relay2_Pin},
    {Relay3_GPIO_Port, Relay3_Pin}
};


/*
*********************************************************************************************************
*	                                  函数定义
*********************************************************************************************************
*/

void Start_Receive(void){
     HAL_UART_Receive_IT(&MDSUARTx,&rx_data, 1);
}


/*
*********************************************************************************************************
*	函 数 名: MODS_Poll
*	功能说明: 解析数据包. 在主程序中轮流调用。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void MODS_Poll(void)
{
    uint16_t addr;
    uint16_t crc1;

    /* 超过3.5个字符时间后执行MODH_RxTimeOut()函数。全局变量 g_rtu_timeout = 1; 通知主程序开始解码 */
    
//    if (g_mods_timeout == 0)
//    {
//        return;								/* 没有超时，继续接收。不要清零 g_tModS.RxCount */
//    }
    if(pdTRUE==xSemaphoreTake(MODRx_SemaphoreHandle,portMAX_DELAY)){
         
        g_mods_timeout = 0;	 					/* 清标志 */

        if (g_tModS.RxCount < 4)				/* 接收到的数据小于4个字节就认为错误，地址（8bit）+指令（8bit）+操作寄存器（16bit） */
        {
            goto err_ret;
        }

        /* 计算CRC校验和，这里是将接收到的数据包含CRC16值一起做CRC16，结果是0，表示正确接收 */
        crc1 = CRC16_Modbus(g_tModS.RxBuf, g_tModS.RxCount);
        if (crc1 != 0)
        {
            goto err_ret;
        }

        /* 站地址 (1字节） */
        addr = g_tModS.RxBuf[0];				/* 第1字节 站号 */
        if (addr != SADDR485)		 			/* 判断主机发送的命令地址是否符合 */
        {
            goto err_ret;
        }

        /* 分析应用层协议 */
				// 加入互斥锁
				LOCK_VAR();
        MODS_AnalyzeApp();
				UNLOCK_VAR();
    
    }
    

err_ret:
    g_tModS.RxCount = 0;					/* 必须清零计数器，方便下次帧同步 */
}


//使用tim2通道1，计时3.5个字符间隔
/*
*********************************************************************************************************
*	函 数 名: StartHardTimer
*	功能说明: 开启定时器计时3.5个字符时间 此处使用定时器2
*	形    参: _uiTimeOut：超时时间
*            _pCallBack：超时回调函数函数指针
*	返 回 值: 无
*********************************************************************************************************
*/
void StartHardTimer(uint32_t _uiTimeOut, void * _pCallBack) {


    __HAL_TIM_SET_AUTORELOAD(&RX_TIMER, _uiTimeOut);
    s_TIM_CallBack1 = (void (*)(void)) _pCallBack;
    // 重置计数器为0
    __HAL_TIM_SET_COUNTER(&RX_TIMER, 0);

    // 清除中断标志
    __HAL_TIM_CLEAR_FLAG(&RX_TIMER, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&RX_TIMER);

}


/*
*********************************************************************************************************
*	函 数 名: HAL_TIM_OC_DelayElapsedCallback
*	功能说明: tim2中断回调函数，判断3.5个字符时间是否达到
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/

//void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
//    if (htim->Instance == TIM2) {
//        // 检查是否是通道1触发中断
//        if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC1) != RESET) {
//            __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC1);
//          //关闭中断
//          __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);
//            if (s_TIM_CallBack1 != NULL) {
//                s_TIM_CallBack1();
//            }
//        }
//    }
//}

/*
*********************************************************************************************************
*	函 数 名: MODS_ReciveNew
*	功能说明: 串口接收中断服务程序会调用本函数。当收到一个字节时，执行一次本函数。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void MODS_ReciveNew(uint8_t _byte)
{
    /*
    	3.5个字符的时间间隔，只是用在RTU模式下面，因为RTU模式没有开始符和结束符，
    	两个数据包之间只能靠时间间隔来区分，Modbus定义在不同的波特率下，间隔时间是不一样的，
    	详情看此C文件开头
    */
    //LCD_ShowString(30,460,210,24,24,"reciving new");
    uint8_t i;

    /* 根据波特率，获取需要延迟的时间 */
    for(i = 0; i < MODBUS_BAUD_RATE_LEN; i++)
    {
        if(SBAUD485 == ModbusBaudRate[i].Bps)
        {

            break;
        }
    }

    g_mods_timeout = 0;

    /* 硬件定时中断，定时精度us 硬件定时器1用于MODBUS从机, 定时器2用于MODBUS主机*/
    StartHardTimer(ModbusBaudRate[i].usTimeOut, (void *)MODS_RxTimeOut);

    if (g_tModS.RxCount < S_RX_BUF_SIZE)
    {
        g_tModS.RxBuf[g_tModS.RxCount++] = _byte;
    }
}


/*
*********************************************************************************************************
*	函 数 名: MODS_RxTimeOut
*	功能说明: 超过3.5个字符时间后执行本函数。 设置全局变量 g_mods_timeout = 1，通知主程序开始解码。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODS_RxTimeOut(void)
{
    
    xSemaphoreGiveFromISR(MODRx_SemaphoreHandle,NULL);
    g_mods_timeout = 1;
    
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
*	函 数 名: MODS_SendWithCRC
*	功能说明: 发送一串数据, 自动追加2字节CRC
*	形    参: _pBuf 数据；
*			  _ucLen 数据长度（不带CRC）
*	返 回 值: 无
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
*	函 数 名: MODS_SendAckErr
*	功能说明: 发送错误应答
*	形    参: _ucErrCode : 错误代码
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODS_SendAckErr(uint8_t _ucErrCode)
{
    uint8_t txbuf[3];

    txbuf[0] = g_tModS.RxBuf[0];					/* 485地址 */
    txbuf[1] = g_tModS.RxBuf[1] | 0x80;				/* 异常的功能码 */
    txbuf[2] = _ucErrCode;							/* 错误代码(01,02,03,04) */

    MODS_SendWithCRC(txbuf, 3);
}

/*
*********************************************************************************************************
*	函 数 名: MODS_SendAckOk
*	功能说明: 发送正确的应答.
*	形    参: 无
*	返 回 值: 无
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
*	函 数 名: MODS_AnalyzeApp
*	功能说明: 分析应用层协议
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODS_AnalyzeApp(void)
{
    //LCD_ShowString(30,400,210,24,24,"modbus_analyzing");
    switch (g_tModS.RxBuf[1])				/* 第2个字节 功能码 */
    {
    case 0x01:							/* 读取线圈状态*/
        MODS_01H();
        break;

    case 0x03:							/* 读取保持寄存器（存在g_tVar中）*/
        MODS_03H();
        break;


    case 0x05:							/* 强制单线圈*/
        MODS_05H();
        break;

    case 0x06:							/* 写单个保存寄存器（改写g_tVar中的参数）*/
        MODS_06H();
        break;

    case 0x10:							/* 写多个保存寄存器（改写g_tVar中的参数）*/
        MODS_10H();

        break;

    default:
        g_tModS.RspCode = RSP_ERR_CMD;
        MODS_SendAckErr(g_tModS.RspCode);	/* 告诉主机命令错误 */
        break;
    }
}

/*
*********************************************************************************************************
*	函 数 名: MODS_01H
*	功能说明: 读取线圈状态（对应远程开关D01/D02/D03）
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
/* 说明:这里用LED代替继电器,便于观察现象 */
static void MODS_01H(void)
{
    /*
     举例：
    	主机发送:
    		01 从机地址
    		01 功能码
    		00 寄存器起始地址高字节
    		13 寄存器起始地址低字节
    		00 寄存器数量高字节
    		25 寄存器数量低字节
    		0E CRC校验低字节
    		84 CRC校验高字节

    	从机应答: 	1代表ON，0代表OFF。若返回的线圈数不为8的倍数，则在最后数据字节未尾使用0代替. BIT0对应第1个
    		01 从机地址
    		01 功能码
    		05 返回字节数
    		CD 数据1(线圈0013H-线圈001AH)
    		6B 数据2(线圈001BH-线圈0022H)
    		B2 数据3(线圈0023H-线圈002AH)
    		0E 数据4(线圈0032H-线圈002BH)
    		1B 数据5(线圈0037H-线圈0033H)
    		45 CRC校验低字节
    		E6 CRC校验高字节

    	例子:
    		01 01 00 01 00 03   xx xx	--- 查询D01开始的3个继电器状态
    		01 01 00 03 00 01   xx xx   --- 查询D03继电器的状态
    */
    uint16_t reg;
    uint16_t num;
    uint16_t i;
    uint16_t m;
    uint8_t status[10];

    g_tModS.RspCode = RSP_OK;

    /** 第1步： 判断接到指定个数数据 ===============================================================*/
    /*  没有外部继电器，直接应答错误
    	地址（8bit）+指令（8bit）+寄存器起始地址高低字节（16bit）+寄存器个数（16bit）+ CRC16
    */
    if (g_tModS.RxCount != 8)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;				/* 数据值域错误 */
        return;
    }

    /** 第2步： 数据解析 ===========================================================================*/
    /* 数据是大端，要转换为小端 0是高字节 */
    reg = BEBufToUint16(&g_tModS.RxBuf[2]); 			/* 寄存器号 */
    num = BEBufToUint16(&g_tModS.RxBuf[4]);				/* 寄存器个数 */

    /* 不足字节整数倍，补齐 */
    m = (num + 7) / 8;
    
    /* 解析主机命令要读取的状态 */
    if ( (num > 0) && (reg + num+ REG_D01 <= REG_DXX + 1))
    {
        for (i = 0; i < m; i++)
        {
            status[i] = 0;
        }

        for (i = 0; i < num; i++)
        {
            //读取继电器状态
            GPIO_PinState state = HAL_GPIO_ReadPin(relayPins[i].port, relayPins[i].pin);
            switch(i) {
                case 0: g_tVar.D01 = state; break;
                case 1: g_tVar.D02 = state; break;
                case 2: g_tVar.D03 = state; break;
                case 3: g_tVar.D04 = state; break;
            }
            status[i / 8]|=(state<< (i % 8));
        }
    }
    else
    {
        g_tModS.RspCode = RSP_ERR_REG_ADDR;				/* 寄存器地址错误 */
    }
    
    /** 第3步： 应答回复 =========================================================================*/
    if (g_tModS.RspCode == RSP_OK)						/* 正确应答 */
    {
        g_tModS.TxCount = 0;
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[0]; /* 返回从机地址 */
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[1]; /* 返回从机指令 */
        g_tModS.TxBuf[g_tModS.TxCount++] = m;				 /* 返回字节数 */

        for (i = 0; i < m; i++)
        {
            g_tModS.TxBuf[g_tModS.TxCount++] = status[i];	/* 返回继电器状态 */
        }
        MODS_SendWithCRC(g_tModS.TxBuf, g_tModS.TxCount);
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);				/* 告诉主机命令错误 */
    }
}




/*
*********************************************************************************************************
*	函 数 名: MODS_03H
*	功能说明: 读取保持寄存器，可读float和uint16_t 在一个或多个保持寄存器中取得当前的二进制值
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODS_03H(void)
{
    /*
    	从机地址为11H。保持寄存器的起始地址为006BH，结束地址为006DH。该次查询总共访问3个保持寄存器。

    	主机发送:
    		11 从机地址
    		03 功能码
    		00 寄存器地址高字节
    		6B 寄存器地址低字节
    		00 寄存器数量高字节
    		03 寄存器数量低字节
    		76 CRC低字节
    		87 CRC高字节

    	从机应答: 	保持寄存器的长度为2个字节。对于单个保持寄存器而言，寄存器高字节数据先被传输，
    				低字节数据后被传输。保持寄存器之间，低地址寄存器先被传输，高地址寄存器后被传输。
    		11 从机地址
    		03 功能码
    		06 字节数
    		00 数据1高字节(006BH)
    		6B 数据1低字节(006BH)
    		00 数据2高字节(006CH)
    		13 数据2 低字节(006CH)
    		00 数据3高字节(006DH)
    		00 数据3低字节(006DH)
    		38 CRC低字节
    		B9 CRC高字节

    	例子:
    		01 03 30 06 00 01  6B0B      ---- 读 3006H, 触发电流
    		01 03 4000 0010 51C6         ---- 读 4000H 倒数第1条浪涌记录 32字节
    		01 03 4001 0010 0006         ---- 读 4001H 倒数第1条浪涌记录 32字节

    		01 03 F000 0008 770C         ---- 读 F000H 倒数第1条告警记录 16字节
    		01 03 F001 0008 26CC         ---- 读 F001H 倒数第2条告警记录 16字节

    		01 03 7000 0020 5ED2         ---- 读 7000H 倒数第1条波形记录第1段 64字节
    		01 03 7001 0020 0F12         ---- 读 7001H 倒数第1条波形记录第2段 64字节

    		01 03 7040 0020 5F06         ---- 读 7040H 倒数第2条波形记录第1段 64字节
    */
    uint16_t reg;
    uint16_t num;
    uint16_t i;
    uint8_t reg_value[128];

    g_tModS.RspCode = RSP_OK;

    /** 第1步： 判断接到指定个数数据 ===============================================================*/
    /* 地址（8bit）+指令（8bit）+寄存器起始地址高低字节（16bit）+寄存器个数（16bit）+ CRC16 */
    if (g_tModS.RxCount != 8)								/* 03H命令必须是8个字节 */
    {
        g_tModS.RspCode = RSP_ERR_VALUE;					/* 数据值域错误 */
        goto err_ret;
    }

    /** 第2步： 数据解析 ===========================================================================*/
    /* 数据是大端，要转换为小端 */
    reg = BEBufToUint16(&g_tModS.RxBuf[2]); 				/* 寄存器号 */
    num = BEBufToUint16(&g_tModS.RxBuf[4]);					/* 寄存器个数 */

    /* 读取的数据个数要在范围内 */
    //
//    if(reg+SLAVE_REG_START+num-1>SLAVE_REG_END){
//        g_tModS.RspCode = RSP_ERR_VALUE;					/* 数据值域错误 */
//        goto err_ret;    
//    }
    if (num > sizeof(reg_value) / 2)
	{
		g_tModS.RspCode = RSP_ERR_VALUE;					/* 数据值域错误 */
		goto err_ret;
	}
    
    /* 读取的数据存入到reg_value里面 */
    for (i = 0; i < num; i++)
    {

         uint8_t read_state=MODS_ReadRegValue(reg, &reg_value[2 * i]);
         if ( read_state== 0)	/* 读出寄存器值放入reg_value，此函数已经做了大端转小端处理 */
        {
            g_tModS.RspCode = RSP_ERR_REG_ADDR;				/* 寄存器地址错误 */
            break;
        }else if(read_state==2){
            //浮点数
            ++i;
            ++reg;
        }

         ++reg;

    }
    
    /** 第3步： 应答回复 =========================================================================*/
err_ret:
    if (g_tModS.RspCode == RSP_OK)							 /* 正确应答 */
    {
        g_tModS.TxCount = 0;
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[0]; /* 返回从机地址 */
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[1]; /* 返回从机指令 */

        g_tModS.TxBuf[g_tModS.TxCount++] =num * 2;			 /* 返回字节数 */
      
        

        for (i = 0; i < num; i++)                            /* 返回数据*/
        {
            g_tModS.TxBuf[g_tModS.TxCount++] = reg_value[2*i];
            g_tModS.TxBuf[g_tModS.TxCount++] = reg_value[2*i+1];
        }

        MODS_SendWithCRC(g_tModS.TxBuf, g_tModS.TxCount);	/* 发送正确应答 */
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);					/* 发送错误应答 */
    }
}


/*
*********************************************************************************************************
*	函 数 名: MODS_05H
*	功能说明: 强制写单线圈（对应D01/D02/D03）
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODS_05H(void)
{
    /*
    	主机发送: 写单个线圈寄存器。FF00H值请求线圈处于ON状态，0000H值请求线圈处于OFF状态
    	。05H指令设置单个线圈的状态，15H指令可以设置多个线圈的状态。
    		11 从机地址
    		05 功能码
    		00 寄存器地址高字节
    		AC 寄存器地址低字节
    		FF 数据1高字节
    		00 数据2低字节
    		4E CRC校验高字节
    		8B CRC校验低字节

    	从机应答:
    		11 从机地址
    		05 功能码
    		00 寄存器地址高字节
    		AC 寄存器地址低字节
    		FF 寄存器1高字节
    		00 寄存器1低字节
    		4E CRC校验高字节
    		8B CRC校验低字节

    	例子:
    	01 05 10 01 FF 00   D93A   -- D01打开
    	01 05 10 01 00 00   98CA   -- D01关闭

    	01 05 10 02 FF 00   293A   -- D02打开
    	01 05 10 02 00 00   68CA   -- D02关闭

    	01 05 10 03 FF 00   78FA   -- D03打开
    	01 05 10 03 00 00   390A   -- D03关闭
    */
    uint16_t reg;
    uint16_t value;

    g_tModS.RspCode = RSP_OK;

    /** 第1步： 判断接到指定个数数据 ===============================================================*/
    /* 地址（8bit）+指令（8bit）+寄存器起始地址高低字节（16bit）+寄存器个数（16bit）+ CRC16 */
    if (g_tModS.RxCount != 8)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;		/* 数据值域错误 */
        goto err_ret;
    }

    /** 第2步： 数据解析 ===========================================================================*/
    /* 数据是大端，要转换为小端 */
    reg = BEBufToUint16(&g_tModS.RxBuf[2]); 	/* 寄存器号 */
    value = BEBufToUint16(&g_tModS.RxBuf[4]);	/* 数据 */

    if (value != 0x0000 && value != 0xFF00)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;		/* 数据值域错误 */
        goto err_ret;
    }
    if(value == 0xFF00)value =1;
    
    
    /* 设置数值 ，控制继电器，FF00H值请求线圈处于ON状态，0000H值请求线圈处于OFF状态*/
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
        g_tModS.RspCode = RSP_ERR_REG_ADDR;		/* 寄存器地址错误 */
    }
    
    /** 第3步： 应答回复 =========================================================================*/
err_ret:
    if (g_tModS.RspCode == RSP_OK)				/* 正确应答 */
    {
        MODS_SendAckOk();
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);		/* 告诉主机命令错误 */
    }
}

/*
*********************************************************************************************************
*	函 数 名: MODS_06H
*	功能说明: 写单个寄存器
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODS_06H(void)
{

    /*
    	写保持寄存器。注意06指令只能操作单个保持寄存器，16指令可以设置单个或多个保持寄存器

    	主机发送:
    		11 从机地址
    		06 功能码
    		00 寄存器地址高字节
    		01 寄存器地址低字节
    		00 数据1高字节
    		01 数据1低字节
    		9A CRC校验低字节
    		9B CRC校验高字节

    	从机响应:
    		11 从机地址
    		06 功能码
    		00 寄存器地址高字节
    		01 寄存器地址低字节
    		00 数据1高字节
    		01 数据1低字节
    		1B CRC校验低字节
    		5A	CRC校验高字节

    	例子:
    		01 06 30 06 00 25  A710    ---- 触发电流设置为 2.5
    		01 06 30 06 00 10  6707    ---- 触发电流设置为 1.0


    		01 06 30 1B 00 00  F6CD    ---- SMA 滤波系数 = 0 关闭滤波
    		01 06 30 1B 00 01  370D    ---- SMA 滤波系数 = 1
    		01 06 30 1B 00 02  770C    ---- SMA 滤波系数 = 2
    		01 06 30 1B 00 05  36CE    ---- SMA 滤波系数 = 5

    		01 06 30 07 00 01  F6CB    ---- 测试模式修改为 T1
    		01 06 30 07 00 02  B6CA    ---- 测试模式修改为 T2

    		01 06 31 00 00 00  8736    ---- 擦除浪涌记录区
    		01 06 31 01 00 00  D6F6    ---- 擦除告警记录区

    */

    uint16_t reg;

    g_tModS.RspCode = RSP_OK;

    /** 第1步： 判断接到指定个数数据 ===============================================================*/
    /* 地址（8bit）+指令（8bit）+寄存器起始地址高低字节（16bit）+写入的值（16bit）+ CRC16 */
    if (g_tModS.RxCount != 8)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;		/* 数据值域错误 */
        goto err_ret;
    }

    /** 第2步： 数据解析 ===========================================================================*/
    /* 数据是大端，要转换为小端 */
    reg = BEBufToUint16(&g_tModS.RxBuf[2]); 	/* 寄存器号 */
    //value = BEBufToUint16(&g_tModS.RxBuf[4]);	/* 寄存器值 */
    
    uint8_t write_state=MODS_WriteRegValue(reg, &g_tModS.RxBuf[4]);
    
    if (write_state!=0)	/* 该函数会把写入的值存入寄存器 */
    {
        ;
    }
    else
    {
        g_tModS.RspCode = RSP_ERR_REG_ADDR;		/* 寄存器地址错误 */
         
    }

    /** 第3步： 应答回复 =========================================================================*/
err_ret:
    if (g_tModS.RspCode == RSP_OK)				/* 正确应答 */
    {
        MODS_SendAckOk();
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);		/* 告诉主机命令错误 */
    }
}

/*
*********************************************************************************************************
*	函 数 名: MODS_10H
*	功能说明: 连续写多个寄存器.  
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODS_10H(void)
{
    /*
    	从机地址为11H。保持寄存器的其实地址为0001H，寄存器的结束地址为0002H。总共访问2个寄存器。
    	保持寄存器0001H的内容为000AH，保持寄存器0002H的内容为0102H。

    	主机发送:
    		11 从机地址
    		10 功能码
    		00 寄存器起始地址高字节
    		01 寄存器起始地址低字节
    		00 寄存器数量高字节
    		02 寄存器数量低字节
    		04 字节数
    		00 数据1高字节
    		0A 数据1低字节
    		01 数据2高字节
    		02 数据2低字节
    		C6 CRC校验高字节
    		F0 CRC校验低字节

    	从机响应:
    		11 从机地址
    		06 功能码
    		00 寄存器地址高字节
    		01 寄存器地址低字节
    		00 数据1高字节
    		01 数据1低字节
    		1B CRC校验高字节
    		5A	CRC校验低字节

    	例子:
    		01 10 30 00 00 06 0C  07 DE  00 0A  00 01  00 08  00 0C  00 00     389A    ---- 写时钟 2014-10-01 08:12:00
    		01 10 30 00 00 06 0C  07 DF  00 01  00 1F  00 17  00 3B  00 39     5549    ---- 写时钟 2015-01-31 23:59:57

    */
    uint16_t reg_addr;
    uint16_t reg_num;
    uint16_t byte_num;
    uint8_t i;
//    uint16_t value;


    g_tModS.RspCode = RSP_OK;

    /** 第1步： 判断接到指定个数数据 ===============================================================*/
    /* 地址（8bit）+指令（8bit）+寄存器起始地址高低字节（16bit）+寄存器个数（16bit）+ 字节数（8bit）+ 数据高低字节（16bit）+ CRC16 */
    if (g_tModS.RxCount < 11)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;			/* 数据值域错误 */
        goto err_ret;
    }

    /** 第2步： 数据解析 ===========================================================================*/
    /* 数据是大端，要转换为小端 */
    reg_addr = BEBufToUint16(&g_tModS.RxBuf[2]); 	/* 寄存器号 */
    reg_num = BEBufToUint16(&g_tModS.RxBuf[4]);		/* 寄存器个数 */
    byte_num = g_tModS.RxBuf[6];					/* 后面的数据体字节数 */

    /* 判断寄存器个数和后面数据字节数是否一致 */
    if (byte_num != 2 * reg_num)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;			/* 数据值域错误 */
        goto err_ret;
    }
    
    
    /* 数据写入 */
    for (i = 0; i < reg_num; i++)
    {
        
        //value = BEBufToUint16(&g_tModS.RxBuf[7 + 2 * i]);	/* 寄存器值 */
        
        uint8_t write_state=MODS_WriteRegValue(reg_addr, &g_tModS.RxBuf[7 + 2 * i]);
        if (write_state == 2)
        {
            ++i;
            ++reg_addr;
        }
        else if(write_state == 0)
        {
            g_tModS.RspCode = RSP_ERR_REG_ADDR;		/* 寄存器地址错误 */
           
            break;
        }
        ++reg_addr;
    }
    
    /** 第3步： 应答回复 =========================================================================*/
err_ret:
    if (g_tModS.RspCode == RSP_OK)					/* 正确应答 */
    {
        MODS_SendAckOk();
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);			/* 告诉主机命令错误 */
    }
}

/*
*********************************************************************************************************
*	函 数 名: MODS_ReadRegValue
*	功能说明: 读取保持寄存器的值
*	形    参: reg_addr 寄存器地址
*			  reg_value 存放寄存器结果
*	返 回 值: 1表示OK 0表示错误
*********************************************************************************************************
*/
//直接强制转换为 uint32_t 不是正确获取其内部二进制表示的方式,需要用联合体
union {
    float f;
    uint32_t u;
} converter;
static uint8_t MODS_ReadRegValue(uint16_t reg_addr, uint8_t *reg_value)
{
    
    uint16_t value;
    //浮点数判断
    float f_value;
    uint8_t f_flag;
    switch (reg_addr+SLAVE_REG_START)									/* 判断寄存器地址 */
    {
    case SLAVE_REG_P01:
        if(sizeof(g_tVar.P01)==4){
           f_value = g_tVar.P01;
           f_flag=1;
        }else{
            value =	g_tVar.P01;
        }

        break;

    case SLAVE_REG_P02:
        if(sizeof(g_tVar.P02)==4){
           f_value = g_tVar.P02;
           f_flag=1;
        }else{
            value =	g_tVar.P02;
        }
        break;
        
    case SLAVE_REG_P03:
        if(sizeof(g_tVar.P03)==4){
           f_value = g_tVar.P03;
           f_flag=1;
        }else{
            value =	g_tVar.P03;
        }
        break;
        
    case SLAVE_REG_P04:
        if(sizeof(g_tVar.P04)==4){
           f_value = g_tVar.P04;
           f_flag=1;
        }else{
            value =	g_tVar.P04;
        }
        break;
        
    case SLAVE_REG_P05:
        if(sizeof(g_tVar.P05)==4){
           f_value = g_tVar.P05;
           f_flag=1;
        }else{
            value =	g_tVar.P05;
        }
        break;
        
    case SLAVE_REG_P06:
        if(sizeof(g_tVar.P06)==4){
           f_value = g_tVar.P06;
           f_flag=1;
        }else{
            value =	g_tVar.P06;
        }
        break;
        
    case SLAVE_REG_P07:
        if(sizeof(g_tVar.P07)==4){
           f_value = g_tVar.P07;
           f_flag=1;
        }else{
            value =	g_tVar.P07;
        }
        break;
        
    case SLAVE_REG_P08:
        if(sizeof(g_tVar.P08)==4){
           f_value = g_tVar.P08;
           f_flag=1;
        }else{
            value =	g_tVar.P08;
        }
        break;
    case SLAVE_REG_P09:
        if(sizeof(g_tVar.P09)==4){
           f_value = g_tVar.P09;
           f_flag=1;
        }else{
            value =	g_tVar.P09;
        }
        break;
    case SLAVE_REG_P10:
        if(sizeof(g_tVar.P10)==4){
           f_value = g_tVar.P10;
           f_flag=1;
        }else{
            value =	g_tVar.P10;
        }
        break;
    case SLAVE_REG_P11:
        if(sizeof(g_tVar.P11)==4){
           f_value = g_tVar.P11;
           f_flag=1;
        }else{
            value =	g_tVar.P11;
        }
        break;  
    case SLAVE_REG_P12:
        if(sizeof(g_tVar.P12)==4){
           f_value = g_tVar.P12;
           f_flag=1;
        }else{
            value =	g_tVar.P12;
        }
        break; 
    case SLAVE_REG_P13:
        if(sizeof(g_tVar.P13)==4){
           f_value = g_tVar.P13;
           f_flag=1;
        }else{
            value =	g_tVar.P13;
        }
        break; 
        
    case SLAVE_REG_P14:
        if(sizeof(g_tVar.P14)==4){
           f_value = g_tVar.P14;
           f_flag=1;
        }else{
            value =	g_tVar.P14;
        }
        break;
        
    case SLAVE_REG_P15:
        if(sizeof(g_tVar.P15)==4){
           f_value = g_tVar.P15;
           f_flag=1;
        }else{
            value =	g_tVar.P15;
        }
        break;
        
    case SLAVE_REG_P16:
        if(sizeof(g_tVar.P16)==4){
           f_value = g_tVar.P16;
           f_flag=1;
        }else{
            value =	g_tVar.P16;
        }
        break;
        
    case SLAVE_REG_P17:
        if(sizeof(g_tVar.P17)==4){
           f_value = g_tVar.P17;
           f_flag=1;
        }else{
            value =	g_tVar.P17;
        }
        break; 
    case SLAVE_REG_P18:
        if(sizeof(g_tVar.P18)==4){
           f_value = g_tVar.P18;
           f_flag=1;
        }else{
            value =	g_tVar.P18;
        }
        break;
    case SLAVE_REG_P19:
        if(sizeof(g_tVar.P19)==4){
           f_value = g_tVar.P19;
           f_flag=1;
        }else{
            value =	g_tVar.P19;
        }
        break;
    case SLAVE_REG_P20:
        if(sizeof(g_tVar.P20)==4){
           f_value = g_tVar.P20;
           f_flag=1;
        }else{
            value =	g_tVar.P20;
        }
        break;
    case SLAVE_REG_P21:
        if(sizeof(g_tVar.P21)==4){
           f_value = g_tVar.P21;
           f_flag=1;
        }else{
            value =	g_tVar.P21;
        }
        break; 
    case SLAVE_REG_P22:
        if(sizeof(g_tVar.P22)==4){
           f_value = g_tVar.P22;
           f_flag=1;
        }else{
            value =	g_tVar.P22;
        }
        break; 
    case SLAVE_REG_P23:
        if(sizeof(g_tVar.P23)==4){
           f_value = g_tVar.P23;
           f_flag=1;
        }else{
            value =	g_tVar.P23;
        }
        break;   
    case SLAVE_REG_P24:
        if(sizeof(g_tVar.P24)==4){
           f_value = g_tVar.P24;
           f_flag=1;
        }else{
            value =	g_tVar.P24;
        }
        break;  
    case SLAVE_REG_P25:
        if(sizeof(g_tVar.P25)==4){
           f_value = g_tVar.P25;
           f_flag=1;
        }else{
            value =	g_tVar.P25;
        }
        break;  
    default:
        return 0;									/* 参数异常，返回 0 */
    }
    
    if(f_flag==1){
        converter.f = f_value;
        reg_value[0] = (converter.u >> 24) & 0xFF;
        reg_value[1] = (converter.u >> 16) & 0xFF;
        reg_value[2] = (converter.u >> 8) & 0xFF;
        reg_value[3] = converter.u & 0xFF;
        return 2;
    
    }else{
        reg_value[0] = value >> 8;                          /* 注意数据是大端  */
        reg_value[1] = value;
    }

    return 1;											/* 读取成功 */
}

/*
*********************************************************************************************************
*	函 数 名: MODS_WriteRegValue
*	功能说明: 写保持寄存器的值
*	形    参: reg_addr 寄存器地址
*			  reg_value 寄存器值
*	返 回 值: 1表示OK 0表示错误
*********************************************************************************************************
*/
static uint8_t MODS_WriteRegValue(uint16_t reg_addr, uint8_t* reg_value)
{
    
    uint8_t f_flag=0;
    uint16_t value =BEBufToUint16(reg_value);
    uint16_t value1;
    
    switch (reg_addr+SLAVE_REG_START)							/* 判断寄存器地址 */
    {
    case SLAVE_REG_P01:
        				/* 将值写入保存寄存器 */
        if(sizeof(g_tVar.P01)==4){
           value1 =BEBufToUint16(reg_value+2);
           g_tVar.P01 = RegistersToFloat_BE(value,value1);
           f_flag=1;
        }else{
            g_tVar.P01 = value;
        }
        break;

    case SLAVE_REG_P02:
        if(sizeof(g_tVar.P02)==4){
            value1 =BEBufToUint16(reg_value+2);
           g_tVar.P02 = RegistersToFloat_BE(value,value1);
           f_flag=1;
        }else{
            g_tVar.P02 = value;
        }
        break;
    case SLAVE_REG_P03:
        if(sizeof(g_tVar.P03)==4){
            value1 =BEBufToUint16(reg_value+2);
           g_tVar.P03 = RegistersToFloat_BE(value,value1);
           f_flag=1;
        }else{
            g_tVar.P03 = value;
        }
        break;
    case SLAVE_REG_P04:
        if(sizeof(g_tVar.P04)==4){
            value1 =BEBufToUint16(reg_value+2);
           g_tVar.P04 = RegistersToFloat_BE(value,value1);
           f_flag=1;
        }else{
            g_tVar.P04 = value;
        }
        break;
    case SLAVE_REG_P05:
        if(sizeof(g_tVar.P05)==4){
            value1 =BEBufToUint16(reg_value+2);
           g_tVar.P05 = RegistersToFloat_BE(value,value1);
           f_flag=1;
        }else{
            g_tVar.P05 = value;
        }
        break;
    case SLAVE_REG_P06:
        if(sizeof(g_tVar.P06)==4){
            value1 =BEBufToUint16(reg_value+2);
           g_tVar.P06 = RegistersToFloat_BE(value,value1);
           f_flag=1;
        }else{
            g_tVar.P06 = value;
        }
        break;        
    case SLAVE_REG_P08:
        if(sizeof(g_tVar.P08)==4){
            value1 =BEBufToUint16(reg_value+2);
           g_tVar.P08 = RegistersToFloat_BE(value,value1);
           f_flag=1;
        }else{
            g_tVar.P08 = value;
        }
        break;  
    case SLAVE_REG_P09:
        if(sizeof(g_tVar.P09)==4){
            value1 =BEBufToUint16(reg_value+2);
           g_tVar.P09 = RegistersToFloat_BE(value,value1);
           f_flag=1;
        }else{
            g_tVar.P09 = value;
        }
        break; 
    case SLAVE_REG_P10:
        if(sizeof(g_tVar.P10)==4){
            value1 =BEBufToUint16(reg_value+2);
           g_tVar.P10 = RegistersToFloat_BE(value,value1);
           f_flag=1;
        }else{
            g_tVar.P10 = value;
        }
        break;
    case SLAVE_REG_P11:
        if(sizeof(g_tVar.P11)==4){
            value1 =BEBufToUint16(reg_value+2);
           g_tVar.P11 = RegistersToFloat_BE(value,value1);
           f_flag=1;
        }else{
            g_tVar.P11 = value;
        }
        break;  
    case SLAVE_REG_P12:
        if(sizeof(g_tVar.P12)==4){
            value1 =BEBufToUint16(reg_value+2);
           g_tVar.P12 = RegistersToFloat_BE(value,value1);
           f_flag=1;
        }else{
            g_tVar.P12 = value;
        }
        break;
    case SLAVE_REG_P13:
        if(sizeof(g_tVar.P13)==4){
            value1 =BEBufToUint16(reg_value+2);
           g_tVar.P13 = RegistersToFloat_BE(value,value1);
           f_flag=1;
        }else{
            g_tVar.P13 = value;
        }
        break; 
    case SLAVE_REG_P14:
        if(sizeof(g_tVar.P14)==4){
            value1 =BEBufToUint16(reg_value+2);
           g_tVar.P14 = RegistersToFloat_BE(value,value1);
           f_flag=1;
        }else{
            g_tVar.P14 = value;
        }
        break; 
    case SLAVE_REG_P15:
        if(sizeof(g_tVar.P15)==4){
            value1 =BEBufToUint16(reg_value+2);
           g_tVar.P15 = RegistersToFloat_BE(value,value1);
           f_flag=1;
        }else{
            g_tVar.P15 = value;
        }
        break; 
    case SLAVE_REG_P16:
        if(sizeof(g_tVar.P16)==4){
            value1 =BEBufToUint16(reg_value+2);
           g_tVar.P16 = RegistersToFloat_BE(value,value1);
           f_flag=1;
        }else{
            g_tVar.P16 = value;
        }
        break; 
    case SLAVE_REG_P17:
        if(sizeof(g_tVar.P17)==4){
            value1 =BEBufToUint16(reg_value+2);
           g_tVar.P17 = RegistersToFloat_BE(value,value1);
           f_flag=1;
        }else{
            g_tVar.P17 = value;
        }
        break; 

    case SLAVE_REG_P18:
        if(sizeof(g_tVar.P18)==4){
            value1 =BEBufToUint16(reg_value+2);
           g_tVar.P18 = RegistersToFloat_BE(value,value1);
           f_flag=1;
        }else{
            g_tVar.P18 = value;
        }
        break;  
    case SLAVE_REG_P19:
        if(sizeof(g_tVar.P19)==4){
            value1 =BEBufToUint16(reg_value+2);
           g_tVar.P19 = RegistersToFloat_BE(value,value1);
           f_flag=1;
        }else{
            g_tVar.P19 = value;
        }
        break; 
    case SLAVE_REG_P20:
        if(sizeof(g_tVar.P20)==4){
            value1 =BEBufToUint16(reg_value+2);
           g_tVar.P20 = RegistersToFloat_BE(value,value1);
           f_flag=1;
        }else{
            g_tVar.P20 = value;
        }
        break;
    case SLAVE_REG_P21:
        if(sizeof(g_tVar.P21)==4){
            value1 =BEBufToUint16(reg_value+2);
           g_tVar.P21 = RegistersToFloat_BE(value,value1);
           f_flag=1;
        }else{
            g_tVar.P21 = value;
        }
        break; 
    case SLAVE_REG_P24:
        if(sizeof(g_tVar.P24)==4){
            value1 =BEBufToUint16(reg_value+2);
           g_tVar.P24 = RegistersToFloat_BE(value,value1);
           f_flag=1;
        }else{
            g_tVar.P24 = value;
        }
        break;
    case SLAVE_REG_P25:
        if(sizeof(g_tVar.P25)==4){
            value1 =BEBufToUint16(reg_value+2);
           g_tVar.P25 = RegistersToFloat_BE(value,value1);
           f_flag=1;
        }else{
            g_tVar.P25 = value;
        }
        break;         
    default:
        return 0;		/* 参数异常，返回 0 */
    }

    if(f_flag==1) return 2;
    
    return 1;		/* 读取成功 */
}

// 从两个寄存器恢复浮点数（高字节在前）
float RegistersToFloat_BE(uint16_t reg1, uint16_t reg2) {
    converter.u = ((uint32_t)reg1 << 16) | reg2;
    return converter.f;
}

//直接强制转换为 uint32_t 不是正确获取其内部二进制表示的方式,需要用联合体
union {
    float f;
    uint32_t u;
} flash_converter;
int InternalFlash_Write(void)
 {

     uint32_t Address = 0x00;        //记录写入的地址
     
     uint32_t DATA_32[12];      //记录写入的数据
     uint32_t NbrOfPage = 0x00;      //记录写入多少页
     __IO uint32_t Data32 = 0;

     uint32_t SECTORError = 0;
     int MemoryProgramStatus = 1;//记录整个测试结果

     static FLASH_EraseInitTypeDef EraseInitStruct;
     /* 解锁 */
     HAL_FLASH_Unlock();

     /* 计算要擦除多少页 */
     NbrOfPage = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;
     EraseInitStruct.TypeErase     = FLASH_TYPEERASE_PAGES;
     EraseInitStruct.NbPages       = NbrOfPage;
     EraseInitStruct.PageAddress   = FLASH_USER_START_ADDR;

     if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK) {
         /*擦除出错，返回，实际应用中可加入处理 */
         return -1;
     }
     flash_converter.f= g_tVar.P03;
     DATA_32[0]=flash_converter.u;
     flash_converter.f= g_tVar.P04;
     DATA_32[1]=flash_converter.u;
     flash_converter.f= g_tVar.P05;
     DATA_32[2]=flash_converter.u;
     flash_converter.f= g_tVar.P06;
     DATA_32[3]=flash_converter.u;
     flash_converter.f= g_tVar.P14;
     DATA_32[4]=flash_converter.u;
     flash_converter.f= g_tVar.P15;
     DATA_32[5]=flash_converter.u;
     flash_converter.f= g_tVar.P16;
     DATA_32[6]=flash_converter.u;
     flash_converter.f= g_tVar.P17;
     DATA_32[7]=flash_converter.u;
     flash_converter.f= g_tVar.P18;
     DATA_32[8]=flash_converter.u;
     flash_converter.f= g_tVar.P19;
     DATA_32[9]=flash_converter.u;  
     flash_converter.f= g_tVar.P20;
     DATA_32[10]=flash_converter.u;
     flash_converter.f= g_tVar.P21;
     DATA_32[11]=flash_converter.u;


     /* 向内部FLASH写入数据 */
     Address = FLASH_USER_START_ADDR;
     for (int i=0;i<12;i++){
       if (Address < FLASH_USER_END_ADDR) {
         if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, DATA_32[i])
             == HAL_OK) {
             Address = Address + 4;
         } else {
             /*写入出错，返回，实际应用中可加入处理
             */
             return -1;
         }
        }
     }

     HAL_FLASH_Lock();

     /* 检查写入的数据是否正确 */
     Address = FLASH_USER_START_ADDR;
     int j=0;
     while ((Address < FLASH_USER_END_ADDR) && (MemoryProgramStatus !=
         0)) {
         if ((*(__IO uint32_t*) Address) != DATA_32[j]) {
             MemoryProgramStatus = 0;
         }
         Address += 4;
         ++j;
     }
     return MemoryProgramStatus;
 }


 
void LoadRegistersFromFlash(void) {
    uint32_t Address = FLASH_USER_START_ADDR;

   //14~21 3~6  标定点和斜率
   flash_converter.u=*(uint32_t *)Address;
   g_tVar.P03=  flash_converter.f;
   Address += 4;   
   flash_converter.u=*(uint32_t *)Address;
   g_tVar.P04=  flash_converter.f;
   Address += 4;
    flash_converter.u=*(uint32_t *)Address;
   g_tVar.P05=  flash_converter.f;
   Address += 4;
    flash_converter.u=*(uint32_t *)Address;
   g_tVar.P06=  flash_converter.f;
   Address += 4;   
   flash_converter.u=*(uint32_t *)Address;
   g_tVar.P14 = flash_converter.f;
   Address += 4;
   flash_converter.u=*(uint32_t *)Address;
   g_tVar.P15 = flash_converter.f;
   Address += 4;
   flash_converter.u=*(uint32_t *)Address;
   g_tVar.P16 = flash_converter.f;
   Address += 4;
   flash_converter.u=*(uint32_t *)Address;
   g_tVar.P17 = flash_converter.f;
   Address += 4;
   flash_converter.u=*(uint32_t *)Address;
   g_tVar.P18 = flash_converter.f;
   Address += 4;
   flash_converter.u=*(uint32_t *)Address;
   g_tVar.P19 = flash_converter.f;
   Address += 4;
   flash_converter.u=*(uint32_t *)Address;
   g_tVar.P20 = flash_converter.f;
   Address += 4;
   flash_converter.u=*(uint32_t *)Address;
   g_tVar.P21 = flash_converter.f;
   Address += 4;

   
}


// ========================== 线圈寄存器（D系列）实现 ==========================
void Var_Write_D01(uint16_t value) {
    LOCK_VAR();
    g_tVar.D01 = value;
    UNLOCK_VAR();
}
uint16_t Var_Read_D01(void) {
    uint16_t ret;
    LOCK_VAR();
    ret = g_tVar.D01;
    UNLOCK_VAR();
    return ret;
}

void Var_Write_D02(uint16_t value) {
    LOCK_VAR();
    g_tVar.D02 = value;
    UNLOCK_VAR();
}
uint16_t Var_Read_D02(void) {
    uint16_t ret;
    LOCK_VAR();
    ret = g_tVar.D02;
    UNLOCK_VAR();
    return ret;
}

void Var_Write_D03(uint16_t value) {
    LOCK_VAR();
    g_tVar.D03 = value;
    UNLOCK_VAR();
}
uint16_t Var_Read_D03(void) {
    uint16_t ret;
    LOCK_VAR();
    ret = g_tVar.D03;
    UNLOCK_VAR();
    return ret;
}

void Var_Write_D04(uint16_t value) {
    LOCK_VAR();
    g_tVar.D04 = value;
    UNLOCK_VAR();
}
uint16_t Var_Read_D04(void) {
    uint16_t ret;
    LOCK_VAR();
    ret = g_tVar.D04;
    UNLOCK_VAR();
    return ret;
}

// ========================== 保持寄存器-只读（P01~P07/P14~P17）实现 ==========================
float Var_Read_P01(void) {
    float ret;
    LOCK_VAR();
    ret = g_tVar.P01;
    UNLOCK_VAR();
    return ret;
}

float Var_Read_P02(void) {
    float ret;
    LOCK_VAR();
    ret = g_tVar.P02;
    UNLOCK_VAR();
    return ret;
}

float Var_Read_P03(void) {
    float ret;
    LOCK_VAR();
    ret = g_tVar.P03;
    UNLOCK_VAR();
    return ret;
}

float Var_Read_P04(void) {
    float ret;
    LOCK_VAR();
    ret = g_tVar.P04;
    UNLOCK_VAR();
    return ret;
}

float Var_Read_P05(void) {
    float ret;
    LOCK_VAR();
    ret = g_tVar.P05;
    UNLOCK_VAR();
    return ret;
}

float Var_Read_P06(void) {
    float ret;
    LOCK_VAR();
    ret = g_tVar.P06;
    UNLOCK_VAR();
    return ret;
}

uint16_t Var_Read_P07(void) {
    uint16_t ret;
    LOCK_VAR();
    ret = g_tVar.P07;
    UNLOCK_VAR();
    return ret;
}

float Var_Read_P14(void) {
    float ret;
    LOCK_VAR();
    ret = g_tVar.P14;
    UNLOCK_VAR();
    return ret;
}

float Var_Read_P15(void) {
    float ret;
    LOCK_VAR();
    ret = g_tVar.P15;
    UNLOCK_VAR();
    return ret;
}

float Var_Read_P16(void) {
    float ret;
    LOCK_VAR();
    ret = g_tVar.P16;
    UNLOCK_VAR();
    return ret;
}

float Var_Read_P17(void) {
    float ret;
    LOCK_VAR();
    ret = g_tVar.P17;
    UNLOCK_VAR();
    return ret;
}

// ========================== 保持寄存器-读写（P08~P13/P18~P25）实现 ==========================
void Var_Write_P08(uint16_t value) {
    LOCK_VAR();
    g_tVar.P08 = value;
    UNLOCK_VAR();
}
uint16_t Var_Read_P08(void) {
    uint16_t ret;
    LOCK_VAR();
    ret = g_tVar.P08;
    UNLOCK_VAR();
    return ret;
}

void Var_Write_P09(uint16_t value) {
    LOCK_VAR();
    g_tVar.P09 = value;
    UNLOCK_VAR();
}
uint16_t Var_Read_P09(void) {
    uint16_t ret;
    LOCK_VAR();
    ret = g_tVar.P09;
    UNLOCK_VAR();
    return ret;
}

void Var_Write_P10(uint16_t value) {
    LOCK_VAR();
    g_tVar.P10 = value;
    UNLOCK_VAR();
}
uint16_t Var_Read_P10(void) {
    uint16_t ret;
    LOCK_VAR();
    ret = g_tVar.P10;
    UNLOCK_VAR();
    return ret;
}

void Var_Write_P11(uint16_t value) {
    LOCK_VAR();
    g_tVar.P11 = value;
    UNLOCK_VAR();
}
uint16_t Var_Read_P11(void) {
    uint16_t ret;
    LOCK_VAR();
    ret = g_tVar.P11;
    UNLOCK_VAR();
    return ret;
}

void Var_Write_P12(float value) {
    LOCK_VAR();
    g_tVar.P12 = value;
    UNLOCK_VAR();
}
float Var_Read_P12(void) {
    float ret;
    LOCK_VAR();
    ret = g_tVar.P12;
    UNLOCK_VAR();
    return ret;
}

void Var_Write_P13(float value) {
    LOCK_VAR();
    g_tVar.P13 = value;
    UNLOCK_VAR();
}
float Var_Read_P13(void) {
    float ret;
    LOCK_VAR();
    ret = g_tVar.P13;
    UNLOCK_VAR();
    return ret;
}

void Var_Write_P18(float value) {
    LOCK_VAR();
    g_tVar.P18 = value;
    UNLOCK_VAR();
}
float Var_Read_P18(void) {
    float ret;
    LOCK_VAR();
    ret = g_tVar.P18;
    UNLOCK_VAR();
    return ret;
}

void Var_Write_P19(float value) {
    LOCK_VAR();
    g_tVar.P19 = value;
    UNLOCK_VAR();
}
float Var_Read_P19(void) {
    float ret;
    LOCK_VAR();
    ret = g_tVar.P19;
    UNLOCK_VAR();
    return ret;
}

void Var_Write_P20(float value) {
    LOCK_VAR();
    g_tVar.P20 = value;
    UNLOCK_VAR();
}
float Var_Read_P20(void) {
    float ret;
    LOCK_VAR();
    ret = g_tVar.P20;
    UNLOCK_VAR();
    return ret;
}

void Var_Write_P21(float value) {
    LOCK_VAR();
    g_tVar.P21 = value;
    UNLOCK_VAR();
}
float Var_Read_P21(void) {
    float ret;
    LOCK_VAR();
    ret = g_tVar.P21;
    UNLOCK_VAR();
    return ret;
}

void Var_Write_P22(uint16_t value) {
    LOCK_VAR();
    g_tVar.P22 = value;
    UNLOCK_VAR();
}
uint16_t Var_Read_P22(void) {
    uint16_t ret;
    LOCK_VAR();
    ret = g_tVar.P22;
    UNLOCK_VAR();
    return ret;
}

void Var_Write_P23(uint16_t value) {
    LOCK_VAR();
    g_tVar.P23 = value;
    UNLOCK_VAR();
}
uint16_t Var_Read_P23(void) {
    uint16_t ret;
    LOCK_VAR();
    ret = g_tVar.P23;
    UNLOCK_VAR();
    return ret;
}

void Var_Write_P24(uint16_t value) {
    LOCK_VAR();
    g_tVar.P24 = value;
    UNLOCK_VAR();
}
uint16_t Var_Read_P24(void) {
    uint16_t ret;
    LOCK_VAR();
    ret = g_tVar.P24;
    UNLOCK_VAR();
    return ret;
}

void Var_Write_P25(uint16_t value) {
    LOCK_VAR();
    g_tVar.P25 = value;
    UNLOCK_VAR();
}
uint16_t Var_Read_P25(void) {
    uint16_t ret;
    LOCK_VAR();
    ret = g_tVar.P25;
    UNLOCK_VAR();
    return ret;
}

// ========================== 批量操作接口（减少锁的获取/释放次数） ==========================
void Var_Update_SensorCore(float nox, float o2, uint16_t state) {
    LOCK_VAR();
    g_tVar.P01 = nox;
    g_tVar.P02 = o2;
    g_tVar.P07 = state;
    UNLOCK_VAR();
}

void Var_Update_ParamSection1(float p03, float p04, float p05, float p06) {
    LOCK_VAR();
    g_tVar.P03 = p03;
    g_tVar.P04 = p04;
    g_tVar.P14 = p05;
    g_tVar.P15 = p06;
    UNLOCK_VAR();
}
extern void Var_Read_ParamSection1(float* p03, float* p04, float* p14, float* p15){
    LOCK_VAR();
    *p03 = g_tVar.P03;
    *p04 = g_tVar.P04;
    *p14 = g_tVar.P14;
    *p15 = g_tVar.P15;
    UNLOCK_VAR();

}
void Var_Update_ParamSection2(float p05, float p06, float p16, float p17) {
    LOCK_VAR();
    g_tVar.P05 = p05;
    g_tVar.P06 = p06;
    g_tVar.P16 = p16;
    g_tVar.P17 = p17;
    UNLOCK_VAR();
}
extern void Var_Read_ParamSection2(float* p05, float* p06, float* p16, float* p17){
    LOCK_VAR();
    *p05 = g_tVar.P05;
    *p06 = g_tVar.P06;
    *p16 = g_tVar.P16;
    *p17 = g_tVar.P17;
    UNLOCK_VAR();
}
void Var_Update_CalibPoint(float p18, float p19, float p20, float p21) {
    LOCK_VAR();
    g_tVar.P18 = p18;
    g_tVar.P19 = p19;
    g_tVar.P20 = p20;
    g_tVar.P21 = p21;
    UNLOCK_VAR();
}


/*****************************  (END OF FILE) *********************************/
