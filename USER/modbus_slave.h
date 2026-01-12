/*
*********************************************************************************************************
*
*	模块名称 : MODEBUS 通信模块 (从站）
*	文件名称 : modbus_slave.h
*	版    本 : V1
*	说    明 : 头文件，修改寄存器需要同时修改.c文件
*
*
*********************************************************************************************************
*/

#ifndef __MODBUY_SLAVE_H
#define __MODBUY_SLAVE_H
#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "gpio.h"
#include "usart.h"
#include "modbus.h"

//从机地址和波特率
extern uint8_t SADDR485;
extern uint32_t SBAUD485;

// 声明寄存器访问互斥信号量
extern SemaphoreHandle_t g_hVarMutex;  

//封装读写宏，简化调用
#define LOCK_VAR()    xSemaphoreTake(g_hVarMutex, portMAX_DELAY)
#define UNLOCK_VAR()  xSemaphoreGive(g_hVarMutex)

/* 01H 读强制单线圈 */
/* 05H 写强制单线圈 */

//4路继电器
/* 正常情况电磁阀1 2 3 4均不导通*/
/* 反吹电磁阀1 2 导通34不导通*/
/* 校准 1 2 不导通 3 导通*/
//反吹电磁阀
#define REG_D01		00001
//电磁换向阀
#define REG_D02		00002
//校准阀 
#define REG_D03		00003
//预留阀
#define REG_D04		00004
#define REG_DXX 	REG_D04


/* 03H 读保持寄存器 */
/* 06H 写保持寄存器 */
/* 10H 写多个保存寄存器 */
/* 一个寄存器存2字节 ,float占两位地址*/
//开始
#define SLAVE_REG_START 	    SLAVE_REG_P01
//只读
//NOx值
#define SLAVE_REG_P01		40001
//O2值
#define SLAVE_REG_P02		40003
//第一段4个参数
#define SLAVE_REG_P03		40005
#define SLAVE_REG_P04		40007
#define SLAVE_REG_P05		40009
#define SLAVE_REG_P06		40011
//传感器状态
/*
   9位2进制表示传感器状态
   |连接状态：0断开，1连接|NOx有效： 0无效，1有效|O2有效： 0无效，1有效|电压在范围:0无效，1有效|温度在范围：0不在范围，1在范围|加热状态：0不在加热，1在加热|FMI温度：0开路或短路，1没有错误或妹使用|FMINOx：|FMIO2：|
              1                    2                     3                   4                           5                           6                                  7                   8       9
   连接状态为0时后面全部无效，即小于256时无效
   
*/
#define SLAVE_REG_P07		40013

//可读写

/*3点标定8，9为一组NOx，10，11为一组O2，8，10默认为0xffff,发送0x0001进行标定,标定失败返回0x0005，标定成功0x000f,发送0x0002恢复初始值成功返回0x0010*/

/*9，11为选择标定的点位，0~2有效，默认0*/
#define SLAVE_REG_P08		40014
#define SLAVE_REG_P09		40015
#define SLAVE_REG_P10		40016
#define SLAVE_REG_P11		40017

//NOx报警值
#define SLAVE_REG_P12		40018
//O2报警值
#define SLAVE_REG_P13		40020

//仅读
//第二段4个参数
#define SLAVE_REG_P14		40022
#define SLAVE_REG_P15		40024
#define SLAVE_REG_P16		40026
#define SLAVE_REG_P17		40028

//读写
//第二个标定点  NOx和O2
#define SLAVE_REG_P18		40030
#define SLAVE_REG_P19   	40032
//第三个标定点  NOx和O2
#define SLAVE_REG_P20		40034
#define SLAVE_REG_P21		40036
//4-20mA电流输出
//在MODH_ReadParam_03H中更新
#define SLAVE_REG_P22		40038
#define SLAVE_REG_P23		40039
//反吹间隔（s） /*写入0立刻反吹,反吹中寄存器状态为0，写入0xffff立即停止反吹,写入0x01立即反吹，并重置基准时间*/
#define SLAVE_REG_P24		40040
//反吹时间(s)
#define SLAVE_REG_P25		40041

/* RTU 应答代码 */
#define RSP_OK				0		/* 成功 */
#define RSP_ERR_CMD			0x01	/* 不支持的功能码 */
#define RSP_ERR_REG_ADDR	0x02	/* 寄存器地址错误 */
#define RSP_ERR_VALUE		0x03	/* 数据值域错误 */
#define RSP_ERR_WRITE		0x04	/* 写入失败 */

#define S_RX_BUF_SIZE		60
#define S_TX_BUF_SIZE		128

//大容量产品，页32
#define FLASH_USER_START_ADDR   0x08010000  // 选择合适的Flash起始地址
#define FLASH_USER_END_ADDR     0x08010800  // 选择合适的Flash结束地址

typedef struct
{
    uint8_t RxBuf[S_RX_BUF_SIZE];
    uint8_t RxCount;
//    uint8_t RxStatus;
//    uint8_t RxNewFlag;

    uint8_t RspCode;

    uint8_t TxBuf[S_TX_BUF_SIZE];
    uint8_t TxCount;
} MODS_T;

typedef struct
{
    /* 03H 06H 10H读保持寄存器 */
    float P01;
    float P02;
    
    float P03;
    float P04;
    float P05;
    float P06;
    uint16_t P07;
    /* 03H 06H 10H读写保持寄存器 */
    uint16_t P08;
    uint16_t P09;
    uint16_t P10;
    uint16_t P11;
    float  P12;
    float  P13;
    
    float  P14;
    float  P15;
    float  P16;
    float  P17;
    
    float  P18;
    float  P19;
    float  P20;
    float  P21;
    uint16_t  P22;
    uint16_t  P23;
    uint16_t  P24;
    uint16_t  P25;
    
    /* 01H 05H 读写单个线圈 */
    uint16_t D01;
    uint16_t D02;
    uint16_t D03;
    uint16_t D04;

} VAR_T;




void MODS_Poll(void);
void MODS_ReciveNew(uint8_t _byte);

extern void (*s_TIM_CallBack1)(void);
extern MODS_T g_tModS;
//寄存器数据存储
extern VAR_T g_tVar;

extern SemaphoreHandle_t MODRx_SemaphoreHandle;

extern void Start_Receive(void);

extern int InternalFlash_Write(void);

void LoadRegistersFromFlash(void);

// -------------------------- 线圈寄存器（01H/05H）读写接口 --------------------------
extern void Var_Write_D01(uint16_t value);
extern uint16_t Var_Read_D01(void);
extern void Var_Write_D02(uint16_t value);
extern uint16_t Var_Read_D02(void);
extern void Var_Write_D03(uint16_t value);
extern uint16_t Var_Read_D03(void);
extern void Var_Write_D04(uint16_t value);
extern uint16_t Var_Read_D04(void);

// -------------------------- 保持寄存器（03H/06H/10H）只读接口 --------------------------
// 浮点型（占2个寄存器地址）
extern float Var_Read_P01(void);    // NOx值
extern float Var_Read_P02(void);    // O2值
extern float Var_Read_P03(void);    // 第一段参数1
extern float Var_Read_P04(void);    // 第一段参数2
extern float Var_Read_P05(void);    // 第一段参数3
extern float Var_Read_P06(void);    // 第一段参数4
// 整型
extern uint16_t Var_Read_P07(void); // 传感器状态
// 第二段参数（只读）
extern float Var_Read_P14(void);    // 第二段参数1
extern float Var_Read_P15(void);    // 第二段参数2
extern float Var_Read_P16(void);    // 第二段参数3
extern float Var_Read_P17(void);    // 第二段参数4

extern void Var_Write_P01(float value);    // NOx值
extern void Var_Write_P02(float value);    // O2值
extern void Var_Write_P03(float value);    // 第一段参数1
extern void Var_Write_P04(float value);    // 第一段参数2
extern void Var_Write_P05(float value);    // 第一段参数3
extern void Var_Write_P06(float value);    // 第一段参数4
// 整型
extern void Var_Write_P07(uint16_t value); // 传感器状态

// 第二段参数（只读）
extern void Var_Write_P14(float value);    // 第二段参数1
extern void Var_Write_P15(float value);    // 第二段参数2
extern void Var_Write_P16(float value);    // 第二段参数3
extern void Var_Write_P17(float value);    // 第二段参数4



// -------------------------- 保持寄存器（03H/06H/10H）读写接口 --------------------------
// 整型
extern void Var_Write_P08(uint16_t value);
extern uint16_t Var_Read_P08(void);
extern void Var_Write_P09(uint16_t value);
extern uint16_t Var_Read_P09(void);
extern void Var_Write_P10(uint16_t value);
extern uint16_t Var_Read_P10(void);
extern void Var_Write_P11(uint16_t value);
extern uint16_t Var_Read_P11(void);
// 浮点型
extern void Var_Write_P12(float value);
extern float Var_Read_P12(void);    // NOx报警值
extern void Var_Write_P13(float value);
extern float Var_Read_P13(void);    // O2报警值
// 标定点参数（浮点型）
extern void Var_Write_P18(float value);
extern float Var_Read_P18(void);    // 第二个标定点NOx
extern void Var_Write_P19(float value);
extern float Var_Read_P19(void);    // 第二个标定点O2
extern void Var_Write_P20(float value);
extern float Var_Read_P20(void);    // 第三个标定点NOx
extern void Var_Write_P21(float value);
extern float Var_Read_P21(void);    // 第三个标定点O2
// 电流输出（整型）
extern void Var_Write_P22(uint16_t value);
extern uint16_t Var_Read_P22(void);
extern void Var_Write_P23(uint16_t value);
extern uint16_t Var_Read_P23(void);
// 反吹参数（整型）
extern void Var_Write_P24(uint16_t value);
extern uint16_t Var_Read_P24(void); // 反吹间隔
extern void Var_Write_P25(uint16_t value);
extern uint16_t Var_Read_P25(void); // 反吹时间

// -------------------------- 批量操作接口（减少锁开销） --------------------------
// 批量更新传感器核心数据（NOx/O2/状态）
extern void Var_Update_SensorCore(float nox, float o2, uint16_t state);
// 批量更新NOx参数
extern void Var_Update_ParamSection1(float p03, float p04, float p14, float p15);
extern void Var_Read_ParamSection1(float* p03, float* p04, float* p14, float* p15);
// 批量更新O2参数
extern void Var_Update_ParamSection2(float p05, float p06, float p16, float p17);
extern void Var_Read_ParamSection2(float* p05, float* p06, float* p16, float* p17);
// 批量更新标定点参数
extern void Var_Update_CalibPoint(float p18, float p19, float p20, float p21);

#endif

/*****************************  (END OF FILE) *********************************/
