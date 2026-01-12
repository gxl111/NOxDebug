#ifndef __NOX_H
#define __NOX_H

#include "main.h"
#include "J1939.h"
#include <string.h>
#include <stdio.h>
#include "can.h"
#include "usart.h"
#include "gpio.h"





//传感器转换参数默认值
#define DEFAULTNOX_A   0.05
#define DEFAULTNOX_B   -200
#define DEFAULTO2_A    0.000514
#define DEFAULTO2_B    -12

//5点标定，标定点y值 FLASH中没有时使用
#define NOX_CALIBRATION_NUM  3
#define NOX_Y0 0
#define NOX_Y1 1500
#define NOX_Y2 2500


#define O2_CALIBRATION_NUM  3
#define O2_Y0 0
#define O2_Y1 12.5
#define O2_Y2 25

//反吹持续时间 （ms）
//#define BLOWTIME 10000

//斜率和截距
typedef struct
{
    float a;
    float b;
    
} Parameter;

//转换公式参数 ，第一段0点到标定点1
extern Parameter NOx_parameter;
extern Parameter O2_parameter;
//第二段参数
extern Parameter NOx_parameter1;
extern Parameter O2_parameter1;
//报警高低值
extern float NOx_High;
extern float O2_Low;




void TxMsg_Init(J1939_MESSAGE *TxMsgPtr);
void NOx_Handle(J1939_MESSAGE *RxMsgPtr);
void hexArrayToString(const j1939_uint8_t *array, size_t length, char *result);




void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) ;

void RS485_Send_Data_IT(uint8_t *pData, uint16_t Size);
#endif
