

#ifndef __J1939_H
#define __J1939_H
#include "FreeRTOS.h"
#include "queue.h"


/**********************变量类型定义**************************/
//特定声明
#define J1939_TRUE         1
#define J1939_FALSE        0


typedef unsigned int   j1939_uint32_t;
typedef int            j1939_int32_t;

typedef unsigned short j1939_uint16_t;
typedef short          j1939_int16_t;

typedef unsigned char  j1939_uint8_t;
typedef char           j1939_int8_t;


//函数返回代码
#define RC_SUCCESS          0  //成功
#define RC_QUEUEEMPTY       1  //列队为空
#define RC_QUEUEFULL        1  //列队满
#define RC_CANNOTRECEIVE    2  //不能接收
#define RC_CANNOTTRANSMIT   2  //不能传输
#define RC_PARAMERROR       3  //参数错误

//设备默认的地址
#define J1939_STARTING_ADDRESS 0x55

//内部常量
#define J1939_FALSE       0
#define J1939_TRUE        1

#define J1939_RX_QUEUE_SIZE 3

#define J1939_MSG_LENGTH  5
#define J1939_DATA_LENGTH 8
#pragma anon_unions
union J1939_MESSAGE_UNION
{
    struct   me
    {
        j1939_uint8_t DataPage          : 1;
        j1939_uint8_t Res               : 1;
        j1939_uint8_t Priority          : 3;
        j1939_uint8_t PDUFormat_Top     : 3;
        j1939_uint8_t PDUFormat;
        j1939_uint8_t PDUSpecific;
        j1939_uint8_t SourceAddress;
        j1939_uint8_t DataLength        : 4;
        j1939_uint8_t RTR               : 4;//这几位总是为0  发送和接受时均没用到
        j1939_uint8_t Data[J1939_DATA_LENGTH];
    };
    struct me Mxe;
    j1939_uint8_t   Array[J1939_MSG_LENGTH + J1939_DATA_LENGTH];
};

#define GroupExtension    PDUSpecific
#define DestinationAddress  PDUSpecific
typedef union J1939_MESSAGE_UNION J1939_MESSAGE;


#pragma anon_unions

// 向外部声明的全局变量
extern J1939_MESSAGE    OneMessage;

//用户可用的全局变量（如有多线程操作，请加互斥）
extern j1939_uint8_t  J1939_Address;
extern QueueHandle_t Rx_QueueHandle;
void J1939_CAN_Transmit(J1939_MESSAGE *MsgPtr);

//初始化
extern void       J1939_Initialization(void);

//void TransmitMessageTask(void);
//void ReceiveMessageTask(void);
#endif



