
//#ifndef __J1939_Config_H
//#define __J1939_Config_H

//#include "J1939.H"
//#include "main.h"
//#include "can.h"
//#include "oled.h"
////#include "stm32f10x.h"

///***************************J1939 标识符和地址配置*************************************/
////设备默认的地址（地址命名是有规定的，参考J1939的附录B 地址和标识符的分配）
//#define J1939_STARTING_ADDRESS 0x55


///******************************J1939功能配置******************************************/
////是否使用接受协议TP对设备的地址进行远程配置（参考J1939-21，J1939-81）
//#define J1939_RX_QUEUE_SIZE 3
////当mcu来不及处理消息，接收消息列队是否允许被新的消息覆盖
//#define J1939_OVERWRITE_RX_QUEUE J1939_FALSE
//#define J1939_TX_QUEUE_SIZE 1
////当mcu来不及处理消息，发送消息列队是否允许被新的消息覆盖
//#define J1939_OVERWRITE_TX_QUEUE J1939_FALSE
////是否使用轮询模式（否则使用中断模式）
//#define J1939_POLL_ECAN J1939_TRUE


///******************************J1939移植配置函数******************************************/

//#define Port_CAN_Transmit(MsgPtr) J1939_CAN_Transmit(MsgPtr)
//#define Port_CAN_Receive(MsgPtr) J1939_CAN_Receive(MsgPtr)
//#define Port_SetAddressFilter(Address) J1939_SetAddressFilter(Address)


///********************************************************************
//** 函数名:  j1939_int8_t J1939_TP_RX_Message(j1939_int8_t *data,j1939_uint16_t data_num)
//** 说  明:  基于SAE J1939协议，我们需要CAN控制器提供至少3个滤波器给J1939协议代码。三个滤波器分别配置如下：
//        1. 设置滤波器0，只接受广播信息（PF = 240 -255）。
//        2. 设置设置滤波器1，2只接受全局地址（J1939_GLOBAL_ADDRESS）
//        3. 随着程序的运行，将改变滤波器2，来适应程序逻辑。
//        J1939_SetAddressFilter() 是用来设置滤波器2的， 函数主要设置PS位（目标地址），其目的是，让控制器
//        只接受发送给本设备的消息。
//** 输  入： 0~0xFF
//** 输  出： 无
//** 返  回： 无
//** 异  常： 无
//*********************************************************************/
//void J1939_SetAddressFilter(unsigned char Ps_Address)
//{
//    ;
//}






///********************************************************************
//** 函数名:  void J1939_CAN_Transmit(J1939_MESSAGE *MsgPtr)
//** 说  明:  从MsgPtr加载到CAN驱动自带的结构体中
//            将和ChangeGroupIDofLMO（）联合移植（可不这样设计）
//            先将传入函数的MsgPtr中的数据写到CAN的结构体（加载），调用CAN驱动的发送函数
//** 输  入： J1939_MESSAGE *MsgPtr ，协议要发送的消息，
//** 输  出： 无
//** 返  回： 无
//** 异  常： 无
//*********************************************************************/
//void J1939_CAN_Transmit(J1939_MESSAGE *MsgPtr)
//{
//    //uint8_t mbox;
//    //uint32_t i = 0;
//    CAN_TxHeaderTypeDef TxMessage;
//    uint8_t TxMessageData[8];
//    uint32_t _id = 0;

//    _id  = ( ( MsgPtr -> Array[0] << (8*3) ) |
//             ( MsgPtr -> Array[1] << (8*2) ) |
//             ( MsgPtr -> Array[2] << (8*1) ) |
//             ( MsgPtr -> Array[3] << (8*0) ) );

//    TxMessage.ExtId=_id;                            // 设置扩展标示符（29位）
//    TxMessage.IDE=CAN_ID_EXT;                  // 使用扩展标识符
//    TxMessage.RTR=CAN_RTR_DATA;                     // 消息类型为数据帧，一帧8位
//    TxMessage.DLC=MsgPtr->Mxe.DataLength;           // 发送两帧信息

//    TxMessage.TransmitGlobalTime=DISABLE;

//    TxMessageData[0]=MsgPtr->Mxe.Data[0];          // 第一帧信息
//    TxMessageData[1]=MsgPtr->Mxe.Data[1];
//    TxMessageData[2]=MsgPtr->Mxe.Data[2];
//    TxMessageData[3]=MsgPtr->Mxe.Data[3];
//    TxMessageData[4]=MsgPtr->Mxe.Data[4];
//    TxMessageData[5]=MsgPtr->Mxe.Data[5];
//    TxMessageData[6]=MsgPtr->Mxe.Data[6];
//    TxMessageData[7]=MsgPtr->Mxe.Data[7];



//    uint32_t TxMailbox;
//    if( HAL_CAN_AddTxMessage(&hcan, &TxMessage, TxMessageData, &TxMailbox)!=HAL_OK) {
//        //send failed
//    }

//}



/////********************************************************************
////** 函数名:  void J1939_CAN_Transmit(J1939_MESSAGE *MsgPtr)
////** 说  明:  读取CAN驱动的数据，如果没有数据，返回0
////            将设备CAN中的数据取出，存入J1939_MESSAGE结构体中
////** 输  入： J1939_MESSAGE *MsgPtr 数据要存入的内存的指针
////** 输  出： 无
////** 返  回： 1：成功    0：失败
////** 异  常： 无
////*********************************************************************/
////int J1939_CAN_Receive(J1939_MESSAGE *MsgPtr)
////{
////    //CanRxMsg RxMessage;
////    CAN_RxHeaderTypeDef RxMessage;
////    uint8_t RxMessageData[8];

////    if(HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0,&RxMessage, RxMessageData)==HAL_OK) {
////        //LCD_ShowString(30,500,200,24,24,(uint8_t*)"RECEIVE OK");
////        //将29位标志位（can_identifier）写入J1939的结构中
////        MsgPtr->Array[0] = RxMessage.ExtId>>(8*3);
////        MsgPtr->Array[1] = RxMessage.ExtId>>(8*2);
////        MsgPtr->Array[2] = RxMessage.ExtId>>(8*1);
////        MsgPtr->Array[3] = RxMessage.ExtId>>(8*0);
////        //读取数据长度
////        MsgPtr->Mxe.DataLength = RxMessage.DLC;
////        if (MsgPtr->Mxe.DataLength > 8)
////            MsgPtr->Mxe.DataLength = 8;
////        //读取数据
////        MsgPtr->Mxe.Data[0] = RxMessageData[0];
////        MsgPtr->Mxe.Data[1] = RxMessageData[1];
////        MsgPtr->Mxe.Data[2] = RxMessageData[2];
////        MsgPtr->Mxe.Data[3] = RxMessageData[3];
////        MsgPtr->Mxe.Data[4] = RxMessageData[4];
////        MsgPtr->Mxe.Data[5] = RxMessageData[5];
////        MsgPtr->Mxe.Data[6] = RxMessageData[6];
////        MsgPtr->Mxe.Data[7] = RxMessageData[7];
////        return 1;
////    } else {
////        return 0;
////    }

////}

//// 定义一个全局的J1939_MESSAGE结构体用于存储接收到的消息
//J1939_MESSAGE receivedMsg;

//// CAN接收中断处理函数
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//    CAN_RxHeaderTypeDef RxMessage;
//    uint8_t RxMessageData[8];

//    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, RxMessageData) == HAL_OK) {
//        //LCD_ShowString(30,500,200,24,24,(uint8_t*)"RECEIVE OK");
//        //将29位标志位（can_identifier）写入J1939的结构中
//        receivedMsg.Array[0] = RxMessage.ExtId >> (8*3);
//        receivedMsg.Array[1] = RxMessage.ExtId >> (8*2);
//        receivedMsg.Array[2] = RxMessage.ExtId >> (8*1);
//        receivedMsg.Array[3] = RxMessage.ExtId >> (8*0);
//        //读取数据长度
//        receivedMsg.Mxe.DataLength = RxMessage.DLC;
//        if (receivedMsg.Mxe.DataLength > 8)
//            receivedMsg.Mxe.DataLength = 8;
//        //读取数据
//        receivedMsg.Mxe.Data[0] = RxMessageData[0];
//        receivedMsg.Mxe.Data[1] = RxMessageData[1];
//        receivedMsg.Mxe.Data[2] = RxMessageData[2];
//        receivedMsg.Mxe.Data[3] = RxMessageData[3];
//        receivedMsg.Mxe.Data[4] = RxMessageData[4];
//        receivedMsg.Mxe.Data[5] = RxMessageData[5];
//        receivedMsg.Mxe.Data[6] = RxMessageData[6];
//        receivedMsg.Mxe.Data[7] = RxMessageData[7];

//        // 处理接收到的J1939消息的代码
//        xQueueSendToBackFromISR(
//            QueueHandle_t xQueue,
//            const void *pvItemToQueue,
//            BaseType_t *pxHigherPriorityTaskWoken
//        );
//    }
//}



//#endif
