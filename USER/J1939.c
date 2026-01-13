/******************************************************************
**
** Copyright (c) 2018 by GUANGXI BOYAO NEW ENERGY  TECH.
**
** 文 件 名:  J1939
** 说    明:  J1939数据链路层
** 版    本:  V1.0
** 创建日期:
** 创 建 人:
** 修改信息：
** 修改人    修改日期       修改内容
**
*********************************************************************/
#ifndef         __J1939_SOURCE
#define         __J1939_SOURCE
#endif

#include "J1939.H"

#include "main.h"

#include "task.h"
#include "can.h"
#include "NOx.h"

j1939_uint8_t                   J1939_Address;

J1939_MESSAGE                   receivedMsg;
//FreeRTOS接受和发送队列
QueueHandle_t Rx_QueueHandle;




/********************************************************************
** 函数名:  J1939_Initialization( j1939_uint8_t InitNAMEandAddress )
** 说  明:  这段代码被调用，在系统初始化中。（放在CAN设备初始化之后）
            初始化J1939全局变量
            然后在总线上，声明设备自己的地址
            如果设备需要初始化自己的标识符和地址，将InitNAMEandAddress置位
** 输  入： InitNAMEandAddress  是否需要初始化标识符
** 输  出： 无
** 返  回： 无
** 异  常： 无
*********************************************************************/
void J1939_Initialization(void)
{

    //初始化接受队列句柄
    Rx_QueueHandle=xQueueCreate( (UBaseType_t)J1939_RX_QUEUE_SIZE, (UBaseType_t) J1939_MSG_LENGTH+J1939_DATA_LENGTH );
    if (Rx_QueueHandle == NULL )
    {
        // 队列创建失败处理
        Error_Handler();
    }

    //初始化can过滤器
    CAN_FilterTypeDef can_filter;
    can_filter.FilterIdHigh = 0x0000;
    can_filter.FilterIdLow = 0x0000;
    can_filter.FilterMaskIdHigh = 0x0000;
    can_filter.FilterMaskIdLow = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.FilterBank = 0;
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterActivation = ENABLE;
    can_filter.SlaveStartFilterBank=14;
    if (HAL_CAN_ConfigFilter(&hcan, &can_filter) != HAL_OK) {
        Error_Handler();
    }


    J1939_Address = J1939_STARTING_ADDRESS;

		HAL_CAN_Start(&hcan); // 启动CAN控制器
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 启用FIFO0中断
}


//发送消息
void J1939_CAN_Transmit(J1939_MESSAGE *MsgPtr)
{

    CAN_TxHeaderTypeDef TxMessage;
    uint8_t TxMessageData[8];
    uint32_t _id = 0;

    _id  = ( ( MsgPtr -> Array[0] << (8*3) ) |
             ( MsgPtr -> Array[1] << (8*2) ) |
             ( MsgPtr -> Array[2] << (8*1) ) |
             ( MsgPtr -> Array[3] << (8*0) ) );

    TxMessage.ExtId=_id;                            // 设置扩展标示符（29位）
    TxMessage.IDE=CAN_ID_EXT;                  // 使用扩展标识符
    TxMessage.RTR=CAN_RTR_DATA;                     // 消息类型为数据帧，一帧8位
    TxMessage.DLC=MsgPtr->Mxe.DataLength;           // 发送两帧信息

    TxMessage.TransmitGlobalTime=DISABLE;

    TxMessageData[0]=MsgPtr->Mxe.Data[0];          // 第一帧信息
    TxMessageData[1]=MsgPtr->Mxe.Data[1];
    TxMessageData[2]=MsgPtr->Mxe.Data[2];
    TxMessageData[3]=MsgPtr->Mxe.Data[3];
    TxMessageData[4]=MsgPtr->Mxe.Data[4];
    TxMessageData[5]=MsgPtr->Mxe.Data[5];
    TxMessageData[6]=MsgPtr->Mxe.Data[6];
    TxMessageData[7]=MsgPtr->Mxe.Data[7];



    uint32_t TxMailbox;
    while( HAL_CAN_GetTxMailboxesFreeLevel(&hcan)<1) {
        //LCD_ShowString(30,40,300,24,24,(uint8_t*)"TX ing       ");
        ;
    }
    if( HAL_CAN_AddTxMessage(&hcan, &TxMessage, TxMessageData, &TxMailbox)==HAL_OK) {
        //send failed

        //LCD_ShowString(30,40,300,24,24,(uint8_t*)"TX Succeed   ");

    } else {
        //发送失败

        //LCD_ShowString(30,40,300,24,24,(uint8_t*)"TX Failed   ");

    }

}
/**
 * @brief  解析J1939标识符，判断是否为指定地址的消息
 * @param  ext_id: 29位CAN扩展标识符
 * @retval pdTRUE: 是指定地址消息，pdFALSE: 不是
 */
static BaseType_t J1939_CheckTargetAddress(uint32_t ext_id)
{
    // 1. 提取J1939标识符各字段
    uint8_t priority = (ext_id >> 26) & 0x07;  // 优先级（3位）
    uint8_t data_page = (ext_id >> 24) & 0x01; // 数据页（1位）
    uint8_t pf = (ext_id >> 16) & 0xFF;        // PDU格式（8位）
    uint8_t ps = (ext_id >> 8) & 0xFF;         // PDU特定（8位）
    uint8_t sa = ext_id & 0xFF;                // 源地址SA（8位）

    // 2. 过滤源地址（最常用：只接收指定SA的消息）
    if (sa == NOx_ADDRESS )
    {
        return pdTRUE;
    }

    // 3. 可选：过滤目标地址（仅PF<240时，PS为目标地址DA）
    // 若需要只接收“发送给自己（或指定DA）”的消息，取消下面注释
    /*
    if (pf < 240 && ps == TARGET_DEST_ADDRESS)
    {
        return pdTRUE;
    }
    */

    // 4. 非指定地址，丢弃
    return pdFALSE;
}
// CAN接收中断处理函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxMessage;
    uint8_t RxMessageData[8];


    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, RxMessageData) == HAL_OK) {
			
				// 先判断是否为指定地址的消息，非目标地址直接返回
        if (J1939_CheckTargetAddress(RxMessage.ExtId) == pdFALSE)
        {
            // 不是目标地址，丢弃消息，直接退出
            return;
        }
        //将29位标志位（can_identifier）写入J1939的结构中
        receivedMsg.Array[0] = RxMessage.ExtId >> (8*3);
        receivedMsg.Array[1] = RxMessage.ExtId >> (8*2);
        receivedMsg.Array[2] = RxMessage.ExtId >> (8*1);
        receivedMsg.Array[3] = RxMessage.ExtId >> (8*0);
        //读取数据长度
        receivedMsg.Mxe.DataLength = RxMessage.DLC;
        if (receivedMsg.Mxe.DataLength > 8)
            receivedMsg.Mxe.DataLength = 8;
        //读取数据
        receivedMsg.Mxe.Data[0] = RxMessageData[0];
        receivedMsg.Mxe.Data[1] = RxMessageData[1];
        receivedMsg.Mxe.Data[2] = RxMessageData[2];
        receivedMsg.Mxe.Data[3] = RxMessageData[3];
        receivedMsg.Mxe.Data[4] = RxMessageData[4];
        receivedMsg.Mxe.Data[5] = RxMessageData[5];
        receivedMsg.Mxe.Data[6] = RxMessageData[6];
        receivedMsg.Mxe.Data[7] = RxMessageData[7];

        // 处理接收到的J1939消息的代码,消息入队列
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // xQueueSendToBackFromISR(Rx_QueueHandle,&receivedMsg,NULL);
				xQueueSendToBackFromISR(Rx_QueueHandle,&receivedMsg,&xHigherPriorityTaskWoken);
				
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // 必要时触发任务切换
    }

}


