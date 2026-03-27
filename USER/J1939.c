/******************************************************************
 * J1939.c - J1939 data link over CAN.
 * Receives frames from NOx sensors (SA 0x52 ch0, SA 0x51 ch1), pushes to Rx_QueueHandle with channel index.
 * J1939_CAN_Transmit() sends heater command.
 *****************************************************************/
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
QueueHandle_t Rx_QueueHandle;
static uint8_t s_tx_mailbox_timeout_cnt;

void J1939_Initialization(void)
{
    /* Queue element = channel index + J1939 message (for dual/3-sensor). */
    Rx_QueueHandle = xQueueCreate((UBaseType_t)J1939_RX_QUEUE_SIZE, (UBaseType_t)sizeof(J1939_RX_ITEM));
    if (Rx_QueueHandle == NULL)
        Error_Handler();

    /* Configure CAN filter: 32-bit mask mode, accept all; SA check in callback */
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
    can_filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan, &can_filter) != HAL_OK) {
        Error_Handler();
    }

    J1939_Address = J1939_STARTING_ADDRESS;
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}


/* Build 29-bit ID from J1939 header bytes and send 8-byte payload */
void J1939_CAN_Transmit(J1939_MESSAGE *MsgPtr)
{

    CAN_TxHeaderTypeDef TxMessage;
    uint8_t TxMessageData[8];
    uint32_t _id = 0;

    _id  = ( ( MsgPtr -> Array[0] << (8*3) ) |
             ( MsgPtr -> Array[1] << (8*2) ) |
             ( MsgPtr -> Array[2] << (8*1) ) |
             ( MsgPtr -> Array[3] << (8*0) ) );

    TxMessage.ExtId = _id;
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = MsgPtr->Mxe.DataLength;
    TxMessage.TransmitGlobalTime = DISABLE;
    TxMessageData[0] = MsgPtr->Mxe.Data[0];
    TxMessageData[1]=MsgPtr->Mxe.Data[1];
    TxMessageData[2]=MsgPtr->Mxe.Data[2];
    TxMessageData[3]=MsgPtr->Mxe.Data[3];
    TxMessageData[4]=MsgPtr->Mxe.Data[4];
    TxMessageData[5]=MsgPtr->Mxe.Data[5];
    TxMessageData[6]=MsgPtr->Mxe.Data[6];
    TxMessageData[7]=MsgPtr->Mxe.Data[7];



    uint32_t TxMailbox;
    uint32_t t0 = HAL_GetTick();
    /* Avoid dead-lock: if mailbox stays busy (e.g. no ACK/bus errors), skip this cycle. */
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) < 1U) {
        if ((HAL_GetTick() - t0) > 5U) {
            /* Mailboxes can stay busy on severe bus errors; try to recover CAN periodically. */
            if (++s_tx_mailbox_timeout_cnt >= 3U) {
                (void)HAL_CAN_Stop(&hcan);
                (void)HAL_CAN_Start(&hcan);
                (void)HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
                s_tx_mailbox_timeout_cnt = 0U;
            }
            return;
        }
    }
    s_tx_mailbox_timeout_cnt = 0U;
    (void)HAL_CAN_AddTxMessage(&hcan, &TxMessage, TxMessageData, &TxMailbox);
}
/**
 * @brief  Check if frame is from a known NOx sensor SA; return channel index.
 * @param  ext_id: 29-bit CAN extended identifier
 * @return 0 for SA 0x52, 1 for SA 0x51, -1 to discard
 */
static int8_t J1939_GetChannelFromSa(uint32_t ext_id)
{
    uint8_t sa = (uint8_t)(ext_id & 0xFF);
    if (sa == NOx_ADDRESS_CH0) return 0;
    if (sa == NOx_ADDRESS_CH1) return 1;
    return -1;
}

/* Copy received frame into J1939_RX_ITEM (channel + msg) and push to queue */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxMessage;
    uint8_t RxMessageData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, RxMessageData) != HAL_OK)
        return;

    int8_t ch = J1939_GetChannelFromSa(RxMessage.ExtId);
    if (ch < 0)
        return;

    J1939_RX_ITEM item;
    item.channel_index = (uint8_t)ch;
    item.msg.Array[0] = (j1939_uint8_t)(RxMessage.ExtId >> (8*3));
    item.msg.Array[1] = (j1939_uint8_t)(RxMessage.ExtId >> (8*2));
    item.msg.Array[2] = (j1939_uint8_t)(RxMessage.ExtId >> (8*1));
    item.msg.Array[3] = (j1939_uint8_t)(RxMessage.ExtId >> (8*0));
    item.msg.Mxe.DataLength = (RxMessage.DLC > 8) ? 8 : (uint8_t)RxMessage.DLC;
    item.msg.Mxe.Data[0] = RxMessageData[0];
    item.msg.Mxe.Data[1] = RxMessageData[1];
    item.msg.Mxe.Data[2] = RxMessageData[2];
    item.msg.Mxe.Data[3] = RxMessageData[3];
    item.msg.Mxe.Data[4] = RxMessageData[4];
    item.msg.Mxe.Data[5] = RxMessageData[5];
    item.msg.Mxe.Data[6] = RxMessageData[6];
    item.msg.Mxe.Data[7] = RxMessageData[7];

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendToBackFromISR(Rx_QueueHandle, &item, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


