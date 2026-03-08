/**
 * @file    J1939.H
 * @brief   J1939 over CAN: message layout, RX queue (with channel index), transmit API.
 *          Accepts SA 0x52 (ch0) and 0x51 (ch1) for dual NOx sensors.
 */
#ifndef __J1939_H
#define __J1939_H
#include "FreeRTOS.h"
#include "queue.h"


/********************** Variable type definitions **************************/
/* J1939-specific type declarations */
#define J1939_TRUE         1
#define J1939_FALSE        0


typedef unsigned int   j1939_uint32_t;
typedef int            j1939_int32_t;

typedef unsigned short j1939_uint16_t;
typedef short          j1939_int16_t;

typedef unsigned char  j1939_uint8_t;
typedef char           j1939_int8_t;


/* Function return codes */
#define RC_SUCCESS          0  /* Success */
#define RC_QUEUEFULL        1  /* Queue full */
#define RC_CANNOTRECEIVE    2  /* Cannot receive */
#define RC_CANNOTTRANSMIT   2  /* Cannot transmit */
#define RC_PARAMERROR       3  /* Parameter error */

/* Default device address */
#define J1939_STARTING_ADDRESS 0x55
/* NOx sensor source addresses: 0x52 = outlet (ch0), 0x51 = inlet (ch1) */
#define NOx_ADDRESS_CH0 0x52
#define NOx_ADDRESS_CH1 0x51

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
        j1939_uint8_t RTR               : 4;  /* These bits always 0; not used on TX/RX */
        j1939_uint8_t Data[J1939_DATA_LENGTH];
    };
    struct me Mxe;
    j1939_uint8_t   Array[J1939_MSG_LENGTH + J1939_DATA_LENGTH];
};

#define GroupExtension    PDUSpecific
#define DestinationAddress  PDUSpecific
typedef union J1939_MESSAGE_UNION J1939_MESSAGE;

/** RX queue element: channel index (0/1) + one J1939 frame (for dual-sensor or future 3-sensor). */
typedef struct {
    uint8_t        channel_index;
    J1939_MESSAGE  msg;
} J1939_RX_ITEM;

#pragma anon_unions

/* Externally declared global variables */
extern J1939_MESSAGE    OneMessage;

/* User-visible globals (use mutex when accessed from multiple threads) */
extern j1939_uint8_t  J1939_Address;
extern QueueHandle_t Rx_QueueHandle;
void J1939_CAN_Transmit(J1939_MESSAGE *MsgPtr);

/* Initialization */
extern void       J1939_Initialization(void);

#endif



