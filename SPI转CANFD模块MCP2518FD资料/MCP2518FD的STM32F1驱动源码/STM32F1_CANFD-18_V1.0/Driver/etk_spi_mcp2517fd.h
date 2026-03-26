/********************************************************************
filename : etk_spi_mcp2518fd.h
discript : can driver
version  : V0.0
editor   : Icy - etk
time     : 2019.9.26
statement: All rights reserved.
contact  : edreamtek@163.com
           edreamtek.taobao.com
********************************************************************/

#ifndef __ETK_SPI_MCP2518FD_H__
#define __ETK_SPI_MCP2518FD_H__

#include "stm32f10x.h"
#include "etk_delay.h"


//SPI2 for CAN
#define CAN1_SPI                           SPI2
#define CAN1_SPI_CLK                       RCC_APB1Periph_SPI2

#define CAN1_SPI_SCK_PIN                   GPIO_Pin_13
#define CAN1_SPI_SCK_GPIO_PORT             GPIOB              
#define CAN1_SPI_SCK_GPIO_CLK              RCC_APB2Periph_GPIOB

#define CAN1_SPI_MISO_PIN                  GPIO_Pin_14
#define CAN1_SPI_MISO_GPIO_PORT            GPIOB
#define CAN1_SPI_MISO_GPIO_CLK             RCC_APB2Periph_GPIOB

#define CAN1_SPI_MOSI_PIN                  GPIO_Pin_15
#define CAN1_SPI_MOSI_GPIO_PORT            GPIOB           
#define CAN1_SPI_MOSI_GPIO_CLK             RCC_APB2Periph_GPIOB

#define CAN1_CS_PIN                        GPIO_Pin_12           
#define CAN1_CS_GPIO_PORT                  GPIOB                  
#define CAN1_CS_GPIO_CLK                   RCC_APB2Periph_GPIOB

#define CAN1_INT_PIN                       GPIO_Pin_8
#define CAN1_INT_GPIO_PORT                 GPIOA  
#define CAN1_INT_GPIO_CLK                  RCC_APB2Periph_GPIOA

#define CAN1_TX_PIN                        GPIO_Pin_11     
#define CAN1_TX_GPIO_PORT                  GPIOA       
#define CAN1_TX_GPIO_CLK                   RCC_APB2Periph_GPIOA

#define CAN1_RX_PIN                        GPIO_Pin_12
#define CAN1_RX_GPIO_PORT                  GPIOA 
#define CAN1_RX_GPIO_CLK                   RCC_APB2Periph_GPIOA


#define Dummy_Byte            	0xFF


/* Exported macro -------------------------------------------------*/
#define CAN1_SPI_CS_LOW()       GPIO_ResetBits(CAN1_CS_GPIO_PORT, CAN1_CS_PIN)
#define CAN1_SPI_CS_HIGH()      GPIO_SetBits(CAN1_CS_GPIO_PORT, CAN1_CS_PIN)


#define CAN1_SPI_CLK_LOW()       GPIO_ResetBits(CAN1_SPI_SCK_GPIO_PORT, CAN1_SPI_SCK_PIN)
#define CAN1_SPI_CLK_HIGH()      GPIO_SetBits(CAN1_SPI_SCK_GPIO_PORT, CAN1_SPI_SCK_PIN)

#define CAN1_SPI_MOSI_LOW()       GPIO_ResetBits(CAN1_SPI_MOSI_GPIO_PORT, CAN1_SPI_MOSI_PIN)
#define CAN1_SPI_MOSI_HIGH()      GPIO_SetBits(CAN1_SPI_MOSI_GPIO_PORT, CAN1_SPI_MOSI_PIN)


#define CAN1_INT()							GPIO_ReadInputDataBit(CAN1_INT_GPIO_PORT,CAN1_INT_PIN)
#define CAN1_TX()								GPIO_ReadInputDataBit(CAN1_TX_GPIO_PORT,CAN1_TX_PIN)
#define CAN1_RX()								GPIO_ReadInputDataBit(CAN1_RX_GPIO_PORT,CAN1_RX_PIN)

#define CAN1_MISO()							GPIO_ReadInputDataBit(CAN1_SPI_MISO_GPIO_PORT,CAN1_SPI_MISO_PIN)


// Index to SPI channel
// Used when multiple MCP25xxFD are connected to the same SPI interface, but with different CS
#define CANFD_CH1         0


// Index to SPI channel
// Used when multiple MCP25xxFD are connected to the same SPI interface, but with different CS
#define SPI_DEFAULT_BUFFER_LENGTH 96



/* Exported functions ---------------------------------------------*/
//Normally called by user.
void etk_can_spi_port_init(void);

//数据读写
int8_t DRV_SPI_TransferData(uint8_t spiSlaveDeviceIndex, uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize);



/* Private functions ----------------------------------------------*/
//Normally won't be called by user.
int8_t spi2_master_transfer(uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize);
uint8_t etk_spi2_read_byte(void);
uint8_t etk_spi2_send_byte(uint8_t byte);
uint16_t etk_spi2_send_halfword(uint16_t HalfWord);


void SPI2_Init(void);			 //初始化SPI口
void SPI2_SetSpeed(u8 SpeedSet); //设置SPI速度   
u8 SPI2_ReadWriteByte(u8 TxData);//SPI总线读写一个字节



#endif

