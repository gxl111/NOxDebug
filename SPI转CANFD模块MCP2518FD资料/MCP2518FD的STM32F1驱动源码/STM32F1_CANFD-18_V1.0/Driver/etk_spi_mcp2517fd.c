/********************************************************************
filename : etk_spi_mcp2517fd.c
discript : can driver
editor   : Icy - etk
time     : 2019.9.26
statement: All rights reserved.
contact  : edreamtek@163.com
           edreamtek.taobao.com
********************************************************************/

#include "etk_spi_mcp2517fd.h"


inline int8_t spi_master_transfer(uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize);



/*******************************************************************************
* Function Name  : etk_spi_port_init(void)
* Description    : SPI port for MCP2517FD
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void etk_can_spi_port_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );//PORTB时钟使能 
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );//PORTB时钟使能 
	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI2,  ENABLE );//SPI2时钟使能 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
	//CAN1 SPI2
  GPIO_InitStructure.GPIO_Pin = CAN1_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(CAN1_CS_GPIO_PORT, &GPIO_InitStructure); 

	GPIO_InitStructure.GPIO_Pin = CAN1_INT_PIN | CAN1_TX_PIN | CAN1_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(CAN1_INT_GPIO_PORT, &GPIO_InitStructure); 

  GPIO_InitStructure.GPIO_Pin = CAN1_SPI_SCK_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(CAN1_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = CAN1_SPI_MISO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(CAN1_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = CAN1_SPI_MOSI_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(CAN1_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

 	GPIO_SetBits(GPIOB,CAN1_SPI_MISO_PIN);  //PB13/14/15上拉

  CAN1_SPI_CLK_LOW();
  CAN1_SPI_CS_HIGH();


//   /* SPI2 configuration */
//  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
//  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
//  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
//  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
//  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
//  SPI_InitStructure.SPI_CRCPolynomial = 7;
//  SPI_Init(SPI2, &SPI_InitStructure);

	
//	SPI_Cmd(SPI2, ENABLE); //使能SPI外设
}




//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
uint8_t SPI2_ReadWriteByte(uint8_t TxData)
{
	u8 dat=0,i;
	
	for(i=0;i<8;i++)
	{
		if(TxData & 0x80)
		{
			CAN1_SPI_MOSI_HIGH();
		}
		else
		{
			CAN1_SPI_MOSI_LOW();
		}
		TxData<<= 1;
		dat <<= 1;
		__NOP();
		__NOP();
		if(CAN1_MISO())
		{
			dat += 1;
		}
		CAN1_SPI_CLK_HIGH();
		__NOP();
		__NOP();
		CAN1_SPI_CLK_LOW();
		__NOP();
		__NOP();
	}

	CAN1_SPI_MOSI_LOW();
	return dat;
	
	/*
	uint8_t retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
	{
//		retry++;
//		if(retry>200)return 0;
	}			  
	SPI_I2S_SendData(SPI2, TxData); //通过外设SPIx发送一个数据
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)//检查指定的SPI标志位设置与否:接受缓存非空标志位
	{
//		retry++;
//		if(retry>200)return 0;
	}	  						    
	return SPI_I2S_ReceiveData(SPI2); //返回通过SPIx最近接收的数据					    
	*/
}


/*******************************************************************************
* Function Name  : DRV_SPI_TransferData(void)
* Description    : SPI 数据读写
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int8_t DRV_SPI_TransferData(uint8_t spiSlaveDeviceIndex, uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize)
{
	return spi2_master_transfer(SpiTxData, SpiRxData, spiTransferSize);
}





int8_t spi2_master_transfer(uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t size)
{
	uint16_t pos = 0;

	CAN1_SPI_CS_LOW();
	__NOP();
//	__NOP();
	
	while(pos < size)
	{
		SpiRxData[pos] = SPI2_ReadWriteByte(SpiTxData[pos]);
		
		pos++;
	}

	__NOP();
//	__NOP();
	CAN1_SPI_CS_HIGH();
	
	return 0;
}




/********************************************************************
function: uint8_t etk_spi_read_byte(void)
discript: Reads a byte from the SPI.
          This function must be used only if the Start_Read_Sequence
          function has been previously called.
entrance: none
return  : Byte Read from the SPI Flash.
other   : none
********************************************************************/
//uint8_t etk_spi2_read_byte(void)
//{
//  return (SPI2_ReadWriteByte(Dummy_Byte));
//}

/********************************************************************
function: uint8_t etk_spi_send_byte(uint8_t byte)
discript: Sends a byte through the SPI interface and return the byte
          received from the SPI bus.
entrance: byte : byte to send.
return  : The value of the received byte.
other   : none
********************************************************************
uint8_t etk_spi2_send_byte(uint8_t byte)
{
  // Loop while DR register in not emplty 
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

  // Send byte through the SPI peripheral
  SPI_I2S_SendData(SPI2, byte);

  // Wait to receive a byte
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

  // Return the byte read from the SPI bus
  return SPI_I2S_ReceiveData(SPI2);
}
*/




