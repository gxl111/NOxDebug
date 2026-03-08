#include "sdcard.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
/*
*********************************************************************************************************
*	                                  用户代码包含的头文件
*********************************************************************************************************
*/
#include "oled.h"
#include "modbus_slave.h"
#include "NOx.h"

uint8_t DFF=0xFF;
uint8_t test;
uint8_t SD_TYPE=0x00;

MSD_CARDINFO SD0_CardInfo;


uint8_t spi_readwrite(uint8_t Txdata){
	uint8_t Rxdata;	
	HAL_SPI_TransmitReceive(&hspi2,&Txdata,&Rxdata,1,100);
	return Rxdata;
}
//SPI1波特率设置
void SPI_setspeed(uint8_t speed){
	hspi2.Init.BaudRatePrescaler = speed;
}


//发送命令，发完释放
//
int SD_sendcmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    uint8_t r1;
    uint16_t retry = 0xFFF;

    SD_CS_DISABLE();  // 拉高确保前一个命令结束
    spi_readwrite(0xFF);  // 给一位 NOP 时钟
    SD_CS_ENABLE();  // 选中卡

    // 等待卡准备好
    retry = 20;
    while (spi_readwrite(0xFF) != 0xFF && retry--) ;
    if (retry == 0) 
	{
        SD_CS_DISABLE();
        return 0xFF;  // 卡未准备好
    }

    // 发送命令包
    spi_readwrite(cmd | 0x40);        // 命令字节（带起始位）
    spi_readwrite((arg >> 24) & 0xFF);
    spi_readwrite((arg >> 16) & 0xFF);
    spi_readwrite((arg >> 8) & 0xFF);
    spi_readwrite(arg & 0xFF);
    spi_readwrite(crc);               // CRC（只有 CMD0 和 CMD8 需要有效）

    // CMD12 后必须多读取一个字节
    if (cmd == CMD12) spi_readwrite(0xFF);

    // 等待响应（最多等待 20 个周期）
    retry = 20;
    do {
        r1 = spi_readwrite(0xFF);
    } while ((r1 & 0x80) && retry--);

    return r1;  // 正常返回响应字节（R1 格式）
}


//SD卡初始化
uint8_t SD_init(void)
{
    uint8_t r1, i;
    uint8_t buff[4];
    uint16_t retry = 0;

    // 降低 SPI 速率，初始化阶段使用低速
    SPI_setspeed(SPI_BAUDRATEPRESCALER_256);

    SD_CS_DISABLE();  // 先取消片选
    for (i = 0; i < 10; i++) spi_readwrite(0xFF);  // 发送至少 74 个时钟以进入 SPI 模式

    SD_CS_ENABLE();  // 选中卡片

    // 发送 CMD0，使 SD 卡进入 IDLE 状态
    retry = 0x10;
    do {
        r1 = SD_sendcmd(CMD0, 0, 0x95);
    } while (r1 != 0x01 && retry--);

    if (retry == 0) {
        SD_CS_DISABLE();
        return 1; // 卡无响应
    }

    SD_TYPE = 0;
    r1 = SD_sendcmd(CMD8, 0x1AA, 0x87);  // 检测是否为 SD V2 卡

    if (r1 == 0x01) {
        // 收到 R7 响应
        for (i = 0; i < 4; i++) buff[i] = spi_readwrite(0xFF);
        if (buff[2] == 0x01 && buff[3] == 0xAA) {
            // 电压范围正确，尝试使用 HCS 初始化
            retry = 0xFFFE;
            do {
                SD_sendcmd(CMD55, 0, 0x01);
                r1 = SD_sendcmd(CMD41, 0x40000000, 0x01);
            } while (r1 && retry--);

            if (retry && SD_sendcmd(CMD58, 0, 0x01) == 0) {
                for (i = 0; i < 4; i++) buff[i] = spi_readwrite(0xFF);
                SD_TYPE = (buff[0] & 0x40) ? V2HC : V2;
            }
        }
    } else {
        // 非 V2 卡，可能是 V1 或 MMC
        retry = 0xFFF;
        do {
            SD_sendcmd(CMD55, 0, 0x01);
            r1 = SD_sendcmd(CMD41, 0, 0x01);
        } while (r1 && retry--);

        if (retry) {
            SD_TYPE = V1;
        } else {
            retry = 0xFFF;
            do {
                r1 = SD_sendcmd(CMD1, 0, 0x01);
            } while (r1 && retry--);

            if (retry) SD_TYPE = MMC;
        }

        if (SD_sendcmd(CMD16, 512, 0x01) != 0) SD_TYPE = ERR;
    }

    SD_CS_DISABLE(); // 释放 SD 卡
    SPI_setspeed(SPI_BAUDRATEPRESCALER_2);  // 提高速率（后期通信）

    return (SD_TYPE ? 0 : 1);  // 0表示成功，1表示失败
}

 


//读取指定长度数据
uint8_t SD_ReceiveData(uint8_t *data, uint16_t len)
{

   uint8_t r1;
   SD_CS_ENABLE();									   
   do
   { 
      r1 = spi_readwrite(0xFF);	
      delay_ms(100);
		}while(r1 != 0xFE);	
  while(len--)
  {
   *data = spi_readwrite(0xFF);
   data++;
  }
  spi_readwrite(0xFF);
  spi_readwrite(0xFF); 										  		
  return 0;
}
//向sd卡写入一个数据包的内容 512字节
uint8_t SD_SendBlock(uint8_t*buf,uint8_t cmd)
{	
	uint16_t t;	
uint8_t r1;	
	do{
		r1=spi_readwrite(0xFF);
	}while(r1!=0xFF);
	
	spi_readwrite(cmd);
	if(cmd!=0XFD)//不是结束指令
	{
		for(t=0;t<512;t++)spi_readwrite(buf[t]);//提高速度,减少函数传参时间
	    spi_readwrite(0xFF);//忽略crc
	    spi_readwrite(0xFF);
		t=spi_readwrite(0xFF);//接收响应
		if((t&0x1F)!=0x05)return 2;//响应错误									  					    
	}						 									  					    
    return 0;//写入成功
}

//获取CID信息
uint8_t SD_GETCID (uint8_t *cid_data)
{
		uint8_t r1;
	  r1=SD_sendcmd(CMD10,0,0x01); //读取CID寄存器
		if(r1==0x00){
			r1=SD_ReceiveData(cid_data,16);
		}
		SD_CS_DISABLE();
		if(r1)return 1;
		else return 0;
}
//获取CSD信息
uint8_t SD_GETCSD(uint8_t *csd_data){
		uint8_t r1;	 
    r1=SD_sendcmd(CMD9,0,0x01);//发CMD9命令，读CSD寄存器
    if(r1==0)
	{
    	r1=SD_ReceiveData(csd_data, 16);//接收16个字节的数据 
    }
	SD_CS_DISABLE();//取消片选
	if(r1)return 1;
	else return 0;
}
//获取SD卡的总扇区数
uint32_t SD_GetSectorCount(void)
{
    uint8_t csd[16];
    uint32_t Capacity;  
    uint8_t n;
		uint16_t csize;  					    
	//取CSD信息，如果期间出错，返回0
    if(SD_GETCSD(csd)!=0) return 0;	    
    //如果为SDHC卡，按照下面方式计算
    if((csd[0]&0xC0)==0x40)	 //V2.00的卡
    {	
		csize = csd[9] + ((uint16_t)csd[8] << 8) + 1;
		Capacity = (uint32_t)csize << 10;//得到扇区数	 		   
    }else//V1.XX的卡
    {	
		n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
		csize = (csd[8] >> 6) + ((uint16_t)csd[7] << 2) + ((uint16_t)(csd[6] & 3) << 10) + 1;
		Capacity= (uint32_t)csize << (n - 9);//得到扇区数   
    }
    return Capacity;
}
int MSD0_GetCardInfo(PMSD_CARDINFO SD0_CardInfo)
{
  uint8_t r1;
  uint8_t CSD_Tab[16];
  uint8_t CID_Tab[16];

  /* Send CMD9, Read CSD */
  r1 = SD_sendcmd(CMD9, 0, 0xFF);
  if(r1 != 0x00)
  {
    return r1;
  }

  if(SD_ReceiveData(CSD_Tab, 16))
  {
	return 1;
  }

  /* Send CMD10, Read CID */
  r1 = SD_sendcmd(CMD10, 0, 0xFF);
  if(r1 != 0x00)
  {
    return r1;
  }

  if(SD_ReceiveData(CID_Tab, 16))
  {
	return 2;
  }  

  /* Byte 0 */
  SD0_CardInfo->CSD.CSDStruct = (CSD_Tab[0] & 0xC0) >> 6;
  SD0_CardInfo->CSD.SysSpecVersion = (CSD_Tab[0] & 0x3C) >> 2;
  SD0_CardInfo->CSD.Reserved1 = CSD_Tab[0] & 0x03;
  /* Byte 1 */
  SD0_CardInfo->CSD.TAAC = CSD_Tab[1] ;
  /* Byte 2 */
  SD0_CardInfo->CSD.NSAC = CSD_Tab[2];
  /* Byte 3 */
  SD0_CardInfo->CSD.MaxBusClkFrec = CSD_Tab[3];
  /* Byte 4 */
  SD0_CardInfo->CSD.CardComdClasses = CSD_Tab[4] << 4;
  /* Byte 5 */
  SD0_CardInfo->CSD.CardComdClasses |= (CSD_Tab[5] & 0xF0) >> 4;
  SD0_CardInfo->CSD.RdBlockLen = CSD_Tab[5] & 0x0F;
  /* Byte 6 */
  SD0_CardInfo->CSD.PartBlockRead = (CSD_Tab[6] & 0x80) >> 7;
  SD0_CardInfo->CSD.WrBlockMisalign = (CSD_Tab[6] & 0x40) >> 6;
  SD0_CardInfo->CSD.RdBlockMisalign = (CSD_Tab[6] & 0x20) >> 5;
  SD0_CardInfo->CSD.DSRImpl = (CSD_Tab[6] & 0x10) >> 4;
  SD0_CardInfo->CSD.Reserved2 = 0; /* Reserved */
  SD0_CardInfo->CSD.DeviceSize = (CSD_Tab[6] & 0x03) << 10;
  /* Byte 7 */
  SD0_CardInfo->CSD.DeviceSize |= (CSD_Tab[7]) << 2;
  /* Byte 8 */
  SD0_CardInfo->CSD.DeviceSize |= (CSD_Tab[8] & 0xC0) >> 6;
  SD0_CardInfo->CSD.MaxRdCurrentVDDMin = (CSD_Tab[8] & 0x38) >> 3;
  SD0_CardInfo->CSD.MaxRdCurrentVDDMax = (CSD_Tab[8] & 0x07);
  /* Byte 9 */
  SD0_CardInfo->CSD.MaxWrCurrentVDDMin = (CSD_Tab[9] & 0xE0) >> 5;
  SD0_CardInfo->CSD.MaxWrCurrentVDDMax = (CSD_Tab[9] & 0x1C) >> 2;
  SD0_CardInfo->CSD.DeviceSizeMul = (CSD_Tab[9] & 0x03) << 1;
  /* Byte 10 */
  SD0_CardInfo->CSD.DeviceSizeMul |= (CSD_Tab[10] & 0x80) >> 7;
  SD0_CardInfo->CSD.EraseGrSize = (CSD_Tab[10] & 0x7C) >> 2;
  SD0_CardInfo->CSD.EraseGrMul = (CSD_Tab[10] & 0x03) << 3;
  /* Byte 11 */
  SD0_CardInfo->CSD.EraseGrMul |= (CSD_Tab[11] & 0xE0) >> 5;
  SD0_CardInfo->CSD.WrProtectGrSize = (CSD_Tab[11] & 0x1F);
  /* Byte 12 */
  SD0_CardInfo->CSD.WrProtectGrEnable = (CSD_Tab[12] & 0x80) >> 7;
  SD0_CardInfo->CSD.ManDeflECC = (CSD_Tab[12] & 0x60) >> 5;
  SD0_CardInfo->CSD.WrSpeedFact = (CSD_Tab[12] & 0x1C) >> 2;
  SD0_CardInfo->CSD.MaxWrBlockLen = (CSD_Tab[12] & 0x03) << 2;
  /* Byte 13 */
  SD0_CardInfo->CSD.MaxWrBlockLen |= (CSD_Tab[13] & 0xc0) >> 6;
  SD0_CardInfo->CSD.WriteBlockPaPartial = (CSD_Tab[13] & 0x20) >> 5;
  SD0_CardInfo->CSD.Reserved3 = 0;
  SD0_CardInfo->CSD.ContentProtectAppli = (CSD_Tab[13] & 0x01);
  /* Byte 14 */
  SD0_CardInfo->CSD.FileFormatGrouop = (CSD_Tab[14] & 0x80) >> 7;
  SD0_CardInfo->CSD.CopyFlag = (CSD_Tab[14] & 0x40) >> 6;
  SD0_CardInfo->CSD.PermWrProtect = (CSD_Tab[14] & 0x20) >> 5;
  SD0_CardInfo->CSD.TempWrProtect = (CSD_Tab[14] & 0x10) >> 4;
  SD0_CardInfo->CSD.FileFormat = (CSD_Tab[14] & 0x0C) >> 2;
  SD0_CardInfo->CSD.ECC = (CSD_Tab[14] & 0x03);
  /* Byte 15 */
  SD0_CardInfo->CSD.CSD_CRC = (CSD_Tab[15] & 0xFE) >> 1;
  SD0_CardInfo->CSD.Reserved4 = 1;

  if(SD0_CardInfo->CardType == V2HC)
  {
	 /* Byte 7 */
	 SD0_CardInfo->CSD.DeviceSize = (uint16_t)(CSD_Tab[8]) *256;
	 /* Byte 8 */
	 SD0_CardInfo->CSD.DeviceSize += CSD_Tab[9] ;
  }

  SD0_CardInfo->Capacity = SD0_CardInfo->CSD.DeviceSize * MSD_BLOCKSIZE * 1024;
  SD0_CardInfo->BlockSize = MSD_BLOCKSIZE;

  /* Byte 0 */
  SD0_CardInfo->CID.ManufacturerID = CID_Tab[0];
  /* Byte 1 */
  SD0_CardInfo->CID.OEM_AppliID = CID_Tab[1] << 8;
  /* Byte 2 */
  SD0_CardInfo->CID.OEM_AppliID |= CID_Tab[2];
  /* Byte 3 */
  SD0_CardInfo->CID.ProdName1 = CID_Tab[3] << 24;
  /* Byte 4 */
  SD0_CardInfo->CID.ProdName1 |= CID_Tab[4] << 16;
  /* Byte 5 */
  SD0_CardInfo->CID.ProdName1 |= CID_Tab[5] << 8;
  /* Byte 6 */
  SD0_CardInfo->CID.ProdName1 |= CID_Tab[6];
  /* Byte 7 */
  SD0_CardInfo->CID.ProdName2 = CID_Tab[7];
  /* Byte 8 */
  SD0_CardInfo->CID.ProdRev = CID_Tab[8];
  /* Byte 9 */
  SD0_CardInfo->CID.ProdSN = CID_Tab[9] << 24;
  /* Byte 10 */
  SD0_CardInfo->CID.ProdSN |= CID_Tab[10] << 16;
  /* Byte 11 */
  SD0_CardInfo->CID.ProdSN |= CID_Tab[11] << 8;
  /* Byte 12 */
  SD0_CardInfo->CID.ProdSN |= CID_Tab[12];
  /* Byte 13 */
  SD0_CardInfo->CID.Reserved1 |= (CID_Tab[13] & 0xF0) >> 4;
  /* Byte 14 */
  SD0_CardInfo->CID.ManufactDate = (CID_Tab[13] & 0x0F) << 8;
  /* Byte 15 */
  SD0_CardInfo->CID.ManufactDate |= CID_Tab[14];
  /* Byte 16 */
  SD0_CardInfo->CID.CID_CRC = (CID_Tab[15] & 0xFE) >> 1;
  SD0_CardInfo->CID.Reserved2 = 1;

  return 0;  
}


//写SD卡
//buf:数据缓存区
//sector:起始扇区
//cnt:扇区数
//返回值:0,ok;其他,失败.
uint8_t SD_WriteDisk(uint8_t*buf,uint32_t sector,uint8_t cnt)
{
	uint8_t r1;
	if(SD_TYPE!=V2HC)sector *= 512;//转换为字节地址
	if(cnt==1)
	{
		r1=SD_sendcmd(CMD24,sector,0X01);//读命令
		if(r1==0)//指令发送成功
		{
			r1=SD_SendBlock(buf,0xFE);//写512个字节	   
		}
	}else
	{
		if(SD_TYPE!=MMC)
		{
			SD_sendcmd(CMD55,0,0X01);	
			SD_sendcmd(CMD23,cnt,0X01);//发送指令	
		}
 		r1=SD_sendcmd(CMD25,sector,0X01);//连续读命令
		if(r1==0)
		{
			do
			{
				r1=SD_SendBlock(buf,0xFC);//接收512个字节	 
				buf+=512;  
			}while(--cnt && r1==0);
			r1=SD_SendBlock(0,0xFD);//接收512个字节 
		}
	}   
	SD_CS_DISABLE();//取消片选
	return r1;//
}	
//读SD卡
//buf:数据缓存区
//sector:扇区
//cnt:扇区数
//返回值:0,ok;其他,失败.
uint8_t SD_ReadDisk(uint8_t*buf,uint32_t sector,uint8_t cnt)
{
	uint8_t r1;
	if(SD_TYPE!=V2HC)sector <<= 9;//转换为字节地址
	if(cnt==1)
	{
		r1=SD_sendcmd(CMD17,sector,0X01);//读命令
		if(r1==0)//指令发送成功
		{
			r1=SD_ReceiveData(buf,512);//接收512个字节	   
		}
	}else
	{
		r1=SD_sendcmd(CMD18,sector,0X01);//连续读命令
		do
		{
			r1=SD_ReceiveData(buf,512);//接收512个字节	 
			buf+=512;  
		}while(--cnt && r1==0); 	
		SD_sendcmd(CMD12,0,0X01);	//发送停止命令
	}   
	SD_CS_DISABLE();//取消片选
	return r1;//
}
/*
*********************************************************************************************************
*	                                  用户代码
*********************************************************************************************************
*/

typedef struct {
    uint8_t address;
    uint32_t BaudRate;
//    uint8_t  Parity;
    uint8_t StopBits;
//    Parameter NOx_p;
//    Parameter NOx_p1;
//    Parameter O2_p;
//    Parameter O2_p1;
    // 添加更多参数...
} SystemConfig;
//默认无校验位，1个停止位
//parit可以为0，1 ,2 无奇偶
//StopBits可以为1 2
SystemConfig config = {
    .address=1,
    .BaudRate = 115200,  // 默认值
//    .Parity=0,
    .StopBits=1,
//    .NOx_p={DEFAULTNOX_A,DEFAULTNOX_B},
//    .NOx_p1={DEFAULTNOX_A,DEFAULTNOX_B},
//    .O2_p={DEFAULTO2_A,DEFAULTO2_B},
//    .O2_p1={DEFAULTO2_A,DEFAULTO2_B},

};
char sdbuffer[30];
void WritetoSD(char filename[], BYTE write_buff[], uint8_t bufSize)
{
	FATFS fs;
	FIL file;
	uint8_t res=0;
	UINT Bw;	

	res = SD_init();		//SD卡初始化
	
	if(res == 1)
	{
		//printf("SD卡初始化失败! \r\n");
        OLED_PrintASCIIString(0, 30, "sd init failed", &afont16x8, OLED_COLOR_NORMAL);
	}
	else
	{
		//printf("SD卡初始化成功！ \r\n");
        OLED_PrintASCIIString(0, 30, "sd init succeed", &afont16x8, OLED_COLOR_NORMAL);
        OLED_ShowFrame();
	}
	
	res=f_mount(&fs,"0:",1);		//挂载
	
//	if(test_sd == 0)		//用于测试格式化
	if(res == FR_NO_FILESYSTEM)		//没有文件系统，格式化
	{
//		test_sd =1;				//用于测试格式化
		//printf("没有文件系统! \r\n");		
		res = f_mkfs("", 0, 0);		//格式化sd卡
		if(res == FR_OK)
		{
			//printf("格式化成功! \r\n");		
			res = f_mount(NULL,"0:",1); 		//格式化后先取消挂载
			res = f_mount(&fs,"0:",1);			//重新挂载	
			if(res == FR_OK)
			{
				//printf("SD卡已经成功挂载，可以进进行文件写入测试!\r\n");
                OLED_PrintASCIIString(0, 30, "sd mount succeed", &afont16x8, OLED_COLOR_NORMAL);
                OLED_ShowFrame();                   
			}	
		}
		else
		{
			//printf("格式化失败! \r\n");		
		}
	}
	else if(res == FR_OK)
	{
		//printf("挂载成功! \r\n");
        OLED_PrintASCIIString(0, 30, "sd mount succeed", &afont16x8, OLED_COLOR_NORMAL);
        OLED_ShowFrame();
	}
	else
	{
		//printf("挂载失败! \r\n");
        OLED_PrintASCIIString(0, 30, "sd mount failed", &afont16x8, OLED_COLOR_NORMAL);
	}	

	res = f_open(&file,filename,FA_OPEN_ALWAYS |FA_WRITE);
	if((res & FR_DENIED) == FR_DENIED)
	{
//		printf("卡存储已满，写入失败!\r\n");		
	}
	
	f_lseek(&file, f_size(&file));//确保写词写入不会覆盖之前的数据
	if(res == FR_OK)
	{
//		printf("打开成功/创建文件成功！ \r\n");
        OLED_PrintASCIIString(0, 30, "open succeed", &afont16x8, OLED_COLOR_NORMAL);
        OLED_ShowFrame();
		res = f_write(&file,write_buff,bufSize,&Bw);		//写数据到SD卡
		if(res == FR_OK)
		{
//			printf("文件写入成功！ \r\n");
            snprintf(sdbuffer,sizeof(sdbuffer),"w s%d",Bw);
            OLED_PrintASCIIString(0, 30,sdbuffer, &afont16x8, OLED_COLOR_NORMAL);	
//            res = f_sync(&file); // 强制同步缓存到SD卡
//            if (res == FR_OK) {
//                OLED_PrintASCIIString(0, 30, "sync succeed", &afont16x8, OLED_COLOR_NORMAL);
//            } else {
//               snprintf(sdbuffer,sizeof(sdbuffer),"sync failed%d",res);
//                OLED_PrintASCIIString(0, 30,sdbuffer, &afont16x8, OLED_COLOR_NORMAL);
//            }
		}
		else
		{
//			printf("文件写入失败！ \r\n");
            OLED_PrintASCIIString(0, 30, "write failed", &afont16x8, OLED_COLOR_NORMAL);	
		}		
	}
	else
	{
//		printf("打开文件失败!\r\n");
        
        switch (res) {
            case FR_DISK_ERR: strcpy(sdbuffer, "Physical drive error\n"); break;
            case FR_INT_ERR: strcpy(sdbuffer,"Assertion failed\n"); break;
            case FR_NOT_READY: strcpy(sdbuffer,"Drive not ready\n"); break;
            case FR_NO_FILE:strcpy(sdbuffer,"File not found\n"); break;
            case FR_NO_PATH: strcpy(sdbuffer,"Path not found\n"); break;
            case FR_INVALID_NAME: strcpy(sdbuffer,"Invalid file name\n"); break;
            case FR_DENIED: strcpy(sdbuffer,"Access denied\n"); break;
            case FR_EXIST: strcpy(sdbuffer,"File already exists\n"); break;
            case FR_INVALID_OBJECT: strcpy(sdbuffer,"Invalid parameter\n"); break;
            case FR_WRITE_PROTECTED: strcpy(sdbuffer,"Write protected\n"); break;
            case FR_INVALID_DRIVE: strcpy(sdbuffer,"Invalid drive\n"); break;
            case FR_NOT_ENABLED: strcpy(sdbuffer,"Volume not mounted\n"); break;
            case FR_NO_FILESYSTEM: strcpy(sdbuffer,"No filesystem\n"); break;
            case FR_MKFS_ABORTED: strcpy(sdbuffer,"mkfs aborted\n"); break;
            case FR_TIMEOUT: strcpy(sdbuffer,"Timeout\n"); break;
            case FR_LOCKED: strcpy(sdbuffer,"File locked\n"); break;
            case FR_NOT_ENOUGH_CORE: strcpy(sdbuffer,"Not enough memory\n"); break;
            case FR_TOO_MANY_OPEN_FILES: strcpy(sdbuffer,"Too many open files\n"); break;
            case FR_INVALID_PARAMETER: strcpy(sdbuffer,"Invalid parameter\n"); break;
            default: strcpy(sdbuffer,"Unknown error\n"); break;
        }
        
        OLED_PrintASCIIString(0, 30, sdbuffer, &afont16x8, OLED_COLOR_NORMAL);
        OLED_ShowFrame();
	}	
	
	f_close(&file);						//关闭文件		
	//f_mount(NULL,"0:",1);		 //取消挂载
	
}



// 读取配置文件并解析参数
FRESULT readConfig(const char* filename, SystemConfig* config) {
    FIL file;
    char sdbuffer[32];
    char key[32], value[32];
    
    FATFS fs;
    uint8_t sd_res = 0;
    FRESULT res;

    sd_res = SD_init();		//SD卡初始化
	
	if (sd_res == 1)
	{
        OLED_PrintASCIIString(0, 30, "sd init failed", &afont16x8, OLED_COLOR_NORMAL);
        OLED_ShowFrame();
        return FR_NOT_READY;
	}
	else
	{
        OLED_PrintASCIIString(0, 30, "sd init succeed", &afont16x8, OLED_COLOR_NORMAL);
        OLED_ShowFrame();		
	}
	
	res = f_mount(&fs, "0:", 1);		//挂载
	if (res == FR_NO_FILESYSTEM)
    {
        OLED_PrintASCIIString(0, 30, "No Filesystem!", &afont16x8, OLED_COLOR_NORMAL);
        OLED_ShowFrame();
        return FR_NO_FILESYSTEM;
    }
    else if (res != FR_OK)
    {
        OLED_PrintASCIIString(0, 30, "mount failed!", &afont16x8, OLED_COLOR_NORMAL);
        OLED_ShowFrame();
        return res;
    }


    res = f_open(&file, filename, FA_READ);
    if (res != FR_OK) return res;

    while(f_gets(sdbuffer, sizeof(sdbuffer), &file)) {
        if(sscanf(sdbuffer, "%31[^=]=%31s", key, value) == 2) {
            if(strcmp(key, "BaudRate") == 0) {
                config->BaudRate = atoi(value);
            }
            else if(strcmp(key, "address") == 0) {
                config->address = atoi(value);
            }
//            else if(strcmp(key, "Parity") == 0) {
//                config->Parity = atoi(value);
//            }
            else if(strcmp(key, "StopBits") == 0) {
                config->StopBits = atoi(value);
            }
//            }else if(strcmp(key, "NOx_A") == 0){
//                config->NOx_p.a= (float)atof(value);
//            }else if(strcmp(key, "NOx_B") == 0){
//                config->NOx_p.b= (float)atof(value);
//            }else if(strcmp(key, "NOx_A1") == 0){
//                config->NOx_p1.a= (float)atof(value);
//            }else if(strcmp(key, "NOx_B1") == 0){
//                config->NOx_p1.b= (float)atof(value);
//            }else if(strcmp(key, "O2_A") == 0){
//                config->O2_p.a= (float)atof(value);
//            }else if(strcmp(key, "O2_B") == 0){
//                config->O2_p.b= (float)atof(value);
//            }else if(strcmp(key, "O2_A1") == 0){
//                config->O2_p1.a= (float)atof(value);
//            }else if(strcmp(key, "O2_B1") == 0){
//                config->O2_p1.b= (float)atof(value);
//            }

            // 添加更多参数...
        }
    }
    
    f_close(&file);	
	//f_mount(NULL,"0:",1);		 //取消挂载
    
    return FR_OK;
}


char getbuffer[32];
char newLine[32];
char buf[20];
FIL tmpFile;
FIL file;
// 更新配置文件中的参数
FRESULT updateConfigParam(const char* filename, const char* key, uint32_t newValue) {
    
    FRESULT res;
    static uint32_t n=0;
    char tmpName[] = "temp.txt";  // 临时文件名
    
    int found = 0;
    UINT bw;
    // 创建新行
    sprintf(newLine, "%s=%lu\r\n", key, (unsigned long)newValue);
       
    // 打开原文件
    res = f_open(&file, filename, FA_READ);
    if(res != FR_OK){
        ++n;
        sprintf(buf,"update open f%d",n);
         OLED_PrintASCIIString(0, 30, buf, &afont16x8, OLED_COLOR_NORMAL);
         OLED_ShowFrame();        
         return res;
    } 
    
 
    // 创建临时文件
    
    res = f_open(&tmpFile, tmpName, FA_WRITE | FA_CREATE_ALWAYS);
    if(res != FR_OK) {
        f_close(&file);
       ++n;
        sprintf(buf,"update open f%d",n);
        OLED_PrintASCIIString(0, 30, buf, &afont16x8, OLED_COLOR_NORMAL);
        OLED_ShowFrame();
        return res;
    }
    
    // 逐行处理
    while(f_gets(getbuffer, sizeof(getbuffer), &file)) {
        
        char currentKey[32];
        if(sscanf(getbuffer, "%31[^=]", currentKey) == 1) {
            if(strcmp(currentKey, key) == 0) {
                f_write(&tmpFile, newLine, strlen(newLine), &bw);  // 替换为新值
                found = 1;
                continue;
            }else{
                f_write(&tmpFile, getbuffer, strlen(getbuffer), &bw);
            } 
           
        }
         memset(getbuffer, 0, sizeof(getbuffer));
        //f_puts(sdbuffer, &tmpFile);  // 保留原行
    }
    
    // 如果参数不存在则添加
    if(!found)
        f_write(&tmpFile, newLine, strlen(newLine), &bw);  
        //f_puts(newLine, &tmpFile);
    
    f_close(&file);
    f_close(&tmpFile);
    
       
    // 原文件换名字
    res = f_rename(filename, "config1.txt");
    if (res != FR_OK) {
        res = f_unlink(tmpName);
        sprintf(buf,"rename1 state:%d",res);
        OLED_PrintASCIIString(0, 30, buf, &afont16x8, OLED_COLOR_NORMAL);
        OLED_ShowFrame();
        return res;
    }
    
    //零时文件换位原文件的名字
    res = f_rename(tmpName, "config.txt");
    if (res != FR_OK) {
        //失败换回原文件的名字
        res = f_rename("config1.txt", "config.txt");
        res = f_unlink(tmpName);
        
        sprintf(buf,"rename2 state:%d",res);
        OLED_PrintASCIIString(0, 30, buf, &afont16x8, OLED_COLOR_NORMAL);
        OLED_ShowFrame();
        return res;
    }
    
    res = f_unlink("config1.txt");
    if (res != FR_OK) {
        sprintf(buf,"unlink state:%d",res);
        OLED_PrintASCIIString(0, 30, buf, &afont16x8, OLED_COLOR_NORMAL);
        OLED_ShowFrame();
        return res;
    }
    ++n;
    sprintf(buf,"update succeed%d",n);
    OLED_PrintASCIIString(0, 30, buf, &afont16x8, OLED_COLOR_NORMAL);

    OLED_ShowFrame();
    return FR_OK;
}

void handleConfig(void) {

    
    // 读取配置
    if(readConfig("config.txt", &config) == FR_OK) {
       OLED_PrintASCIIString(0, 30, "read succeed      ", &afont16x8, OLED_COLOR_NORMAL);
        
        //配置初始化
        SBAUD485=config.BaudRate;
        SADDR485=config.address;
//        NOx_parameter= config.NOx_p;
//        NOx_parameter1= config.NOx_p1;
//        O2_parameter= config.O2_p;
//        O2_parameter1= config.O2_p1;
        
        MDSUARTx.Init.BaudRate = SBAUD485;
//        switch(config.Parity){
//            case 0:
//                MDSUARTx.Init.Parity= UART_PARITY_NONE;
//                break;
//            case 1:
//                MDSUARTx.Init.Parity= UART_PARITY_EVEN ;
//                break;
//            case 2:
//                MDSUARTx.Init.Parity= UART_PARITY_ODD;
//                break;            
//            default:
//                break;
//        }
        switch(config.StopBits){
            case 1:
                MDSUARTx.Init.StopBits=UART_STOPBITS_1;
                break;
            case 2:
                MDSUARTx.Init.StopBits=UART_STOPBITS_2;
                break;
            default:
                break;            
        }
        HAL_UART_Init(&MDSUARTx);
        
    }else{
       OLED_PrintASCIIString(0, 30, "read failed      ", &afont16x8, OLED_COLOR_NORMAL);    
    }
    OLED_ShowFrame();
       
    
}


//毫秒级的延时
void delay_ms(uint16_t time)
{    
   uint16_t i=0;  
   while(time--)
   {
      i=12000;  //自己定义
      while(i--) ;    
   }
}
///END//


