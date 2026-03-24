#include "sdcard.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
/*
*********************************************************************************************************
*	                                  User application includes
*********************************************************************************************************
*/
// #include "oled.h"  /* OLED 已禁用 */
#include "modbus_slave.h"
#include "NOx.h"

uint8_t DFF=0xFF;
uint8_t test;
uint8_t SD_TYPE=0x00;

MSD_CARDINFO SD0_CardInfo;


uint8_t spi_readwrite(uint8_t Txdata){
	uint8_t Rxdata;	
	HAL_SPI_TransmitReceive(&hspi1,&Txdata,&Rxdata,1,100);
	return Rxdata;
}

/* SD 卡走 SPI1（PA5/6/7）；SPI2 专用于 MCP2515 */
void SPI_setspeed(uint8_t speed){
	hspi1.Init.BaudRatePrescaler = speed;
	(void)HAL_SPI_Init(&hspi1);
}


// Send command; CS toggled inside this routine.
//
int SD_sendcmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    uint8_t r1;
    uint16_t retry = 0xFFF;

    SD_CS_DISABLE();  // Deassert CS so previous command completes
    spi_readwrite(0xFF);  // One NOP clock
    SD_CS_ENABLE();  // Assert CS (select card)

// Wait until card is ready
    retry = 20;
    while (spi_readwrite(0xFF) != 0xFF && retry--) ;
    if (retry == 0) 
	{
        SD_CS_DISABLE();
        return 0xFF;  // Card not ready
    }

// Send command packet
    spi_readwrite(cmd | 0x40);        // Command byte (start bit set)
    spi_readwrite((arg >> 24) & 0xFF);
    spi_readwrite((arg >> 16) & 0xFF);
    spi_readwrite((arg >> 8) & 0xFF);
    spi_readwrite(arg & 0xFF);
    spi_readwrite(crc);               // CRC (valid only for CMD0 and CMD8)

// After CMD12 clock one extra byte
    if (cmd == CMD12) spi_readwrite(0xFF);

// Wait for response (up to 20 clocks)
    retry = 20;
    do {
        r1 = spi_readwrite(0xFF);
    } while ((r1 & 0x80) && retry--);

    return r1;  // R1 response byte
}


// SD card initialization
uint8_t SD_init(void)
{
    uint8_t r1, i;
    uint8_t buff[4];
    uint16_t retry = 0;

// Lower SPI speed for init phase
    SPI_setspeed(SPI_BAUDRATEPRESCALER_256);

    SD_CS_DISABLE();  // Deassert CS first
    for (i = 0; i < 10; i++) spi_readwrite(0xFF);  // At least 74 clocks to enter SPI mode

    SD_CS_ENABLE();  // Assert CS

// CMD0: put card in IDLE
    retry = 0x10;
    do {
        r1 = SD_sendcmd(CMD0, 0, 0x95);
    } while (r1 != 0x01 && retry--);

    if (retry == 0) {
        SD_CS_DISABLE();
        return 1; // No response
    }

    SD_TYPE = 0;
    r1 = SD_sendcmd(CMD8, 0x1AA, 0x87);  // Check SD V2 card

    if (r1 == 0x01) {
// R7 response
        for (i = 0; i < 4; i++) buff[i] = spi_readwrite(0xFF);
        if (buff[2] == 0x01 && buff[3] == 0xAA) {
// Voltage OK; try HCS init
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
// Not V2; may be V1 or MMC
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

    SD_CS_DISABLE(); // Release SD card
    SPI_setspeed(SPI_BAUDRATEPRESCALER_2);  // Higher speed for normal transfer

    return (SD_TYPE ? 0 : 1);  // 0 success, 1 fail
}

 


// Read fixed-length data
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
// Write one 512-byte block to SD
uint8_t SD_SendBlock(uint8_t*buf,uint8_t cmd)
{	
	uint16_t t;	
uint8_t r1;	
	do{
		r1=spi_readwrite(0xFF);
	}while(r1!=0xFF);
	
	spi_readwrite(cmd);
	if(cmd!=0XFD)// Not stop token
	{
		for(t=0;t<512;t++)spi_readwrite(buf[t]);// Fast path: inline SPI
	    spi_readwrite(0xFF);// Ignore CRC
	    spi_readwrite(0xFF);
		t=spi_readwrite(0xFF);// Read response token
		if((t&0x1F)!=0x05)return 2;// Bad token
	}						 									  					    
    return 0;// Write OK
}

// Read CID
uint8_t SD_GETCID (uint8_t *cid_data)
{
		uint8_t r1;
	  r1=SD_sendcmd(CMD10,0,0x01); // CMD10 read CID
		if(r1==0x00){
			r1=SD_ReceiveData(cid_data,16);
		}
		SD_CS_DISABLE();
		if(r1)return 1;
		else return 0;
}
// Read CSD
uint8_t SD_GETCSD(uint8_t *csd_data){
		uint8_t r1;	 
    r1=SD_sendcmd(CMD9,0,0x01);// CMD9 read CSD
    if(r1==0)
	{
    	r1=SD_ReceiveData(csd_data, 16);// 16 bytes
    }
	SD_CS_DISABLE();// Deassert CS
	if(r1)return 1;
	else return 0;
}
// Total sector count
uint32_t SD_GetSectorCount(void)
{
    uint8_t csd[16];
    uint32_t Capacity;  
    uint8_t n;
		uint16_t csize;  					    
// Get CSD; return 0 on error
    if(SD_GETCSD(csd)!=0) return 0;	    
// SDHC capacity
    if((csd[0]&0xC0)==0x40)	 // V2.00 card
    {	
		csize = csd[9] + ((uint16_t)csd[8] << 8) + 1;
		Capacity = (uint32_t)csize << 10;// Sector count
    }else// V1.x card
    {	
		n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
		csize = (csd[8] >> 6) + ((uint16_t)csd[7] << 2) + ((uint16_t)(csd[6] & 3) << 10) + 1;
		Capacity= (uint32_t)csize << (n - 9);// Sector count
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


// Write SD card
// buf: buffer
// sector: start sector
// cnt: sector count
// return 0 ok else fail
uint8_t SD_WriteDisk(uint8_t*buf,uint32_t sector,uint8_t cnt)
{
	uint8_t r1;
	if(SD_TYPE!=V2HC)sector *= 512;// Byte address for non-HC
	if(cnt==1)
	{
		r1=SD_sendcmd(CMD24,sector,0X01);// CMD24 single block write
		if(r1==0)// Command accepted
		{
			r1=SD_SendBlock(buf,0xFE);// Write 512 bytes
		}
	}else
	{
		if(SD_TYPE!=MMC)
		{
			SD_sendcmd(CMD55,0,0X01);	
			SD_sendcmd(CMD23,cnt,0X01);// Predefine block count
		}
 		r1=SD_sendcmd(CMD25,sector,0X01);// CMD25 multi-block write
		if(r1==0)
		{
			do
			{
				r1=SD_SendBlock(buf,0xFC);// Send 512-byte block
				buf+=512;  
			}while(--cnt && r1==0);
			r1=SD_SendBlock(0,0xFD);// Stop token
		}
	}   
	SD_CS_DISABLE();// Deassert CS
	return r1;//
}	
// Read SD card
// buf: buffer
// sector: sector
// cnt: sector count
// return 0 ok else fail
uint8_t SD_ReadDisk(uint8_t*buf,uint32_t sector,uint8_t cnt)
{
	uint8_t r1;
	if(SD_TYPE!=V2HC)sector <<= 9;// Byte address for non-HC
	if(cnt==1)
	{
		r1=SD_sendcmd(CMD17,sector,0X01);// CMD17 read single block
		if(r1==0)// Command accepted
		{
			r1=SD_ReceiveData(buf,512);// Read 512 bytes
		}
	}else
	{
		r1=SD_sendcmd(CMD18,sector,0X01);// CMD18 multi-block read
		do
		{
			r1=SD_ReceiveData(buf,512);// Read 512 bytes
			buf+=512;  
		}while(--cnt && r1==0); 	
		SD_sendcmd(CMD12,0,0X01);	// Stop transmission
	}   
	SD_CS_DISABLE();// Deassert CS
	return r1;//
}
/*
*********************************************************************************************************
*	                                  User application code
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
    /* Add more fields here if needed */
} SystemConfig;
/* Default: no parity, 1 stop bit. Parity 0/1/2; StopBits 1 or 2 */
SystemConfig config = {
    .address=1,
    .BaudRate = 115200,  /* default */
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

	res = SD_init();		/* SD init */

	if(res == 1)
	{
		/* SD init failed (was printf) */
        // OLED_PrintASCIIString(0, 30, "sd init failed", &afont16x8, OLED_COLOR_NORMAL);
	}
	else
	{
		/* SD init ok (was printf) */
        // OLED_PrintASCIIString(0, 30, "sd init succeed", &afont16x8, OLED_COLOR_NORMAL);
        // OLED_ShowFrame();
	}
	
	res=f_mount(&fs,"0:",1);		/* mount volume */

/* if(test_sd == 0) test format path */
	if(res == FR_NO_FILESYSTEM)		/* no FS: format */
	{
/* test_sd = 1 */
		/* no filesystem (was printf) */
		res = f_mkfs("", 0, 0);		/* format SD */
		if(res == FR_OK)
		{
			/* format ok (was printf) */
			res = f_mount(NULL,"0:",1); 		/* unmount after mkfs */
			res = f_mount(&fs,"0:",1);			/* remount */
			if(res == FR_OK)
			{
				/* mount ok (was printf) */
                // OLED_PrintASCIIString(0, 30, "sd mount succeed", &afont16x8, OLED_COLOR_NORMAL);
                // OLED_ShowFrame();                   
			}	
		}
		else
		{
			/* format failed (was printf) */
		}
	}
	else if(res == FR_OK)
	{
		/* mount ok (was printf) */
        // OLED_PrintASCIIString(0, 30, "sd mount succeed", &afont16x8, OLED_COLOR_NORMAL);
        // OLED_ShowFrame();
	}
	else
	{
		/* mount failed (was printf) */
        // OLED_PrintASCIIString(0, 30, "sd mount failed", &afont16x8, OLED_COLOR_NORMAL);
	}	

	res = f_open(&file,filename,FA_OPEN_ALWAYS |FA_WRITE);
	if((res & FR_DENIED) == FR_DENIED)
	{
/* card full (was printf) */
	}

	f_lseek(&file, f_size(&file));/* append: do not overwrite */
	if(res == FR_OK)
	{
/* open ok (was printf) */
        // OLED_PrintASCIIString(0, 30, "open succeed", &afont16x8, OLED_COLOR_NORMAL);
        // OLED_ShowFrame();
		res = f_write(&file,write_buff,bufSize,&Bw);		/* write to SD */
		if(res == FR_OK)
		{
/* write ok (was printf) */
            snprintf(sdbuffer,sizeof(sdbuffer),"w s%d",Bw);
            // OLED_PrintASCIIString(0, 30,sdbuffer, &afont16x8, OLED_COLOR_NORMAL);	
/* optional f_sync to flush */
//            if (res == FR_OK) {
//                OLED_PrintASCIIString(0, 30, "sync succeed", &afont16x8, OLED_COLOR_NORMAL);
//            } else {
//               snprintf(sdbuffer,sizeof(sdbuffer),"sync failed%d",res);
//                OLED_PrintASCIIString(0, 30,sdbuffer, &afont16x8, OLED_COLOR_NORMAL);
//            }
		}
		else
		{
/* write failed (was printf) */
            // OLED_PrintASCIIString(0, 30, "write failed", &afont16x8, OLED_COLOR_NORMAL);	
		}		
	}
	else
	{
/* open failed (was printf) */

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
        
        // OLED_PrintASCIIString(0, 30, sdbuffer, &afont16x8, OLED_COLOR_NORMAL);
        // OLED_ShowFrame();
	}	
	
	f_close(&file);						/* close file */
	/* f_mount(NULL,"0:",1); unmount */

}



/* Read config file and parse key=value */
FRESULT readConfig(const char* filename, SystemConfig* config) {
    FIL file;
    char sdbuffer[32];
    char key[32], value[32];
    
    FATFS fs;
    uint8_t sd_res = 0;
    FRESULT res;

    sd_res = SD_init();		/* SD init */
	
	if (sd_res == 1)
	{
        // OLED_PrintASCIIString(0, 30, "sd init failed", &afont16x8, OLED_COLOR_NORMAL);
        // OLED_ShowFrame();
        return FR_NOT_READY;
	}
	else
	{
        // OLED_PrintASCIIString(0, 30, "sd init succeed", &afont16x8, OLED_COLOR_NORMAL);
        // OLED_ShowFrame();		
	}
	
	res = f_mount(&fs, "0:", 1);		/* mount */
	if (res == FR_NO_FILESYSTEM)
    {
        // OLED_PrintASCIIString(0, 30, "No Filesystem!", &afont16x8, OLED_COLOR_NORMAL);
        // OLED_ShowFrame();
        return FR_NO_FILESYSTEM;
    }
    else if (res != FR_OK)
    {
        // OLED_PrintASCIIString(0, 30, "mount failed!", &afont16x8, OLED_COLOR_NORMAL);
        // OLED_ShowFrame();
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

            /* add more keys here */
        }
    }

    f_close(&file);
	/* optional unmount */
    
    return FR_OK;
}


char getbuffer[32];
char newLine[32];
char buf[20];
FIL tmpFile;
FIL file;
/* Update one key=value in config file via temp file */
FRESULT updateConfigParam(const char* filename, const char* key, uint32_t newValue) {
    
    FRESULT res;
    static uint32_t n=0;
    char tmpName[] = "temp.txt";  /* temp file name */

    int found = 0;
    UINT bw;
    /* build new line */
    sprintf(newLine, "%s=%lu\r\n", key, (unsigned long)newValue);

    /* open source file */
    res = f_open(&file, filename, FA_READ);
    if(res != FR_OK){
        ++n;
        sprintf(buf,"update open f%d",n);
         // OLED_PrintASCIIString(0, 30, buf, &afont16x8, OLED_COLOR_NORMAL);
         // OLED_ShowFrame();        
         return res;
    }

    /* create temp file */
    res = f_open(&tmpFile, tmpName, FA_WRITE | FA_CREATE_ALWAYS);
    if(res != FR_OK) {
        f_close(&file);
       ++n;
        sprintf(buf,"update open f%d",n);
        // OLED_PrintASCIIString(0, 30, buf, &afont16x8, OLED_COLOR_NORMAL);
        // OLED_ShowFrame();
        return res;
    }

    /* copy lines; replace matching key */
    while(f_gets(getbuffer, sizeof(getbuffer), &file)) {
        
        char currentKey[32];
        if(sscanf(getbuffer, "%31[^=]", currentKey) == 1) {
            if(strcmp(currentKey, key) == 0) {
                f_write(&tmpFile, newLine, strlen(newLine), &bw);  /* write new line */
                found = 1;
                continue;
            }else{
                f_write(&tmpFile, getbuffer, strlen(getbuffer), &bw);
            } 
           
        }
         memset(getbuffer, 0, sizeof(getbuffer));
        /* keep original line */
    }

    /* append key if missing */
    if(!found)
        f_write(&tmpFile, newLine, strlen(newLine), &bw);  
        //f_puts(newLine, &tmpFile);
    
    f_close(&file);
    f_close(&tmpFile);

    /* rename original aside */
    res = f_rename(filename, "config1.txt");
    if (res != FR_OK) {
        res = f_unlink(tmpName);
        sprintf(buf,"rename1 state:%d",res);
        // OLED_PrintASCIIString(0, 30, buf, &afont16x8, OLED_COLOR_NORMAL);
        // OLED_ShowFrame();
        return res;
    }

    /* promote temp to config.txt */
    res = f_rename(tmpName, "config.txt");
    if (res != FR_OK) {
        /* rollback */
        res = f_rename("config1.txt", "config.txt");
        res = f_unlink(tmpName);
        
        sprintf(buf,"rename2 state:%d",res);
        // OLED_PrintASCIIString(0, 30, buf, &afont16x8, OLED_COLOR_NORMAL);
        // OLED_ShowFrame();
        return res;
    }
    
    res = f_unlink("config1.txt");
    if (res != FR_OK) {
        sprintf(buf,"unlink state:%d",res);
        // OLED_PrintASCIIString(0, 30, buf, &afont16x8, OLED_COLOR_NORMAL);
        // OLED_ShowFrame();
        return res;
    }
    ++n;
    sprintf(buf,"update succeed%d",n);
    // OLED_PrintASCIIString(0, 30, buf, &afont16x8, OLED_COLOR_NORMAL);

    // OLED_ShowFrame();
    return FR_OK;
}

void handleConfig(void) {

    /* load config from SD */
    if(readConfig("config.txt", &config) == FR_OK) {
       // OLED_PrintASCIIString(0, 30, "read succeed      ", &afont16x8, OLED_COLOR_NORMAL);

        /* apply to UART/Modbus */
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
       // OLED_PrintASCIIString(0, 30, "read failed      ", &afont16x8, OLED_COLOR_NORMAL);    
    }
    // OLED_ShowFrame();
       
    
}


/* Busy-wait delay (~ms); tune i for CPU clock */
void delay_ms(uint16_t time)
{
   uint16_t i=0;
   while(time--)
   {
      i=12000;  /* loop count per ms */
      while(i--) ;    
   }
}
///END//


