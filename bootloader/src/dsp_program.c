
#include "dsp_program.h"
#include "i2c_bitbang.h"
#include "stm32-mci/mmci.h"
#include "fat/common/port_f.h"
#include "fatSD.h"
#include "hardware.h"
#include "spi.h"
#include "stdio.h"


#define DSP_SPI SPI2
#define DSP_EEPROM_PAGE_SIZE 64

#define 	EEPROM_ADDR_RD			0xA1		/* 24LC128 I2C Eeprom*/
#define 	EEPROM_ADDR_WR			0xA0		/* 24LC128 I2C Eeprom*/
#define 	EEPROM_ADDR			0xA0		/* 24LC128 I2C Eeprom*/

#define ACK       0x00
#define NACK      0x80

bool isDspSpiInit = FALSE;





// copies firmware into sram
int program_dsp_eeprom(void)
{
  F_FILE *file;
  int ret;
  int filesize;
  int addr;
  int i;
  bool success = FALSE;
  uint8_t databuf[DSP_EEPROM_PAGE_SIZE];
  int32_t bytesToGo;
  int progressPercentage;
  char percentageString[8];        
  
  
    //open firmware file 
    file = f_open("dsp.boot", "r");
    // get length of file
    filesize = f_filelength("dsp.boot");
    bytesToGo = filesize;
    // check if valid
    if (!file) return -1;
    if (filesize <=0) return -2;
    
    
    //Write Boot file to EEPROM - also reads back data to check if valid
    f_seek(file,0,F_SEEK_SET); //start of file
    success = TRUE; //assume true
    for(addr=0;bytesToGo>0;addr+=DSP_EEPROM_PAGE_SIZE,bytesToGo-=DSP_EEPROM_PAGE_SIZE)
    {
      if (bytesToGo > DSP_EEPROM_PAGE_SIZE)
        f_read(databuf,1,DSP_EEPROM_PAGE_SIZE,file);
      else
      {
        i = f_read(databuf,1,bytesToGo,file);
        for(;i<DSP_EEPROM_PAGE_SIZE;i++) databuf[i] =0;  //zero pad rest of block
      } 
      DspEEPROM_WritePage(addr, databuf, DSP_EEPROM_PAGE_SIZE);
      
      if (!DspEEPROM_VerifyPage(addr, databuf, DSP_EEPROM_PAGE_SIZE))  //if verify fails then exit loop
      {
       success = FALSE;
       break;
      }
      
      progressPercentage =  ((filesize -  bytesToGo) * 100) / filesize;
      sprintf(percentageString,"%%%u",progressPercentage);
      
      LCD_Addr(SECONDLINE_START_INDEX + 2);
      write_LCD((uint8_t*)percentageString); 
    }
    
    f_close(file);       
	
  if (!success) return 1;
  return 0;
}






int boot_dsp(void)
{
  int boot_ret;
  int ret;
  
  ret = _f_poweron();                     // power on sd card
  if (ret!=F_ERR_NOTFORMATTED)
  {  
  
    boot_ret = program_dsp_eeprom();
  }
  ret=_f_poweroff();
  if (ret) return -3; 
  
  return boot_ret;
}




void DspEEPROM_WritePage(uint16_t addr, uint8_t *srcData, uint8_t recordlength)
{
  int count = 0;
  
  i2c2_start();
  i2c2_wr(EEPROM_ADDR_WR);   //DIR= WR
  i2c2_wr((uint8_t)(addr>>8));
  i2c2_wr((uint8_t)(addr));
  
  while ( count < recordlength)
  {
          i2c2_wr(*srcData);
          srcData++;
          count++;
  }	
  i2c2_stop(); 
  
  DspEEPROM_WaitEepromStandbyState();
}



bool DspEEPROM_VerifyPage(uint16_t addr, uint8_t *comparebuf, uint8_t length)
{
  int i;
  bool match = TRUE;
        
  i2c2_start();
  i2c2_wr(EEPROM_ADDR_WR);   //DIR= WR
  i2c2_wr((uint8_t)(addr>>8));
  i2c2_wr((uint8_t)(addr));
  
  i2c2_start();   //second start bit
  i2c2_wr(EEPROM_ADDR_RD);
  
  for (i=0;i<DSP_EEPROM_PAGE_SIZE-1;i++)
  {
    if(comparebuf[i] != i2c2_rd(ACK)) match = FALSE;
  }
  
  if ( comparebuf[i] != i2c2_rd(NACK) ) match = FALSE;
  i2c2_stop(); 
  
  return match;
}


uint8_t DspEEPROM_ReadByte (uint16_t addr)
{
	uint8_t data1;

	i2c2_start();
        i2c2_wr(EEPROM_ADDR_WR);   //DIR= WR
	i2c2_wr((uint8_t)(addr>>8));
        i2c2_wr((uint8_t)(addr));
        
        i2c2_start();   //second start bit
        i2c2_wr(EEPROM_ADDR_RD);  //DIR = RD

	data1 = i2c2_rd(NACK);

        i2c2_stop();        
        return data1;           
}


void DspEEPROM_WriteByte( uint16_t addr, uint8_t data)
{
	i2c2_start();
        i2c2_wr(EEPROM_ADDR_WR);   //DIR= WR
	
        i2c2_wr((uint8_t)(addr>>8));
        i2c2_wr((uint8_t)(addr));
        i2c2_wr(data);
        
        i2c2_stop();  
        DspEEPROM_WaitEepromStandbyState();
}

void DspEEPROM_WaitEepromStandbyState(void)
{
   uint8_t ack;
   int timeout;
  ack = 1;
  
  timeout = 500;
  while((ack) && (timeout))
  {
    delay_us(10);
    i2c2_start();
    ack = i2c2_wr(EEPROM_ADDR_WR);   //DIR= WR
    timeout--;
  } 
  i2c2_stop();
}


////////////////////////////////////////////////////////EOF/////////////////////////////////////////////////////////