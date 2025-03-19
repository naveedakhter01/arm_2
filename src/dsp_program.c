
#include "dsp_program.h"
#include "i2c_bitbang.h"

#include "fatSD.h"
#include "hardware.h"
#include "spi.h"
#include "stdio.h"



#define DSP_SPI SPI2
#define DSP_EEPROM_PAGE_SIZE 64

#define 	EEPROM_ADDR_RD			0xA1		/* 24LC128 I2C Eeprom*/
#define 	EEPROM_ADDR_WR			0xA0		/* 24LC128 I2C Eeprom*/
#define 	EEPROM_ADDR			0xA0		/* 24LC128 I2C Eeprom*/


#define 	DSP_ADDR_RD		        ((0x22 << 1) | 0x01)		/* DSP i2c address*/
#define 	DSP_ADDR_WR			((0x22 << 1))			/* Dsp i2c adddress*/



#define ACK       0x00
#define NACK      0x80


bool isDspSpiInit = FALSE;


// copies firmware into sram
int program_dsp_eeprom(void)
{
  FRESULT fr;     /* FatFs return code */
  //fr = f_mount(&fs, "", 0);
  //if (fr) return (int)fr;
  
  FIL file;
  
 
  
  int filesize;
  int addr;
  int i;
  bool success = FALSE;
  uint8_t databuf[DSP_EEPROM_PAGE_SIZE];
  int32_t bytesToGo;
  int progressPercentage;
  char percentageString[8];   
  UINT bytesread;
  
  
    //open firmware file 
  fr = f_open(&file, "dsp.boot", FA_READ);
  // get length of file
  if (fr) return (int)fr;
  
  filesize = f_size(&file);
  
    bytesToGo = filesize;
    // check if valid
    if (filesize <=0) return -2;

    //Write Boot file to EEPROM - also reads back data to check if valid
    fr = f_lseek(&file, 0);  //start of file

    success = TRUE; //assume true
    for(addr=0;bytesToGo>0;addr+=DSP_EEPROM_PAGE_SIZE,bytesToGo-=DSP_EEPROM_PAGE_SIZE)
    {
      if (bytesToGo > DSP_EEPROM_PAGE_SIZE)
        f_read(&file, databuf,DSP_EEPROM_PAGE_SIZE,&bytesread);
      
      else
      {
        i = f_read(&file, databuf,bytesToGo,&bytesread);
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
    
    f_close(&file);       
	
  if (!success) return 1;
  return 0;
}


int boot_dsp(void)
{
  int boot_ret;
  int ret;
  
  ret = _f_poweron();                     // power on sd card
  if (ret == FR_OK)
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
  i2c2_wr(EEPROM_ADDR_RD);  //DIR = RD
  
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


void   DspEEPROM_WriteByte( uint16_t addr, uint8_t data)
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






/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions to setup otpitons in dsp
//








void DspWriteCommand(uint8_t command, uint8_t data)
{
  i2c2_start();
  i2c2_wr(DSP_ADDR_WR);   //DIR= WR
  
  i2c2_wr(command);
  i2c2_wr(data);
  i2c2_stop();  
}





uint8_t DspReadCommand(uint8_t command)
{
  uint8_t data;
  
  //set command
  i2c2_start();
  i2c2_wr(DSP_ADDR_WR);   //DIR= WR
  i2c2_wr(command);   //DIR= WR
  i2c2_stop();   
  
  //now get data
  i2c2_start();   //second start bit
  i2c2_wr(DSP_ADDR_RD);
  
  data = i2c2_rd(NACK);
  
  i2c2_stop();        
  return data;           
}






//
//
////send a pulse to dsp telling it there is a command ready to be recived via SPI
//bool DspReadCommand(uint8_t command, uint8_t *data)
//{
//  bool ret = FALSE;
//  uint16_t sendWord;
//  uint16_t rcvWord;
//  int timeout;
// 
//  
//  //load data into spi
// 
//  //DSP_SPI->DR; //read the buffer to clear it
//  if (!isDspSpiInit) InitDspSpi();
//  
//  
//  sendWord = (uint16_t)(command <<8);
//  sendWord |=*data;
// 
//  DSP_SPI->DR = sendWord;   //load in the syncronizing word
//  while (SPI_I2S_GetFlagStatus(DSP_SPI, SPI_I2S_FLAG_RXNE))  //read receive buffer to clear crap out of it
//  {
//    rcvWord = DSP_SPI->DR; //read the buffer to clear it
//  }      
//  
//  LED_Display(LED_ON);
//  DSP_SendDataReady();
//  LED_Display(LED_OFF);
//  //wait for data to be sent
//  timeout = 5000;
//  while (!SPI_I2S_GetFlagStatus(DSP_SPI, SPI_I2S_FLAG_TXE)) 
//  {
//    if ( timeout-- <=0) return ret;
//      delay_us(1);
//  }
//  while (!SPI_I2S_GetFlagStatus(DSP_SPI, SPI_I2S_FLAG_RXNE))
//  {
//    if ( timeout-- <=0) return ret;
//    delay_us(1);
//  }     
//
//  rcvWord = DSP_SPI->DR; //read the buffer to clear it
//  rcvWord = 0; 
//  LED_Display(LED_ON);
//  //wait for data to come in
//  timeout = 5000;
//  while (!SPI_I2S_GetFlagStatus(DSP_SPI, SPI_I2S_FLAG_RXNE))
//  {
//    if ( timeout-- <=0) return ret;
//    delay_us(1);
//  }    
//  
//  rcvWord = DSP_SPI->DR;
//  LED_Display(LED_OFF);
//  *data = (uint8_t)rcvWord;
//  command &=0x7F; //remove write bit
//  if (command != (uint8_t)(rcvWord>>8)) return ret;
//  
//  ret = TRUE;
//  return ret;
//}
//
//
//bool DspWriteCommand(uint8_t command, uint8_t data)
//{
//  //load data into spi
//  command |= 0x80; // set to writemode
//  return DspReadCommand(command, &data);
//}
//


//return 4byte version number 
DspVersion_typedef *GetDspVersion(void)
{
  static DspVersion_typedef dsp_version;
  
  dsp_version.valid = FALSE;

  if (!IsDspInCommandMode()) return &dsp_version;  // returns with invalid flag if can't connect dsp
  
  dsp_version.majorVersion = DspReadCommand(VERSION_ID_0);
  dsp_version.minorVersion = DspReadCommand(VERSION_ID_1);
  
  dsp_version.valid = TRUE;
  return &dsp_version;
}










bool SetDspSamplingMode(DspSamplingOption option)
{
  if (!IsDspInCommandMode()) return FALSE;
  
  DspWriteCommand(SAMPLING_MODE, (uint8_t)option);
  
  if (DspReadCommand(SAMPLING_MODE) != (uint8_t)option) return FALSE;
  
  return TRUE;
}



#define TIMEOUT 50  // arbitrary value set based on system clock and machine cycle delays
#define TOGGLE_VALUE1 0x55
#define TOGGLE_VALUE2 0xAA
//reads from the toggle command to determine if in command mode   
bool IsDspInCommandMode()
{
  uint8_t value1;
  uint8_t value2;
  int timeout3= TIMEOUT;
  
  while(timeout3-- > 0)
  { 
    value1 = DspReadCommand(TOGGLE_READ);
    value2 = ~DspReadCommand(TOGGLE_READ);
    if (value1 == value2 ) return TRUE;

    delay_ms(20);
  }
  
  return FALSE;
}

//waits for event line to go high from dsp
bool WaitForDspEvent()
{
  int timeout= MAGIC(2000);
  
  while (timeout-- > 0)
  {
    if (DSP_IsEvent(LEVEL))  
    {
      delay_us(2000); // don't use int driven function
      return TRUE;
    }
    delay_us(10);
  }
  
  return FALSE;
}



/**************************
* AttemptConnectDsp(void)
*
* - if fails to connect will reset dsp and try one more time
*/
bool AttemptConnectDsp(void)
{
  
  if (WaitForDspEvent() && IsDspInCommandMode()) return TRUE;
  
  //reset DSP
  DSP_Reset(HOLD_IN_RESET);
  delay_ms(100);
  DSP_Reset(RELEASE_RESET); 
  
  Delay_Dsp_Start();//wait set amount of time for dsp to start then wait for response, we don't need to do this if running menu as that's enough of a delay
  
  if (WaitForDspEvent() && IsDspInCommandMode()) return TRUE;

  
  LED_Flash(); 
  
  //reset DSP
  DSP_Reset(HOLD_IN_RESET);
  delay_ms(100);
  DSP_Reset(RELEASE_RESET); 
  
  Delay_Dsp_Start();//wait set amount of time for dsp to start then wait for response, we don't need to do this if running menu as that's enough of a delay
  Delay_Dsp_Start();
  
  LED_Flash();

  
  if (WaitForDspEvent() && IsDspInCommandMode()) return TRUE;  
  
  return FALSE;
}




////////////////////////////////////////////////////////EOF/////////////////////////////////////////////////////////