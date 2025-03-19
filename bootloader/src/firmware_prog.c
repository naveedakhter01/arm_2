/**************************************************************************
* File Name          : firmware_prog.c
* Author             : DOC Electronics Development Team
* Version            : V1.0.0
* Date               : 17/08/2016
* Description        : functions for programming arm and dsp
***************************************************************************/

#include "firmware_prog.h"
#include "FatSD.h"
#include "hardware.h"
#include "stm32f10x_flash.h"
#include "eeprom.h"




#define DSP_SPI SPI2
#define DSP_EEPROM_PAGE_SIZE 64




void DspEEPROM_WritePage(uint16_t addr, uint8_t *srcData, uint8_t recordlength);
bool DspEEPROM_VerifyPage(uint16_t addr, uint8_t *comparebuf, uint8_t length);
uint8_t DspEEPROM_ReadByte (uint16_t addr);
void DspEEPROM_WriteByte( uint16_t addr, uint8_t data);
void DspEEPROM_WaitEepromStandbyState(void);

int32_t FlashArmFile(void);
int32_t FlashDspFile(void);



bool FlashARM(void)
{
  int i;
  int error;
 
  LCD_On();
  for(i=0;i<8;i++)
  {
    LED_On();
    delay_us(50);
    LED_Off();
    delay_us(50);      
  }
      
  LED_On();
  clear_LCD();
  write_LCD_line1("Flashing ARM");
  write_LCD_line2("            ");

  
  error = FlashArmFile();
  LED_Off();
      
  write_LCD_line2("           ");       
  if (error == 0)
  {
    write_LCD_line2("   Done.    ");      
    delay_us(2000);
    LCD_Off();
    return TRUE;
  }
  else
    write_LCD_line2("   Error.   ");
   return FALSE;
  
}


bool FlashDSP(void)
{
  int error;
 
  LCD_On();
  LED_On();
  clear_LCD();

  write_LCD_line1("Flashing DSP");
  write_LCD_line2(" waiting..  ");

  WaitForI2c2();
  i2c2_init(); //     
  write_LCD_line2("            ");
  error = FlashDspFile();
  LED_Off();
      
  write_LCD_line2("           ");       
  if (error == 0)
  {
    write_LCD_line2("   Done.    ");      
    delay_us(2000);
    LCD_Off();
    return TRUE;
  }
  else
  {
    
    write_LCD_line2("   Error.");
    LCD_out_hex((char)error); 
  }
    return FALSE;
 
}




uint32_t CheckForFirmware(void)
{
  int ret;
  uint32_t flash_mask; 
  //int i;
  
  /* 1st power on */
  ret = _f_poweron();                     // power on sd card
  if (ret)
  {
    _f_poweroff();
    return 0;                       //not formatted
  }  

  
  flash_mask = 0; //reset mask
  
  FIL file;
  FILINFO fno;
  
  
  ret=f_stat(ARM_MAIN_FIRMWARE_FILENAME, &fno); //mark if arm firmware present 
  if (!ret)
  {
    flash_mask |= ARM_FIRMWARE;
  }
  
  ret=f_stat(DSP_FIRMWARE_FILENAME, &fno); //mark if dsp firmware present
  if (!ret)
  {
    flash_mask |= DSP_FIRMWARE;
  }
  
  ret=f_stat(ARM_BOOTLOADER_FIRMWARE_FILENAME, &fno); //mark if bootloader firmware present
  if (!ret)
  {
    flash_mask |= BOOTLOADER_FIRMWARE; 
  }
  
  ret=f_stat(ARM_BOTH_FIRMWARE_FILENAME, &fno); //mark if arm and bootloader firmware present
  if (!ret)
  {
    flash_mask |= ARM_FIRMWARE; 
    flash_mask |= BOOTLOADER_FIRMWARE; 
  }
  
  ret=f_stat(SETTINGS_FILENAME, &fno); //mark if settings file present
  if (!ret)
  {
    flash_mask |= SETTINGS_MASK; 
  }
  
   /* simple restart */
  ret=_f_poweroff();
  if (ret) flash_mask = 0; // if error then clear mask
  
  return flash_mask;  
}








int32_t FlashArmFile(void)
{
  FIL file;
  int ret;
  int32_t  filesize; //size of boot file
  int32_t filesizeCopy;
  uint32_t EraseCounter = 0x00, SrcAddress = 0x00, DstAddress = 0x00;
  int progressPercentage;
  char percentageString[8];
  
  uint16_t i;
  
  __IO uint32_t NbrOfPage = 0x00;
  volatile FLASH_Status FLASHStatus;
  volatile TestStatus MemoryProgramStatus;
  
  FLASHStatus = FLASH_COMPLETE;
  MemoryProgramStatus = PASSED;
  uint8_t dataarray[FLASH_PAGE_SIZE];
  
  /* 1st power on */
  ret = _f_poweron();                     // power on sd card
  
  if (ret)
  {
    return -2;                       //not formatted
  }  
  
  FILINFO fno;
  
  //open dsp firmware file - thre are two possible files either just the arm main firmware, or the one with the bootloader.
  if ( !f_open(&file,  ARM_BOTH_FIRMWARE_FILENAME, FA_READ))
  {  
    filesize = f_size(&file);
  }
  else if ( !f_open(&file,  ARM_MAIN_FIRMWARE_FILENAME, FA_READ) )
  {  
    filesize = f_size(&file);
  }
  
  else
  {
    ret=_f_poweroff();
    return -1;
  }         
             
             
  //check if filesize != 0
  filesize -= PROGRAM_START_OFFSET; //remove the size of the bootloader code
  filesizeCopy = filesize;
  if (filesize <= 0) 
  {
    //f_close(file);   
    ret=_f_poweroff();
    return -1;
  }     
  UINT notneeded;
  /* Unlock the Flash Program Erase controller */
  FLASH_Unlock();
  
  /* Define the number of page to be erased */
  NbrOfPage = filesize / FLASH_PAGE_SIZE;
  if ((filesize%FLASH_PAGE_SIZE) !=0) NbrOfPage++;
  if (NbrOfPage > 254) return 1;  //over limit
  
  /* Clear All pending flags */
  FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	
  
  /* Erase the FLASH pages */
  for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
  {
    FLASHStatus = FLASH_ErasePage(PROGRAM_START_ADDR + (FLASH_PAGE_SIZE * EraseCounter));
  }
  
  /*  FLASH Word program of data at addresses defined by StartAddr and EndAddr*/
  DstAddress = ((uint32_t)PROGRAM_START_ADDR);
  f_lseek(&file,PROGRAM_START_OFFSET); // seek offset into file ignoring bootloader code
  // read data out into buffer to write to flash          
  for(;filesize>0;filesize-=FLASH_PAGE_SIZE)
  {     
    if(filesize >=FLASH_PAGE_SIZE) 
      f_read(&file, dataarray,FLASH_PAGE_SIZE, &notneeded);  
    else
    {
      UINT bytesread;
      ret = f_read(&file, dataarray,filesize,&bytesread); 
      if(ret) return ret;
      for(;bytesread<FLASH_PAGE_SIZE;bytesread++)
        dataarray[bytesread] =0;  //zero pad rest of block
    }     
    i = 0;
    SrcAddress = (uint32_t)dataarray;   
    while((i < (FLASH_PAGE_SIZE/2)) && (FLASHStatus == FLASH_COMPLETE))
    {
      FLASHStatus = FLASH_ProgramHalfWord(DstAddress, *(uint16_t*)SrcAddress);
      DstAddress += 2; //16 bit wide data
      SrcAddress += 2; //16 bit wide data
      i++;
    }
    
    progressPercentage =  ((filesizeCopy -  filesize) * 100) / filesizeCopy;
    sprintf(percentageString,"%%%u",progressPercentage);
    
    LCD_Addr(SECONDLINE_START_INDEX + 2);
    write_LCD(" ");
    write_LCD((uint8_t*)percentageString); 
    write_LCD("   ");
    
  }      
  
  // Check the correctness of written data 
  /*  FLASH Word program of data at addresses defined by StartAddr and EndAddr*/
  DstAddress = ((uint32_t)PROGRAM_START_ADDR);
  f_lseek(&file,PROGRAM_START_OFFSET); //start of file
  // read data out into buffer to write to flash          
  for(;filesize>0;filesize-=FLASH_PAGE_SIZE)
  {     
    if(filesize >=FLASH_PAGE_SIZE) 
      f_read(&file, dataarray,FLASH_PAGE_SIZE,&notneeded);  
    else
    {
      UINT bytesread;
      ret = f_read(&file, dataarray,filesize,&bytesread); 
      for(;bytesread<FLASH_PAGE_SIZE;bytesread++)
        dataarray[bytesread] =0;  //zero pad rest of block
    }     
    i = 0;
    SrcAddress = (uint32_t)dataarray;   
    while((i < (FLASH_PAGE_SIZE/2)) && (MemoryProgramStatus != FAILED))
    {
      if(*(uint16_t*)DstAddress != *(uint16_t*)SrcAddress)
      {
        MemoryProgramStatus = FAILED;
      }
      DstAddress += 2; //16 bit wide data
      SrcAddress += 2; //16 bit wide data
      i++;
    }
  }    
  
  //lock flash
  FLASH_Lock();    
  
  f_close(&file);   
  //f_delete("arm.boot");
  
  
  /* simple restart */
  ret=_f_poweroff();
  if (ret) return ret;
  if(MemoryProgramStatus == FAILED) return -3;  
  
  return 0;
}


// copies firmware into sram
int FlashDspFile(void)
{
  FIL file;
  int ret;
  int filesize;
  int addr;
  int i;
  bool success = FALSE;
  uint8_t databuf[DSP_EEPROM_PAGE_SIZE];
  int32_t bytesToGo;
  int progressPercentage;
  char percentageString[8];        
  UINT notneeded;
  ret = _f_poweron();                     // power on sd card
  if (ret)
  {  
    return 1;
  }
  
  
    //open firmware file 
    ret = f_open(&file, "dsp.boot", FA_READ);
    // get length of file
    filesize = f_size(&file);
    bytesToGo = filesize;
    // check if valid
    if (ret) return 1;
    if (filesize <=0) return 2;
    
    
    //Write Boot file to EEPROM - also reads back data to check if valid
    ret = f_lseek(&file,0); //start of file
    if (ret) return 3;
    success = TRUE; //assume true
    for(addr=0;bytesToGo>0;addr+=DSP_EEPROM_PAGE_SIZE,bytesToGo-=DSP_EEPROM_PAGE_SIZE)
    {
      if (bytesToGo > DSP_EEPROM_PAGE_SIZE)
        f_read(&file,databuf,DSP_EEPROM_PAGE_SIZE,&notneeded);
      else
      {
        UINT bytesread;
        ret = f_read(&file,databuf,bytesToGo,&bytesread);
        for(;bytesread<DSP_EEPROM_PAGE_SIZE;bytesread++) databuf[bytesread] =0;  //zero pad rest of block
      } 
      DspEEPROM_WritePage(addr, databuf, DSP_EEPROM_PAGE_SIZE);
      
      if (!DspEEPROM_VerifyPage(addr, databuf, DSP_EEPROM_PAGE_SIZE))  //if verify fails then exit loop
      {
       success = FALSE;
       break;
      }
      
      progressPercentage =  ((filesize -  bytesToGo) * 100) / filesize;
      sprintf(percentageString,"%%%u",progressPercentage);
      
      LCD_Addr(SECONDLINE_START_INDEX + 4);
      write_LCD((uint8_t*)percentageString); 
    }
    
    ret =f_close(&file);       
    if (ret) return 4;
  ret=_f_poweroff();
  if (ret) return 5;     
	
  if (!success) return 6;
  return 0;
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




/****************************************EOF************************************/

