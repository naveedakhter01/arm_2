/*********************************************************************
*
*  FAT SD 
*
*  functions to access the sd card                     
**********************************************************************/





//#include "stm32-mci/mmci.h"
#include "fat/ff.h"
#include "FatSD.h"
#include "spi.h"
#include "record.h"
#include "stdio.h"
#include "stdlib.h"

/****************************************************************************
*
* _f_poweron
*
* This function is called to simulate power on
* Function which should call f_initvolume for the drive to be
* tested - which must be drive 0 ("A").
*
* RETURNS
*
* 0 if success or error code
*
***************************************************************************/


FATFS fs;


int _f_poweron()
{
  int ret;
  ret = SD_PowerON();
  return ret;
}

/****************************************************************************
*
* _f_poweroff
*
* This function is called to simulate power off
*
* RETURNS
*
* 0 if success or error code
*
***************************************************************************/

int _f_poweroff()
{
  int ret;
  //ret = f_unmount("");
  ret = SD_PowerOFF();
  return ret;
}






/*************************************************************************
*
* SIMPLE TESTS
*
*************************************************************************/

int _f_createdir()
{
  int ret;          /* FatFs function common result code */
  //ret = f_mount(&fs, "", 0);
  //if (ret) return (int)ret;
  FIL fsrc, fdst;      /* File objects */
  BYTE buffer[512];   /* File copy buffer */
  
  UINT br, bw;         /* File read/write count */
  
  /* Open source file on the drive 1 */
  ret = f_open(&fsrc, "file.bin", FA_READ);
  if (ret) return (int)ret;
  
  /* Create destination file on the drive 0 */
  ret = f_open(&fdst, "file2.bin", FA_WRITE | FA_CREATE_ALWAYS);
  if (ret) return (int)ret;
  
  /* Copy source to destination */
  for (;;) {
    ret = f_read(&fsrc, buffer, sizeof buffer, &br); /* Read a chunk of data from the source file */
    if (br == 0) break; /* error or eof */
    ret = f_write(&fdst, buffer, br, &bw);           /* Write it to the destination file */
    if (bw < br) break; /* error or disk full */
  }
  
  ret=f_mkdir("Apple");
  if (ret) return (int)ret;
  
  ret=f_chdir("Apple");
  if (ret) return (int)ret;
  
  
  /* Close open files */
  f_close(&fsrc);
  f_close(&fdst);
  
  ret=_f_poweroff();
  if (ret) return ret;
  
  return 0;
  
}


/******************************************************************************
* checks to see if card has boot file
*
* 0- no file, 1 - arm firmware, 2 - dsp firmware
******************************************************************************/

int CheckBootFile(void)
{
  int ret;
  //ret = f_mount(&fs, "", 0);
  //if (ret) return (int)ret;
  FIL file;
  
  //int i;
  
  
  /* 1st power on */
  ret = _f_poweron();                     // power on sd card
  
  if (ret!=FR_OK)
  {
    return 0;
  }  
  
  
  // f_chdir("Firmware");
  // if (ret) return ret;
  
  //open arm firmware file 
  ret = f_open(&file, "arm.boot", FA_READ);
  // check if valid
  if (ret==FR_OK) 
  {
    f_close(&file);
    _f_poweroff();
    //valid file
    return 1;
  }         
  f_close(&file);
  
  
  //open dsp firmware file 
  ret = f_open(&file, "dsp.boot", FA_READ);
  // check if valid
  if (ret==FR_OK) 
  {
    f_close(&file);
    _f_poweroff();
    //valid file
    return 2;
  }                          
  f_close(&file);    
  
  //open dsp firmware file 
  ret = f_open(&file, "fancy.test", FA_READ);
  // check if valid
  if (ret==FR_OK) 
  {
    f_close(&file);
    _f_poweroff();
    //valid file
    return 3;
  }                          
  f_close(&file);  
  
  
  /* simple restart */
  ret=_f_poweroff();
  
  return 0;  
  
}

/*******************************************************************************
* decodes dsp boot file
*
*******************************************************************************/

int DecodeBootFile(void)
{
  int ret;
  //ret = f_mount(&fs, "", 0);
  //if (ret) return (int)ret;
  FIL file;
  
  int32_t  filesize; //size of boot file
  uint8_t  decHByte;
  uint8_t  decLByte;
  uint16_t inWord;
  //int i;
  uint16_t HexArray[64];
  uint8_t *pbuf = (uint8_t*)0x68000000;   //start of external sram
  uint32_t datasize =0; 
  
  
  /* 1st power on */
  ret = _f_poweron();                     // power on sd card
  
  if (ret!=FR_OK)
  {
    return 0;                      
  }  
  
  //open dsp firmware file 
  ret = f_open(&file, "dsp.boot", FA_READ);
  filesize = (f_size(&file)/2);  //filesize = number of words
  
  // check if valid
  if (ret!=FR_OK) 
  {
    ret=_f_poweroff();
    return -1;
  }             
  
  datasize =0; 
  
  // read words out into buffer to decode          
  for(;filesize>0;filesize-=64)
  {                   
    UINT bytesread;
    f_read(&file,HexArray,64, &bytesread);
    
    //decode hex array and store in ext SRAM
    for(uint8_t i=0;i<64;i++)
    {
      decHByte  = *(uint8_t*)&HexArray[i];
      //decHByte -= 0x30;
      (decHByte  > 0x39) ? (decHByte -=0x37) : (decHByte -=0x30);
      
      decLByte = *((uint8_t*)&HexArray[i]+1);
      //decLByte -= 0x30;
      (decLByte  > 0x39) ? (decLByte -=0x37) : (decLByte -=0x30);
      
      *pbuf = decLByte | (decHByte <<4);
      pbuf++;
      
      
    }
    
    datasize +=64;
    f_read(&file,&inWord,1,&bytesread);
    filesize -= 1;
    if(inWord != 0x0a0d) break;
    
    
    
  }      
  
  f_close(&file);                 
  
  pbuf = (uint8_t*)0x68000000;   //start of external sram
  
  
  //open dsp firmware file 
  ret = f_open(&file, "dsp.ram", FA_OPEN_APPEND | FA_WRITE | FA_READ);
  
  UINT byteswritten;
  f_write(&file, (char*)pbuf,datasize,&byteswritten);
  f_close(&file);
  
  /* simple restart */
  ret=_f_poweroff();
  
  return datasize;  
  
}



/*********************************************
* Returns percentage of used card space
*
*********************************************/
int UsedCardSpace(void)
{
  int ret;
  //ret = f_mount(&fs, "", 0);
  //if (ret) return (int)ret;
  
  long Totalspace, Freespace;
  
    
  /* 1st power on */
  
  
  //ret = _f_poweron();
  //if (ret!=FR_OK)
  //{
  //  return(-1);
  //}
  //if (ret) return(-2);
  
  //ret = f_checkvolume(f_getdrive());  
  //if (ret) return(-2);     

  DWORD fre_clust, fre_sect, tot_sect;
  
  
  /* Get volume information and free clusters of drive 1 */
  FATFS *pFatFs = &fs;
  ret = f_getfree("0:", &fre_clust, &pFatFs);
  
  if (ret) return(-2); 
  /* Get total sectors and free sectors */
  Totalspace = (pFatFs->n_fatent - 2) * pFatFs->csize;
  Freespace = fre_clust * pFatFs->csize;
  
  Totalspace=Totalspace/2;
  Freespace=Freespace/2;
  
  //  
  //  *((long*)&Totalspace+1) = space.total_high;      
  //  *(long*)&Totalspace = space.total;  
  //  
  //  *((long*)&Freespace+1) = space.free_high;      
  //  *(long*)&Freespace = space.free;             
  //  
  //  
  /* simple restart */
  //ret=_f_poweroff();
  //if (ret) return(-2);
//  
  return (int)((( (double)Totalspace - (double)Freespace) / (double)Totalspace) *100)   ;
//  return 0;
  
}

/******************************************************************************
* DeleteAll - deletes everything from card
*
*
******************************************************************************/
//Deletes the oldest folder to free up memory
int DeleteCard(void)
{
  //	int ret;         
  //        F_FIND find;
  //	ret = _f_poweron();
  //
  //	if (ret==F_ERR_NOTFORMATTED)
  //        {
  //		ret=f_chdrive(0);	/* this is supported to select a drive which is not formatted */
  //		if (ret) return ret;
  //	}    
  //    
  //          if (!f_findfirst("A:/*.*",&find))
  //          {
  //            do
  //            {
  //              if(strcmp(".",find.filename) == 0) continue;
  //              if(strcmp("..",find.filename) == 0) continue;
  //              
  //              if (find.attr&F_ATTR_DIR)  //if a directory then call then recursive call
  //              {
  //                ret = DeleteDirectory(find.filename);
  //                if (ret) return ret; 
  //              }
  //              else
  //              {
  //                ret = f_delete(find.filename);
  //                if (ret) return ret; 
  //              }
  //            } while (!f_findnext(&find));
  //          }
  //        
  //	ret=_f_poweroff();
  //	if (ret) return ret;
  //    
  return 0;
}


//deletes all files within directory
int DeleteDirectory(char* directoryname)
{
  //  int ret;
  //  F_FIND find;
  //       
  //       
  //        ret = f_chdir(directoryname);
  //        if (ret) return ret; 
  //        
  //          if (!f_findfirst("*.*",&find))
  //          {
  //            do
  //            {
  //              if(strcmp(".",find.filename) == 0) continue;
  //              if(strcmp("..",find.filename) == 0) continue;
  //              
  //              if (find.attr&F_ATTR_DIR)  //if a directory then call then recursive call
  //              {
  //                ret = DeleteDirectory(find.filename);
  //                if (ret) return ret; 
  //              }
  //              else
  //              {
  //                ret = f_delete(find.filename);
  //                if (ret) return ret; 
  //              }
  //            } while (!f_findnext(&find));
  //          }
  //          //go back on directory
  //        ret = f_chdir("..");
  //        if (ret) return ret; 
  //        
  //        //now delete the directory
  //        ret =  f_rmdir(directoryname);
  //        if (ret) return ret; 
  
  return 0;  
}


/******************************************************
* Flash Boot file
* - is used to reprogram bootloader with file from sd card
*/

int FlashBootloader(void)
{
  bool ret = FALSE;
  write_LCD_line1 ("FlashingBoot");    
  write_LCD_line2 ("            ");  
  if (!FlashBootFile())
  {  
    write_LCD_line2 (" Done       ");
    ret = TRUE;
  }          
  else
  {
    write_LCD_line2 (" Failed     ");  
    delay_ms(1000);
  }
  delay_ms(1000);
  return ret;
}




int FlashBootFile(void)
{
  int ret;
  /* 1st power on */
    ret = _f_poweron();                     // power on sd card

    FIL file;
    
  ret = f_mount(&fs, "", 0);
  if (ret) return (int)ret;
  
    
    int32_t  filesize; //size of boot file
    int32_t filesizeCopy;
    uint32_t eraseCounter = 0, srcAddress = 0, dstAddress = 0;
    int progressPercentage;
    char percentageString[8];
    
    UINT i;
    
    __IO uint32_t nbrOfPage = 0x00;
    volatile FLASH_Status flashStatus;
    volatile TestStatus memoryProgramStatus;
    
    flashStatus = FLASH_COMPLETE;
    memoryProgramStatus = PASSED;
    uint8_t dataarray[FLASH_PAGE_SIZE];
    
        
    if (ret!=FR_OK)
    {
      return 1;                       //not formatted
    }  
    
    
    ret = f_open(&file, ARM_BOTH_FIRMWARE_FILENAME, FA_OPEN_EXISTING);
    if (ret == FR_OK) filesize=f_size(&file);
    else 
    {
      ret = f_open(&file, ARM_BOOTLOADER_FIRMWARE_FILENAME, FA_OPEN_EXISTING);
      if (ret == FR_OK) filesize=f_size(&file);
    }
    
    
    // check if valid
    if (ret!=FR_OK) 
    {
      ret=_f_poweroff();
      return -1;
    }     
    
    
    //check if filesize != 0
    
    if (filesize > BOOTLOADER_MEMORY) filesize = BOOTLOADER_MEMORY; //remove the size of the bootloader code
    filesizeCopy = filesize;
    if (filesize <= 0) 
    {
      f_close(&file);   
      ret=_f_poweroff();
      return 1;
    }     
    
    /* Unlock the Flash Program Erase controller */
    FLASH_Unlock();
    
    /* Define the number of page to be erased */
    nbrOfPage = filesize / FLASH_PAGE_SIZE;
    if ((filesize%FLASH_PAGE_SIZE) !=0) nbrOfPage++;
    if (nbrOfPage > FLASH_PAGE_LIMIT) return 1;  //over limit
    
    /* Clear All pending flags */
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	
    
    /* Erase the FLASH pages */
    for(eraseCounter = 0; (eraseCounter < nbrOfPage) && (flashStatus == FLASH_COMPLETE); eraseCounter++)
    {
      flashStatus = FLASH_ErasePage(BOOTLOADER_START_ADDR + (FLASH_PAGE_SIZE * eraseCounter));
    }
    
    /*  FLASH Word program of data at addresses defined by StartAddr and EndAddr*/
    dstAddress = ((uint32_t)BOOTLOADER_START_ADDR);
    f_lseek(&file,BOOTLOADER_START_OFFSET); // seek offset into file ignoring bootloader code
    // read data out into buffer to write to flash  
    UINT bytesread=0;
    for(;filesize>0;filesize-=FLASH_PAGE_SIZE)
    {     
      if(filesize >=FLASH_PAGE_SIZE) 
        f_read(&file,dataarray,FLASH_PAGE_SIZE, &bytesread);  
      else
      {
        i = f_read(&file,dataarray,filesize, &i); 
        for(;i<FLASH_PAGE_SIZE;i++)
          dataarray[i] =0;  //zero pad rest of block
      }     
      i = 0;
      srcAddress = (uint32_t)dataarray;   
      while((i < (FLASH_PAGE_SIZE/2)) && (flashStatus == FLASH_COMPLETE))
      {
        flashStatus = FLASH_ProgramHalfWord(dstAddress, *(uint16_t*)srcAddress);
        dstAddress += 2; //16 bit wide data
        srcAddress += 2; //16 bit wide data
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
    dstAddress = ((uint32_t)BOOTLOADER_START_ADDR);
    f_lseek(&file,BOOTLOADER_START_OFFSET); //start of file
    // read data out into buffer to write to flash          
    for(;filesize>0;filesize-=FLASH_PAGE_SIZE)
    {     
      if(filesize >=FLASH_PAGE_SIZE) 
        f_read(&file,dataarray,FLASH_PAGE_SIZE, &bytesread);  
      else
      {
        i = f_read(&file,dataarray,filesize, &i); 
        for(;i<FLASH_PAGE_SIZE;i++)
          dataarray[i] =0;  //zero pad rest of block
      }     
      i = 0;
      srcAddress = (uint32_t)dataarray;   
      while((i < (FLASH_PAGE_SIZE/2)) && (memoryProgramStatus != FAILED))
      {
        if(*(uint16_t*)dstAddress != *(uint16_t*)srcAddress)
        {
          memoryProgramStatus = FAILED;
        }
        dstAddress += 2; //16 bit wide data
        srcAddress += 2; //16 bit wide data
        i++;
      }
      
      
      
    }    
    
    //lock flash
    FLASH_Lock();    
    
    f_close(&file);   
    //f_delete("arm.boot");
    
    
    /* simple restart */
    ret=_f_poweroff();
    if(memoryProgramStatus == FAILED) return -3;  
    if (ret==FR_OK) return 0;
    
  
  return 1;
}
















/**************************************EOF**********************************/