/*********************************************************************
*
*  FAT SD 
*
*  functions to access the sd card                     
**********************************************************************/




#include "stm32f10x_flash.h"
#include "fat/ff.h"
//#include "stm32-mci/mmci.h"
//#include "fat/common/port_f.h"
#include "FatSD.h"
#include "LCD.h"
#include "string.h"
#include "stdio.h"

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
  ret = SD_PowerOFF();
  return ret;
  
  //return f_delvolume(0);
}






/*************************************************************************
*
* SIMPLE TESTS
*
*************************************************************************/
int _f_createdir()
{
        FIL file;
	int ret;
        char buffer[100] = {0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F};
       // int i;
        
        
        
/* 1st power on */
	ret = _f_poweron();
        
	if (ret)
        {
		//ret=f_chdrive(0);	/* this is supported to select a drive which is not formatted */
		//if (ret) return ret;
        
		//ret=f_format(0,F_FAT16_MEDIA);
		//if (ret) return ret;
	}  
        else
       

	{
		 f_mkdir("Apple");
                 if (ret) return ret;
       
                 //set time date for folder
                 
                 
                // ret = f_settimedate("Apple",0x1032,0x2244);
                // if (ret) return ret;
                 
                 
                 f_chdir("Apple");
                 if (ret) return ret;
                 
                 ret = f_open(&file, "Pie.bin", FA_OPEN_APPEND | FA_WRITE | FA_READ);
                 if (ret) return 1;
             
                 //f_seek(file,0x10,F_SEEK_SET);
                 //f_write(buffer,1,8,file);
                 //for(i=0;i<1024;i++)                 //create file of about 32Mbyte
                         //led on DEBUGGING ONLY
                  GPIO_WriteBit(GPIOC, GPIO_Pin_7, Bit_RESET);                    
                  f_write(&file, buffer,100, NULL);     
                  GPIO_WriteBit(GPIOC, GPIO_Pin_7, Bit_SET);   
        
         delay_ms(5000);
                  GPIO_WriteBit(GPIOC, GPIO_Pin_7, Bit_RESET);                    
                  f_write(&file, buffer,100, NULL);     
                  GPIO_WriteBit(GPIOC, GPIO_Pin_7, Bit_SET);            
                  
                 f_close(&file);   
                 
	}

/* simple restart */
	ret=_f_poweroff();
	if (ret) return ret;
        
         return 0;
  
}

/**************************************EOF**********************************/