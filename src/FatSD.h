/*********************************************************************
*
*  FAT SD 
*
* functions to access SD card                     
**********************************************************************/
#ifndef FATSD_H
#define FATSD_H


#include "fat/ff.h"
#include "fat/diskio.h"
//#include "fat/common/common.h"




#define BOOTLOADER_MEMORY 0x00010000
#define ROM_BASE              ((uint32_t)0x08000000)
#define BOOTLOADER_START_OFFSET  ((uint32_t)0x00000000)
#define BOOTLOADER_START_ADDR (ROM_BASE + BOOTLOADER_START_OFFSET)



#define ARM_BOOTLOADER_FIRMWARE_FILENAME "bootloader.boot"  // arm bootloader only
#define ARM_BOTH_FIRMWARE_FILENAME "arm_bootloader.boot"  // both the main program and bootloader




typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private define ------------------------------------------------------------*/
/* Define the STM32F10x FLASH Page Size depending on the used STM32 device */
#ifdef STM32F10X_LD
  #define FLASH_PAGE_SIZE    ((uint16_t)0x400)
  #define FLASH_PAGE_LIMIT 126   // number of pages that are allowed to be written over
#elif defined STM32F10X_MD
  #define FLASH_PAGE_SIZE    ((uint16_t)0x400)
  #define FLASH_PAGE_LIMIT 126   // number of pages that are allowed to be written over
#elif defined STM32F10X_HD
  #define FLASH_PAGE_SIZE    ((uint16_t)0x800)
  #define FLASH_PAGE_LIMIT 254   // number of pages that are allowed to be written over
#elif defined STM32F10X_CL
  #define FLASH_PAGE_SIZE    ((uint16_t)0x800)  
  #define FLASH_PAGE_LIMIT 254   // number of pages that are allowed to be written over
#endif /* STM32F10X_LD */



extern FATFS fs;  // Declare fs as external

int _f_poweron();
int _f_poweroff();

int _f_createdir();

int CheckBootFile(void);
int DecodeBootFile(void);

void SDCardOn(void);
void SDCardOff(void);

int UsedCardSpace(void);

int DeleteCard(void);
int DeleteDirectory(char* directoryname);

int FlashBootloader(void);
int FlashBootFile(void);


#endif
/**********************************EOF*********************************/