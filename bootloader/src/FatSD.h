/*********************************************************************
*
*  FAT SD 
*
* functions to access SD card                   
**********************************************************************/


#include "stm32f10x.h"
//#include "fat/common/fat.h"
//#include "fat/common/common.h"
#include "fat/ff.h"
#include "fat/diskio.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;


/* Private define ------------------------------------------------------------*/
/* Define the STM32F10x FLASH Page Size depending on the used STM32 device */
#ifdef STM32F10X_LD
  #define FLASH_PAGE_SIZE    ((uint16_t)0x400)
#elif defined STM32F10X_MD
  #define FLASH_PAGE_SIZE    ((uint16_t)0x400)
#elif defined STM32F10X_HD
  #define FLASH_PAGE_SIZE    ((uint16_t)0x800)
#elif defined STM32F10X_CL
  #define FLASH_PAGE_SIZE    ((uint16_t)0x800)  
#endif /* STM32F10X_LD */








extern FATFS fs;  // Declare fs as external

int _f_poweron();
int _f_poweroff();

int _f_createdir();

int copy_firmware();
uint8_t CheckBootFile(void);
int32_t FlashBootFile(void);

void SDCardOn(void);
void SDCardOff(void);

char IsFileExist(char* Filename);







/**********************************EOF*********************************/