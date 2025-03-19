/*****************************************************************
*hardware.h
*
*****************************************************************/

#ifndef __HARDWARE
#define __HARDWARE


#include "stm32f10x.h"



typedef enum
{
   HOLD_IN_RESET = 0,
   RELEASE_RESET
} ResetState;




typedef enum
{
  GREEN = 0,
  RED,
} LED_TypeDef;

//These memory address are setup to pass data to the main program. since the two are not linked . these address are at the end of ram
//These definitions must match the ones in the main program's hardware.h file
#define BOOTLOADER_INSTRUCTION_CODEWORD ((uint32_t)0xFFAA6699)
#define BOOTLOADER_FLASH_CODEWORD       ((uint32_t)0x12345678)
#define BOOTLOADER_SETTINGS_CODEWORD    ((uint32_t)0x9BDF1357)


#define BOOTLOADER_SETTINGS_INSTRUCT      ((uint32_t*)0x2000FFEC) // this is the memory address to tell the main program to update settings from card

#define BOOTLOADER_MAJOR_VERSION      ((uint16_t*)0x2000FFF0) //this is the memory address to tell the main program it's bootloader version
#define BOOTLOADER_MINOR_VERSION      ((uint16_t*)0x2000FFF2) //this is the memory address to tell the main program it's bootloader version
#define BOOTLOADER_FLASH_INSTRUCT      ((uint32_t*)0x2000FFF4) // this is the memory address to tell the main program to re flash the bootloader
#define BOOTLOADER_PAGE_INSTRUCT      ((uint32_t*)0x2000FFF8) // this is the memory address that the main progam uses when reseting to instruct bootloader to enable the page button on flag 
#define BOOTLOADER_PAGE_DETECT      ((uint32_t*)0x2000FFFC)   //this is the memory address that the bootloader writes to to indicate that the page button was pushed
////////////////////////////////////////////////////////////////////////////


#define LED_GREEN_PIN   GPIO_Pin_7
#define LED_GREEN_PORT  GPIOA
#define LED_GREEN_CLK   RCC_APB2Periph_GPIOA


#define SDCARD_DETECT_PIN   GPIO_Pin_8
#define SDCARD_DETECT_PORT  GPIOB
#define SDCARD_DETECT_CLK   RCC_APB2Periph_GPIOB


#define POK_PORT  GPIOA
#define POK_CLK   RCC_APB2Periph_GPIOA 
#define POK_PIN   GPIO_Pin_5


#define POWER_CNTRL_PORT          GPIOA
#define POWER_CNTRL_CLK           RCC_APB2Periph_GPIOA
#define POWER_CNTRL_PIN           GPIO_Pin_12    



#define PAGE_BUTTON_PORT          GPIOC
#define PAGE_BUTTON_CLK           RCC_APB2Periph_GPIOC
#define PAGE_BUTTON_PIN           GPIO_Pin_0

#define NEXT_BUTTON_PORT          GPIOC
#define NEXT_BUTTON_CLK           RCC_APB2Periph_GPIOC
#define NEXT_BUTTON_PIN           GPIO_Pin_1


#define RTC_INT_PORT          GPIOA
#define RTC_INT_CLK           RCC_APB2Periph_GPIOA
#define RTC_INT_PIN           GPIO_Pin_1    



#define DSP_RESET_PORT              GPIOB
#define DSP_RESET_CLK               RCC_APB2Periph_GPIOB  
#define DSP_RESET_PIN               GPIO_Pin_5


//GPS 
#define GPS_ON_OFF_PIN   GPIO_Pin_4
#define GPS_ON_OFF_PORT  GPIOA
#define GPS_ON_OFF_CLK   RCC_APB2Periph_GPIOA

void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);
int IsSDCardDetected(void);
void Hardware_GPIO_config(void);
void PowerCntrl(FunctionalState newState);
void PageButtonInit(void);
bool IsPageButtonPushed(void);
bool IsNextButtonPushed(void);

bool IsRtcInterrupt(void);
void GpsOnOffInit(void);


void DSP_Reset(ResetState state);


#endif /* __HARDWARE*/




/********************************************EOF*******************************/


