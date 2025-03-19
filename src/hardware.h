#ifndef __HARDWARE_H
#define __HARDWARE_H



#include "stm32f10x.h"


//These definitions must match the ones in the bootloaders hardware.h file
#define BOOTLOADER_INSTRUCTION_CODEWORD ((uint32_t)0xFFAA6699)
#define BOOTLOADER_FLASH_CODEWORD ((uint32_t)0x12345678)
#define BOOTLOADER_SETTINGS_CODEWORD ((uint32_t)0x9BDF1357)




#define BOOTLOADER_MAJOR_VERSION          ((uint16_t*)0x2000FFF0) //this is the memory address to tell the main program it's bootloader version
#define BOOTLOADER_MINOR_VERSION          ((uint16_t*)0x2000FFF2) //this is the memory address to tell the main program it's bootloader version
#define BOOTLOADER_FLASH_INSTRUCT         ((uint32_t*)0x2000FFF4) // this is the memory address to tell the main program to re flash the bootloader
#define BOOTLOADER_PAGE_INSTRUCT          (uint32_t*)0x2000FFF8   //this is the memory address that we use to tell bootloader apon reset to flag page button as set
#define BOOTLOADER_PAGE_DETECT            (uint32_t*)0x2000FFFC   //this is the memory address that the bootloader writes to to indicate that the page button was pushed
#define BOOTLOADER_SETTINGS_INSTRUCT      (uint32_t*)0x2000FFEC  // this is the memory address to tell the main program to update settings from card

///////////////////////////////////////////////////////////////



#define LED_DISPLAY_PORT              GPIOA
#define LED_DISPLAY_CLK               RCC_APB2Periph_GPIOA  
#define LED_DISPLAY_PIN               GPIO_Pin_7


#define LED_DEBUG_PORT              GPIOA
#define LED_DEBUG_CLK               RCC_APB2Periph_GPIOA  
#define LED_DEBUG_PIN               GPIO_Pin_5

//SD DETECT
#define SD_DETECT_PORT              GPIOB
#define SD_DETECT_CLK               RCC_APB2Periph_GPIOB  
#define SD_DETECT_PIN               GPIO_Pin_8


// IO1
#define DSP_EVENT_PORT              GPIOB
#define DSP_EVENT_CLK               RCC_APB2Periph_GPIOB  
#define DSP_EVENT_PIN               GPIO_Pin_6
#define DSP_EVENT_PORT_SOURCE       GPIO_PortSourceGPIOB
#define DSP_EVENT_PIN_SOURCE        GPIO_PinSource6
#define DSP_EVENT_EXTI_LINE         EXTI_Line6
#define DSP_EVENT_IRQn              EXTI9_5_IRQn 

//DSP_IO5
#define DSP_COMMAND_READY_PORT              GPIOB
#define DSP_COMMAND_READY_CLK               RCC_APB2Periph_GPIOB 
#define DSP_COMMAND_READY_PIN               GPIO_Pin_7



//IO4
#define DSP_RESERVED_PORT              GPIOA
#define DSP_RESERVED_CLK               RCC_APB2Periph_GPIOA 
#define DSP_RESERVED_PIN               GPIO_Pin_10

//IO3
#define DSP_COMMAND_MODE_PORT              GPIOA
#define DSP_COMMAND_MODE_CLK               RCC_APB2Periph_GPIOA  
#define DSP_COMMAND_MODE_PIN               GPIO_Pin_11


#define DSP_RESET_PORT              GPIOB
#define DSP_RESET_CLK               RCC_APB2Periph_GPIOB  
#define DSP_RESET_PIN               GPIO_Pin_5




#define POK_PORT  GPIOA
#define POK_CLK   RCC_APB2Periph_GPIOA 
#define POK_PIN   GPIO_Pin_5


#define POWER_CNTRL_PORT          GPIOA
#define POWER_CNTRL_CLK           RCC_APB2Periph_GPIOA
#define POWER_CNTRL_PIN           GPIO_Pin_12    



#define RTC_INT_PORT          GPIOA
#define RTC_INT_CLK           RCC_APB2Periph_GPIOA
#define RTC_INT_PIN           GPIO_Pin_1




//SD DETECT
#define SD_DETECT_PORT              GPIOB
#define SD_DETECT_CLK               RCC_APB2Periph_GPIOB  
#define SD_DETECT_PIN               GPIO_Pin_8



typedef enum
{
  LED_OFF,
  LED_ON,
  LED_TOGGLE
} LED_State_TypeDef;


typedef enum
{
  DSP_NO_EVENT,
  DSP_END_EVENT,
  DSP_START_EVENT
}
DspEvent_TypeDef;



typedef enum
{
   HOLD_IN_RESET = 0,
   RELEASE_RESET
} ResetState;



typedef enum
{
	LEVEL,
	EDGE
} DetectType;


typedef enum
{ 
  SHORT,
  LONG
}WD_Timeout;

void LED_Init(void);
void LED_Debug(LED_State_TypeDef ledState);
void LED_Display( LED_State_TypeDef ledState);
void LED_Flash(void);


void DSP_IO_Init(void);
void DSP_Reset(ResetState state);
void DSP_SendDataReady();
void Dsp_RunCommandMode(FunctionalState NewState);
void DspEventInterrupt(FunctionalState state);
void SetDspEventInterruptState(DspEvent_TypeDef dsp_event);
DspEvent_TypeDef Dsp_IsEventInterrupt(void);

void SetLedDisplayNthCount(int nthCount);
void LED_DisplayOnCount(void);



bool IsPowerOk(void);

void PowerCntrl(FunctionalState newState);

bool DSP_IsEvent(DetectType detectType);

void WaitForInterrupt(void);

void ResetWatchdog(void);
void WatchDogInit(WD_Timeout wdt_timeout);

  
void SD_Detect_Init(void);
bool IsSDCardDetected(void);


void PowerOffPeripherals(void);
void PowerOnPeripherals(void);



void ReducePowerOnce(void);


#endif   //HARDWARE