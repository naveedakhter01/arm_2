/**
  ******************************************************************************
  * @file    main.c 
  * @author  DOC
  * @version V3.1.0
  * @date    06/19/2009
  * @brief   Main program body.
  ******************************************************************************
  * @copy
  * bootloader main
  * - detects firmware files on sd card to perform updates
  *   
  *
  * 
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FatSD.h"
#include "target/target.h"
#include "sysclk.h"
#include "hardware.h"
#include "stm32f10x_it.h"
#include "LCD.h"
#include "firmware_prog.h"
#include "i2c_bitbang.h"
#include "../stm32f10xreg.h"




#define MAJOR_VERSION ((uint16_t)1)
                              // .
#define MINOR_VERSION ((uint16_t)2)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  ADC_InitTypeDef ADC_InitStructure;
  SPI_InitTypeDef SPI_InitStructure; 

/* Private function prototypes -----------------------------------------------*/
//void Delay(__IO uint32_t nTime);
void NVIC_Configuration(void);
/* Private functions ---------------------------------------------------------*/




// these are set by the bootloader to indicate events to the main program 
uint32_t *flashBootloaderInstruction;// = BOOTLOADER_FLASH_INSTRUCT;   //use to flash the bootloader 
uint32_t *pageButtonDetect;// = BOOTLOADER_PAGE_DETECT; 
uint32_t *pageButtonInstruction;// = BOOTLOADER_PAGE_INSTRUCT;  
uint32_t *settingsInstruction;// = BOOTLOADER_SETTINGS_INSTRUCT; // use to signal to main program to update settings from card



/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{  
  bool isPagePushed = FALSE;
  bool isRtcAlarm = FALSE;
  bool wasResetByWatchdog = FALSE;
  
  //these are variables that get passed to the main program by sticking them at end of ram
  flashBootloaderInstruction = BOOTLOADER_FLASH_INSTRUCT; 
  pageButtonDetect = BOOTLOADER_PAGE_DETECT; 
  pageButtonInstruction = BOOTLOADER_PAGE_INSTRUCT; 
  settingsInstruction = BOOTLOADER_SETTINGS_INSTRUCT; // use to signal to main program to update settings from card
  
    
  *BOOTLOADER_MAJOR_VERSION = MAJOR_VERSION;
  *BOOTLOADER_MINOR_VERSION = MINOR_VERSION;  
  
//jump to application via function pointer
  delay_us(80);  
 
  
    /* Check if the system has resumed from IWDG reset */
  if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
  {    
    wasResetByWatchdog = TRUE;
  }
  
   hcc_u32 tmp;
  
  RCC_AHBENR |= BIT10 | BIT2; /* Enable SDIO + DMA2 clock */
  
  
  /* GPIOA, GPIOB, SPI1 clocks */
  RCC_APB2ENR |=  RCC_APB2_PERIPH_GPIOC | RCC_APB2_PERIPH_GPIOD |
                         RCC_APB2_PERIPH_SPI1 | RCC_APB2_PERIPH_AFIO ;
  RCC_AHBENR |=  RCC_AHBENR_PERIPH_DMA2EN;

  tmp = GPIOx_CRL(GPIO_C_BASE);
  tmp &= ~(GPIO_MODE_OUT_OD << (PORT12+2));
  tmp |= (GPIO_SPEED_50MHz  | (GPIO_MODE_OUT_PP <<2)) << (PORT12); /* PC12 */
  GPIOx_CRH(GPIO_C_BASE) = tmp;
  

  
  /* Setup hardware for sdio*/
  PageButtonInit(); 
  GpsOnOffInit();   // do this as this line isn't pulled down and could acidentally turn on gps if floating
  
  
  //detect page button  before checking card and indicate to next program that it was pushed 
  *pageButtonDetect = FALSE;
  if (IsPageButtonPushed() || (*pageButtonInstruction == BOOTLOADER_INSTRUCTION_CODEWORD) )
  {
    isPagePushed = TRUE;
    *pageButtonDetect = TRUE;
    *pageButtonInstruction = 0;  // reset the codeword so we don't trigger again unless set intentionally
  }  
  
  PowerCntrl(ENABLE); //
  Hardware_GPIO_config();
  
  isRtcAlarm = IsRtcInterrupt();

  if (!isPagePushed && !isRtcAlarm && !wasResetByWatchdog)      // ONLY CONTINUE IF PAGE BUTTON WAS PUSHED OR RTC ALARM WAS TRIGGERED
  {
    PowerCntrl(DISABLE); // 
    while(1) {};
  }
  
  
  //SDHWInit(); 
  //f_init();
  SD_Init();
  f_mount(&fs, "", 0);

  // reset memory that tells the main program to start reprogramming this bootloader
  *flashBootloaderInstruction = 0;
  *settingsInstruction = 0;
  
  
  //check for arm and dsp firmware to flash
  if (IsNextButtonPushed() && IsSDCardDetected())
  {
    uint32_t firmware_mask = CheckForFirmware();
    if (firmware_mask)
    {
      // arm main program flash0
      if ( (firmware_mask & ARM_FIRMWARE) == ARM_FIRMWARE)
      {  
        if (!FlashARM()) while(1) {}  // lock up if problem flashing arm
      }
      //dsp eeprom flash
      if ((firmware_mask & DSP_FIRMWARE) == DSP_FIRMWARE)
      {
        //we have to make sure dsp isn't booting up beacuse it will be read from eeprom so we monitor i2c lines and wait for 
       if (!FlashDSP()) while(1) {}  // lock up if problem flashing dsp
       DSP_Reset(HOLD_IN_RESET);
       delay_us(180);
       DSP_Reset(RELEASE_RESET);
      }
      
      if ((firmware_mask & SETTINGS_MASK) == SETTINGS_MASK)
      {
       *settingsInstruction = BOOTLOADER_SETTINGS_CODEWORD;
      }
      
      // arm bootloader flash (this code), will have to be done when main program runs, so for now just set the 
      // codeword in memory to instruct the main program to flash apon startup
      if ((firmware_mask & BOOTLOADER_FIRMWARE) == BOOTLOADER_FIRMWARE)
      {
        *flashBootloaderInstruction = BOOTLOADER_FLASH_CODEWORD;
      }
    }

    
  }
  
  
  
  
  
  //check that there is valid vector table before jump and no error from firmware flashing
  if(((*(uint32_t*)0x08010000) == (uint32_t)0xFFFFFFFF) )
  {
    LED_On();
    PowerCntrl(DISABLE);
    while(1) {}
  }
  
  int *address = (int*)0x08010004;
//  *address = 0x08001000;
// jump; actually call
  __set_MSP(*(uint32_t*)0x08010000); //set stack pointer to that of application
  *(int*)0xE000ED08 = 0x08010000; // VTOR: set the vector table of app
((void (*)())(*address))();

  while(1){ ; };
  
  
}

/**
  * @brief  Configure the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x00);             
}




void IntExtOnOffConfig(FunctionalState NewState) //
{
  EXTI_InitTypeDef EXTI_InitStructure;

  /* Initializes the EXTI_InitStructure */
  //EXTI_StructInit(&EXTI_InitStructure);

  /* Disable the EXTI line 0,1,2 and 3 on falling edge */
  if(NewState == DISABLE)
  {
    EXTI_InitStructure.EXTI_Line = EXTI_Line0|EXTI_Line1;          
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);
  }
  /* Enable the EXTI line 0,1,2 and 3 on falling edge */
  else
  {
    /* Clear the the EXTI line 0,1,2 and 3 interrupt pending bit */
    EXTI_ClearITPendingBit(EXTI_Line0|EXTI_Line1);     

    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Line = EXTI_Line0|EXTI_Line1;         
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    //usb detect / OutOfSync pin
    //EXTI_InitStructure.EXTI_Line = EXTI_Line9;         
    //EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    //EXTI_Init(&EXTI_InitStructure);    
  }

}



#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
