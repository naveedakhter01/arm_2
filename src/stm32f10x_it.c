/**
  ******************************************************************************
  * @file    Project/Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.1.0
  * @date    06/19/2009
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "delay.h"
#include "main.h"
#include "menu.h"
#include "record.h"
#include "pushbutton.h"
#include "gps.h"
#include "hardware.h"



/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t capture = 0;
static uint32_t resetIWDGcount = 0; 
static bool rtcFlag = FALSE;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1) // release mode - watchdog will reset mcu. Debug no watchdog so infinite loop  
  {
  }
  
  
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{

    TimingDelay_Decrement();
  if(resetIWDGcount!= 0) 
  {
      IWDG->KR =  ((uint16_t)0xAAAA); //KR_KEY_Reload;  //WatchDog reload
      resetIWDGcount--;
  }
}





//allows clearing of watchdog in systick interrupt
//used for function calls that may not return longer than the max watchdog timeout

void ClearWatchdogInInterrupt(uint32_t secs)  //allow watchdog reset every systick interrupt
{  
  resetIWDGcount = secs * 1000;   
}


/*****************************************************************************
*
*
*
*****************************************************************************/

void TIM2_IRQHandler(void)
{


    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

    GPIO_WriteBit(GPIOC, GPIO_Pin_7, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_7)));  
  
  
  
}


/*****************************************************************************
*
*
*
*****************************************************************************/

void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)  //capture/compare int 1 
  { 
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
    //AddBatVal();  // reads adc and writes bat voltage value into buffer
  }
}



/*****************************************************************************
*
*
*
*****************************************************************************/

void TIM4_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)  //capture/compare int 1 
  { 
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);  
    if (GetWatchdogExtendCount() > 0)
      IWDG->KR =  ((uint16_t)0xAAAA); //KR_KEY_Reload;  //WatchDog reload
    else
    {
      TIM_ITConfig(TIM4, TIM_IT_CC1, DISABLE); 
      /* TIM4 enable counter */
      TIM_Cmd(TIM4, DISABLE);   
    }
      
    //AddBatVal();  // reads adc and writes bat voltage value into buffer
  }
}






/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/





void EXTI0_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
     SetPageButtonPushed();
     SetButtonFlagFunc(PAGE_KEY);
    /* Clear the Key Button EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line0);
  }
}
void EXTI1_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  {
    
     SetButtonFlagFunc(NEXT_KEY);
    /* Clear the Key Button EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line1);
  }
}
void EXTI2_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line2) != RESET)
  {
    
     SetButtonFlagFunc(PLUS_KEY);
    /* Clear the Key Button EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line2);
  }  
}

void EXTI3_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line3) != RESET)
  {
    
     SetButtonFlagFunc(MINUS_KEY);
    /* Clear the Key Button EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line3);
  }
}


void EXTI4_IRQHandler(void)
{

}


void EXTI9_5_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line6) != RESET)
  {
    
    if (GPIO_ReadInputDataBit(DSP_EVENT_PORT, DSP_EVENT_PIN)) 
      SetDspEventInterruptState(DSP_START_EVENT);
    else
      SetDspEventInterruptState(DSP_END_EVENT);
    /* Clear the Key Button EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line6);
  }  
}  
  

/**
  * @brief  This function handles RTC global interrupt request.
  * @param  None
  * @retval None
  */
void RTCAlarm_IRQHandler(void)
{
 if(RTC_GetITStatus(RTC_IT_ALR) != RESET)
  {
    /* Check if the Wake-Up flag is set */
    if(PWR_GetFlagStatus(PWR_FLAG_WU) != RESET)
    {
      /* Clear Wake Up flag */
      PWR_ClearFlag(PWR_FLAG_WU);
    }
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();  
      RTC_SetCounter(0);
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();   
    /* Clear RTC Alarm interrupt pending bit */
    RTC_ClearITPendingBit(RTC_IT_ALR);
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
    
    rtcFlag = TRUE;
  }
 
     /* Clear EXTI line17 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line17); /*!< External interrupt line 17 Connected to the RTC Alarm event */    
 
}

bool GetRTCFlag(void)
{
  bool ret = rtcFlag;
  if (rtcFlag) rtcFlag =FALSE;
  return ret;
}



/******************************************************************************
*
*
******************************************************************************/
void SPI2_IRQHandler(void)
{
  SPI2->DR;

  SPI_I2S_ClearITPendingBit(SPI2, SPI_I2S_IT_RXNE);
    /* Get the SPI IT index */
  //itpos = 0x01 << (SPI_I2S_IT_RXNE & 0x0F);
  /* Clear the selected SPI CRC Error (CRCERR) interrupt pending bit */
  SPI2->SR = (uint16_t)0xfffe;  

}







void USART2_IRQHandler(void)
{
  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  // if receive register not empty 
  {
    ReadToBuffer();
  }

  if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)   // if transmit register empty 
  {   
    WriteFromBuffer();
  }
}


/*******************************************************************************
* Function Name  : DMA1 Ch4_IRQHandler
* Description    : This function handles DMA High Priority interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel4_IRQHandler(void)
{
  RecordSession_TypeDef *recordSession;
   // GPIO_WriteBit(GPIOC, GPIO_Pin_7, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_7)));  
  recordSession = GetCurrentRecordSession();
  
  if(DMA_GetITStatus(DMA1_IT_HT4) != RESET)     //first half of buffer
  {    
    DMA_ClearITPendingBit(DMA1_IT_HT4);
    
    if (recordSession != NULL)  //if no filesession then skip
    {
      LED_DisplayOnCount();  // turns on led when upcounter matches a set value, in this case will only turn on every so often. value set by function SetDisplayNthCount(int nthCount)
      
      if ( recordSession->RecordInterrupt != NULL)
        recordSession->RecordInterrupt(SAMPLE_BUFFER_FIRSTHALF, SAMPLE_BUFFER_HALF_SIZE);
      LED_Display(LED_OFF);
      
    }

  } 
  else if(DMA_GetITStatus(DMA1_IT_TC4) != RESET)             //second half of buffer
  {    
    DMA_ClearITPendingBit(DMA1_IT_TC4);
    
    if (recordSession != NULL)  //if no filesession then skip
    {
      LED_DisplayOnCount();  // turns on led when upcounter matches a set value, in this case will only turn on every so often. value set by function SetDisplayNthCount(int nthCount)

      if ( recordSession->RecordInterrupt != NULL)
        recordSession->RecordInterrupt(SAMPLE_BUFFER_SECONDHALF, SAMPLE_BUFFER_HALF_SIZE); // write the PONG buffer to card when full

      
      LED_Display(LED_OFF);
      
    }
  }
  
  
}

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
