/*****************************************************************
*hardware.c
*
*****************************************************************/
#include "hardware.h"

void Hardware_GPIO_config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;  
  
    // Enable PWR ON
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC 
                         | RCC_APB2Periph_ADC1| RCC_APB2Periph_ADC2 | RCC_APB2Periph_AFIO, ENABLE);  
      //SETUP OUTPUTS
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      //LED GREEN
  GPIO_InitStructure.GPIO_Pin = LED_GREEN_PIN;
  GPIO_Init(LED_GREEN_PORT, &GPIO_InitStructure);
  
  //INPUT PULLUPS
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  GPIO_InitStructure.GPIO_Pin = SDCARD_DETECT_PIN;
  GPIO_Init(SDCARD_DETECT_PORT, &GPIO_InitStructure); 
  
  
  //INPUT PULLUPS
 // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
 // GPIO_InitStructure.GPIO_Pin = RTC_INT_PIN;
 // GPIO_Init(RTC_INT_PORT, &GPIO_InitStructure); 
  
  
}

void LED_On(void)
{
   GPIO_WriteBit(LED_GREEN_PORT,LED_GREEN_PIN, Bit_SET);
}

void LED_Off(void)
{
   GPIO_WriteBit(LED_GREEN_PORT,LED_GREEN_PIN, Bit_RESET);
}

void LED_Toggle(void)
{
   GPIO_WriteBit(LED_GREEN_PORT, LED_GREEN_PIN, (BitAction)(1 - GPIO_ReadOutputDataBit(LED_GREEN_PORT, LED_GREEN_PIN)));     //Toggle LED 
}

int IsSDCardDetected(void)
{
  if(!GPIO_ReadInputDataBit(SDCARD_DETECT_PORT, SDCARD_DETECT_PIN)) return 1;
  return 0;
}


bool IsRtcInterrupt(void)
{
  if(!GPIO_ReadInputDataBit(RTC_INT_PORT, RTC_INT_PIN)) return TRUE;
  return FALSE;
}


//HARDWARE
void PowerCntrl(FunctionalState newState)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  //KEEP POWER LOCKED ON ASAP
  /* Enable PWR_ON */
  RCC_APB2PeriphClockCmd(POWER_CNTRL_CLK, ENABLE);  
   /* Configure the PWR_ON pin */
  GPIO_InitStructure.GPIO_Pin = POWER_CNTRL_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(POWER_CNTRL_PORT, &GPIO_InitStructure);
  
  if (newState == ENABLE)
    GPIO_WriteBit(POWER_CNTRL_PORT,POWER_CNTRL_PIN, Bit_SET); //Keep pwr on
  else
    GPIO_WriteBit(POWER_CNTRL_PORT,POWER_CNTRL_PIN, Bit_RESET); //turn off
  
}
  

void GpsOnOffInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(GPS_ON_OFF_CLK, ENABLE); 
  
  GPIO_InitStructure.GPIO_Pin =  GPS_ON_OFF_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPS_ON_OFF_PORT, &GPIO_InitStructure);
  GPIO_WriteBit(GPS_ON_OFF_PORT, GPS_ON_OFF_PIN, Bit_RESET);
}





void PageButtonInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(PAGE_BUTTON_CLK, ENABLE);    
   
     //SETUP INPUTS + PULL UP
  /* Configure Button pin as input floating */
  GPIO_InitStructure.GPIO_Pin = PAGE_BUTTON_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(PAGE_BUTTON_PORT, &GPIO_InitStructure);    
  
  GPIO_InitStructure.GPIO_Pin = NEXT_BUTTON_PIN;
  GPIO_Init(NEXT_BUTTON_PORT, &GPIO_InitStructure);      
  
}



bool IsPageButtonPushed(void)
{
  if (GPIO_ReadInputDataBit(PAGE_BUTTON_PORT, PAGE_BUTTON_PIN) == 0) return TRUE; 
  return FALSE;
}

bool IsNextButtonPushed(void)
{
  if (GPIO_ReadInputDataBit(NEXT_BUTTON_PORT, NEXT_BUTTON_PIN) == 0) return TRUE; 
  return FALSE;
}





void DSP_Reset(ResetState state)
{
  GPIO_InitTypeDef GPIO_InitStructure; 
  
  RCC_APB2PeriphClockCmd(DSP_RESET_CLK , ENABLE);
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = DSP_RESET_PIN;
  
  if (state == HOLD_IN_RESET)
  {
    GPIO_WriteBit(DSP_RESET_PORT, DSP_RESET_PIN, Bit_RESET);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
    GPIO_Init(DSP_RESET_PORT, &GPIO_InitStructure);
    GPIO_WriteBit(DSP_RESET_PORT, DSP_RESET_PIN, Bit_RESET);
  } 
  else
  {
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(DSP_RESET_PORT, &GPIO_InitStructure);    
  }
}







/********************************************EOF*******************************/