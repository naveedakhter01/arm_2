/******************************************************
*
*  sysclk.c
*
* 
*******************************************************/

#include "sysclk.h"



void SysTickInterrupt(FunctionalState state)
{
 if (state == ENABLE)
 {
    //ENABLE THE SYSTICK TIMER
  if (SysTick_Config(SystemFrequency / 1000))   //for 1usec overflow = 8MHz/1Mhz = 8, 8 x 0.125nS = 1uS
  { 
    /* Capture error */ 
    while (1);
  } 
   
   
 }
 else //DISABLE
   SysTick->CTRL = (1 << SYSTICK_CLKSOURCE) | (0<<SYSTICK_ENABLE) | (0<<SYSTICK_TICKINT);   


}
  
  
  
void SetClkHSE(void)
{
    /* Select HSE as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_HSE;    

    /* Wait till HSE is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x04)
    {
    }
  

}


void SetClkPLL(void)
{
    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;   
  
    /* Select PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;    

    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
    {
    }
}



void SetSysClk2MHz(void)
{
        //change PCLK
  RCC_PCLK1Config(RCC_HCLK_Div1);
  //set 8Mhz Clk Rate
  RCC_HCLKConfig(RCC_SYSCLK_Div4);   //div 4
  SetClkHSE();       // set sysclk to 2MHz     , HCLK = 4MHz  , PCLK1 = 4MHz , PCLK2 = 4MHz 
  FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
  FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_0;    
// AddTestLowSession();
}



void SetSysClk8MHz(void)
{
    RCC_PCLK1Config(RCC_HCLK_Div1);
    //set 8Mhz Clk Rate
    RCC_HCLKConfig(RCC_SYSCLK_Div1);   //div 4
    SetClkHSE();       // set sysclk to 2MHz     , HCLK = 4MHz  , PCLK1 = 4MHz , PCLK2 = 4MHz 
    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_0;    
}





/**
  * @brief  Configures system clock after wake-up from STOP: enable HSE, PLL
  *   and select PLL as system clock source.
  * @param  None
  * @retval None
  */
void SYSCLKConfig_STOP(void)
{
  ErrorStatus HSEStartUpStatus;
  
  
  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    SetSysClk8MHz();
  }
  
  
}






/****************************************EOF****************************************/










