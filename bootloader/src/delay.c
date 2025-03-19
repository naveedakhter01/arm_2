

#include "delay.h"



static uint16_t Ticks;      // tick count = 1ms * 65535 = must use if counting no greater than 65.535 secs
static uint16_t FiveSecCounts;


static __IO uint32_t TimingDelay;

void delay_ms(uint32_t nTime)
{ 

    TimingDelay = nTime;
    while(TimingDelay != 0);    
}

//only when systick set up to one sec interrupts
void delay_sec(uint32_t nTime)
{ 
    TimingDelay = nTime;
    while(TimingDelay != 0)
    {
         
    }
}





/**
  * @brief  Inserts a delay time.
  * @param nTime: specifies the delay time length, in milliseconds.
  * @retval : None
  */
void delay_us(uint32_t nTime)
{ 
  int i,j;

  for(i=0; i<nTime;i++)
  {
    j=0;
    while(j<500) j++;
    
  }

    //TimingDelay = nTime;
    //while(TimingDelay != 0);

}
  
   
   
   
          

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
  
  Ticks++;

}


uint16_t GetTicks(void)
{
   return(Ticks); 
}





void FiveSecDelay_Decrement(void)
{  
  FiveSecCounts++;
}


uint16_t Get5SecTicks(void)
{
   return(FiveSecCounts); 
}










//


void DelayTimer_init(sCLOCK Clock)
{
  TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
  TIM_OCInitTypeDef         TIM_OCInitStructure; 
    NVIC_InitTypeDef          NVIC_InitStructure;
     
  uint16_t TimebasePrescaler = 24999; //25
  uint16_t TIMPrescaler;
  
  if(Clock == FAST_CLK) TIMPrescaler = 7199;  //FAST clock
     //  else    TIMPrescaler = 609;   //SLOW clock @8MHz
       else    TIMPrescaler = 99;   //@ 500kHz
       
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);   
       
       
  //reset periph
   RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM5, ENABLE);
   RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM5, DISABLE);

		  /* Time base configuration */
		TIM_TimeBaseStructure.TIM_Period = TimebasePrescaler;
		TIM_TimeBaseStructure.TIM_Prescaler = 0;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	
		/* Prescaler configuration */
		TIM_PrescalerConfig(TIM5, TIMPrescaler, TIM_PSCReloadMode_Immediate);
	
		/* Output Compare Timing Mode configuration: Channel1 */
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = TimebasePrescaler;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OC1Init(TIM5, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Disable);
	
	/* Enable the TIM4 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 8;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);   
        
		/* TIM IT enable */
		TIM_ITConfig(TIM5, TIM_IT_CC1, ENABLE); 
	        /* TIM5 enable counter */
		TIM_Cmd(TIM5, ENABLE); 
        
}


void DelayTimer_Prescaler(sCLOCK Clock)
{
uint16_t TIMPrescaler;
    
  if(Clock == FAST_CLK) TIMPrescaler = 7199;  //FAST clock
     //  else    TIMPrescaler = 609;   //SLOW clock @8MHz
       else    TIMPrescaler = 99; 
       
  /* Prescaler configuration */
    TIM_PrescalerConfig(TIM5, TIMPrescaler, TIM_PSCReloadMode_Immediate);
}
void DisableDelayTimer()
{
		/* TIM IT disable */
		TIM_ITConfig(TIM5, TIM_IT_CC1, DISABLE); 
	/* TIM4 disable counter */
		TIM_Cmd(TIM5, DISABLE); 
  
}