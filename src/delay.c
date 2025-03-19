

#include "delay.h"
#include "pushbutton.h"



static uint16_t Ticks;      // tick count = 1ms * 65535 = must use if counting no greater than 65.535 secs



static __IO uint32_t TimingDelay;

void delay_ms(uint32_t nTime)
{ 

  //for (i=0;i<1000;i++)
  //{ 
    TimingDelay = nTime;
    while(TimingDelay != 0);    
  //}
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
    while(j<100) j++;
    
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



// delays turn on of dsp is roughly 3s
bool Delay_Dsp_Start(void)
{
  bool ret = TRUE;
  for (int i=0;i<MAGIC(350);i++)
  {
    if (IsPagebuttonPushed())
    {
      ret = FALSE;
      break;
    }
    delay_ms(10);
  }
  
  return ret;
}










