



#include "hardware.h"
#include "pushbutton.h"







static DspEvent_TypeDef dspEventInterruptState = DSP_NO_EVENT;






void LED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure; 
  
  RCC_APB2PeriphClockCmd(LED_DISPLAY_CLK | LED_DEBUG_CLK, ENABLE);    
  
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = LED_DISPLAY_PIN;
  GPIO_Init(LED_DISPLAY_PORT, &GPIO_InitStructure);
  LED_Display( LED_OFF );
  

  
  
  
  //GPIO_InitStructure.GPIO_Pin = LED_DEBUG_PIN;
 // GPIO_Init(LED_DEBUG_PORT, &GPIO_InitStructure); 
 // LED_Debug( LED_OFF );  
}
  
void SD_Detect_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure; 
  
  RCC_APB2PeriphClockCmd(SD_DETECT_CLK, ENABLE);    
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = SD_DETECT_PIN;
  GPIO_Init(SD_DETECT_PORT, &GPIO_InitStructure);
}



bool IsSDCardDetected(void)
{
 if (GPIO_ReadInputDataBit(SD_DETECT_PORT, SD_DETECT_PIN) != 0) return FALSE;
 return TRUE;
}




/***********************************************************
* LED Display Nth Count 
*
* turn on led every nth count
* - is used in recording mode to flash led every so often 
*/
static int displayNthCount = 1;


void SetLedDisplayNthCount(int nthCount)
{
  displayNthCount = nthCount;
}


// Turns on led every n'th time this function is called
void LED_DisplayOnCount(void)
{
  static int count = 0;
  if (++count >= displayNthCount)
  {
    count = 0;
    GPIO_WriteBit(LED_DISPLAY_PORT,LED_DISPLAY_PIN, Bit_SET);
  }
}






// LED on display board
void LED_Display( LED_State_TypeDef ledState)
{
  if (ledState == LED_OFF)
  {
       GPIO_WriteBit(LED_DISPLAY_PORT,LED_DISPLAY_PIN, Bit_RESET);
  }
  else if (ledState == LED_ON)
  {
    GPIO_WriteBit(LED_DISPLAY_PORT,LED_DISPLAY_PIN, Bit_SET);
  }
  else if (ledState == LED_TOGGLE)
  {
    GPIO_WriteBit(LED_DISPLAY_PORT, LED_DISPLAY_PIN, (BitAction)(1 - GPIO_ReadOutputDataBit(LED_DISPLAY_PORT, LED_DISPLAY_PIN))); 
  }
}

//LED on control board used for debug
void LED_Debug( LED_State_TypeDef ledState)
{
#ifdef DEBUG  
  if (ledState == LED_OFF)
  {
       GPIO_WriteBit(LED_DEBUG_PORT,LED_DEBUG_PIN, Bit_RESET);
  }
  else if (ledState == LED_ON)
  {
    GPIO_WriteBit(LED_DEBUG_PORT,LED_DEBUG_PIN, Bit_SET);
  }
  else if (ledState == LED_TOGGLE)
  {
    GPIO_WriteBit(LED_DEBUG_PORT, LED_DEBUG_PIN, (BitAction)(1 - GPIO_ReadOutputDataBit(LED_DEBUG_PORT, LED_DEBUG_PIN))); 
  }
#endif  
}


void LED_Flash(void)
{
  LED_Display(LED_ON);  //F_Init LED indication
  delay_ms(40);  
  LED_Display(LED_OFF);  //F_Init LED indication
  delay_ms(40);  
}



void DSP_IO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure; 
    NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_APB2PeriphClockCmd(DSP_RESERVED_CLK | DSP_COMMAND_READY_CLK | DSP_COMMAND_MODE_CLK | DSP_EVENT_CLK, ENABLE);    
  

  //OUTPUT
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
  GPIO_InitStructure.GPIO_Pin = DSP_COMMAND_READY_PIN;
  GPIO_Init(DSP_COMMAND_READY_PORT, &GPIO_InitStructure);
  GPIO_WriteBit(DSP_COMMAND_READY_PORT,DSP_COMMAND_READY_PIN, Bit_RESET);
 
  GPIO_InitStructure.GPIO_Pin = DSP_RESERVED_PIN;
  GPIO_Init(DSP_RESERVED_PORT, &GPIO_InitStructure);
  GPIO_WriteBit(DSP_RESERVED_PORT,DSP_RESERVED_PIN, Bit_RESET);
  
  GPIO_InitStructure.GPIO_Pin = DSP_COMMAND_MODE_PIN;
  GPIO_Init(DSP_COMMAND_MODE_PORT, &GPIO_InitStructure);
  GPIO_WriteBit(DSP_COMMAND_MODE_PORT,DSP_COMMAND_MODE_PIN, Bit_RESET);
  
  
  //INPUT
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
  
  GPIO_InitStructure.GPIO_Pin = DSP_EVENT_PIN;
  GPIO_Init(DSP_EVENT_PORT, &GPIO_InitStructure);
  
  
  GPIO_EXTILineConfig(DSP_EVENT_PORT_SOURCE, DSP_EVENT_PIN_SOURCE);
  
  NVIC_InitStructure.NVIC_IRQChannel = DSP_EVENT_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
  
}

void DspEventInterrupt(FunctionalState state) // enable interrupt for dsp events both falling and rising edge triggered
{

 EXTI_InitTypeDef EXTI_InitStructure; 
  
  if (state == DISABLE)
  {
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Line = DSP_EVENT_EXTI_LINE;          
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);    
  }
  /* Enable the EXTI line 0,1,2 and 3 on falling edge */
  else
  {
    /* Clear the the EXTI line 0,1,2 and 3 interrupt pending bit */
    EXTI_ClearITPendingBit(DSP_EVENT_EXTI_LINE);     

    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Line = DSP_EVENT_EXTI_LINE;         
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
 
  }
  //clear flag
  dspEventInterruptState = DSP_NO_EVENT;
  
}


void DSP_SendDataReady()
{
  GPIO_WriteBit(DSP_COMMAND_READY_PORT,DSP_COMMAND_READY_PIN, Bit_SET);
  delay_us(100);
  GPIO_WriteBit(DSP_COMMAND_READY_PORT,DSP_COMMAND_READY_PIN, Bit_RESET);
}



void Dsp_RunCommandMode(FunctionalState NewState)
{
  if (NewState == DISABLE)
    GPIO_WriteBit(DSP_COMMAND_MODE_PORT,DSP_COMMAND_MODE_PIN, Bit_RESET);
  else
    GPIO_WriteBit(DSP_COMMAND_MODE_PORT,DSP_COMMAND_MODE_PIN, Bit_SET);
}

void SetDspEventInterruptState(DspEvent_TypeDef dsp_event)
{
  dspEventInterruptState = dsp_event;
}



// returns the dsp event if occured, returns "event end" if both start and end are present 
DspEvent_TypeDef Dsp_IsEventInterrupt(void)
{
  DspEvent_TypeDef event;
  
  event = dspEventInterruptState;
  if ( dspEventInterruptState != DSP_NO_EVENT)
    dspEventInterruptState = DSP_NO_EVENT;
  
  return event;
}




bool DSP_IsEvent(DetectType detectType)
{
  bool state;
  static uint8_t lastState =  0; 
  uint8_t nowState;
  
  state = FALSE;
  
  if (detectType == LEVEL)
  {
    if (GPIO_ReadInputDataBit(DSP_EVENT_PORT, DSP_EVENT_PIN) != 0) state = TRUE;
  }
  else
  {
    nowState = GPIO_ReadInputDataBit(DSP_EVENT_PORT, DSP_EVENT_PIN);
    if ((nowState == 0 ) && (lastState != 0)) state = TRUE;
    lastState = nowState;
  }
  return state;
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

//HARDWARE
void PowerCntrl(FunctionalState newState)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  //KEEP POWER LOCKED ON ASAP
  /* Enable PWR_ON */
  RCC_APB2PeriphClockCmd(POWER_CNTRL_CLK | POK_CLK, ENABLE);  
   /* Configure the PWR_ON pin */
  GPIO_InitStructure.GPIO_Pin = POWER_CNTRL_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(POWER_CNTRL_PORT, &GPIO_InitStructure);
  
  if (newState == ENABLE)
    GPIO_WriteBit(POWER_CNTRL_PORT,POWER_CNTRL_PIN, Bit_SET); //Keep pwr on
  else
    GPIO_WriteBit(POWER_CNTRL_PORT,POWER_CNTRL_PIN, Bit_RESET); //turn off
  
  
  //setup pok
  GPIO_InitStructure.GPIO_Pin = POK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(POK_PORT, &GPIO_InitStructure);
  
}



#define POWER_FAIL_COUNT 10
bool IsPowerOk(void)
{ 
  static int power_fail_count = POWER_FAIL_COUNT;
  bool ret = TRUE; //assume power is good 
  uint8_t input;  
  
  input = GPIO_ReadInputDataBit(POK_PORT,POK_PIN);
  
  if (input == 0)
  {
    if (power_fail_count > 0)  
      power_fail_count--;
    else
      ret = FALSE;
  }
  else
  {
    power_fail_count = POWER_FAIL_COUNT;
  }
  
  return ret;
}







/***************************************************
* Power On/Off peripherals
*
* - attempts to minimise power consumption by powering off periherals
* - also sets unused pins to either a set level or ADC input(best result)
* - 
*
****************************************************/
void PowerOffPeripherals(void)
{
  
  
  
   
}






void PowerOnPeripherals(void)
{
  
  
  
}


void ReducePowerOnce(void)
{  
#ifndef DEBUG 
  GPIO_InitTypeDef GPIO_InitStructure;

  
  
  
  
  // Disable the Serial Wire Jtag Debug Port SWJ-DP 
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);  
  
  //set JTAG pins to Outputs
  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15 ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
  GPIO_Init(GPIOB, &GPIO_InitStructure);   
  
  ADC_Cmd(ADC3, DISABLE);  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3 , DISABLE);
    
    
#endif    
    
  

}








/***************************************************
* WatchDog Initialise
****************************************************/
#define WATCHDOG_LONG_TIMEOUT 4095
#define WATCHDOG_SHORT_TIMEOUT 250
void WatchDogInit(WD_Timeout wdt_timeout)
{
#ifdef WATCHDOG
  
  static uint8_t setupflag = 0;

  // Enable write access to IWDG_PR and IWDG_RLR registers 
      IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);    
  
  if(!setupflag)
  {  
    // IWDG counter clock: 40KHz(LSI) / 32 = 1.25 KHz 
    IWDG_SetPrescaler(IWDG_Prescaler_256);
    
    // Set counter reload value to 349 
    if(wdt_timeout ==LONG) IWDG_SetReload(WATCHDOG_LONG_TIMEOUT);  //~12-13 secs
    else  IWDG_SetReload(WATCHDOG_SHORT_TIMEOUT);   //~ .8 - 1 sec           
    // Reload IWDG counter
    IWDG_ReloadCounter();
    // Enable IWDG (the LSI oscillator will be enabled by hardware)
    IWDG_Enable(); 
    setupflag=1;
  }
  else  //just change the counter timeout value
  {
    while((IWDG->SR & 0x00000002) != 0);  
    if(wdt_timeout ==LONG) IWDG_SetReload(WATCHDOG_LONG_TIMEOUT);       //~12-13 secs
    else  IWDG_SetReload(WATCHDOG_SHORT_TIMEOUT);     // ~0.9-1.0 secs
    // Reload IWDG counter
    IWDG_ReloadCounter();       
  }
  
#endif  
  
}

void ResetWatchdog(void)
{
#ifdef WATCHDOG
    IWDG_ReloadCounter();
#endif
}

#define SCB_SysCtrl              ((uint32_t)0xE000ED10)/* Cortex System Control register address */
#define SysCtrl_SLEEPDEEP_Set    ((uint32_t)0x00000004)/* SLEEPDEEP bit mask */
void WaitForInterrupt(void)
{

  *(__IO uint32_t *) SCB_SysCtrl &= ~SysCtrl_SLEEPDEEP_Set; // make sure not in 
  __WFI();    // LOOP here waiting for data from dsp to come in

}



/*****************************************EOF*****************************************/