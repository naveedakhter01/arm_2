/******************************************************************************
* functions relating to battery checking
*
******************************************************************************/

#include "rtc.h"
#include "battery.h"


#define BAT_MEASURE_PORT              GPIOA
#define BAT_MEASURE_CLK               RCC_APB2Periph_GPIOA  
#define BAT_MEASURE_PIN               GPIO_Pin_2

#define BAT_ON_PORT              GPIOB
#define BAT_ON_CLK               RCC_APB2Periph_GPIOB  
#define BAT_ON_PIN               GPIO_Pin_9

#define BATTERY_ADC ADC1
#define BATTERY_ADC_CHANNEL ADC_Channel_2

#define MAX_ADC_VALUE (4096) // ADC 12 bit
#define ADC_FULLSCALE_VOLTAGE 3.3f
#define VOLTAGE_DIVIDER 0.25f //adc input voltage is a quarter of battery voltage

#define TRIGGER_LEVEL_BATTERY 3.6f  // battery voltage threshold level

#define THRESHOLD_VALUE (uint16_t)(((TRIGGER_LEVEL_BATTERY*VOLTAGE_DIVIDER)/ADC_FULLSCALE_VOLTAGE)* MAX_ADC_VALUE)


void BatteryMeasurementConfig(FunctionalState newState);

//Turns on the fet for the voltage divider, and sets up adc

void InitBatteryMeasure(void)
{
  ADC_InitTypeDef  ADC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  //SETUP ADC for BATTERY MEASURE 
  RCC_APB2PeriphClockCmd(BAT_MEASURE_CLK | BAT_ON_CLK , ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  //SETUP ANALOG INPUTS
  GPIO_InitStructure.GPIO_Pin = BAT_MEASURE_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(BAT_MEASURE_PORT, &GPIO_InitStructure);
  

  //SETUP ANALOG INPUTS
  GPIO_InitStructure.GPIO_Pin = BAT_ON_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(BAT_ON_PORT, &GPIO_InitStructure);
  GPIO_WriteBit(BAT_ON_PORT,BAT_ON_PIN, Bit_RESET);
  
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(BATTERY_ADC, &ADC_InitStructure);

  // ADC1 regular channel14 configuration 
  ADC_RegularChannelConfig(BATTERY_ADC, BATTERY_ADC_CHANNEL , 1, ADC_SampleTime_28Cycles5 );


}


void BatteryMeasurementConfig(FunctionalState newState)
{
 
  if (newState != DISABLE)
  {
        //voltage divider on
    GPIO_WriteBit(BAT_ON_PORT,BAT_ON_PIN, Bit_SET);  
    delay_us(10);
    // Enable ADC1
    ADC_Cmd(BATTERY_ADC, ENABLE);
    // Enable ADC1 reset calibaration register   
    ADC_ResetCalibration(BATTERY_ADC);
    
    // Check the end of ADC1 reset calibration register
    while(ADC_GetResetCalibrationStatus(BATTERY_ADC));
    
    // Start ADC1 calibaration
    ADC_StartCalibration(BATTERY_ADC);
    
    // Check the end of ADC1 calibration
    while(ADC_GetCalibrationStatus(BATTERY_ADC)); 

    
    ADC_SoftwareStartConvCmd(BATTERY_ADC, ENABLE);
    while(ADC_GetSoftwareStartConvStatus(BATTERY_ADC));
    ADC_GetConversionValue(BATTERY_ADC);     
    
    
  }
  else
  {
    // Disable ADC1
    ADC_Cmd(ADC1, DISABLE);
    //voltage divider off
    GPIO_WriteBit(BAT_ON_PORT,BAT_ON_PIN, Bit_RESET);  
  }
  
}


uint16_t GetBatteryMeasurement(void)
{
  //SETUP ADC for BATTERY MEASURE  
  //BATTERY STATUS
  /* Start ADC1 Software Conversion */ 

  
  ADC_SoftwareStartConvCmd(BATTERY_ADC, ENABLE);
  while(ADC_GetSoftwareStartConvStatus(BATTERY_ADC));
  /* get the value */
  return((uint16_t)  ADC_GetConversionValue(BATTERY_ADC)); 
  
}


float GetBatteryMeasurementFloat(void)
{
  uint16_t val;
  float    fVal;
  //SETUP ADC for BATTERY MEASURE  
  //BATTERY STATUS
  /* Start ADC1 Software Conversion */ 
  
  ADC_SoftwareStartConvCmd(BATTERY_ADC, ENABLE);
  while(ADC_GetSoftwareStartConvStatus(BATTERY_ADC));
  /* get the value */
  
  val = ADC_GetConversionValue(BATTERY_ADC); 

  fVal = val * ADC_FULLSCALE_VOLTAGE;
  fVal /=MAX_ADC_VALUE;
  fVal /=VOLTAGE_DIVIDER;

  return fVal; 
}






bool IsBatteryVoltageBelowThreshold(void)
{ 
  uint16_t value;
  BatteryMeasurementConfig(ENABLE);

  //
  value = GetBatteryMeasurement();
  
  BatteryMeasurementConfig(DISABLE); 
  
  return value >= THRESHOLD_VALUE ? FALSE: TRUE;
  
}


#define BATTERY_STATE_READS 4 // number of reads before confirming battery is low
bool MonitorBattery(void)
{
  static int batteryReads = BATTERY_STATE_READS;
  bool ret = TRUE;
  
  if (batteryReads > 0)
  {
    if (IsBatteryVoltageBelowThreshold()) 
      batteryReads--;
    else
      batteryReads = BATTERY_STATE_READS; //reset reads
  }
  else
  {
    ret = FALSE;
    batteryReads = BATTERY_STATE_READS; //reset reads
  }
  
  return ret;                                  
}



  
void MarkBatteryState(BatteryState_TypeDef state)
{
  RtcSetUser2Reg(state);
}


bool IsBatteryOk(void)
{
  return (bool) (RtcGetUser2Reg() == BATTERY_OK);
}




