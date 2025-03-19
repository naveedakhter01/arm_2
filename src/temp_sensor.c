


#include "temp_sensor.h"
#include "rtc.h"
#include "log.h"
#include "stdio.h"
#include <math.h>

#define TEMP_SENSOR_PART "MAXIM DS600" 


#define TEMP_ADC          ADC2
#define TEMP_ADC_CHANNEL  ADC_Channel_0


//CALIBRATION
#define ADC_BIT_VALUE (3250.0/4096.0)  
//#define ADC_BIT_VALUE 0.8056640625   //how many millivolts an adc value of one is (3.3v/4096) * 1000
#define ZERO_VALUE_MILIVOLTS 509.0   
#define ONE_DEGREE_MILIVOLTS 6.45  



#define WORKSHOP_AVERAGE_TEMP 22.0 // used to auto detect which temp sensor may be present on board



typedef struct
{
  float adc_bit;
  float zero_value_millivolts;
  float millivolts_per_degree;
  uint8_t name[8];
} TempSensorTypedef;




TempSensorTypedef temp_sensors[] = { 
/*********************TEMP SENSORS************************************/  

  //"MAXIM DS600"  
  { .adc_bit = ADC_BIT_VALUE,  //how many millivolts an adc value of one is assuming port output not quite 3.3v. 3280mv / 4096 (adc bits)
    .zero_value_millivolts = 509.0,  //the milivolt output of sensor at 0 degrees
    .millivolts_per_degree = 6.45, //the milivolt change per one degree
    .name = "DS600"  
  },  

  //"TMP235"  
  { .adc_bit = ADC_BIT_VALUE ,//how many millivolts an adc value of one is assuming port output not quite 3.3v. 3280mv / 4096 (adc bits)
    .zero_value_millivolts = 500.0,  //the milivolt output of sensor at 0 degrees
    .millivolts_per_degree = 10.0, //the milivolt change per one degree
    .name = "TMS235"  
  }
};




void TempSensorInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure; 
  ADC_InitTypeDef ADC_InitStructure;
  
  //init power enable  and signal pins  
   RCC_APB2PeriphClockCmd(TEMP_POWER_CLK | TEMP_SENSOR_CLK, ENABLE);  
  
  
  //first setup power enable pin
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = TEMP_POWER_PIN;
  GPIO_Init(TEMP_POWER_PORT, &GPIO_InitStructure);  
  GPIO_WriteBit(TEMP_POWER_PORT,TEMP_POWER_PIN, Bit_RESET);
    
  //now setup pin as adc to read level from temp sensor
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 
  GPIO_InitStructure.GPIO_Pin = TEMP_SENSOR_PIN;
  GPIO_Init(TEMP_SENSOR_PORT, &GPIO_InitStructure);  
  
  
  //init ADC 
  /* Enable PWR_ON */
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);      
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2 , ENABLE);      
  
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(TEMP_ADC, &ADC_InitStructure);

  /* ADC1 regular channel0 configuration */ 
  ADC_RegularChannelConfig(TEMP_ADC, TEMP_ADC_CHANNEL, 1, ADC_SampleTime_28Cycles5 ); 
  
  
}

void TempSensorPower(FunctionalState state)
{
  
  state == ENABLE ?   
    GPIO_WriteBit(TEMP_POWER_PORT,TEMP_POWER_PIN, Bit_SET) : GPIO_WriteBit(TEMP_POWER_PORT,TEMP_POWER_PIN, Bit_RESET);
}

void TempSensorADCState(FunctionalState state)
{
  static bool isEnabled = FALSE;
  if((state == ENABLE) && (!isEnabled))
  {
    /* Enable ADC1 */
    ADC_Cmd(TEMP_ADC, ENABLE);
  
    /* Enable ADC1 reset calibaration register */   
    ADC_ResetCalibration(TEMP_ADC);
    /* Check the end of ADC1 reset calibration register */
    while(ADC_GetResetCalibrationStatus(TEMP_ADC));
  
    /* Start ADC1 calibaration */
    ADC_StartCalibration(TEMP_ADC);
    /* Check the end of ADC1 calibration */
    while(ADC_GetCalibrationStatus(TEMP_ADC));
    
    
    isEnabled = TRUE;
  }
  else
  {
     ADC_Cmd(TEMP_ADC, DISABLE);
     isEnabled = FALSE;
  }
}




uint16_t TempSensorValue()
{  
  //BATTERY STATUS
  uint32_t tempVal; 
  
  //get the temp val
  ADC_SoftwareStartConvCmd(TEMP_ADC, ENABLE);
  while(ADC_GetSoftwareStartConvStatus(TEMP_ADC));
  /* get the value */
  ADC_GetConversionValue(TEMP_ADC);
  tempVal = ADC_GetConversionValue(TEMP_ADC);
   
  return((uint16_t) tempVal );
}


float * TempConvertValue(uint16_t value)
{
  double g;
  static float f;
  
  g = value * ADC_BIT_VALUE; // convert the adc reading to milivolts
  g -= ZERO_VALUE_MILIVOLTS; // subtract the zero value 
  g /=  ONE_DEGREE_MILIVOLTS; //convert to actual temp in degrees
  f = (float)g;
  return  &f; // get the temp 
}


float * TempConvertValue2(uint16_t value, int sensor_id)
{
  double g;
  static float f;
  
  
  g = value * temp_sensors[sensor_id].adc_bit; // convert the adc reading to milivolts
  g -= temp_sensors[sensor_id].zero_value_millivolts; // subtract the zero value 
  g /=  temp_sensors[sensor_id].millivolts_per_degree; //convert to actual temp in degrees
  f = (float)g;
  return  &f; // get the temp 
}


//Call this to get temperature
// - init adc
// - turns on power to temp and gets reading
// - converts adc reading to temp float value
// - turns off power to sensor
float GetTemperature()
{
  float *ret;
  uint8_t sensor_id;
  
  TempSensorADCState(ENABLE);
  TempSensorPower(ENABLE);
  delay_us(300);
  
  //get temp sensor id 
  sensor_id = EEPROM_ReadTempSensorId();
  if (sensor_id >= AVAILABLE_TEMP_SENSORS) sensor_id = 0; //default ds600
  
  TempSensorValue();
 
  ret = TempConvertValue2(TempSensorValue(), sensor_id);
    
  TempSensorPower(DISABLE);
  TempSensorADCState(DISABLE);
  return *ret;
}





// rtc time must have been refreshed 
// logs a temp every hour
void LogTemp(void)
{
  uint8_t text_string[64];
  rtcTimeTypeDef *time;
  rtcDateTypeDef *rtc_date;
  
  //time = RtcGetTime();
  time = RtcGetBufferedTime(); // if time was recently aquired from rtc
  rtc_date = RtcGetBufferedDate();
 
  sprintf(text_string,"%02d/%02d/%02d %02d:%02d Temp= %.1f\r\n",rtc_date->date, rtc_date->month,rtc_date->year, time->hour, time->min, GetTemperature());
  
  if (WriteLogFileToCard(text_string, FALSE) == 0, FALSE)
  { 
    //success in writing log file
  }
  else
  { //Failed to write 

  }  
}




// without knowing which temp sensor is on board, applys the adc value to each of the sensors algorithms and 
// determines which on it is by the closest to the workshop average temperature
uint8_t GetMostLikelySensorId(uint16_t adc_value)
{
  float nearest_temp = 400.0;
  float candidate_temp;
  uint8_t likely_id = 0;
  
  // get temps assuming each a different sensor
  for (int i=0;i< AVAILABLE_TEMP_SENSORS;i++)
  {
     candidate_temp = *TempConvertValue2(adc_value, i);
     candidate_temp = candidate_temp - WORKSHOP_AVERAGE_TEMP;
     candidate_temp = fabs(candidate_temp);
     
     if (candidate_temp < nearest_temp)  //subtract off the workshop average temp to determine the lowest
     {
       nearest_temp = candidate_temp;
       likely_id = i;
     }
  }

  return likely_id;
}


uint8_t *GetSensorIdName(int sensor_id)
{
 return temp_sensors[sensor_id].name; 
  
}


uint8_t GetTempSensorId(void)
{
  uint8_t id; 
  id = EEPROM_ReadTempSensorId();
  if (id >= AVAILABLE_TEMP_SENSORS)
  {
    id = 0; //default ds600
    EEPROM_WriteTempSensorId(id);
  }
 return id;
  
}

void SetTempSensorId(uint8_t id)
{
  id = id%AVAILABLE_TEMP_SENSORS;
  EEPROM_WriteTempSensorId(id);
}