
#ifndef _TEMP_SENSOR_H_
#define _TEMP_SENSOR_H_


#include "stm32f10x.h"

//DEFINE PINS

#define TEMP_POWER_PORT          GPIOB
#define TEMP_POWER_CLK           RCC_APB2Periph_GPIOB
#define TEMP_POWER_PIN           GPIO_Pin_9       
   
#define TEMP_SENSOR_PORT         GPIOA
#define TEMP_SENSOR_CLK          RCC_APB2Periph_GPIOA
#define TEMP_SENSOR_PIN          GPIO_Pin_0    

// temp sensor id's - must match with temp_sensor struct array

#define AVAILABLE_TEMP_SENSORS 2 //DS600, TMP235

#define  TEMP_SENSOR_DS600 0
#define  TEMP_SENSOR_TMP235 1





void TempSensorInit(void);
void TempSensorADCState(FunctionalState state); 
uint16_t TempSensorValue();
void TempSensorPower(FunctionalState state);
float * TempConvertValue(uint16_t value);
float * TempConvertValue2(uint16_t value, int sensor_id);
void LogTemp(void);
uint8_t GetMostLikelySensorId(uint16_t adc_value);
uint8_t *GetSensorIdName(int sensor_id);

uint8_t GetTempSensorId(void);
void SetTempSensorId(uint8_t id);
#endif //_TEMP_SENSOR_H_
/********************************************EOF**************************************************/