/****************************************************************************
* BATTERY.H
*
******************************************************************************/


#ifndef BATTERY_H
#define BATTERY_H


#include "stm32f10x.h"
#include "stm32_eval.h"


typedef enum {
 BATTERY_OK,
 BATTERY_FAIL
} BatteryState_TypeDef;
  
void MarkBatteryState(BatteryState_TypeDef state);
bool IsBatteryOk(void);


void InitBatteryMeasure(void);
bool IsBatteryVoltageBelowThreshold(void);

float GetBatteryMeasurementFloat(void);


uint8_t CompBatCondition(uint16_t Val);

bool MonitorBattery(void);
void BatteryMeasurementConfig(FunctionalState newState);



#endif

