/***************************************************************************
* File Name          : Rtc.h
* Author             :  DOC Electronics Development Team
* Version            : V1.0.0
* Date               : 06/08/2009
* Description        : This file contains all the functions prototypes for the
*                      RTC driver.
***************************************************************************/

#ifndef RTC_H
#define RTC_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f10x.h"
#include "i2c_bitbang.h"
#include <time.h>



/* Rtc Variable, structure declaration----------------------------------------------*/

typedef struct
{
	int8_t		sec;
	int8_t		min;
	int8_t		hour;	
}rtcTimeTypeDef;


typedef struct
{
	int8_t		dayOfWeek;
	int8_t		date;
	int8_t		month;
	int8_t		year;
}rtcDateTypeDef;


//#define HR_CHK(HR)  			if(HR>23){ HR=0;} else if(HR<0) {HR=23;}	
//#define MIN_SEC_CHK(MIN) 	if(MIN>59){MIN=0;} else if (MIN<0) {MIN=59;}
//#define DATE_CHK(DATE,MON) 		if(DATE>days[MON]){DATE=1;} else if (DATE<1) {DATE=days[MON];}
//#define MONTH_CHK(MON)		if(MON>12) {MON=1;} else if (MON<1) {MON=12;}
//#define YEAR_CHK(YR)			if(YR>99) {YR=0;} else if (YR<0){YR=99;}


/*Function Declaration ----------------------------------------------------------*/
bool HasRtcPowerFailed();
void RtcSetTime( rtcTimeTypeDef *rtcTime );
void RtcSetAlarm( rtcTimeTypeDef *rtcTime );
void RtcDisableAlarm(void);
void RtcSetDate( rtcDateTypeDef *rtcDate );
int8_t RtcGetAlamMatchStatus();
void RtcClearAlarmFlag( void);
uint8_t RtcGetAlarmFlag(void);

rtcTimeTypeDef *RtcGetAlarm(void);
rtcTimeTypeDef *RtcGetTime(void);
rtcDateTypeDef *RtcGetDate(void);
void RtcStart();
uint8_t	RtcGetUser1Reg(void);
void RtcSetUser1Reg(uint8_t Byte);
uint8_t	RtcGetUser2Reg(void);
void RtcSetUser2Reg(uint8_t Byte);

uint8_t Rtc_bin2bcd(uint8_t bin_value);
uint8_t Rtc_bcd2bin(uint8_t bcd_value);

void RtcRegWrite(uint8_t RegName, uint16_t RegValue);
uint8_t RtcRegRead(uint8_t RegName);

bool IsOnAlarmTime(void);

//void ModifyTimeAndDateWithAddedSeconds(rtcTimeTypeDef *updatedTime ,rtcDateTypeDef *updatedDate, int addedSeconds);


rtcDateTypeDef *RtcGetBufferedDate(void);
rtcTimeTypeDef *RtcGetBufferedTime(void);

//uint8_t	RTC_GetUser1Reg(void);
//void RTC_SetUser1Reg(uint8_t Byte);
//
//unsigned int date_time_to_epoch(rtcTimeTypeDef *pTime, rtcDateTypeDef * pDate);
//void epoch_to_date_time(rtcTimeTypeDef *pTime, rtcDateTypeDef * pDate,unsigned int epoch);

void Internal_RTC_Configuration(FunctionalState NewState, int interruptSeconds);
time_t GetRtcTimeInTicks();
void SetRtcTimeInTicks(time_t ticks);



#endif /* __RTC_H */


