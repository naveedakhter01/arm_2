/***************************************************************************
* File Name          : Menu.h
* Author             :  Raje.K DOC Electronics Development Team
* Version            : V1.0.0
* Date               : 06/08/2009
* Description        : This file contains all the functions prototypes for the
*                      Menu.
***************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MENU_H
#define __MENU_H

#include "stm32f10x.h"
/*
typedef enum 
{
	TIME = 0,
		RTCDATE,
		START_TIME,
		STOP_TIME,
		SURVEY,
		GPS,
		USB,
                WAV

}eMenuType;
*/
typedef enum 
{
                TIME = 0,
		RTCDATE,
		START_TIME,
		DUR_TIME,
                SURVEY,
                RECORD,
	        TEST,
                CODE,
                USB,
                BAT,
                SAMP,
                DMA,
                SYNC,
                PWROFF,
                BOOT

}eMenuType;



typedef enum
{
		HOUR =1,
		MINUTES,
		SECONDS,
		NONE
}eTime;		

typedef enum
{
		DATE = 1,
		MONTH,
		YEAR,
}eDate;		

typedef enum
{
		_PLUS =  0x01,
		_MINUS = 0x02,
	        _NEXT =  0x04,
                UPDATE = 0x08,
                INIT =   0x10,
                CLOSE =  0x20
}bACTION;	


typedef void (* pMenuFunc)(bACTION);

struct sMenuItem
{
	uint8_t	*PageName;
	pMenuFunc pfMenuFunc;
	int	MenuType;	

};

struct sRtcTime
{
	uint8_t	Hour;
	uint8_t	Min;
	uint8_t	Sec;
};

struct sRtcDate
{
	uint8_t day;
	uint8_t date;
	uint8_t month;
	uint8_t year;
};	



/* Function Declaration*/
void Menu_Init(void);
void SetMenuCursorPos( int8_t curpos);
int8_t GetMenuCursorPos (void);
void CallUpdate();
int16_t RefreshMenu();
void TimeDisplay(bACTION);
void DateDisplay(bACTION);
void StartTimeDisplay(bACTION);
void DurationDisplay(bACTION);
void SurveyNameDisplay(void);
void GPSLocationDisplay(void);
void WaveFile_Play(void);
void Set_SELStatus(void);

uint8_t StartBlinkField( uint8_t  iField);

void Record(bACTION);
void Boot(bACTION event);
void USB_Start(bACTION event);
void Test(bACTION);
void MainPowerOff(bACTION event);
void SurveyDisplay(bACTION event);
void Battery(bACTION event);
void CodeTest(bACTION event);
void Syncing(bACTION event);
void Sampling(bACTION event);

void DMA_Start(bACTION event);

#endif /* __MENU_H */

