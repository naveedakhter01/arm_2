/***************************************************************************
* File Name          : Menu.h
* Author             :  DOC Electronics Development Team
* Version            : V1.0.0
* Date               : 06/08/2009
* Description        : This file contains all the functions prototypes for the
*                      Menu.
***************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MENU_H
#define __MENU_H

#include "stm32f10x.h"

#define SURVEY_NAME_LENGTH 7




typedef enum 
{
  NO_OTHER_MENU, // used in the protocol options to indicate end of addition menus
  TIME,
  RTCDATE,
  DUR_TIME,
  SURVEY,
  STATION,
  RECORD,
  TEST,
  START_TIME,
  CODE,
  ENTRY1,
  ENTRY2,
  ENTRY3,
  ENTRY4,
  ENTRY5,
  ENTRY6,  
  
  
  USB,
  BAT,
  SAMP,
  DMA,
  CARD,
  PWROFF,
  LOWBATTERY,
  TEMP,
  GPS,
  DSP,
  DSP_SPI,
  SLEEP,
  CONTRAST_LCD,
  BOOT,
  RANDOM_DELAY,
  NFC  

}eMenuType;




typedef enum
{
  NO_EXIT,   
  TIMED_OUT,
  USER_BUTTON,
  POWER_FAILED
} MenuExitCode;


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


typedef bool (* pMenuFunc)(bACTION);

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

static uint8_t charTable[] = {'0','1','2','3','4','5','6','7','8','9',
                              'A','B','C','D','E','F','G','H','I','J',
                              'K','L','M','N','O','P','Q','R','S','T',
                              'U','V','W','X','Y','Z','_'}; 
static uint8_t  *stationItem[] =  {"BIRX",
                                   "BIRA",
                                   "BIRD",
                                   "BIRM",
                                   "BIRP"};  








/* Function Declaration*/
void Menu_Init(void);
void SetMenuCursorPos( int8_t curpos);
int8_t GetMenuCursorPos (void);
void CallUpdate();
void InitTier1Settings();
MenuExitCode RefreshMenu();
void SetMenuSwitchesFromProtocols(void);

bool TimeDisplay(bACTION);
bool DateDisplay(bACTION);
bool StartTimeDisplay(bACTION);
bool DurationDisplay(bACTION);
bool SurveyNameDisplay(void);
bool GPSLocationDisplay(void);
void Set_SELStatus(void);

uint8_t StartBlinkField( uint8_t  iField);

bool Record(bACTION);
bool Test(bACTION);
bool MainPowerOff(bACTION event);
bool SurveyDisplay(bACTION event);
bool CodeTest(bACTION event);
bool Sampling(bACTION event);
bool DMA_Start(bACTION event);
bool StartAndDuration(bACTION event);
bool TimeAndDate (bACTION event);
bool CardDisplay(bACTION event);
bool Temp (bACTION event);
bool Gps(bACTION event);
bool DspBoot(bACTION event);
bool Sleep(bACTION event);
bool Contrast(bACTION event);
bool StartupRandomDelay(bACTION event);
bool StationDisplay(bACTION event);

bool ProtocolDisplay(bACTION event);
bool GpsOptionsDisplay(bACTION event);
bool NFCTestDelay(bACTION event);


uint8_t* GetSurveyName(void);
uint8_t* GetStationID(void);



#endif /* __MENU_H */

