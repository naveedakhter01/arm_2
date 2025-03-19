/******************************************************************************
* @file    gps.h
* @author  DOC
* @version V1.0
* @date    10/07/2011
* @brief   gps m10372 module
******************************************************************************/

#ifndef __GPS_H
#define __GPS_H


#include "stm32f10x.h"
#include <time.h>


//GPS 
#define GPS_ON_OFF_PIN   GPIO_Pin_4
#define GPS_ON_OFF_PORT  GPIOA
#define GPS_ON_OFF_CLK   RCC_APB2Periph_GPIOA

//GPS 
#define GPS_TX_PIN   GPIO_Pin_2
#define GPS_TX_PORT  GPIOA
#define GPS_TX_CLK   RCC_APB2Periph_GPIOA

//GPS 
#define GPS_RX_PIN   GPIO_Pin_3
#define GPS_RX_PORT  GPIOA
#define GPS_RX_CLK   RCC_APB2Periph_GPIOA



typedef struct
{
  char  Latitude[12];      //Latitude.
  char  NorthSouth[2];        //NorthSouth indicator
  char  Longitude[12] ;   //Longitude  
  char  EastWest[2];           //EastWest indicator
  char  UTCTime[12];  
  int  PositionFix;       //0 = Fix not available or in valid, 1 GPS SPS Mode -fix Valid,  2 = DGPS,SPS Mode -fix Valid, 3-5 Not supported,6 Dead reckoning mode
  int  SatellitesUsed;   //
  int HorizontalDilutionOfPosition;
}GlobalPosition_TypeDef;

typedef struct
{
  char  Latitude[12];      //Latitude.
  char  NorthSouth[2];        //NorthSouth indicator
  char  Longitude[12] ;   //Longitude  
  char  EastWest[2];           //EastWest indicator
  char  UTCTime[12];  
  char  Status[2];       //0 = Fix not available or in valid, 1 GPS SPS Mode -fix Valid,  2 = DGPS,SPS Mode -fix Valid, 3-5 Not supported,6 Dead reckoning mode
  char  Mode[2];   //
  float MeasuredFlow;  // Measured flow from the flow sensor 
}GeographicPosition_TypeDef;

typedef struct
{
  char  Latitude[12];      //Latitude.
  char  NorthSouth[2];        //NorthSouth indicator
  char  Longitude[12] ;   //Longitude  
  char  EastWest[2];           //EastWest indicator
  char  LatitudeDecDeg[15];      //Latitude in decimal degrees. 
  char  LongitudeDecDeg[15] ;   //Longitude in decimal degrees  
  char  UTCTime[12];  
  char  Status[2];       //0 = Fix not available or in valid, 1 GPS SPS Mode -fix Valid,  2 = DGPS,SPS Mode -fix Valid, 3-5 Not supported,6 Dead reckoning mode
  char  Mode[2];   //
  char  Date[7];   //
  float MeasuredFlow;  // Measured flow from the flow sensor 
}RMCData_TypeDef;

typedef struct
{
  char  HrMinSec[12];      //time
  char  Day[4];        //date
  char  Month[4];        //date
  char  Year[6];        //date  
  char  LoacalZoneHours[4] ;   //zone hours
  char  LocalZoneMins[4];   //zone mins
}ZDAData_TypeDef;


#define  BAUD_4800 (uint32_t)4800
#define  BAUD_9600 (uint32_t)9600



typedef enum
{
  GPSLOG_RESET_TRIES,
  GPSLOG_FAILED,
  GPSLOG_PASSED,
} GpsLogOptions;
  
typedef enum
{
  GPS_IS_FIRST_START,
  GPS_NOT_FIRST_START
} GpsFirstStartOptions;


typedef enum
{ GGA_Msg = 0x00,
  GLL_Msg = 0x01,
  GSA_Msg = 0x02,
  GSV_Msg = 0x03,
  RMC_Msg = 0x04,
  VTG_Msg = 0x05,
  MSS_Msg = 0x06,
  ZDA_Msg = 0x08
}MsgTypedef;

//struct gpsPositionData
//{
//  char lattitude[16];
//  char longitude[16];
//  char elevation[8];
//  char time[24];
//  int32_t hdop;
//  bool isFixValid;
//};
typedef struct
{     
  int sec;
  int min;
  int hour;
  int day;
  int month;
  int year;
} time_typedef;


typedef struct
{
  double latitude; 
  double longitude;
  time_typedef time;
  int hdop;  
  int timeout; // is used to detect if data is not been updated 
  int satellites;
  bool isValid;
}GPS_Position_TypeDef;

typedef struct
{
  double latitude; 
  double longitude;
  int hdop;
  char  latStr[12];      //Latitude.
  char  northSouth[2];        //NorthSouth indicator
  char  longStr[12] ;   //Longitude  
  char  eastWest[2];           //EastWest indicator
} GpsCoordinates_TypeDef;




void GpsInit(void);
void DisableGpsUSART(void);
void EnableGpsUSART(void);

void GpsIOLowPower(void);

void GPS_On(void);
void GPS_Off(void);
bool StartGPSModule(void);
bool StopGPSModule(void);
bool IsGpsOn();

void ReadToBuffer(void);
void WriteFromBuffer(void);
uint8_t GpsSendCommand(uint8_t* command);
void DecodeMessageTable(void);
bool IsValidFix(void);
bool IsFix(void);
double ConvertToDecimalDegrees(const uint8_t* DegreesMinutes,char NESW);
GPS_Position_TypeDef * GetGPSData(void);

bool IsGpsModuleOnWithAutoBaud(void);
bool IsGpsModuleOn(void);


bool IsReceivingGPSMessages(void);
double Distance(double lat1, double lon1, double lat2, double lon2);
bool IsLastCoordinateFarAwayEnough(void);
int ConvertStringToInt(char* string, int length);
void InitMessageBuffer(void);
void ProcessIncomingGpsMessages(void);


void TurnOffGpsIfActive(void);

void LogGps();
void LogGpsAtOnce(bool);

void SetGpsLoggedTries(GpsLogOptions options);
bool IsGpsLogged(void);

void SetGpsFirstStart(GpsFirstStartOptions options);
bool IsGpsFirstStart(void);


void GpsEnableZdaMessage();
int32_t SyncToGpsTime(struct tm *gps_time);


//FIFO ROUTINES
bool GPSFifoEnqueue(uint8_t data);
uint8_t GPSFifoDequeue(void);
void GPSFifoClear(void);
int32_t GPSFifoCount(void);

void InitGpsCoordinateAveraging(void);
void AddGpsSample(GpsCoordinates_TypeDef gpsCoordinate);
GpsCoordinates_TypeDef* GetGpsCoordinates(void);
bool IsGpsLogOptionEnabled();
bool IsGpsSyncOptionEnabled();







#endif //__GPS_H











/*************************************EOF**************************************/