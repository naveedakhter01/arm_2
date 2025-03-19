
#ifndef LOG_H
#define LOG_H




#include "stm32f10x.h"



//must match array "LogString" in log.c
typedef enum
{
  LOG_INVALID,
  LOG_FILE_CREATE,
  LOG_FILE_CLOSE,
  LOG_RECORDING_START,
  LOG_RTC_START,
  
  LOG_BATTERY_LOW,
  LOG_WATCHDOG_RESET,
  LOG_SHUTDOWN,
  LOG_UNIT_START,
  LOG_DSP_FAIL_COMMS,
  
  LOG_NO_SD_CARD,
  LOG_RECORDING_TERMINATED,
  LOG_ERROR_CODE,
  LOG_LOW_SAMPLING_MODE,
  LOG_HIGH_SAMPLING_MODE,
  
  LOG_BAT_SAMPLING_MODE,
  LOG_SUSPEND_MODE,
  LOG_SUSPEND_RANDOM_MODE,
  LOG_OTHER_SAMPLING_MODE,
  LOG_DSP_FAIL_SET_SAMPLING,
  
  LOG_FILE_DELETE,
  LOG_FILE_RENAME,
  LOG_PROTOCOL_ID,
  LOG_MENU_ENTER,
  LOG_MENU_EXIT,
  
  LOG_PREVENT_RECORD,
  LOG_FAIL_WRITE_BYTES,
  LOG_FAIL_CLOSE_FILE,
  LOG_FAIL_FILE_CREATE,
  LOG_FAIL_FILE_SEEK,
  
  LOG_FAIL_FILE_DELETE,
  LOG_FAIL_FILE_RENAME,
  LOG_FAIL_CARD_FULL,
  LOG_GPS_STARTED,
  LOG_GPS_STOPPED,
  
  LOG_GPS_LOCATION_FIX,
  LOG_GPS_FAIL_START,
  LOG_GPS_FAIL_FIX, 
  LOG_GPS_LOCATION_SAVED,
  LOG_GPS_FAIL_SAVE,
  
  LOG_NUMBER_OF_ELEMENTS, // this must be the last item in this enum
} LogCode;          // this list must match log string list in .c file



typedef enum
{
  NO_TIMESTAMP,
  TIMESTAMP,
  BUFFERED_TIMESTAMP
} LogTimeStampOptions;


//message type to display in user log file
typedef enum
{
  USERLOG_START,
  USERLOG_STOP,
  USERLOG_BATTERY_LOW,
  USERLOG_CARD_FULL
  
} UserLogMessage; 



typedef struct
{
  uint8_t sec;
  uint8_t min;
  uint8_t hour;
  uint8_t day;
  uint8_t month;
  uint8_t year;
  uint8_t data1;
  uint8_t data2;
  uint8_t data3;
  LogCode code; 
} LogStruct;






int SaveLogToCard(void);
void AddToLog(LogCode code, uint8_t data1, uint8_t data2, uint8_t data3, LogTimeStampOptions timestampOptions);
void ClearLog(void);
void CopyLogFileFromEeprom(void);
int WriteLogFileToCard(uint8_t* text_string, bool rootDir);
int InsertTextToLogFile(uint8_t* text_string);

void CopyLogFileFromEeprom(void);
void SaveLogFileToEeprom(void);

void LogUserMessage(UserLogMessage messageType);
void AddRandomNameToLogFilename(bool state);

bool IsValidLogFile();

#endif