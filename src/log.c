/**
  ******************************************************************************
  * @file    log.c 
  ******************************************************************************
  *
  * for generated log file to be created with recordings
  *
  *
*/ 



#include "log.h"
#include "rtc.h"
#include "dsp_program.h"
#include "eeprom.h"
#include "FatSD.h"
#include "stdio.h"

#ifdef DEBUG
  #define LOGFILE
#endif


#define LOG_ENTRIES 30


//must match enum in log.h
char const *logString[] = {  
  "INVALID",
  "FILE_CREATE",
  "FILE_CLOSE",
  "RECORDING_START",
  "RTC_START",
  
  "BATTERY_LOW",
  "WATCHDOG_RESET",
  "SHUTDOWN",
  "UNIT_START",
  "DSP_FAIL_COMMS",
  
  "NO_SD_CARD",
  "RECORDING_TERMINATED",
  "ERROR_CODE",
  "LOW_SAMPLING_MODE",
  "HIGH_SAMPLING_MODE",
  
  "BAT_SAMPLING_MODE",
  "LOG_SUSPEND_MODE",
  "SUSPEND_RANDOM_MODE",
  "OTHER_SAMPLING_MODE",
  "DSP_FAIL_SET_SAMPLING",
  
  "FILE_DELETE",
  "FILE_RENAME",
  "PROTOCOL_ID",
  "MENU_ENTER",
  "MENU_EXIT",
  
  "PREVENT_RECORD",
  "FAIL_WRITE_BYTES",
  "FAIL_CLOSE_FILE",  
  "FAIL_FILE_CREATE",
  "FAIL_FILE_SEEK",
  
  "FAIL_FILE_DELETE",
  "FAIL_FILE_RENAME",
  "FAIL_FILE_CARD_FULL",
  "GPS_STARTED",
  "GPS_STOPPED",
  
  "GPS_LOCATION_FIX",
  "GPS_FAIL_START",
  "GPS_FAIL_FIX",  
  "GPS_LOCATION_SAVED",
  "GPS_FAIL_SAVE",
  "FAIL_LOG_CODE_ELEMENT"  //this must be the last entry in this array
};                    //this l;ist must match enum list in .h file










//static void WriteLogEntryEeeprom(LogStruct *log);
//static LogStruct *ReadLogEntryEeprom(int logIndex);

#ifdef LOGFILE
__no_init static LogStruct logFileCopy[LOG_ENTRIES]; // copy of eeprom log file, is not initialised upon start so unless the bootloader has overwritten it may still be good
__no_init static uint16_t logIndex;

static bool isLogFileCopyValid = FALSE; // is true once the log has been read from eeprom
#endif

#define MAX_LOG_STRING_LENGTH 10
static uint8_t logFileName[] = "log"; // this may get overwritten if different log filename present on card
static uint8_t logFileNameRandStub[] = "_";
static uint8_t logFileNameExt[] = ".txt";

static bool addRandomName = FALSE;

/********************************************************************
* saves to log file in eeprom to sd card
* - converts it into readable format
*/
int SaveLogToCard(void)
{
#ifdef LOGFILE   
  FIL file;
  int ret;
  int length;
  char string[128];

  ret = _f_poweron();/* 1st power on */
  
  //ret = f_mount(&fs, "", 0);
  //if (ret) return (int)ret;
  
//  if (ret==F_ERR_NOTFORMATTED)
//  {
//    ret=f_chdrive(0);	/* this is supported to select a drive which is not formatted */
//    return ret;
//  }  

    
  ret = f_open(&file, "debug_log.txt", FA_CREATE_ALWAYS | FA_WRITE | FA_READ); //open for appending
  if (ret) return 1;
                                       
  //seek beginning of file & write WAV DATA
  ret = f_lseek(&file,0);
  if (ret) return ret;

  LogStruct *log_entry;
  bool contentsInvalid;
  
  for (int i=0; i< LOG_ENTRIES; i++)
  {
    log_entry = &logFileCopy[i];
    contentsInvalid = FALSE;
    //check that log contents are valid
    if (log_entry->sec > 59) contentsInvalid = TRUE;
    if (log_entry->min > 59) contentsInvalid = TRUE;
    if (log_entry->hour > 23) contentsInvalid = TRUE;
    if (log_entry->day > 31) contentsInvalid = TRUE;
    if (log_entry->month > 12) contentsInvalid = TRUE;
    if (log_entry->year > 99) contentsInvalid = TRUE;
    
    if (!contentsInvalid)
    {     
      
      if (log_entry->code >= LOG_NUMBER_OF_ELEMENTS) log_entry->code = LOG_NUMBER_OF_ELEMENTS;    
      //first part date
      length = sprintf(string,"%02u:%02u:%02u_%02u\\%02u\\%02u %s %02u %02u %02u\r\n",
                  log_entry->hour,
                  log_entry->min,
                  log_entry->sec,
                  log_entry->day,
                  log_entry->month,
                  log_entry->year,
                  logString[log_entry->code],
                  log_entry->data1,
                  log_entry->data2,
                  log_entry->data3);
   
    }
    else
    {
      length = sprintf(string,"No entry.\r\n");
    }
    UINT byteswritten;
      if (length >= 0)
        f_write(&file, string,length,&byteswritten); 

  }
  
  //f_flush(&file);

  f_close(&file);

  /* simple restart */
  ret=_f_poweroff();
  if (ret) return ret;
  
  return 0;
#else

  return -1;
#endif  
  
}



//
// code = specific log code from list
// data1 = general purpose data
// data2 = general purpose data
// data3 = general purpose data
void AddToLog(LogCode code, uint8_t data1, uint8_t data2, uint8_t data3, LogTimeStampOptions timestampOptions)
{
  
#ifdef LOGFILE  
  
  rtcTimeTypeDef rtcTime;
  rtcDateTypeDef rtcDate;
  
  if (!isLogFileCopyValid) return;  // don't write if log file not initialised
  
  if (timestampOptions == TIMESTAMP)
  {
    rtcTime = *RtcGetTime();
    rtcDate = *RtcGetDate();
  }
  else if (timestampOptions == BUFFERED_TIMESTAMP)
  {
    rtcTime = *RtcGetBufferedTime();
    rtcDate = *RtcGetBufferedDate();
  }
  else
  {
    rtcTime.sec = 0;
    rtcTime.min = 0;
    rtcTime.hour = 0;
    rtcDate.date = 0;
    rtcDate.month = 0;
    rtcDate.year = 0;
  }

  logFileCopy[logIndex].sec = rtcTime.sec;
  logFileCopy[logIndex].min = (uint8_t)rtcTime.min;
  logFileCopy[logIndex].hour = rtcTime.hour;
  logFileCopy[logIndex].day = rtcDate.date;
  logFileCopy[logIndex].month = rtcDate.month;
  logFileCopy[logIndex].year = rtcDate.year;
  logFileCopy[logIndex].data1 = data1;  
  logFileCopy[logIndex].data2 = data2;
  logFileCopy[logIndex].data3 = data3;
  logFileCopy[logIndex].code = code;

  if (++logIndex >= LOG_ENTRIES) logIndex = 0;

#endif  
}




void CopyLogFileFromEeprom(void)
{
#ifdef LOGFILE   
  uint8_t *pLogData =  (uint8_t*)logFileCopy;
  
    // copy values from eeprom
  EEPROM_ReadPage(EE_LOG_BASE_ADDRESS, pLogData, sizeof(LogStruct)* LOG_ENTRIES);
  
  //copy the last index pointer
  logIndex = EEPROM_ReadShort(EE_LOG_COUNT_ADDR);
  if (logIndex >= LOG_ENTRIES) logIndex = 0;
  isLogFileCopyValid = TRUE;
#endif  
}




void SaveLogFileToEeprom(void)
{
#ifdef LOGFILE  
  uint8_t *pLogData =  (uint8_t*)logFileCopy;
  int bytesToWrite = sizeof(LogStruct) * LOG_ENTRIES;
  uint16_t addr = EE_LOG_BASE_ADDRESS;
  
  for (int i=0; i < bytesToWrite ; i++)
  {
    EEPROM_WriteByte(addr++,*pLogData++);
    ResetWatchdog();
  }
  
  // save data
  //for (int i=0; bytesToWrite >0; i++, bytesToWrite -= EEPROM_PAGE_SIZE, pLogData += EEPROM_PAGE_SIZE)
  //  EEPROM_WritePage(EE_LOG_BASE_ADDRESS + (i*EEPROM_PAGE_SIZE), pLogData, EEPROM_PAGE_SIZE);

  //save count
  if (logIndex >= LOG_ENTRIES) logIndex = 0;
  EEPROM_WriteShort(EE_LOG_COUNT_ADDR, logIndex);
#endif
}


  
// marks all the entries as invalid 
void ClearLog(void)
{
#ifdef LOGFILE   
  for (int i=0; i< LOG_ENTRIES; i++)
  {  
    logFileCopy[i].sec = 0xff;
    logFileCopy[i].min = 0xff;
    logFileCopy[i].hour = 0xff;
    logFileCopy[i].day = 0xff;
    logFileCopy[i].month = 0xff;
    logFileCopy[i].year = 0xff;
    logFileCopy[i].data1 = 0xff;
    logFileCopy[i].data2 = 0xff;
    logFileCopy[i].data3 = 0xff;
    logFileCopy[i].code = LOG_INVALID;
  }
  logIndex = 0;
  
#endif  
}



// writes a specific 
// addRandomName - can add a random string to filename
#define BUFFLEN 64
int WriteLogFileToCard(uint8_t* text_string, bool rootDir)
{
  FIL file;

  int ret;
  int length;
  char directory[BUFFLEN];
  uint8_t filename[64];
  uint8_t rname[MAX_LOG_STRING_LENGTH];
  uint8_t value;  
    
 
  strcpy((char*)filename, (const char*)logFileName);
  
  //if random file name is required then use this
  if (addRandomName)
  {
    //use the survey name to create a random number string 8 char between 0-9
    EEPROM_ReadSurveyName(rname);
    value = 11 * EEPROM_ReadStation(); // arb magic number
    int i;
    for (i =0;i<8;i++)
    {
      value = value + rname[i];
      rname[i] = (rname[i] + value) % 10;
      rname[i] += 0x30; // ascii number base
    }
    rname[i] = 0; //str terminate
    strcat((char*)filename, (const char*)logFileNameRandStub);
    strcat((char*)filename,(const char*) rname);
  }
  strcat((char*)filename, (const char*)logFileNameExt);
  
  //filename = logFileName;
  //check if filename valid
   
  length = strlen((char const*) filename);
  if (length <= 0 || length > 255) return 1;
  
  length = strlen((char const*) text_string);
  if (length < 0 || length > 255) return 1;
  
  //ret = _f_poweron();/* 1st power on */
  //if (ret==F_ERR_NOTFORMATTED)
  //{
  //  ret=f_chdrive(0);	/* this is supported to select a drive which is not formatted */
  //  return ret;
  //}    
  
  
 
    //first save the current directory
    //go back two folders in the directory
    ret = f_getcwd(directory, BUFFLEN);
    if (ret) return ret;
  

    if (strlen((char*)directory) > 1)
    {
      ret = f_chdir(".."); //go back one 
      if (rootDir) ret |= f_chdir(".."); // go back two dir  
    }
    if (ret) return ret;
   
  
  
  
  ret = f_open(&file, (char const*) filename, FA_OPEN_APPEND | FA_WRITE);
  if (ret) return ret;
                                       
  //seek beginning of file & write WAV DATA
  //ret = f_seek(file,0,F_SEEK_SET);
  //if (ret) return ret;    
  
  UINT byteswritten;
  ret = f_write(&file, text_string,length, &byteswritten); 
  if (ret) return ret;
  //f_flush(&file);

  f_close(&file);

  //restore directory
  ret = f_chdir((char*)directory);
  if (ret) return ret;
  
  
  /* simple restart */
  //ret=_f_poweroff();
  //if (ret) return ret;

  return 0;
}


//inserts text at beginning of file
#define FILEBUFFERSIZE 64
#define DIRBUFFLEN 64
int InsertTextToLogFile(uint8_t* text_string)
{
  FIL logfile;
  FIL logfile_temp;
  int32_t ret;
  int32_t numOfBytesToRead;
  int32_t bytecount;
  int length;
  uint8_t filebuffer[FILEBUFFERSIZE];
  char tempfilename[64];
  char directory[DIRBUFFLEN];
  uint8_t* filename;
  //check if filename valid
  
  filename = logFileName;
  
  
  length = strlen((char const*) filename);
  if (length <= 0 || length > 255) return 1;
  
  length = strlen((char const*) text_string);
  if (length < 0 || length > 255) return 1;
  
  //first save the current directory
  //go back two folders in the directory
  ret = f_getcwd(directory, DIRBUFFLEN);
  if (ret) return ret;
  
  
  if (strlen((char*)directory) > 1)
  {
    ret = f_chdir(".."); //go back one 
    ret |= f_chdir(".."); // go back two dir
  } 
  if (ret) return ret;
  

  
  //create temp filename to copy old log file to
  sprintf(tempfilename,"temp_");
  strcat(tempfilename,(const char*)filename);
  ret = f_open(&logfile_temp, (const char*)tempfilename, FA_OPEN_APPEND | FA_WRITE); //open for appending
  if (ret) return (int)ret;

  // log file to copy
  ret = f_open(&logfile, (const char*)filename, FA_READ); //open for appending
  if (ret) return (int)ret;
  
  numOfBytesToRead = f_size(&logfile);
  if(numOfBytesToRead  < 0) return -1;
  
  f_lseek(&logfile,0);
  f_lseek(&logfile_temp,0);
  
  UINT byteswritten;
  //write text data to beginning of file
  f_write(&logfile_temp, text_string,length, &byteswritten); 
 
  //copy rest of log file over
  while(numOfBytesToRead > 0)
  {
    bytecount = numOfBytesToRead;
    if(bytecount >= FILEBUFFERSIZE)  { bytecount = FILEBUFFERSIZE; }//if file is greater than buffer size then only fill buffer
    
    UINT bytesread;
    f_read(&logfile, filebuffer,bytecount,&bytesread);    //read from old file
    UINT byteswritten;
    f_write(&logfile_temp, (char*)filebuffer,bytecount,&byteswritten);  //write to new file
    numOfBytesToRead -=bytecount;
  }
  
  //f_flush(logfile_temp);

  //close both files
  ret = f_close(&logfile);
  if (ret) return ret;      
  ret = f_close(&logfile_temp);
  if (ret) return ret; 

  //NOW DELETE OLD FILE
  ret = f_unlink((const char*)filename);
  if (ret) return ret;
  
  //RENAME NEW FILE TO OLD ONE
  ret = f_rename((const char*)tempfilename,(const char*)filename);
  if(ret) return ret;
  
  //restore directory
  ret = f_chdir(directory);
  if (ret) return ret;
  
  return 0;  
}




//checks if there is a log file already on card
// -if log file exists sets name to that
// - else sets new name based
// if tier 1 then date is prefixed to "log.txt" eg "2018_12_08_log.txt"
/*
bool SetLogFileName(bool dateLogFilename)
{
  F_FIND find;
  bool status = FALSE;
  int ret;
  rtcDateTypeDef *rtc_date;

 
  
  
  //if (~dateLogFilename) return TRUE; // if no date is required in filename the we're already go to go (filename is init to "log.txt").
  
  // 1st power on 
  ret = _f_poweron();
   
  //check if a log file already exist with date in name. 
  // this just does a dumb check . if first char is a number 
  if (!f_findfirst("A:/log*.txt",&find)) //look for filename with "*****log.txt" in it 
  {
    do 
    {
      if (~(find.attr&F_ATTR_DIR)) //if file not directory
      {
          strcpy(find.filename,logfilename);
          status = TRUE;
          break;
      }
    } while (!f_findnext(&find));
  } 
  
  ret |= _f_poweroff();
  
  return status;
 
}
*/


void AddRandomNameToLogFilename(bool state)
{
  addRandomName = state;
}



// rtc time must have been refreshed 
// logs a temp every hour
void LogUserMessage(UserLogMessage messageType)
{
  uint8_t text_string[48];
  rtcTimeTypeDef *time;
  rtcDateTypeDef *rtc_date;

  // if time was recently aquired from rtc
  time = RtcGetTime(); 
  rtc_date = RtcGetDate();
  switch (messageType)
  {
    case USERLOG_START:
      sprintf((char*)text_string,"Start Rec @ %02d/%02d/%02d %02d:%02d:%02d\r\n",rtc_date->date, rtc_date->month,rtc_date->year, time->hour, time->min, time->sec);
      break;
    case USERLOG_STOP:
      sprintf((char*)text_string,"%02d/%02d/%02d %02d:%02d - Stop Recording\r\n",rtc_date->date, rtc_date->month,rtc_date->year, time->hour, time->min);
      break;
    case USERLOG_BATTERY_LOW:
      sprintf((char*)text_string,"%02d/%02d/%02d %02d:%02d - Battery Low\r\n",rtc_date->date, rtc_date->month,rtc_date->year, time->hour, time->min);
      break;
    case USERLOG_CARD_FULL:
      sprintf((char*)text_string,"%02d/%02d/%02d %02d:%02d:%02d - Card Full\r\n",rtc_date->date, rtc_date->month,rtc_date->year, time->hour, time->min, time->sec);
      break;
    default:
      sprintf((char*)text_string,"\r\n");
      break;
  }
  
  
  if (WriteLogFileToCard(text_string, FALSE) == 0)
  { 
    //success in writing log file
  }
  else
  { //Failed to write 
  
  }  
}




/*************************************EOF***************************************/