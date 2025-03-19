/***************************************************************************
*  record.c     
*  16/10/09
* 
***************************************************************************/

#include "main.h"
#include "record.h"
#include "FatSD.h"
#include "eeprom.h"

#include "stdio.h"
#include "sysclk.h"
//#include "fat/common/port_f.h"
#include "menu.h"
#include "spi.h"
#include "dsp_program.h"
#include "pushbutton.h"
#include "string.h"
#include "random.h"
#include <stdlib.h>


#define NUM_OF_START_STOP_ITEMS 2

typedef struct
{
  int start_in_mins;
  int stop_in_mins;
} StartStop_TypeDef;
  

typedef struct
{
  uint8_t rootFolderName[32];
  uint8_t localFolderName[32];
  int numberOfFiles;
} RecordFolders_TypeDef;




#define MAX_FILE_TIME_OFFSETS 128


static int protocolsUsed = 0;
static RecordFolders_TypeDef rFolders[NUM_OF_START_STOP_ITEMS]; //only 2 protocols 
static TimeOffset_TypeDef fileTimeOffsets[MAX_FILE_TIME_OFFSETS];
static int ftOffsetIndex = 0;
static uint64_t baseTime;
static int32_t gps_adjust_offset = 0;




void SaveFolderNamesForRename(uint8_t *rootFolderName, uint8_t *localFolderName)
{
  
  if (protocolsUsed >= NUM_OF_START_STOP_ITEMS) return;
  
  strcpy((char*)rFolders[protocolsUsed].rootFolderName, (const char*)rootFolderName);
  strcpy((char*)rFolders[protocolsUsed].localFolderName, (const char*)localFolderName);
  
  rFolders[protocolsUsed].numberOfFiles = 0;
  protocolsUsed++;
  
 
}


void IncFileCountForRename()
{
  if (protocolsUsed >0 && protocolsUsed <= NUM_OF_START_STOP_ITEMS)
      rFolders[protocolsUsed-1].numberOfFiles++;
}





StartStop_TypeDef startStopList[NUM_OF_START_STOP_ITEMS];


//private variables
static int8_t SDWriteFlag =0;

static char FolderName[] = "survey";
static char testFolderName[] = "Test";

static uint8_t dateFolderName[10] = "";


static uint32_t colorTable[256];



static WAVE_InitTypeDef WAVE_InitStruct_A;


static int fileTagNumber = 0;


//for temp buf
#define TEMPBUFLENGTH 128



char const *errorString[] = {  
  "NO_ERROR",
  "FAIL_FILE_CREATE",
  "FAIL_FILE_CLOSE",
  "FAIL_CARD_WRITE",
  "FAIL_CARD_WRITE_ITEMS",
  "FAIL_FILE_DELETE",
  "FAIL_CARD_FULL",
  "FAIL_FILE_RENAME",
  "DSP_WAIT_FOR_EVENT_FAIL",
  "DSP_COMMAND_MODE_FAIL",
  "INVALID_RECORD_TYPE"
};






/*****************************************/







//TEMPORARY FUNCTION TO DELETE "record.wav" FILE
void PwrOFF(void)
{
  
  _f_poweron();
  _f_poweroff();
  
}


void EnableRecordingSpi(void)
{
  SetSpiSlave(DMA_On);
}


void DisableRecordingSpi(void)
{
  ResetSpiSlave();
}


uint8_t* GetErrorString(SessionError errorCode)
{
  return (uint8_t*)errorString[errorCode]; 
}

//is used to tag each file with an incremental number
// is more used for debugging errors than anything else
int GetFileTagNumber()
{
  int ret = fileTagNumber;
  
  if (++fileTagNumber > 9) fileTagNumber = 0;
  return ret;
}


/*********************************************
* Checks if valid card
*
*********************************************/
//int CheckCard(void)
//{
//	int ret;
//        F_SPACE space;
//       // char buffer[1024];
//       
///* 1st power on */
//	ret = _f_poweron();
//        
//	if (ret==F_ERR_NOTFORMATTED)
//        {
//         return(1);
//        }
//        if (ret) return(2);
//
//        ret = f_checkvolume(f_getdrive());  
//        if (ret) return(2);     
//
//        ret = f_getfreespace(f_getdrive(),&space);
//        
//        if(!ret)
//        {
//          if((space.free_high == 0) && (space.free < 1000))
//            return(3);
//          
//          
//        }
//        else return(2); //error
//        
//
//        
//        
//        
//        
//
//                
///* simple restart */
//   ret=_f_poweroff();
//   if (ret) return(2);
//        
//   return 0;
//  
//}
//
///*********************************************
//* Checks if valid card
//*
//*********************************************/
//int FormatCardDepreciated(void)
//{
//	int ret;
//        F_SPACE space;
///* 1st power on */
//	ret = _f_poweron();
//        
//        if (ret )
//        {
//          if (ret==F_ERR_NOTFORMATTED)
//          {
//           return(1);
//          }
//        if (ret) return(2);
//        }
//        ret = f_getfreespace(0,&space);
//        if (ret) return(2);
//        
//        
//        if(space.total_high) // greater than 4GB - format FAT32
//        {
// 		ret=f_format(f_getdrive(),F_FAT32_MEDIA);
//		if (ret) return(2);       
//        }
//        else    // up to 4BG so FAT16
//        {
// 		ret=f_format(f_getdrive(),F_FAT16_MEDIA);
//		if (ret) return(2);        
//        }
//        
//
//                 
//
//                
///* simple restart */
//   ret=_f_poweroff();
//   if (ret) return(2);
//        
//   return 0;
//  
//}


/*********************************************
* Checks if valid card
*
*********************************************/
int FormatCard(void)
{
 int ret;
  
  //F_SPACE space;
  /* 1st power on */  
  ret = _f_poweron();
  if(ret) return ret;
  
  
  //FATFS fs;           /* Filesystem object */
  BYTE work[FF_MAX_SS]; /* Work area (larger is better for processing time) */
  ret = f_mkfs("", 0, work, sizeof work);
  if(ret) return ret;
  
//  if (ret == 0)
//  {
//    int drive = f_getdrive();
//    ret=f_format(drive,F_FAT32_MEDIA);
//   
//    if (ret == F_ERR_MEDIATOOSMALL)
//    {
//      ret=f_format(drive,F_FAT16_MEDIA);
//      if (ret) return ret;
//      
//      _f_poweroff();
//      return(ret); 
//    }
//    if (ret) 
//    {
//      _f_poweroff();
// 
//      return(ret); 
//    }
//    
//    //=================================
//    // confirm fat
//    
//    ret = _f_poweroff();
//    if (ret) return ret;
//    delay_ms(1000);
//    
//    ret = _f_poweron();
//    if (ret == 0)
//    { 
//        drive = f_getdrive();
//        ret=f_confirmformat(drive,F_FAT32_MEDIA);
//    }
//  }
 
/* simple restart */
  _f_poweroff();
   return ret;
}

/*******************************************************************
* Create WAV header
* returns a pointer to wav header data
******************************************************************/


WAVE_RIFF_TypeDef* MakeWAVheader(WAVE_InitTypeDef* WAVE_Init)
{
  
    static WAVE_RIFF_TypeDef WAVE_Header;
  
    WAVE_Header.ChunkID = 0x46464952;  //"RIFF" backwards
    WAVE_Header.ChunkSize = (WAVE_Init->WAV_DataSize + WAVEHEADERSIZE) - 8;  //(header size + data size) - 8
    WAVE_Header.RIFFType = 0x45564157; // "WAVE" backwards
      
      
    WAVE_Header.WAVE_Format.FmtChunkID =  0x20746D66;  // "fmt "
    WAVE_Header.WAVE_Format.FmtChunkSize = 16; // 16 + extra format bytes
    WAVE_Header.WAVE_Format.CompressionCode = 0x01;  //PCM/uncompressed
    WAVE_Header.WAVE_Format.NumChannels = WAVE_Init->WAV_NumChannels;
    WAVE_Header.WAVE_Format.SampleRate  = WAVE_Init->WAV_SampleRate;
    WAVE_Header.WAVE_Format.ByteRate =  ((WAVE_Init->WAV_BitsPerSample /8) * WAVE_Init->WAV_NumChannels) * WAVE_Init->WAV_SampleRate;
    WAVE_Header.WAVE_Format.BlockAlign =  (WAVE_Init->WAV_BitsPerSample /8) * WAVE_Init->WAV_NumChannels; 
    
    WAVE_Header.WAVE_Format.BitsPerSample = WAVE_Init->WAV_BitsPerSample;

    WAVE_Header.DATAChunkID = 0x61746164;  //"DATA" backwards
    WAVE_Header.DATASize = WAVE_Init->WAV_DataSize; //size of wave file minus header


  return(&WAVE_Header);
}















/*******************************************************************
* Sets state of Flags Used to determine when to save samples to SD card
*
*****************************************************************/

void SetSDWriteFlag(BitAction NewState)
{
  SDWriteFlag = (int8_t)NewState; 
}


BitAction GetSDWriteFlag(void)
{ 
 return((BitAction)SDWriteFlag);  
}






/*********************************************************************
* CheckRecordTime
* takes the start time and duration and returns flag if time in range
*********************************************************************/

uint8_t CheckRecordTime(void)
{
  uint8_t retVal = FALSE;

  rtcTimeTypeDef *rtcTime;
  
  //get current time
  rtcTime = RtcGetTime(); 
  int slots = GetNumOfProtocolSlots();
  
  for(int i=0;i<slots;i++)   
  {
    retVal |= (IsWithinProtocolOnTime(rtcTime->hour, rtcTime->min, i))->isWithin ;
  }
     
  return retVal;
}








/*********************************************************************
* SetNextAlarm
* set time to the 
*********************************************************************/

void SetNextAlarm(void)
{ 
  rtcTimeTypeDef  *currentTime;
  rtcTimeTypeDef  nextTime;
  Protocol_UserDef_Start_Span_TypeDef *pNextStartStop;
 
 
  currentTime = RtcGetTime();    //get current time  
  
  pNextStartStop  = GetNextActiveProtocolStartSpanTime(currentTime);
 
  nextTime.sec = 0;
  nextTime.min = Rtc_bin2bcd(pNextStartStop->startMin) | 0x80;
  nextTime.hour = Rtc_bin2bcd(pNextStartStop->startHour) | 0x80;
  RtcClearAlarmFlag(); //clear RTC alarm
  RtcSetAlarm(&nextTime);      
}


/****************************************************************
*  
*
*/
int OpenSDCard(void)
{
  int ret;

  ret = _f_poweron();
  //if (ret)
  //{
  //        ret=f_chdrive(0);	/* this is supported to select a drive which is not formatted */
  //        if (ret) return ret;
  //}     

  return 0;
}


int CloseSdCard(void)
{
  return _f_poweroff();
}


/****************************************************************
*  
*
*
*/
int CreateWavDirectory(void)
{
  f_mkdir(FolderName);
  f_chdir(FolderName);
  return 0;
}


int CreateTestDirectory(void)
{
  //get_fattime();
  //f_mount(&fs, "", 0);
  f_mkdir(testFolderName);
  f_chdir(testFolderName);
  return 0;
}


void SetFolderNameByDate(void)
{
  rtcTimeTypeDef *pTime;
  rtcDateTypeDef *pDate;
  
  pTime = RtcGetTime(); //update buffered date
  pDate = RtcGetDate(); //update buffered date
  get_fattime();
  //f_inittime(pTime);
  //f_initdate(pDate);
     
  sprintf((char*)dateFolderName,"20%02u%02u%02u",pDate->year, pDate->month, pDate->date );
          
  //for (int k =0; k<10;k++) blah[k] = 23;
          

}


bool ChangeDirectory(uint8_t *dirName)
{


  uint8_t curDir[32];
  bool success = TRUE;
  int ret;
  
  
  WatchdogExtendTimer(200);   // wd extension . 100 = ~ 1min 
  
  
  // first go back to root directory
  ret = f_getcwd( (char*)curDir, 32 );
  if (ret) success = FALSE;
  
  if (strlen((char*)curDir) > 1)
  {
    f_chdir(".."); //go back one 
    f_chdir(".."); // go back two dir
  }
  // now make and open folder if doesn't exists, will named after protocol 
  f_mkdir((char*)dirName);
  ret = f_chdir((char*)dirName);
  if (ret) success = FALSE;
  
  
  //now make and open folder based on the date recording started
  f_mkdir((char*)dateFolderName);
  ret = f_chdir((char*)dateFolderName);
  if (ret) success = FALSE;
  
  //save folder names for renaming process later
  SaveFolderNamesForRename(dirName, dateFolderName);
  
  WatchdogExtendTimer(0);   // wd extension . 100 = ~ 1min 
  return success;
}




SessionExitCode_Typedef *ChangeSamplingMode(RecordSessionInit_TypeDef* sessionInit)
{
  RecordSession_TypeDef *recordSession;
  DspSamplingOption dspSamplingOption;
  static SessionExitCode_Typedef exitCode;
    
  
  // Create the Record Session here
  AddChangeDspSamplingSession(sessionInit);
  
  
  // now get the record session
  recordSession = GetCurrentRecordSession();
  
  // init default exit code
  exitCode.isButtonPushed = FALSE;
  exitCode.powerOk = TRUE;
  exitCode.error = NO_ERROR;
  
  
  recordSession->state = ACTIVE;
  // first disable interrupts from dsp
  DisableRecordingSpi(); 
   
  Dsp_RunCommandMode(ENABLE); //stick dsp back into command mode to change settings
  
  if (!WaitForDspEvent()) recordSession->errorCode = DSP_WAIT_FOR_EVENT_FAIL;
       
  if (!IsDspInCommandMode()) recordSession->errorCode = DSP_COMMAND_MODE_FAIL; //this function will delay return until command mode, if not set then mark error in fileSession
  
  switch (recordSession->instruction)
  {
     case CHANGE_LOW_SAMPLING:
      dspSamplingOption = DSP_LOW_FILTER;
      AddToLog(LOG_LOW_SAMPLING_MODE,(uint8_t)recordSession->protocolNumber,0,0,TIMESTAMP);
      SetSysClk2MHz();
      break;
    case CHANGE_HIGH_SAMPLING:
      dspSamplingOption = DSP_HIGH_FILTER;
      AddToLog(LOG_HIGH_SAMPLING_MODE,(uint8_t)recordSession->protocolNumber,0,0,TIMESTAMP);
      SetSysClk8MHz();
      break;
    case CHANGE_BAT_SAMPLING:
      dspSamplingOption = DSP_BAT_FILTER;
      AddToLog(LOG_BAT_SAMPLING_MODE,(uint8_t)recordSession->protocolNumber,0,0,TIMESTAMP);
      SetSysClk8MHz();
      break;
    case CHANGE_SUSPEND:
      dspSamplingOption = DSP_NO_FILTER;
      AddToLog(LOG_SUSPEND_MODE,(uint8_t)recordSession->protocolNumber,0,0,TIMESTAMP);
      SetSysClk2MHz();
      break;          
    default:
      dspSamplingOption = DSP_NO_FILTER;
      AddToLog(LOG_OTHER_SAMPLING_MODE,(uint8_t)recordSession->protocolNumber,0,0,TIMESTAMP);
      SetSysClk2MHz();
      break;
  }
    
  if (!SetDspSamplingMode(dspSamplingOption)) // set recording mode of dsp to be equal to next files recording mode
  {
    AddToLog(LOG_DSP_FAIL_SET_SAMPLING,0,0,0,NO_TIMESTAMP);
  }
   
  EnableRecordingSpi();  //initialise the spi with/without the DMA
  Dsp_RunCommandMode(DISABLE); //take dsp out of command mode
  recordSession->state = FINISHED;
  
  //remove record session here
  RemoveRecordSession();
  
  exitCode.error = recordSession->errorCode;
  return &exitCode;
}

/****************************************************************
*  
*
*
*/
// OLD int CreateWavFile(uint8_t *fileName, uint32_t samplingRate)
int CreateWavFile(RecordSession_TypeDef *recordSession)
{
  int ret;
 
  
  WAVE_RIFF_TypeDef *WAVE_Header;
  //copy pointers for buffer       
  //f_mount(&fs, "", 0);
  /* 1st power on */
  
  
  //f_chdir(FolderName);
  //if (ret) return ret;            
  WatchdogExtendTimer(200);
  
 
   
  get_fattime();
  ret = f_open(&recordSession->file, (const char*)recordSession->filename, FA_CREATE_ALWAYS | FA_WRITE | FA_READ); //open for appending
  if (ret) 
  {
    recordSession->errorCode = FAIL_FILE_CREATE;
    AddToLog(LOG_FAIL_FILE_CREATE,(uint8_t)recordSession->tag, FILE_BEGIN, recordSession->instruction, TIMESTAMP);
    WatchdogExtendTimer(0);   
    return 1;
  }
  
  //ret=set_timestamp((char*)recordSession->filename, newTime, newDate);

  

  
  //Create A space for the WAVE File
  WAVE_InitStruct_A.WAV_SampleRate = recordSession->samplingRate;
  WAVE_InitStruct_A.WAV_NumChannels = WAV_MONO;
  WAVE_InitStruct_A.WAV_BitsPerSample = WAV_BITS_PER_SAMPLE_16;
  WAVE_InitStruct_A.WAV_DataSize = recordSession->fileByteCount;    // was  ACTIVEBUFSIZE but will only write in full buffer blocks
  WAVE_Header = MakeWAVheader(&WAVE_InitStruct_A);                                          
  //seek beginning of file & write WAV DATA
  ret = f_lseek(&recordSession->file,0);
  if (ret) 
  {
    AddToLog(LOG_FAIL_FILE_SEEK,(uint8_t)recordSession->tag, FILE_BEGIN, recordSession->instruction, TIMESTAMP);
    WatchdogExtendTimer(0);   
    return ret;
  }
  UINT byteswritten;
  ret = f_write(&recordSession->file, WAVE_Header,44, &byteswritten);
  if (byteswritten != 44)
  {
    AddToLog(LOG_FAIL_WRITE_BYTES,(uint8_t)recordSession->tag, FILE_BEGIN, recordSession->instruction, BUFFERED_TIMESTAMP);
    recordSession->errorCode = FAIL_CARD_FULL;
    WatchdogExtendTimer(0);   
    return 0x7F;     
  }
  
  
  AddToLog(LOG_FILE_CREATE,(uint8_t)recordSession->tag, FILE_BEGIN, recordSession->instruction, BUFFERED_TIMESTAMP);
  // ret = f_close(wavFile);
  // if (ret) return ret;      
  
  
  //mark the time after file has been created so we can change the filename later if need be
  MarkFileRenameTimeOffsets(recordSession->origFileTimeTicks, GetRtcTimeInTicks());  
  IncFileCountForRename();
  
  
  WatchdogExtendTimer(0);
  return 0; // else return no problems
}





int CloseWavFile(RecordSession_TypeDef *recordSession)
{
  int ret;
  
  WAVE_RIFF_TypeDef* WAVE_Header;  
  
  
  if(&recordSession->file) // if valid file
  {
    WatchdogExtendTimer(200);
    WAVE_InitStruct_A.WAV_DataSize = recordSession->fileByteCount; //add total file count
    WAVE_Header = MakeWAVheader(&WAVE_InitStruct_A);                     
       
    //seek beginning of file & write WAV DATA
    ret = f_lseek(&recordSession->file,0);
    if (ret)  
    {
      AddToLog(LOG_FAIL_FILE_SEEK,(uint8_t)recordSession->tag, FILE_END, recordSession->instruction, TIMESTAMP);
      WatchdogExtendTimer(0);   
      return ret;
    }
    UINT byteswritten;
    ret = f_write(&recordSession->file, WAVE_Header,44, &byteswritten);
    if (byteswritten != 44) 
    {
      AddToLog(LOG_FAIL_WRITE_BYTES,(uint8_t)ret, FILE_END, recordSession->instruction, TIMESTAMP);
      WatchdogExtendTimer(0);   
      return 0x7D;                 
    }
    //make sure data saved    
    //f_sync(&recordSession->file);    
    ret = f_close(&recordSession->file);
    if (ret) 
    {
      AddToLog(LOG_FAIL_CLOSE_FILE,(uint8_t)recordSession->tag, FILE_END, recordSession->instruction, TIMESTAMP);
      WatchdogExtendTimer(0);   
      return ret;               
    }
    AddToLog(LOG_FILE_CLOSE,(uint8_t)recordSession->tag, FILE_END, recordSession->instruction, TIMESTAMP);
    
    //if no data in file then delete it.
    if (recordSession->fileByteCount == 0)
    {
      ret = f_unlink((char const*)recordSession->filename);
      if (ret) 
      {
        AddToLog(LOG_FAIL_FILE_DELETE,(uint8_t)recordSession->tag, FILE_END, recordSession->instruction, NO_TIMESTAMP);
        WatchdogExtendTimer(0);   
        return ret;
      }
      AddToLog(LOG_FILE_DELETE,(uint8_t)recordSession->tag, FILE_END, recordSession->instruction, NO_TIMESTAMP);
    }
    
    /* simple restart */
   // ret=_f_poweroff();
   // if (ret) return ret; 
    WatchdogExtendTimer(0);   
  }
  else
  {
    
    //recordSession->file = (void*)0; 
    memset(&recordSession->file, 0, sizeof(FIL));
    return 1; 
  }
  

  //recordSession->file = (void*)0; 
  memset(&recordSession->file, 0, sizeof(FIL));
  return(0);
}


/****************************************************************
* Writes a block of audio samples to the currently open wav file  
*
* - wav file must be open before calling this.
* - if fails to write all bytes then card is flushed and attempts to close wav file
*
* blockAddress = address of buffer where wav data is stored 
* blockCount = number of words (uint16) to save to card
*/
RecordStatus WriteWavBlock(uint32_t blockAddress, int32_t blockCount)
{
    RecordSession_TypeDef *recordSession;
  RecordStatus retStatus;
  static bool lock = FALSE;
  int ret;
  
  
 
  
  retStatus.status = OK;  //assume card will write correctly
  retStatus.fw_error = FR_OK; // reset fat error code
  
  if (lock) return retStatus;
  
  recordSession = GetCurrentRecordSession();
  
  if ( (recordSession != (void*)0) && //if file session isn't void
     (recordSession->state == ACTIVE) && // if file session is currently active 
     (&recordSession->file ) )       // file handle is valid
  {
    lock = TRUE;
    WatchdogExtendTimer(200);
    
    UINT byteswritten;
    UINT total_written = 0;
    
    while (total_written < blockCount) 
    {
        UINT to_write = ((blockCount) - total_written > 512) 
                        ? 512 
                        : ((blockCount) - total_written);

        ret = f_write(&recordSession->file, (char*)blockAddress + total_written, to_write, &byteswritten);
//        if (res != FR_OK || byteswritten != to_write) {
//            return ret;  // Error handling
//        }
        
        total_written += byteswritten;
    }
    
    
    //ret =  f_write(&recordSession->file,(char*)blockAddress,blockcount/2,&byteswritten); // returns data written in bytes
    if (total_written == blockCount) //check if all bytes written, compare against blockCount(uint16_t)
    {
       recordSession->fileByteCount += blockCount; //all bytes written correctly
       recordSession->currentFileLengthInMilliSecs += ((SAMPLE_BUFFER_HALF_SIZE *1000) / (recordSession->samplingRate*2)); //16kb buffer / (sampling rate in bytes per sec) = number o secs to fill buffer. * 1000 = in millisecs 
       if (recordSession->currentFileLengthInMilliSecs >= recordSession->maxFileLengthInMilliSecs) recordSession->state = FINISHED;
    }
    else  // failed to write all bytes - card full?/
    {
      recordSession->fileByteCount += ret/2;   
      recordSession->errorCode = FAIL_CARD_FULL;
      recordSession->state = FINISHED;
      AddToLog(LOG_FAIL_WRITE_BYTES,(uint8_t)recordSession->tag, FILE_MIDDLE, recordSession->instruction, NO_TIMESTAMP);
      
      
      //Close file then reopen it
      retStatus.status  = CARD_ERROR; // assume card error 
      retStatus.fw_error = f_close(&recordSession->file);
      if(retStatus.fw_error == 0) //if no error in closing file
      {
        
        ret = f_open(&recordSession->file, (const char*)recordSession->filename, FA_CREATE_ALWAYS | FA_WRITE | FA_READ); 
        if (ret)  // if file still opens then finish file 
        {
          AddToLog(LOG_FAIL_CARD_FULL,(uint8_t)recordSession->tag, FILE_MIDDLE, recordSession->instruction, NO_TIMESTAMP);
          retStatus.status  = CARD_FULL;
          AddToLog(LOG_FAIL_WRITE_BYTES,(uint8_t)recordSession->tag, FILE_MIDDLE, recordSession->instruction, NO_TIMESTAMP);
          //Modify header with current datacount                   
          retStatus.fw_error = CloseWavFile(recordSession); //TODO update closerecording function
        } 
      }
    }
    //WatchdogExtendTimer(0);
    lock = FALSE;
  }
  else // no wav file open
  {
    
  }
  return retStatus;
}








int CreateBmpFile(RecordSession_TypeDef *recordSession)
{
  BMP_TypeDef bmpHeaderInfo;
  int ret;
  uint8_t *bmpHeaderarray;
  
  //create filename
  ret = f_open(&recordSession->file, (const char*)recordSession->filename, FA_CREATE_ALWAYS | FA_WRITE | FA_READ); //open for appending
  if (ret) 
  {
    AddToLog(LOG_FAIL_FILE_CREATE,(uint8_t)recordSession->tag,FILE_BEGIN,recordSession->instruction,TIMESTAMP);
    recordSession->errorCode = FAIL_FILE_CREATE;
    return 1;
  }
  FillGreyScaleTable(colorTable);
  
  //setup bmp header - becuase we don't know how long the file is until finished, so we'll overwrite these later
  bmpHeaderInfo.width = recordSession->bmpWidth;
  bmpHeaderInfo.height = recordSession->bmpHeight;  // this part is known
  bmpHeaderInfo.datasize = recordSession->fileByteCount;
  bmpHeaderarray = GetBmpHeader(&bmpHeaderInfo);
                 
  //seek beginning of file & write WAV DATA
  ret = f_lseek(&recordSession->file,0);
  if (ret) 
  {
    AddToLog(LOG_FAIL_WRITE_BYTES,(uint8_t)recordSession->tag,FILE_BEGIN,(uint8_t)ret,TIMESTAMP);
    recordSession->errorCode = FAIL_FILE_CREATE; 
    return ret; 
  }
  //copy header
  UINT byteswritten;
  ret = f_write(&recordSession->file, bmpHeaderarray,BMP_HEADER_SIZE, &byteswritten);  
  if (byteswritten != BMP_HEADER_SIZE) 
  {
    AddToLog(LOG_FAIL_WRITE_BYTES,(uint8_t)recordSession->tag,FILE_BEGIN,(uint8_t)ret,TIMESTAMP); 
    recordSession->errorCode = FAIL_CARD_WRITE; 
    return 0x7F;
  }   
  //copy color table
    UINT total_written = 0;
    
    while (total_written < BMP_COLORTABLE_SIZE) 
    {
        UINT to_write = ((BMP_COLORTABLE_SIZE) - total_written > 512) 
                        ? 512 
                        : ((BMP_COLORTABLE_SIZE) - total_written);

        ret = f_write(&recordSession->file, colorTable + total_written, to_write, &byteswritten);        
        total_written += byteswritten;
    }

  
  //ret =  f_write(&recordSession->file,colorTable,BMP_COLORTABLE_SIZE,&byteswritten);
  if (total_written != BMP_COLORTABLE_SIZE) 
  {
    AddToLog(LOG_FAIL_WRITE_BYTES,(uint8_t)recordSession->tag,FILE_BEGIN,(uint8_t)ret,TIMESTAMP);
    recordSession->errorCode = FAIL_CARD_WRITE; 
    return 0x7F;
  }          
  ret= 0;
  
  AddToLog(LOG_FILE_CREATE,(uint8_t)recordSession->tag,FILE_BEGIN,recordSession->instruction,TIMESTAMP);

  return 0;     
  
}



/**********************************************************************
* Closes the currently active bmp file
*
*/
int CloseBmpFile(RecordSession_TypeDef *recordSession)
{
  int ret;
  BMP_TypeDef bmpHeaderInfo;
  uint8_t *bmpHeaderarray;
   
  
  
  
  //attempting to clear any data left in dma buffer after each file write 
  DMA_Cmd(DMA1_Channel4, DISABLE);  
  Spi_Dma_Init(); 
  DMA_Cmd(DMA1_Channel4, ENABLE);  
  
  if(&recordSession->file) // if valid file
  {       
    AddToLog(LOG_FILE_CLOSE,(uint8_t)recordSession->tag,FILE_END,recordSession->instruction,TIMESTAMP);
    //determine if any data stored to file if not then delete file
    if (recordSession->fileByteCount > 0)
    {
      //setup bmp header - becuase we don't know how long the file is until finished, so we'll overwrite these later
      bmpHeaderInfo.width = recordSession->bmpWidth;
      bmpHeaderInfo.height = recordSession->bmpHeight;  // this part is known
      bmpHeaderInfo.datasize = recordSession->fileByteCount;
      bmpHeaderarray = GetBmpHeader(&bmpHeaderInfo);
      
      //seek beginning of file & write WAV DATA
      ret = f_lseek(&recordSession->file,0);
      if (ret) 
      {
        recordSession->errorCode = FAIL_CARD_WRITE; 
        return ret;
      }    
      UINT byteswritten;
      ret = f_write(&recordSession->file,bmpHeaderarray,BMP_HEADER_SIZE, &byteswritten);  //overwrite the header with the updated values
      if (byteswritten != BMP_HEADER_SIZE) 
      {
        recordSession->errorCode = FAIL_CARD_WRITE; 
        return ret;
      }            
    
      //make sure data saved    
      //f_flush(recordSession->file);    
                        
      ret = f_close(&recordSession->file);  //close the file
      if (ret) 
      {
        recordSession->errorCode = FAIL_FILE_CLOSE; 
        return ret;
      }  
      
    }
    else //no data in file so close and delete it
    {
      ret = f_close(&recordSession->file);  //close the file
      if (ret) 
      {
        AddToLog(LOG_FAIL_CLOSE_FILE,(uint8_t)recordSession->tag,FILE_END, recordSession->instruction,NO_TIMESTAMP);
        recordSession->errorCode = FAIL_FILE_CLOSE; 
        return ret;
      }  
      
      ret = f_unlink((char*)recordSession->filename);
      if (ret) 
      {
        AddToLog(LOG_FAIL_FILE_DELETE,(uint8_t)recordSession->tag,FILE_END, recordSession->instruction,NO_TIMESTAMP);
        recordSession->errorCode = FAIL_FILE_DELETE; 
        return ret;
      }  
      AddToLog(LOG_FILE_DELETE,(uint8_t)recordSession->tag,FILE_END, recordSession->instruction,NO_TIMESTAMP);
    }
    
  }
  else
  {
    //recordSession->file = (void*)0; 
    memset(&recordSession->file, 0, sizeof(FIL));
    return 1; 
  }
  
  //recordSession->file = (void*)0; 
  memset(&recordSession->file, 0, sizeof(FIL));
  return(0);
}


void FillGreyScaleTable(uint32_t* table)
{
  int i;
  uint32_t tmp;
  static bool isFilled = FALSE;
  
  if (!isFilled)
  {
    for(i=0;i<256;i++)
    {
     tmp = (i<<16) + (i<<8) + i;
     tmp = ~tmp;
     tmp &= 0x00FFFFFF;
     table[i] = tmp;
    }
    isFilled = TRUE;
  }
}



/****************************************************************
* Writes a block of bitmap samples to the currently open bmp file  
*
* - bmp file must be open before calling this.
* - if fails to write all bytes then card is flushed and attempts to close bmp file
*
* blockAddress = address of buffer where wav data is stored 
* blockCount = number of words (uint16) to save to card
*
*-bitmap scan order is lower left is the high freq with the lower right the low freq
* from bottom to top is the time axis
*- to achieve this incoming data needs to be reversed in blocks (height size)
*
*/
RecordStatus WriteBmpBlock(uint32_t blockAddress, int32_t blockCount)
{
  
  RecordSession_TypeDef *recordSession;
  RecordStatus retStatus; 
  int ret;
  uint8_t reverseArray[BMP_WIDTH];
  uint16_t tmp;
  uint16_t *pSrc;
  int i,j,k;
  static bool lock = FALSE;
  int dataCountWord;
  
  retStatus.status = OK;  //assume card will write correctly
  retStatus.fw_error = FR_OK; // reset fat error code
  
  if (lock) return retStatus;  
  
  recordSession = GetCurrentRecordSession();
  if ( (recordSession != (void*)0) && //if file session isn't void
     (recordSession->state == ACTIVE) && // if file session is currently active 
     (&recordSession->file ) )       // file handle is valid
  {
    lock = TRUE;
    
    dataCountWord = blockCount/2; //convert count from byte to word
    pSrc = (uint16_t*)blockAddress;
    i = j = k = 0; 
    while  (k < dataCountWord)
    {
      //reverse data  and truncate 16bit to 8bit
      for (i=0, j= (BMP_WIDTH-1);i< BMP_WIDTH; i++,j--)
      {
        tmp = pSrc[j+ k];
        if(tmp >= 256) tmp = 0xff; //saturating add
        reverseArray[i] = (uint8_t)tmp;
      }
    
      k+= BMP_WIDTH;
      
      //write data
      UINT byteswritten;
      ret =  f_write(&recordSession->file, (char*)reverseArray,BMP_WIDTH,&byteswritten); // returns data written in bytes
      if (byteswritten == (BMP_WIDTH)) //check if all bytes written, compare against blockCount(uint16_t)
      {
         recordSession->fileByteCount += BMP_WIDTH; //all bytes written correctly
         recordSession->bmpHeight = recordSession->fileByteCount/recordSession->bmpWidth;
      }
      else
      {
        recordSession->errorCode = FAIL_CARD_WRITE;
        retStatus.status  = CARD_ERROR; // assume card error
        retStatus.fw_error = f_close(&recordSession->file);
        if(retStatus.fw_error == 0) //if no error in closing file
        {
          
          ret = f_open(&recordSession->file,(const char*)recordSession->filename, FA_CREATE_ALWAYS | FA_WRITE | FA_READ); 
          if (!ret)  // if file still opens then finish file 
          {
            recordSession->errorCode = FAIL_CARD_FULL;
            retStatus.status  = CARD_FULL;
            //Modify header with current datacount                   
            retStatus.fw_error = CloseBmpFile(recordSession); //TODO update closerecording function
          } 
        }
        break;  //exit loop because of error
      }
    }
    
    //release lock
    lock = FALSE;
  }

  return retStatus;
}




uint8_t * GetBmpHeader(BMP_TypeDef *bmpHeaderInfo)
{
  static uint8_t bmpHeaderarray[BMP_HEADER_SIZE];
  
  *((uint16_t*)&bmpHeaderarray[0]) = 0x4D42; // "BM" ID field (42h, 4Dh)
  *((uint32_t*)&bmpHeaderarray[2]) = bmpHeaderInfo->datasize + BMP_COLORTABLE_SIZE + BMP_HEADER_SIZE; // 	Size of the BMP file
  *((uint16_t*)&bmpHeaderarray[6]) = 0; //Application specific
  *((uint16_t*)&bmpHeaderarray[8]) = 0; //Application specific      
  *((uint32_t*)&bmpHeaderarray[0x0A]) = BMP_COLORTABLE_SIZE + BMP_HEADER_SIZE; //colortable + header =  Offset where the pixel array (bitmap data) can be found
  *((uint32_t*)&bmpHeaderarray[0x0E]) = BMP_DIP_HEADER_SIZE; //Number of bytes in the DIB header (from this point)
  *((int32_t*)&bmpHeaderarray[0x12]) = bmpHeaderInfo->width;   //Width of the bitmap in pixels
  *((int32_t*)&bmpHeaderarray[0x16]) = bmpHeaderInfo->height;   //Height of the bitmap in pixels. Positive for bottom to top pixel order.
  *((uint16_t*)&bmpHeaderarray[0x1A]) = BMP_COLOR_PLANES;     //Number of color planes being used
  *((uint16_t*)&bmpHeaderarray[0x1C]) = BMP_BITSPERPIXEL;    //Number of bits per pixel
  *((uint32_t*)&bmpHeaderarray[0x1E]) = 0;   //no pixel array compression used
  *((uint32_t*)&bmpHeaderarray[0x22]) = bmpHeaderInfo->datasize; // Size of the raw bitmap data (including padding)
  *((int32_t*)&bmpHeaderarray[0x26]) = 0;   //Print resolution of the image W,
  *((int32_t*)&bmpHeaderarray[0x2A]) = 0;   //Print resolution of the image H,
  *((uint32_t*)&bmpHeaderarray[0x2E]) = 0;  //Number of colors in the palette
  *((uint32_t*)&bmpHeaderarray[0x32]) = 0;   //0 means all colors are important
  
  return bmpHeaderarray;
  
}






/***************************************************************************
* FIFO for fileSessions
* 
*
*/

#define NUM_OF_SESSION_FILES 8   //number of record instructions

RecordSession_TypeDef recordSessions[NUM_OF_SESSION_FILES];
int32_t readSessionIndex = 0;
int32_t writeSessionIndex = 0;
int32_t sessionCount = 0;


RecordSession_TypeDef * GetCurrentRecordSession(void)
{
  RecordSession_TypeDef *pRecordSession;
  
  pRecordSession = NULL;  //set pointer to null as default 
  if (sessionCount > 0) pRecordSession = &recordSessions[readSessionIndex];
  
  return pRecordSession;
}                    
                      
//adds file to pending session slot                      
bool AddRecordSession(RecordSession_TypeDef recordSession)
{
  bool recordAdded = FALSE;

  if (sessionCount < NUM_OF_SESSION_FILES)
  {
    sessionCount++;
    recordSession.fileByteCount = 0; 
    recordSession.errorCode = NO_ERROR;  
    
    recordSessions[writeSessionIndex] = recordSession;
    if (++writeSessionIndex >= NUM_OF_SESSION_FILES) writeSessionIndex = 0;
    recordAdded = TRUE;
  }
  
  return recordAdded;
}





void RemoveRecordSession(void)
{
    if(sessionCount > 0)
    {
        if(++readSessionIndex >= NUM_OF_SESSION_FILES) readSessionIndex =0;
        sessionCount--;
    }
}

void ClearSessions(void)
{
  readSessionIndex = 0;
  writeSessionIndex = 0;
  sessionCount = 0;
  
}


int32_t GetSessionCount(void)
{
 return sessionCount; 
}

/**********************************************************************************/

// Adds a file session for low sampling
// //rtc buffered time and date must be up to current
// 
// input - minutesToGo = number of minutes before stop time
//

bool AddRecordingSession(RecordSessionInit_TypeDef *sessionInit, bool isTest)
{
  
  RecordSession_TypeDef recordSession;
  char filenameStub[32];
  rtcDateTypeDef *pDate;
  rtcTimeTypeDef *pTime;
  struct tm datetime = {0};
  

  
  //check filetime is resonable
  if (!isTest && sessionInit->instruction != RECORD_BAT_176KHZ)
  {
    if (sessionInit->sessionTimeInSecs < MINIMUM_REC_TIME || sessionInit->sessionTimeInSecs > MAXIMUM_REC_TIME) return FALSE;
  }
  
  // copy of the init values to main struct
  recordSession.instruction = sessionInit->instruction;
  recordSession.startTime = sessionInit->startTime;  // time and date will either be current time or an estimated future time
  recordSession.startDate = sessionInit->startDate;  //
  
  pTime = &recordSession.startTime;
  pDate = &recordSession.startDate;
  
  
  
  //store the timeticks before file is created
  datetime.tm_sec = pTime->sec;
  datetime.tm_min = pTime->min;
  datetime.tm_hour = pTime->hour;
  
  datetime.tm_mday = pDate->date;
  datetime.tm_mon = pDate->month-1;
  datetime.tm_year = pDate->year + 100;   //
  //unknowns
  datetime.tm_isdst = -1;   //
  datetime.tm_wday = 0;   //
  datetime.tm_yday = 0;   //
  
  
  recordSession.origFileTimeTicks = mktime(&datetime);
  
  
  
  recordSession.protocolNumber = sessionInit->protocolNumber;
  recordSession.maxFileLengthInMilliSecs = sessionInit->sessionTimeInSecs * 1000; // convert secs to millisecs  // total file length of file in millisecs
  
  
  recordSession.state = PENDING;  // 
  recordSession.currentFileLengthInMilliSecs = 0;  // 
  //recordSession.file = NULL;  //no file created for it yet 
  memset(&recordSession.file, 0, sizeof(FIL));  //no file created for it yet  
  
  
  recordSession.errorCode = NO_ERROR;
  recordSession.dspEvent = DSP_NO_EVENT;
  
  
  
  
  recordSession.filename[0] = 0; // apply null char before strcat
  
  //survey name
  if (sessionInit->useSurveyName) 
  {
    strcat((char*)recordSession.filename,(const char*)GetSurveyName());
    strcat((char*)recordSession.filename,"_");
  }
  
  //station name
  if (sessionInit->useStationName) 
  {
    strcat((char*)recordSession.filename,(const char*)GetStationID());
    strcat((char*)recordSession.filename,"_");
  }
  
  // datetime stamp
  sprintf(filenameStub,"20%02u%02u%02u_%02u%02u%02u",  pDate->year, pDate->month, pDate->date, pTime->hour, pTime->min, pTime->sec); 
  strcat((char*)recordSession.filename, filenameStub);
  
  
  //check if recordingtype is valid for this setup
  if (recordSession.instruction == RECORD_LOW_8KHZ)
  {
    //create the filename based on time
    if (isTest)
      sprintf((char*)recordSession.filename,"record_low_test.wav");
    else
      strcat((char*)recordSession.filename, ".wav");
    
    recordSession.tag = GetFileTagNumber();
    recordSession.suspendMode = SLEEP_SUSPEND; 
    recordSession.samplingRate = 8000;
    recordSession.RecordCreate = CreateWavFile;   // when file is created it calls this function
    recordSession.RecordClose =  CloseWavFile;    //when file is closed it calls this function
    recordSession.RecordInterrupt = WriteWavBlock;  //when buffered data from dsp ready to write to card it calls this function
    recordSession.RecordEvent = NULL;  // 
  }
  else if (recordSession.instruction == RECORD_HIGH_32KHZ)
  {
    //create the filename based on time
    if (isTest)
      sprintf((char*)recordSession.filename,"record_high_test.wav");
    else
      strcat((char*)recordSession.filename, ".wav");
      
    recordSession.tag = GetFileTagNumber();
    recordSession.suspendMode = SLEEP_SUSPEND; 
    recordSession.samplingRate = 32000;
    recordSession.RecordCreate = CreateWavFile;   // when file is created it calls this function
    recordSession.RecordClose =  CloseWavFile;    //when file is closed it calls this function
    recordSession.RecordInterrupt = WriteWavBlock;  //when buffered data from dsp ready to write to card it calls this function
    recordSession.RecordEvent = NULL;  // 
  }
  else if (recordSession.instruction == RECORD_BAT_176KHZ)
  {
    if (isTest)
      sprintf((char*)recordSession.filename,"record_bat_test.wav");
    else
      strcat((char*)recordSession.filename, ".wav");
    
    recordSession.tag = GetFileTagNumber();
    recordSession.suspendMode = STOP_SUSPEND; 
    recordSession.samplingRate = 176000;  
    recordSession.bmpWidth = BMP_WIDTH;
    recordSession.bmpHeight = 0;        
    recordSession.RecordCreate = CreateWavFile;   // when file is created it calls this function
    recordSession.RecordClose =  CloseWavFile;    //when file is closed it calles this function
    recordSession.RecordInterrupt = WriteWavBlock;  //when buffered data from dsp ready to write to card it calls this function
    recordSession.RecordEvent = NULL;  // 
  }
  else if (recordSession.instruction == SUSPEND ||
           recordSession.instruction == SUSPEND_RANDOM)
  {
    recordSession.tag = 0;
    recordSession.suspendMode = SLEEP_SUSPEND; 
    recordSession.samplingRate = 0;
    recordSession.RecordCreate = NULL;   // when file is created it calls this function
    recordSession.RecordClose =  NULL;    //when file is closed it calls this function
    recordSession.RecordInterrupt = NULL;  //when buffered data from dsp ready to write to card it calls this function
    recordSession.RecordEvent = NULL;  // 
  }
  else  // something wrong
  {
    while(1) {} 
    recordSession.errorCode = INVALID_RECORD_TYPE;
    recordSession.suspendMode = NO_SUSPEND; 
    return FALSE; // not a valid recording type to create a file for
  }
    
  if (!AddRecordSession(recordSession)) { while(1) {} } 
  
  return TRUE;
}




// Changes the sampling mode of the dsp
//
//  - options are : low, high , bat, suspend
// - This is called when the sampling mode is changed 

void AddChangeDspSamplingSession(RecordSessionInit_TypeDef* sessionInit)
{
  RecordSession_TypeDef recordSession;

  recordSession.instruction = sessionInit->instruction;
    
  // if session is linked to previous session then used estimated time instead
  recordSession.startTime = sessionInit->startTime;  // time and date will either be current time or an estimated future time
  recordSession.startDate = sessionInit->startDate;
  
  recordSession.RecordCreate = NULL;   // when file is created it calls this function
  recordSession.RecordClose =  NULL;    //when file is closed it calls this function
  recordSession.RecordInterrupt = NULL;  //when buffered data from dsp ready to write to card it calls this function
    
   
  recordSession.state = PENDING;  // 
  recordSession.maxFileLengthInMilliSecs = 0;  // total file length of file in millisecs
  recordSession.currentFileLengthInMilliSecs = 0;  // 
  recordSession.protocolNumber = sessionInit->protocolNumber;
  //recordSession.file = NULL;  //no file created for it 
  memset(&recordSession.file, 0, sizeof(FIL));
   
  if (!AddRecordSession(recordSession)) { while(1) {} } 
}


bool IsCurrentSessionFree()
{
  RecordSession_TypeDef *currentSession;
 
  currentSession = GetCurrentRecordSession();
  
  return (bool)(currentSession==NULL);
}



//end session is used when end of recording is detected 
// - create end file session when recording stop time has been detected
void AddEndSession()
{
  RecordSession_TypeDef recordSession;

  recordSession.instruction = END_RECORDING;
  recordSession.state = FINISHED;
  //recordSession.file = NULL;
  memset(&recordSession.file, 0, sizeof(FIL));
  recordSession.suspendMode = NO_SUSPEND;
  
  recordSession.RecordCreate = NULL;   // don't create any file
  recordSession.RecordClose =  NULL;    //no file to close
  recordSession.RecordInterrupt = NULL;  //no file to buffer data to
  
  if (!AddRecordSession(recordSession)) { while(1) {}} 
}





/************************************************************
* GetNextRecordingInit
*
* - determines from the current time and protocol which recording mode to run next 
* - returns RecordSessionInit, this contains the initialisation variables for next recordSession 
*/
RecordSessionInit_TypeDef *GetNextRecordingInit(void)
{
  static RecordSessionInit_TypeDef  sessionInit;
  ProtocolTimeCheck_TypeDef *protocolTimeCheck;
  ProtocolSequence_TypeDef *protocolSequence;  
  static InstructionType lastInstruction = RESERVED;
  static int lastProtocolNumber = -1; // used to log when protocol changes
  
  rtcTimeTypeDef *newTime;
  rtcDateTypeDef *newDate;
  
  newTime = RtcGetTime(); //update buffered time
  newDate = RtcGetDate(); //update buffered date    
  
  //f_inittime(newTime);   //display this time and date in file metadata
  //f_initdate(newDate);   //
  get_fattime();
  //int ret;
  //ret=set_timestamp((char*)sessionInit.filename, newTime, newDate);
  
  
  sessionInit.startTime = *newTime;  //start time that instruction is run.
  sessionInit.startDate = *newDate;  //start date that instruction is run.
  
  
  // check if stop time coming up
  protocolTimeCheck = CheckProtocolTimeQuick(newTime); //
  
  
  
  //check if recording time is still valid and is above minimum record time
  if (protocolTimeCheck->isWithin && (protocolTimeCheck->secondsToGo > MINIMUM_REC_TIME) )
  {    
    //log the protocol number
    if (lastProtocolNumber != protocolTimeCheck->protocolNum)
    {
      AddToLog(LOG_PROTOCOL_ID,(uint8_t)protocolTimeCheck->protocolNum,0,0,NO_TIMESTAMP);
      lastProtocolNumber = protocolTimeCheck->protocolNum;
    }
    
    //Determine what file to create next based on the protocol sequence
    protocolSequence = GetNextProtocolSequence(protocolTimeCheck->protocolNum);   //
    
    sessionInit.protocolNumber = protocolSequence->protocolNumber;  // protocol  that decided this session
    sessionInit.sessionTimeInSecs = protocolSequence->timeInSec;  // time of session
    sessionInit.protocolName = protocolSequence->protocolName; // will be used for folder creation
    
    //custom options
    sessionInit.useSurveyName = HasProtocolOption(protocolSequence->protocolNumber, PROTOCOL_SURVEY); // for tier1 or other custom name
    sessionInit.useStationName = HasProtocolOption(protocolSequence->protocolNumber, PROTOCOL_STATION); //for tier1 only
    sessionInit.forceGpsLog = HasProtocolOption(protocolSequence->protocolNumber, PROTOCOL_GPS); //for tier1 only
    //use to add random name to log file
    AddRandomNameToLogFilename(HasProtocolOption(protocolSequence->protocolNumber, PROTOCOL_RANDOM_LOGNAME));
    
    //if recording method is different to last method then add instruction to change methods
    if (lastInstruction == protocolSequence->instruction)
    {
      sessionInit.instruction = protocolSequence->instruction;
    }
    else // different recording mode so change dsp modes
    {
      switch(protocolSequence->instruction)
      {
      case RECORD_LOW_8KHZ:
        sessionInit.instruction = CHANGE_LOW_SAMPLING;
        break;
      case RECORD_HIGH_32KHZ: 
        sessionInit.instruction = CHANGE_HIGH_SAMPLING;   
        break;
      case RECORD_BAT_176KHZ:
        sessionInit.instruction = CHANGE_BAT_SAMPLING;
        break;
      case SUSPEND:
        sessionInit.instruction = CHANGE_SUSPEND;
        break;
      case SUSPEND_RANDOM:
        sessionInit.instruction = CHANGE_SUSPEND_RANDOM;
        break;
      default:
        sessionInit.instruction = END_RECORDING;
        break;
      };        
      
      lastInstruction = protocolSequence->instruction;
    }
  }    
  else // if out of time range then instruct to exit recording
  {
    sessionInit.instruction = END_RECORDING;
    sessionInit.protocolNumber = -1;  // protocol  that decided this session
    sessionInit.sessionTimeInSecs = 0;  // time of session
  }
  
  return &sessionInit;
}






/**********************************************************
* Audio Record Session
*
* - is called if record session is either low or high sampling
* - creates file based on timestamp of record session
* - loops in sleep mode, wakes up to store data from dsp is to card
* - when finished closes file on card before exiting
*/ 
SessionExitCode_Typedef *AudioRecordSessionRun(RecordSessionInit_TypeDef* sessionInit)
{
  static SessionExitCode_Typedef exitCode;
  RecordSession_TypeDef *currentRecordSession;
  bool isTest = FALSE;
  
  if (sessionInit->protocolNumber == PROTOCOL_TEST) isTest = TRUE;  // if protocol number == -1  then indicates that this is a test recording
  //setup a recording session from the init variables, 
  // 
  if (!AddRecordingSession(sessionInit, isTest)) while(1) {} //loop here if error adding rec session- will be min/max session time out of range
    
  
  //get the recording session and create a file for it
  currentRecordSession = GetCurrentRecordSession();
  if (memcmp(&currentRecordSession->file, &(FIL){0}, sizeof(FIL)) == 0)
  {
    if ((currentRecordSession->state != ACTIVE) && (currentRecordSession->RecordCreate != NULL)) 
    {
      currentRecordSession->RecordCreate(currentRecordSession);             //create wav file for current session if not created
    }
    currentRecordSession->state = ACTIVE; // mark file as  active this will allow the dsp dma int routine to save data to file
  }

 // NOW WAIT FOR FILE TO FINISH, 
  while (currentRecordSession->state == ACTIVE &&
         currentRecordSession->errorCode == NO_ERROR &&
         !(exitCode.isButtonPushed = HasPageButtonTriggered()) &&
         (exitCode.powerOk = IsPowerOk()) )
  {
      /* Set SLEEPDEEP bit of Cortex System Control Register */
      WaitForInterrupt();
      ResetWatchdog();
  }
  
  //NOW CLOSE FILE
  if (memcmp(&currentRecordSession->file, &(FIL){0}, sizeof(FIL)) != 0) //if file present then close it
  {
    currentRecordSession->state = FINISHED;
    if (currentRecordSession->RecordClose != NULL)
      currentRecordSession->RecordClose(currentRecordSession);  //close the file now that it's finished
  }
  

RemoveRecordSession();// mark the session as finished 
//  
  exitCode.error = currentRecordSession->errorCode;
  return &exitCode;
}










/***********************************************************
*BatRecordSessionRun
*
* - this runs differently from the audio recordings, as many files can be created before exiting
* -- is in stop mode waiting for event from dsp
* -- wakes up from stop mode every 15 secs to check if still in recording mode
* -- if event from dsp then don't go into stop mode until file finished and closed
*/
SessionExitCode_Typedef *BatRecordSessionRun(RecordSessionInit_TypeDef* sessionInit)
{
  static SessionExitCode_Typedef exitCode;
  RecordSessionInit_TypeDef batSession;
  RecordSession_TypeDef *currentRecordSession;
  rtcTimeTypeDef *nowTime;
  rtcDateTypeDef *nowDate;
  bool isTest = FALSE;
  
  
  exitCode.error = NO_ERROR;
  
#ifndef DEBUG
  ReducePowerOnce();
#endif  
  
  Internal_RTC_Configuration(ENABLE,15);  // init the internal rtc to 15 sec wakeup
  DspEventInterrupt(ENABLE);  // enable interrupt for dsp events both falling and rising edge triggered
        
   
  
  
  nowTime = RtcGetTime();
  nowDate = RtcGetDate();
  
  // will need to calculate the stop time which is the start time + session time in secs
  while (IsWithinSessionTime(&sessionInit->startTime, nowTime, sessionInit->sessionTimeInSecs)    &&
         !(exitCode.isButtonPushed = HasPageButtonTriggered()) &&
         exitCode.error == NO_ERROR          &&
         (exitCode.powerOk = IsPowerOk()) == TRUE )
  {
    
    
    // reset the internal rtc clock to 0
    RTC_WaitForLastTask();  
    RTC_SetCounter(0);
    RTC_WaitForLastTask();   
    /* Clear RTC Alarm interrupt pending bit */
    RTC_ClearITPendingBit(RTC_IT_ALR);
    NVIC_EnableIRQ(RTCAlarm_IRQn);
    PWR_ClearFlag(PWR_FLAG_WU);
    
    // Enter stop mode and wait for wake from these:
    // - event from dsp
    // - pushbutton 
    // - internal rtc event
    PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI); 
    
    NVIC_DisableIRQ(RTCAlarm_IRQn); 
    ///SetSysClockToHSE();
    SYSCLKConfig_STOP();    //set clks after stop 
    
    //wake up from stop mode
    nowTime = RtcGetTime();
    nowDate = RtcGetDate();
    
    ResetWatchdog();
    //check if event line from dsp is active if so then create new session and 
    // wait for file to finish
    if (DSP_IsEvent(LEVEL) == TRUE)
    {
      batSession.instruction = sessionInit->instruction;
      batSession.startTime = *nowTime;  //start time that instruction is run. maybe actual or estimated time
      batSession.startDate = *nowDate;  //start date that instruction is run maybe actual or estimated date
      batSession.protocolNumber = sessionInit->protocolNumber;  // protocol  that decided this session
      batSession.sessionTimeInSecs = 0 ;  // this isn't used 
      batSession.useSurveyName = FALSE; // for tier1 or other custom name
      batSession.useStationName = FALSE; //for tier1 only  
      if (batSession.protocolNumber == PROTOCOL_TEST) isTest = TRUE;
      AddRecordingSession(&batSession, isTest);
      
      
      currentRecordSession = GetCurrentRecordSession();
      if (memcmp(&currentRecordSession->file, &(FIL){0}, sizeof(FIL)) == 0)
      {
        if ((currentRecordSession->state != ACTIVE) && (currentRecordSession->RecordCreate != NULL)) 
          currentRecordSession->RecordCreate(currentRecordSession);             //create file for current session if not created

        currentRecordSession->state = ACTIVE; // mark file as  active this will allow the dsp dma int routine to save data to file
      }
      
      ResetWatchdog();
      //NOW WAIT FOR FILE TO FINISH, 
      while (DSP_IsEvent(LEVEL) == TRUE)  // just to make sure we've detected end of event
      {
          
      }
      ResetWatchdog();
      currentRecordSession->state = FINISHED;
      if (memcmp(&currentRecordSession->file, &(FIL){0}, sizeof(FIL)) != 0) //if file present then close it
      {
        if (currentRecordSession->RecordClose != NULL)
          currentRecordSession->RecordClose(currentRecordSession);  //close the file now that it's finished
      }
      exitCode.error = currentRecordSession->errorCode;
      RemoveRecordSession();
      
      ResetWatchdog();
      
    }
    
  }
  
  Internal_RTC_Configuration(DISABLE,0);
  DspEventInterrupt(DISABLE);  // enable interrupt for dsp events both falling and rising edge triggered

  return &exitCode;
}

/********************************************************************************
* SuspendSessionRun
*
*
*/
SessionExitCode_Typedef *SuspendSessionRun(RecordSessionInit_TypeDef* sessionInit)
{
  RecordSession_TypeDef *recordSession;
  static SessionExitCode_Typedef exitCode;
  rtcTimeTypeDef *nowTime;
  int8_t compareLastSec; 
  //int rtcErrorCount = 0; // used to detect if rtc error
  
  AddRecordingSession(sessionInit, FALSE); // add record session, set isTest flag to false
  
  //get the recording session and create a file for it
  recordSession = GetCurrentRecordSession();
  
  recordSession->state = ACTIVE; //
  
  int sessionTime = sessionInit->sessionTimeInSecs;
  
  // randomise session time if suspend random mode selected
  //time 
  if (recordSession->instruction == SUSPEND_RANDOM) sessionTime = GetRandomValue(sessionTime); // 

  nowTime = RtcGetTime();  
  
  SysTickInterrupt(ENABLE);
  ResetWatchdog();
  //NOW WAIT FOR FILE TO FINISH, 
  while (IsWithinSessionTime(&sessionInit->startTime, nowTime, sessionTime) &&
         recordSession->errorCode == NO_ERROR &&
         !(exitCode.isButtonPushed = HasPageButtonTriggered()) &&
         (exitCode.powerOk = IsPowerOk()) )
  { 
    delay_ms(1000);
    //lastTime = *nowTime; // save the last time so we can compare it after
    compareLastSec = nowTime->sec;
    nowTime = RtcGetTime(); 
    
    //detect if rtc time isn't changing -As this is the only bad thing that can happen here
    if(nowTime->sec != compareLastSec) ResetWatchdog();
    
  }
  SysTickInterrupt(DISABLE);
  
  exitCode.error = recordSession->errorCode;
  RemoveRecordSession();
  
  return &exitCode;
}



#define SECS_IN_MIN 60
#define SECS_IN_HOUR 3600
#define HOURS_IN_DAY 24
#define HOURS_IN_SECS(x) (x*SECS_IN_HOUR)


bool IsWithinSessionTime(rtcTimeTypeDef *startTime,rtcTimeTypeDef *curTime, int sessionTimeInSecs)
{
  int startTimeInSecs;
  int stopTimeInSecs;
  int curTimeInSecs;
  bool isWithin = FALSE;
  
  startTimeInSecs = ((uint16_t)startTime->hour * SECS_IN_HOUR) + ((uint16_t)startTime->min * SECS_IN_MIN) + (uint16_t)startTime->sec;	
  stopTimeInSecs = startTimeInSecs + sessionTimeInSecs;
  if (stopTimeInSecs >= HOURS_IN_SECS(HOURS_IN_DAY)) stopTimeInSecs -= HOURS_IN_SECS(HOURS_IN_DAY); 
  
  curTimeInSecs = ((uint16_t)curTime->hour * SECS_IN_HOUR) + ((uint16_t)curTime->min * SECS_IN_MIN) + (uint16_t)curTime->sec;	 

    // decision depends on if the stop time is before or after the start time (if survey passes through midnight)
    if (startTimeInSecs > stopTimeInSecs)		// so survey passes across midnight
    {
      if ((curTimeInSecs >= startTimeInSecs) || (curTimeInSecs < stopTimeInSecs)) 
      {
        isWithin = TRUE;
      }
    }
    else		
    {
      if((curTimeInSecs >= startTimeInSecs) && (curTimeInSecs < stopTimeInSecs)) 
      {
        isWithin = TRUE;
      }
    }
    
    return isWithin;
}







//mark filename time offsets           
// - used to rename files to their correct time after recorder has finished.         
// - there is a delay between when file is created and data is written so the filename will need to be changed later            

//filetimeoffset = offset from base time , this should match the timestamp of 
void MarkFileRenameTimeOffsets(time_t fileCreateTicks, time_t recStartTicks)
{
  TimeOffset_TypeDef timeOffsets;
  uint32_t x,y;
   
  if (ftOffsetIndex >= MAX_FILE_TIME_OFFSETS) return;
  
  x = (fileCreateTicks - baseTime);
  timeOffsets.baseTicks = (uint16_t)x; // it doesn't matter if value greter than 65536, should rarely clock over
  
  y = recStartTicks - fileCreateTicks;
  timeOffsets.offsetTicks =  y < 256 ? (uint8_t)y : 255;  // will be under 256 as it shouldn't take this long to write a card. but if it is then just set to 255
  
  fileTimeOffsets[ftOffsetIndex++] = timeOffsets; //
}






//set the base time point from the current datetime from RTC
// this is used as a reference point 
void SetBaseTimePoint()
{
  baseTime = GetRtcTimeInTicks();

  //clear offset
  ftOffsetIndex = 0; 
}

//used in calculation for rename files
//is difference between the rtc time and gps syncd time
//because files were named with rtc time we can adjust filenames to correct sync time after
void SetGpsAdjustOffset(int32_t offset)
{
  gps_adjust_offset = offset;
  
  
}



//
void BulkRenameFiles()
{
  uint32_t btime = baseTime;
  uint16_t lastOffset = 0; /// used to detect an overflow of the 16bit offset
  time_t timestamp;
  struct tm *timeinfo;
  uint8_t filename_org[256];
  uint8_t filename_new[256];
  int ret;
  int error_count = 0;
  int protoIndex = 0;
  bool openFolderFlag = TRUE;
  int fileCount = 0;
  //if (OpenSDCard()) return;

  //get the first protocol used so we can determine which folder to open
  if (protocolsUsed == 0) return;
  
  
  
  
  
  //scan through all offsets
  for (int i=0; i < ftOffsetIndex; i++)
  {
    
    //open folder 
    if ( openFolderFlag && protoIndex < protocolsUsed )
    {
        uint8_t curDir[32];
        bool success = TRUE;
        int ret;
        
        // first go back to root directory
        ret = f_getcwd( (char*)curDir, 32 );
        if (ret) success = FALSE;
        
        if (strlen((char*)curDir) > 1)
        {
          f_chdir(".."); //go back one 
          f_chdir(".."); // go back two dir
        }
        //open folder named after protocol 
        ret = f_chdir((char*)rFolders[protoIndex].rootFolderName);
        if (ret) success = FALSE;
  
  
        //open folder based on the date recording started
        ret = f_chdir((char*)rFolders[protoIndex].localFolderName);
        if (ret) success = FALSE;
        
        openFolderFlag = FALSE;
        
        fileCount = rFolders[protoIndex].numberOfFiles;
        protoIndex++; 
      }
    
      //check if overflow since last offset 
      if (lastOffset > fileTimeOffsets[i].baseTicks) btime += (UINT16_MAX+1); 
      lastOffset = fileTimeOffsets[i].baseTicks;
       
      //get original filename
      timestamp = (time_t)(fileTimeOffsets[i].baseTicks + btime);
      timeinfo = localtime(&timestamp);
      
      sprintf((char*)filename_org,"%04u%02u%02u_%02u%02u%02u.wav",  
              timeinfo->tm_year + 1900, // base year 1900
              timeinfo->tm_mon+1, //index starts at 1
              timeinfo->tm_mday,
              timeinfo->tm_hour,
              timeinfo->tm_min,
              timeinfo->tm_sec
             );
      
      //get new filename
      timestamp = (time_t)(fileTimeOffsets[i].baseTicks + btime + fileTimeOffsets[i].offsetTicks + gps_adjust_offset);
      timeinfo = localtime(&timestamp);
      
      sprintf((char*)filename_new,"%04u%02u%02u_%02u%02u%02u.wav",
              timeinfo->tm_year + // base year 1900 
              timeinfo->tm_mon+1, //index starts at 1
              timeinfo->tm_mday,
              timeinfo->tm_hour,
              timeinfo->tm_min,
              timeinfo->tm_sec
             );
      
      
      if (strcmp((const char*)filename_org, (const char*)filename_new) != 0)
      {
        ret = f_rename((const char*)filename_org,(const char*)filename_new);
        if (ret) error_count++;
      }
      
      
      //dec filecount counter to determine if number of files created per protocol have been used up
      fileCount--;
      if (fileCount == 0) openFolderFlag = TRUE;
  }
  


  //
  
  
  //CloseSdCard();


  
}






// test function

void TestRename()
{
  
  uint32_t initTicks, startTicks;

  SetBaseTimePoint();
   
  delay_ms(8000);
  initTicks = GetRtcTimeInTicks();
  delay_ms(4000);
  startTicks = GetRtcTimeInTicks();
  MarkFileRenameTimeOffsets(initTicks, startTicks);
  
  
  delay_ms(13000);
  initTicks = GetRtcTimeInTicks();
  delay_ms(5000);
  startTicks = GetRtcTimeInTicks();
  MarkFileRenameTimeOffsets(initTicks, startTicks);  
  
  BulkRenameFiles();
}



DWORD get_fattime (void)
{ 
  
  rtcTimeTypeDef *newTime;
  rtcDateTypeDef *newDate;
  
  newTime = RtcGetTime();
  newDate = RtcGetDate();
  
  
  int hour = newTime->hour;
  int min = newTime->min;
  int sec = newTime->sec;
  int month = newDate->month;
  int mday = newDate->date;
  int year = newDate->year;
  year+=2000;

    return (DWORD)(year - 60) << 25 |
           (DWORD)(month) << 21 |
           (DWORD)mday << 16 |
           (DWORD)hour << 11 |
           (DWORD)min << 5 |
           (DWORD)sec >> 1;
}














/*************************************************EOF*******************************************************/