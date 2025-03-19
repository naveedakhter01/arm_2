/***************************************************************************
*  record.h     
*  16/10/09
*
***************************************************************************/

#ifndef RECORD_H
#define RECORD_H


#include "stm32f10x.h"
#include "fat/ff.h"
#include "fat/diskio.h"
#include "rtc.h"
#include "hardware.h"
#include <time.h>

#define SRAMBUFFERSIZE  0x100000  // 1Mb
#define ACTIVEBUFSIZE   0x0EA600  //% of full buffer size     0x0C0000 = %75
                                  //10secs= 9c400, 12 secs =BB800  



// top half of sram( top 32kb),  where we will DMA data from DSP to.
#define SAMPLE_BUFFER_ADDR ((uint32_t)0x20008000) 
#define SAMPLE_BUFFER_SIZE ((int32_t)32000) //equal to one second on low sampling 32/2 (2 half buffers) = 16kb, 8khz @ 16bit(2 bytes) = 16kb. so one second to fill one half buffer  

#define SAMPLE_BUFFER_HALF_SIZE   (SAMPLE_BUFFER_SIZE/2) 
#define SAMPLE_BUFFER_FIRSTHALF   SAMPLE_BUFFER_ADDR
#define SAMPLE_BUFFER_SECONDHALF  (SAMPLE_BUFFER_ADDR + SAMPLE_BUFFER_HALF_SIZE)


#define MINIMUM_REC_TIME  14//(1*60)//1min  in seconds  //min
#define MAXIMUM_REC_TIME  (60*60)//60min  in secs 

//WAV DEFINES
/* Audio Parsing Constants */
//#define  ChunkID             0x52494646  /* correspond to the letters 'RIFF' */
//#define  FileFormat          0x57415645  /* correspond to the letters 'WAVE' */
//#define  FormatID            0x666D7420  /* correspond to the letters 'fmt ' */
//#define  DataID              0x64617461  /* correspond to the letters 'data' */
//#define  FactID              0x66616374  /* correspond to the letters 'fact' */

#define  WAVE_FORMAT_PCM     0x01
#define  FormatChunkSize     0x10
#define  Channel_MONO        0x01
#define  Channel_STEREO      0x02

#define  SampleRate_8000     8000
#define  SampleRate_16000    16000
#define  SampleRate_22050    22050
#define  SampleRate_44100    44100
#define  SampleRate_48000    48000
#define  Bits_Per_Sample_8   8
#define  Bits_Per_Sample_16  16





typedef enum
{
  EMPTY = 0,
  NOTEMPTY,
  FULL,
  DONE                        
}bufSTATE;	


typedef struct 
{
  uint16_t baseTicks; // when the file is being created
  uint8_t offsetTicks;   // when the recording actually starts
} TimeOffset_TypeDef;


/** 
  * @brief  Configuration Mode enumeration  
  */

typedef enum
{ 
  WAV_SAMPLERATE_8000 = 8000,
  WAV_SAMPLERATE_16000 = 16000,
  WAV_SAMPLERATE_22050 = 22050,
  WAV_SAMPLERATE_44100 = 44100,
  WAV_SAMPLERATE_48000 = 48000  
}WAVSampleRate_TypeDef;

typedef enum
{ 
  WAV_MONO = 0x01,
  WAV_STEREO = 0x02
}WAVNumChannels_TypeDef;

typedef enum
{ 
  WAV_BITS_PER_SAMPLE_8 = 8,
  WAV_BITS_PER_SAMPLE_16 = 16
}WAVBitsPerSample_TypeDef;

/** 
  * @brief  GPIO Init structure definition  
  */

typedef struct
{  
  uint16_t WAV_SampleRate; //specifies sample rate of wav data
    WAVNumChannels_TypeDef WAV_NumChannels; //A value of 1 means a mono signal, a value of 2 means a stereo signal
    WAVBitsPerSample_TypeDef WAV_BitsPerSample;   //This value specifies the number of bits used to define each sample. This value is usually 8, 16, 24 or 32.
    int  WAV_DataSize;

}WAVE_InitTypeDef;


/********************************************/
//WAV FILE FORMAT

#define WAVEHEADERSIZE 44           // 44 bytes for wave header without extra data

/* Format chunk */
typedef struct
{
  int   FmtChunkID;
  int   FmtChunkSize; 
  short CompressionCode ;
  short NumChannels;
  int   SampleRate;
  int   ByteRate;
  short BlockAlign;
  short BitsPerSample;
} WAVE_FormatTypeDef;


/* Wave file chunk */
typedef struct
{
  int  ChunkID;
  int  ChunkSize; 
  int  RIFFType;
  WAVE_FormatTypeDef WAVE_Format;
  int  DATAChunkID;
  int  DATASize;
} WAVE_RIFF_TypeDef;



/************************************************************************
* BITMAP DATA
*
*/

#define BMP_HEADER_SIZE 54
#define BMP_COLORTABLE_SIZE 0x400
#define BMP_DIP_HEADER_SIZE 40
#define BMP_COLOR_PLANES 1
#define BMP_BITSPERPIXEL 8
#define BMP_WIDTH 64

typedef struct
{                 
    uint32_t width;
    uint32_t height; 
    uint32_t datasize; 
} BMP_TypeDef;


typedef struct {
  bool isWithin;  //is within the time
  int minutesToGo; //minutes till stop time if within time else -1
  int minutesElapsed; // minutes from start time if within time else -1
  int secondsToGo;
  int secondsElapsed;
  int startstopIndex; //the start stop item number if within time , (protocol start stop number). else -1  
  
} RecordTimeCheck_TypeDef;





enum 
{
 OK, 
 CARD_ERROR,
 CARD_FULL
};


typedef struct 
{
  int status;  //ok, error
  int fw_error; // fat32 file system error code
} RecordStatus;



typedef enum
{
  NO_ERROR = 0,
  FAIL_FILE_CREATE,
  FAIL_FILE_CLOSE,
  FAIL_CARD_WRITE,
  FAIL_CARD_WRITE_ITEMS,
  FAIL_FILE_DELETE,
  FAIL_CARD_FULL,
  FAIL_FILE_RENAME,
  DSP_WAIT_FOR_EVENT_FAIL,
  DSP_COMMAND_MODE_FAIL,
  INVALID_RECORD_TYPE,
  FAIL_RTC_TIME      // if time from rtc doen't change
    
    
} SessionError;         //MUST MATCH ERROR CODE STRINGS IN THE .c FILE
  


//DSP filter mode options - THIS MUST MATCH ENUM IN DSP SOFTWARE
typedef enum{
	DSP_RESERVED = 0,
	DSP_LOW_FILTER,
	DSP_HIGH_FILTER,
	DSP_BAT_FILTER,
	DSP_NO_FILTER	
} DspSamplingOption;






typedef enum {
  RESERVED,
  RECORD_LOW_8KHZ,
  RECORD_HIGH_32KHZ,          
  RECORD_BAT_176KHZ,
  CHANGE_LOW_SAMPLING,
  CHANGE_HIGH_SAMPLING,
  CHANGE_BAT_SAMPLING,
  SUSPEND,
  SUSPEND_RANDOM,    
  CHANGE_SUSPEND,
  CHANGE_SUSPEND_RANDOM,
  END_RECORDING,
  END_RECORDING_HIGH_PRIORITY,
} InstructionType;


enum {
 FILE_BEGIN = 0,
 FILE_MIDDLE,
 FILE_END, 
};



typedef enum {
  PENDING,
  ACTIVE,
  FINISHED
} InstructionState;


typedef enum {
  NO_SUSPEND,
  SLEEP_SUSPEND,
  STOP_SUSPEND
} SuspendMode_Typedef;

  




typedef struct RecordSessionInit{
  InstructionType instruction;
  rtcTimeTypeDef startTime;  //start time that instruction is run.
  rtcDateTypeDef startDate;  //start date that instruction is run.
  int protocolNumber;  // protocol index that decided this session
  int sessionTimeInSecs;  // time of session
  bool useSurveyName; // for tier1 or other custom name
  bool useStationName; //for tier1 only  
  bool forceGpsLog;
  uint8_t* protocolName;
} RecordSessionInit_TypeDef;




typedef struct RecordSession{
  
  InstructionType instruction;
  InstructionState state;
  
  int32_t maxFileLengthInMilliSecs;  // the total time in millisec of the file. once the currentfileLengthInMilliSecs value matchs this value then file is closed
  int32_t currentFileLengthInMilliSecs;   // every time the sampling buffer is written to card this increments by the bufferLengthInMilliSecs value.
  int32_t bufferLengthInMilliSecs; //the amount of time the dma buffer ( half buffer) takes to fill. depends on the data rate sent from dsp.
  


  DspEvent_TypeDef dspEvent;
  rtcTimeTypeDef startTime;  //start time that instruction is run. maybe actual or estimated time
  rtcDateTypeDef startDate;  //start date that instruction is run maybe actual or estimated date

  int protocolNumber; 
  int tag;
  SessionError errorCode;
  SuspendMode_Typedef suspendMode;  // this indicates what suspend mode to put ARM into while waiting for incoming data.
  
  
  int samplingRate;
  
  time_t origFileTimeTicks; // ticks
  
  //bat specific 
  int32_t bmpWidth; //bitmap
  int32_t bmpHeight; //bitmap
  
  //
  FIL file;     //store the file struct used by the fat system
  int32_t fileByteCount; //stroes the amount of bytes written to the card
 //functions specific to file recording will
  RecordStatus (*RecordInterrupt)(uint32_t, int32_t );  // file  interrupt function , is called when dma'd sampling data is ready to be written to card
  int (*RecordCreate)(struct RecordSession *); // record create function, is called when file is to be created
  int (*RecordClose)(struct RecordSession *); //record close function, is called when file is to be opened
  int (*RecordEvent)(struct RecordSession *); // used for bat recording will run this function when dsp sends event trigger.
  
  uint8_t filename[40];
  
  
}RecordSession_TypeDef;





typedef struct
{
  bool isButtonPushed;
  bool powerOk;
  SessionError error;
  
}SessionExitCode_Typedef;




bufSTATE StoreSample(int16_t sample);
bufSTATE StoreSamples(uint16_t* samples);
void SetStopRecFlag(FlagStatus StopFlag);
 
void RecInit(uint16_t SampleRate);


int CreateFile(void);
WAVE_RIFF_TypeDef* MakeWAVheader(WAVE_InitTypeDef* WAVE_Init);
void SetSDWriteFlag(BitAction NewState);
BitAction GetSDWriteFlag(void);



uint8_t CheckRecordTime(void);

void SetNextAlarm(void);

void GetDateString(uint8_t* pString);
void SetFileName(void);

void SwitchFastClock(void);
void SwitchSlowClock(void);

//int CheckCard(void);
int FormatCard(void);


int WriteBatteryLog(float* buffer, uint16_t count);

//temp functions
void PwrOFF(void);


uint8_t* GetErrorString(SessionError errorCode);


//WAV ROUTINES
int CreateWavFile(RecordSession_TypeDef *recordSession);
int OpenSDCard(void);
int CloseSdCard(void);
int CreateWavDirectory(void);
int CreateTestDirectory(void);
int CloseWavFile(RecordSession_TypeDef *recordSession);
RecordStatus WriteWavBlock(uint32_t blockAddress, int32_t blockCount);
bool ChangeDirectory(uint8_t *dirName);




//BMP ROUTINES
void FillGreyScaleTable(uint32_t* table);
int CreateBmpFile(RecordSession_TypeDef *recordSession);

int CloseBmpFile(RecordSession_TypeDef *recordSession);

RecordStatus WriteBmpBlock(uint32_t blockAddress, int32_t blockCount);
uint8_t * GetBmpHeader(BMP_TypeDef *bmpHeaderInfo);


void CreateTimeStampFileName(uint8_t * filename);
bool IsWithinSessionTime(rtcTimeTypeDef *startTime,rtcTimeTypeDef *curTime, int sessionTimeInSecs);


//FIFO FILE SESSIONS
RecordSession_TypeDef * GetCurrentRecordSession(void);
RecordSession_TypeDef * GetNextRecordSession(void);                 
bool AddRecordSession(RecordSession_TypeDef recordSession);
void RemoveRecordSession(void);
void ClearSessions(void);
int32_t GetSessionCount(void);

bool IsCurrentSessionFree();

bool AddRecordingSession(RecordSessionInit_TypeDef *sessionInit, bool isTest);
void AddEndSession();

void AddChangeDspSamplingSession(RecordSessionInit_TypeDef* sessionInit);

void EnableRecordingSpi(void);
void DisableRecordingSpi(void);


SessionExitCode_Typedef *ChangeSamplingMode(RecordSessionInit_TypeDef* sessionInit);

SessionExitCode_Typedef *AudioRecordSessionRun(RecordSessionInit_TypeDef* sessionInit);
SessionExitCode_Typedef *BatRecordSessionRun(RecordSessionInit_TypeDef* sessionInit);
SessionExitCode_Typedef *SuspendSessionRun(RecordSessionInit_TypeDef* sessionInit);

RecordSessionInit_TypeDef *GetNextRecordingInit(void);


void SetFolderNameByDate(void);

void SetGpsAdjustOffset(int32_t offset);




void MarkFileRenameTimeOffsets(time_t fileCreateTicks, time_t recStartTicks);
void SetBaseTimePoint();
void BulkRenameFiles();
void TestRename();

void SaveFolderNamesForRename(uint8_t *rootFolderName, uint8_t *localFolderName);
void IncFileCountForRename();



DWORD get_fattime (void);

#endif



/********************************************EOF**************************************************/
