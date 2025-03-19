



#ifndef PROTOCOL_H
#define PROTOCOL_H


#include "stm32f10x.h"
#include "record.h"
#include "menu.h"


#define MAX_NUM_OF_PROTOCOL_SLOTS 6
#define NUM_OF_VIEWABLE_PROTOCOL_SLOTS 6  //number of slots that show on menu
#if NUM_OF_VIEWABLE_PROTOCOL_SLOTS>MAX_NUM_OF_PROTOCOL_SLOTS
  #error "MAX NUMBER OF VIEWABLE SLOTS GREATER THAN ACTUAL SLOTS"
#endif

#define DEFAULT_NUM_PROTOCOL_SLOTS 2

#define ALL_PROTOCOLS 0
#define PROTOCOL_1 1
#define PROTOCOL_2 2


#define PROTOCOL_NAME_LENGTH 14 //12 bytes for name (to fit in lcd screen)
#define PROTOCOL_NUM_OF_COMMANDS 6

#define PROTOCOL_MAX_NUM_OF_OPTIONS 8 // max number of options to make certain menus available. eg for TIER1 protocol we'd want only need two - "station select" and "survey name".

#define PROTOCOL_USER_DEFINED_TIME ((int8_t)-1)  // used to indicate that the start and span times are stored in eeprom


#define PROTOCOL_TEST -1

#define PROTOCOL_OFF_INDEX 0 // the "off protocol will be always be in the first index





#define TIME_IN_SECS(x) x
#define TIME_IN_MINUTES(x) (x*60)

//used for quicklist
typedef struct
{
  int start_in_mins;
  int stop_in_mins;
} Protocol_StartStop_TypeDef;
  

typedef struct {
  bool isWithin;  //is within the time
  int minutesToGo; //minutes till stop time if within time else -1
  int minutesElapsed; // minutes from start time if within time else -1
  int secondsToGo;
  int secondsElapsed;
  int protocolNum; //the protocol num that the time was within , (protocol start stop number). else -1  
  
} ProtocolTimeCheck_TypeDef;


typedef struct
{   
  int8_t startHour;
  int8_t startMin;
  
  int8_t spanHour;
  int8_t spanMin;
  
  int16_t startOffset;
  int16_t stopOffset;
    
} Protocol_UserDef_Start_Span_TypeDef;





typedef enum { 
  COMMAND_RECORD_LOW_8KHZ,
  COMMAND_RECORD_HIGH_32KHZ,
  COMMAND_RECORD_BAT_176KHZ,
  COMMAND_SUSPEND,
  COMMAND_SUSPEND_RANDOM,
  COMMAND_RECORD_USER_DEF,
  COMMAND_RESET_SEQUENCE
} CommandType;


//extra options that can be applied to the protocol 
// custon code will have to be written for each option
typedef enum{
  PROTOCOL_END_OPTIONS,  // indicates that there is none/no_more options for the protocol 
  PROTOCOL_STATION,  // for TIER1 - indicates that that the menu will display the change station option. also the option will be inserted on to the filenames of the recordings 
  PROTOCOL_SURVEY,    // for TIER1 or custom protocol - indicates that that the menu will display the change survey option. also the survey name will be inserted on to the filenames of the recordings
  PROTOCOL_GPS,      //for gps logging, greates log file with gps coordinates in it
  PROTOCOL_TEMP,      //for temp logging
  PROTOCOL_RANDOM_DELAY, // used to set a random delay period upon startup
  PROTOCOL_RANDOM_LOGNAME //appends a random name to the logfile, should be different for each recorder 
}Protocol_Options_TypeDef;


typedef struct
{
  CommandType command; 
  uint16_t timeInSec; 
}Protocol_Command_TypeDef;


typedef struct
{ 
  Protocol_Options_TypeDef options[PROTOCOL_MAX_NUM_OF_OPTIONS];   //this allows making certain menus available for the protocol
  Protocol_UserDef_Start_Span_TypeDef times;
  Protocol_Command_TypeDef commands[PROTOCOL_NUM_OF_COMMANDS];
  uint8_t name[PROTOCOL_NAME_LENGTH];
} Protocol_Struct_TypeDef;



typedef struct 
{
  InstructionType instruction;
  int timeInSec;   // time of file in secs, or for random suspend = max random time    
  int protocolNumber; // reference index of protocol for which file was created for   
  uint8_t* protocolName;
}ProtocolSequence_TypeDef;



void Protocol_Init(void);
ProtocolSequence_TypeDef * GetNextProtocolSequence(int protocolIndex);


uint8_t *GetNameOfProtocol(int protocolListIndex, bool centered);


void SaveProtocolSlot(int slotIndex);
Protocol_Struct_TypeDef *GetProtocolFromSlot(int slotIndex);
void IncProtocolInSlot(int slotIndex);
void DecProtocolInSlot(int slotIndex);


bool IsProtocolStartSpanLocked(int protocolNum);


void WriteProtocolStartSpanTime(int protocolNum, Protocol_UserDef_Start_Span_TypeDef *startSpanItem);
Protocol_UserDef_Start_Span_TypeDef *ReadProtocolStartSpanTime(int protocolNum);


//PROTOCOL TIME CHECK FUNCTIONS
ProtocolTimeCheck_TypeDef *CheckProtocolTime(rtcTimeTypeDef *rtcTime);
ProtocolTimeCheck_TypeDef *IsWithinProtocolOnTime(int8_t currentHour, int8_t currentMin, int protocolNum);
void FillProtocolStartStopList(void);
ProtocolTimeCheck_TypeDef *CheckProtocolTimeQuick(rtcTimeTypeDef * currentTime);
Protocol_UserDef_Start_Span_TypeDef *GetNextActiveProtocolStartSpanTime(rtcTimeTypeDef * currentTime);

void SetProtocolToSlot(int slotIndex, int protocolNum);

void IncUserDefSamplingMode(void);
void DecUserDefSamplingMode(void);
uint8_t *GetUserDefSamplingModeName();


bool HasAnyProtocolOption(Protocol_Options_TypeDef optionType);
bool HasProtocolOption(int protocolNumber, Protocol_Options_TypeDef optionType);




void WriteProtocolNameToLogFile(RecordSessionInit_TypeDef * session);



int GetNumOfProtocolSlots();
void SetNumOfProtocolSlots(int slots);

int GetIndexOfProtocolFromName(uint8_t* name);

#endif //_PROTOCOL_H



