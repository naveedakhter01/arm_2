/**
  ******************************************************************************
  * @file    protocol.c 
  ******************************************************************************
  * deals with the recording protocols
  *
  *
*/ 

#include "protocol.h"
#include "eeprom.h"
#include "log.h"
#include <stdio.h>




#define MAX_PROTOCOLS_IN_LIST 100


#define PROTOCOL_START_LENGTH 2 //
#define PROTOCOL_SPAN_LENGTH 2 //
#define PROTOCOL_START_OFFSET_LENGTH 2 // signed 16 bit 
#define PROTOCOL_STOP_OFFSET_LENGTH 2 // signed 16 bit

#define PROTOCOL_COMMAND_LENGTH  1
#define PROTOCOL_COMMAND_DATA_LENGTH 2



#define MENUSAMPLINGOPTIONS_FIRST ((InstructionType)RECORD_LOW_8KHZ) // min range for menu sampling options
#define MENUSAMPLINGOPTIONS_LAST ((InstructionType)RECORD_BAT_176KHZ) // max 3 options for user defined sampling . low, high and bat

typedef struct
{
 int index;
 bool isLoaded;
} Protocol_Slot_Typedef;

static int numOfProtocolSlots = DEFAULT_NUM_PROTOCOL_SLOTS;  //DEFAULT _NUM_OF_PROTOCOL_SLOTS;


//the new protocol Structure
static Protocol_Slot_Typedef protocolSlot[MAX_NUM_OF_PROTOCOL_SLOTS] = { {.isLoaded = FALSE},
                                                                     {.isLoaded = FALSE},
                                                                     {.isLoaded = FALSE},
                                                                     {.isLoaded = FALSE},
                                                                     {.isLoaded = FALSE},
                                                                     {.isLoaded = FALSE}
                                                                    };



static InstructionType userDefinedSamplingMode; //copy of value stored in eeprom. when protocol is set to manual then use this sampling mode. 
static bool isUserDefSamplingModeLoaded = FALSE;


static Protocol_StartStop_TypeDef startStopList[MAX_NUM_OF_PROTOCOL_SLOTS]; //used for quicktime check
static bool isStartStopListValid = FALSE;


Protocol_Struct_TypeDef protocolList[] = { 
/*********************START OF PROTOCOLS************************************/  

  //OFF PROTOCOL 
  // - start and span is set to 0
  // - no recording
  { .name = "Off", 
    .times.startHour = 0, 
    .times.startMin = 0,
    .times.spanHour = 0,
    .times.spanMin = 0,
    .times.startOffset = 0,
    .times.stopOffset = 0,
    .commands = { //COMMAND SEQUENCE -UP TO SIX COMMANDS
                    { COMMAND_RECORD_LOW_8KHZ , TIME_IN_MINUTES(15) } , 
                    { COMMAND_RESET_SEQUENCE , 0}  
                 },
    .options = { //additional menus to add to menu screen 
                    PROTOCOL_END_OPTIONS 
                }
  },  
  
/***********************************************************************/
  //LOW PROTOCOL
  // - start and span is defined by user
  // - no start stop offsets
  // - sampling type is defined by user
  // - sequence creates 15 min files 
  { .name = "Low", 
    .times.startHour = PROTOCOL_USER_DEFINED_TIME,
    .times.startMin = PROTOCOL_USER_DEFINED_TIME,
    .times.spanHour = PROTOCOL_USER_DEFINED_TIME,
    .times.spanMin = PROTOCOL_USER_DEFINED_TIME,
    .times.startOffset = 0,
    .times.stopOffset = 0,
    .commands = { //COMMAND SEQUENCE -UP TO SIX COMMANDS
                    { COMMAND_RECORD_LOW_8KHZ , TIME_IN_MINUTES(15)} ,  // user defined recording type , 15 min file 
                    { COMMAND_RESET_SEQUENCE, 0 }  , // repeat  sequence
                 },
    .options = { //additional menus to add to menu screen 
                    PROTOCOL_END_OPTIONS 
                }    
  },  
/***********************************************************************/
  //HIGH PROTOCOL
  // - start and span is defined by user
  // - no start stop offsets
  // - sampling type is set to high sampling mode
  // - sequence creates 15 min files 
  { .name = "High", 
    .times.startHour = PROTOCOL_USER_DEFINED_TIME,
    .times.startMin = PROTOCOL_USER_DEFINED_TIME,
    .times.spanHour = PROTOCOL_USER_DEFINED_TIME,
    .times.spanMin = PROTOCOL_USER_DEFINED_TIME,
    .times.startOffset = 0,
    .times.stopOffset = 0,
    .commands = { //COMMAND SEQUENCE -UP TO SIX COMMANDS
                    { COMMAND_RECORD_HIGH_32KHZ , TIME_IN_MINUTES(15)} ,  // user defined recording type , 15 min file 
                    { COMMAND_RESET_SEQUENCE, 0 }  , // repeat  sequence,
                 },
    .options = { //additional menus to add to menu screen 
                     PROTOCOL_END_OPTIONS 
                }    
  },  
/***********************************************************************/

  //BAT PROTOCOL
  // - start and span is defined by user
  // - no start stop offsets
  // - sampling type set to record bats
  // - sequence creates 15 min files 
  { .name = "Bat", 
    .times.startHour = PROTOCOL_USER_DEFINED_TIME,
    .times.startMin = PROTOCOL_USER_DEFINED_TIME,
    .times.spanHour = PROTOCOL_USER_DEFINED_TIME,
    .times.spanMin = PROTOCOL_USER_DEFINED_TIME,
    .times.startOffset = 0,
    .times.stopOffset = 0,
    .commands = { //COMMAND SEQUENCE -UP TO SIX COMMANDS
                    { COMMAND_RECORD_BAT_176KHZ , TIME_IN_MINUTES(5)} ,  // 5 mins of waiting time, will check if in record time after
                    { COMMAND_RESET_SEQUENCE, 0 }  , // repeat  sequence
                 },
    .options = { //additional menus to add to menu screen 
                     PROTOCOL_TEMP,
                     PROTOCOL_END_OPTIONS 
                }    
  },  

/***********************************************************************/
  //MANUAL PROTOCOL
  // - start and span is defined by user
  // - no start stop offsets
  // - sampling type is defined by user
  // - sequence creates 15 min files 
//  { .name = "Manual", 
//    .times.startHour = PROTOCOL_USER_DEFINED_TIME,
//    .times.startMin = PROTOCOL_USER_DEFINED_TIME,
//    .times.spanHour = PROTOCOL_USER_DEFINED_TIME,
//    .times.spanMin = PROTOCOL_USER_DEFINED_TIME,
//    .times.startOffset = 0,
//    .times.stopOffset = 0,
//    .commands = { //COMMAND SEQUENCE -UP TO SIX COMMANDS
//                    { COMMAND_RECORD_USER_DEF , TIME_IN_MINUTES(2)} ,  // user defined recording type , 15 min file 
//                    { COMMAND_RESET_SEQUENCE, 0 }  , // repeat  sequence
//                 },
//    .options = { //additional menus to add to menu screen 
//                     PROTOCOL_END_OPTIONS 
//                }    
//  }, 

/***********************************************************************/
  //TIER1 DAY PROTOCOL
  // - start and span is set
  // - sampling type set to 
  { .name = "Tier1 Day", 
    .times.startHour = 7,
    .times.startMin = 0,
    .times.spanHour = 6,
    .times.spanMin = 0,
    .times.startOffset = 0,
    .times.stopOffset = 0,
    .commands = { //COMMAND SEQUENCE -UP TO SIX COMMANDS
                    { COMMAND_RECORD_HIGH_32KHZ , TIME_IN_MINUTES(15)} ,  // user defined recording type , 15 min file
                    { COMMAND_RESET_SEQUENCE, 0 }  // repeat  sequence
                 },
    .options = { //additional menus to add to menu screen 
                     PROTOCOL_SURVEY,
                     PROTOCOL_STATION,
                     PROTOCOL_RANDOM_LOGNAME,
                     PROTOCOL_GPS,  //this overrides the user selected option
                     PROTOCOL_END_OPTIONS 
                }    
  }, 
/***********************************************************************/  
  //TIER1 NIGHT PROTOCOL
  // - start and span is set
  // - sampling type set to 
  { .name = "Tier1 Night", 
    .times.startHour = 20,
    .times.startMin = 0,
    .times.spanHour = 10,
    .times.spanMin = 0,
    .times.startOffset = 0,
    .times.stopOffset = 0,
    .commands = { //COMMAND SEQUENCE -UP TO SIX COMMANDS
                    { COMMAND_RECORD_HIGH_32KHZ , TIME_IN_MINUTES(15)} ,  // user defined recording type , 15 min file
                    { COMMAND_RESET_SEQUENCE, 0 }  // repeat  sequence
                 },
    .options = { //additional menus to add to menu screen 
                     PROTOCOL_SURVEY,
                     PROTOCOL_STATION,
                     PROTOCOL_RANDOM_LOGNAME,
                     PROTOCOL_GPS,  //this overrides the user selected option
                     PROTOCOL_END_OPTIONS 
                }    
  }, 
/***********************************************************************/
  //Forest PROTOCOL
  // - start and span is set to user seleceted
  // - sampling type set to low
  // - random delay period between recordings
  //
  { .name = "Forest", 
    .times.startHour = PROTOCOL_USER_DEFINED_TIME,
    .times.startMin = PROTOCOL_USER_DEFINED_TIME,
    .times.spanHour = PROTOCOL_USER_DEFINED_TIME,
    .times.spanMin = PROTOCOL_USER_DEFINED_TIME,
    .times.startOffset = 0,
    .times.stopOffset = 0,
    .commands = { //COMMAND SEQUENCE -UP TO SIX COMMANDS
                    { COMMAND_RECORD_LOW_8KHZ , TIME_IN_MINUTES(1)} ,  // user defined recording type , 15 min file 
                    { COMMAND_SUSPEND, TIME_IN_MINUTES(2) }, 
                         // set suspend time
                    { COMMAND_SUSPEND_RANDOM, TIME_IN_MINUTES(6) }, // random suspend time
                    { COMMAND_RESET_SEQUENCE, 0 }  // repeat  sequence
                 },
    .options = { //additional menus to add to menu screen 
                     PROTOCOL_TEMP,
                     PROTOCOL_END_OPTIONS 
                }    
  }, 
/***********************************************************************/
//  
//  //TEST PROTOCOL
//  // - start and span is set user defined
//  // - changes sampling mode every 10 minutes up till random suspend . sampling order - low, high, bat , suspend, random suspend
//  { .name = "Test", 
//    .times.startHour = PROTOCOL_USER_DEFINED_TIME,

//    .times.startMin = PROTOCOL_USER_DEFINED_TIME,
//    .times.spanHour = PROTOCOL_USER_DEFINED_TIME,
//    .times.spanMin = PROTOCOL_USER_DEFINED_TIME,
//    .times.startOffset = 0,
//    .times.stopOffset = 0,
//    .commands = { //COMMAND SEQUENCE -UP TO SIX COMMANDS
//                    { COMMAND_RECORD_LOW_8KHZ , TIME_IN_MINUTES(10) } ,  
//                    { COMMAND_RECORD_HIGH_32KHZ , TIME_IN_MINUTES(10) } ,  
//                    { COMMAND_RECORD_BAT_176KHZ , TIME_IN_MINUTES(10) } ,  
//                    { COMMAND_SUSPEND , TIME_IN_MINUTES(10) } ,  
//                    { COMMAND_SUSPEND_RANDOM , TIME_IN_MINUTES(10) },  
//                    { COMMAND_RESET_SEQUENCE , 0}  
//                 },
//    .options = { //additional menus to add to menu screen 
//                     PROTOCOL_END_OPTIONS 
//                }    
//  }
  

/***********************************************************************/
//INSERT NEW PROTOCOL HERE

/***********************************************************************/
//INSERT NEW PROTOCOL HERE

/***********************************************************************/
//INSERT NEW PROTOCOL HERE

/***********************************************************************/
//INSERT NEW PROTOCOL HERE

/***********************************************************************/
//INSERT NEW PROTOCOL HERE

/***********************************************************************/
//INSERT NEW PROTOCOL HERE

/***********************************************************************/
//INSERT NEW PROTOCOL HERE

/***********************************************************************/
//INSERT NEW PROTOCOL HERE

/***********************************************************************/
  
 /**********************END OF PROTOCOLS*************************************/ 
};



int GetNumOfProtocolSlots()
{ 
  static bool numLoaded = FALSE;
  
  if (!numLoaded)
  {
    numOfProtocolSlots = EEPROM_ReadNumStartStopTimes();
    numLoaded = TRUE;
  }
      
  return numOfProtocolSlots;
}

void SetNumOfProtocolSlots(int slots)
{
  if (slots > NUM_OF_VIEWABLE_PROTOCOL_SLOTS) slots = NUM_OF_VIEWABLE_PROTOCOL_SLOTS;
  EEPROM_WriteNumStartStopTimes(slots);
  numOfProtocolSlots = slots;
}

int GetLengthOfProtocolList()
{
   return (sizeof(protocolList) / sizeof(Protocol_Struct_TypeDef)); 
}

// tests all protocols for a specific option
bool HasAnyProtocolOption(Protocol_Options_TypeDef optionType)
{

  bool ret = FALSE;   
  int slots = GetNumOfProtocolSlots();
  
  //scan all protocols
  for (int i = 0;i< slots ;i++)
  {
    ret |= HasProtocolOption(i, optionType);
  }
  
  return ret;
}

// tests whether a protocol has a specific option
bool HasProtocolOption(int protocolNumber, Protocol_Options_TypeDef optionType)
{
  bool ret = FALSE; 
  Protocol_Struct_TypeDef *protocol;
  
  protocol = GetProtocolFromSlot(protocolNumber);  
  //scan all menu options - exit if NO_OTHER_MENU found
  for (int j = 0; j < PROTOCOL_MAX_NUM_OF_OPTIONS; j++)
  {
    if (protocol->options[j] == PROTOCOL_END_OPTIONS) break;
    if (protocol->options[j] == optionType) ret = TRUE;
  }
  
  return ret;
}



void SaveProtocolSlot(int slotIndex)
{
  if (protocolSlot[slotIndex].isLoaded)
  {
    EEPROM_WriteSelectedProtocol(slotIndex, protocolSlot[slotIndex].index);
  }
}


Protocol_Struct_TypeDef *GetProtocolFromSlot(int slotIndex)
{
  if (!protocolSlot[slotIndex].isLoaded)  //load the protocol index from eeprom if haven't already
  {
    protocolSlot[slotIndex].index = EEPROM_ReadSelectedProtocol(slotIndex);
    //check if protocol list Index is valid , if not then reset 
    if ( protocolSlot[slotIndex].index < 0 || protocolSlot[slotIndex].index >= GetLengthOfProtocolList())
    {
      protocolSlot[slotIndex].index = 0;
      EEPROM_WriteSelectedProtocol(slotIndex, protocolSlot[slotIndex].index);
    }      
    protocolSlot[slotIndex].isLoaded = TRUE;
  }

  return &protocolList[protocolSlot[slotIndex].index];
}

void SetProtocolToSlot(int slotIndex, int protocolNum)
{

  if (protocolNum < GetLengthOfProtocolList() && protocolNum >= 0)
  {
    protocolSlot[slotIndex].index = protocolNum;
    protocolSlot[slotIndex].isLoaded = TRUE;
    SaveProtocolSlot(slotIndex); //save to eepron new value
    
  }
}

void IncProtocolInSlot(int slotIndex)
{
  
  if (!protocolSlot[slotIndex].isLoaded)  // protocol slot index should have been loaded form eeprom but if not then do this anyway
  {
    GetProtocolFromSlot(slotIndex); //we just call this to load protocol
  }
    
  if (++protocolSlot[slotIndex].index >= GetLengthOfProtocolList()) protocolSlot[slotIndex].index = 0;
  
  SaveProtocolSlot(slotIndex); //save to eepron new value  
}

void DecProtocolInSlot(int slotIndex)
{
  if (!protocolSlot[slotIndex].isLoaded)  // protocol slot index should have been loaded form eeprom but if not then do this anyway
  {
    GetProtocolFromSlot(slotIndex); //we just call this to load protocol
  }
    
  if (--protocolSlot[slotIndex].index < 0 ) protocolSlot[slotIndex].index = GetLengthOfProtocolList()-1;
  
  SaveProtocolSlot(slotIndex); //save to eepron new value  
}




uint8_t *GetNameOfProtocol(int slotIndex, bool centered)
{
  static uint8_t name[PROTOCOL_NAME_LENGTH];
  Protocol_Struct_TypeDef *protocol;
  
  protocol = GetProtocolFromSlot(slotIndex);
  
 
  if (centered)  // gets length of name and places it in centre of 12 char string
  {
    int i,j;
    int str_length;
    
    for(str_length = 0; protocol->name[str_length] != 0; str_length++);
    
    int position = (LCD_LINE_DISPLAY_COUNT - str_length) /2;
    
    for(i = 0; i < position; i++) name[i] = 0x20; //blank space
    for(j = 0; protocol->name[j] != 0; i++,j++) name[i] = protocol->name[j];
    for(;i < LCD_LINE_DISPLAY_COUNT; i++) name[i] = 0x20; //blank space
    name[i] = 0; //null
  }
  else
    return protocol->name;
    
  return name;
}




int GetIndexOfProtocolFromName(uint8_t* name)
{
  
  int length = sizeof(protocolList) / sizeof(Protocol_Struct_TypeDef);
  int i=0;
  
  for (i=0;i<length;i++)
  {
    if (strncmp(name,protocolList[i].name,12) == 0) break;
  }
  
  if (i != length)
    return i;
  else
    return -1;
}




//is the selected protocol have a user changable start time and span time
bool IsProtocolStartSpanLocked(int protocolSlotIndex)
{
  bool ret;
  
  Protocol_Struct_TypeDef *protocol;
  
  protocol = GetProtocolFromSlot(protocolSlotIndex);
  
   // get the protocol that is being referenced by the user selected index
  ret =  (bool)( (protocol->times.startHour != PROTOCOL_USER_DEFINED_TIME) && (protocol->times.startMin != PROTOCOL_USER_DEFINED_TIME));
  return ret;
}


void WriteProtocolStartSpanTime(int protocolNum, Protocol_UserDef_Start_Span_TypeDef *startSpanItem)
{
  //check if 
  isStartStopListValid = FALSE; // beacuse we've modified the values start stop list isn't valid 
  EEPROM_WriteProtocolUserDefinedStartSpanTime(protocolNum, startSpanItem);
}


Protocol_UserDef_Start_Span_TypeDef *ReadProtocolStartSpanTime(int slotIndex)
{
  //check whether to get start span times directly from protocol list if not changable
  //elase get them from eeprom
  Protocol_UserDef_Start_Span_TypeDef * pStartSpanItem;
  Protocol_Struct_TypeDef *protocol;
 
  
  if (IsProtocolStartSpanLocked(slotIndex))
  { 
    protocol = GetProtocolFromSlot(slotIndex);
    pStartSpanItem = &protocol->times;
  }
  else //or if changeable grab from eeeprom
    pStartSpanItem = EEPROM_ReadProtocolUserDefinedStartSpanTime(slotIndex);
  
  
  return pStartSpanItem;
}






//determines if current
ProtocolTimeCheck_TypeDef *IsWithinProtocolOnTime(int8_t currentHour, int8_t currentMin, int protocolNum)
{
  static ProtocolTimeCheck_TypeDef protocolTimeCheck;
  int endMin, endHour;
  int carry;
  int startCalc, stopCalc, currentCalc;
  Protocol_UserDef_Start_Span_TypeDef *pStartStopItem;
  
  
 
  protocolTimeCheck.isWithin = FALSE; //reset flag
  
  pStartStopItem = ReadProtocolStartSpanTime(protocolNum);
  
  currentCalc = (currentHour * 60) + currentMin;	
  
  //get EndTime                    
  endMin = pStartStopItem->startMin + pStartStopItem->spanMin;
  
  carry =0;
  if (endMin > 59) 
  {
    endMin -= 60;
    carry = 1;
  }
  
  endHour = pStartStopItem->startHour + pStartStopItem->spanHour + carry;
  if (endHour > 23) endHour -= 24;
            
  startCalc = ((uint16_t)pStartStopItem->startHour * 60) + (uint16_t)pStartStopItem->startMin;		//59minutes hex = 89 
  stopCalc = ((uint16_t)endHour * 60) + (uint16_t)endMin;
                 
  // decision depends on if the stop time is before or after the start time (if survey passes through midnight)
  if (startCalc > stopCalc)		// so survey passes across midnight
  {
    if ( currentCalc >= startCalc || currentCalc < stopCalc) 
    {
      protocolTimeCheck.isWithin = TRUE;
      protocolTimeCheck.protocolNum = protocolNum;

        protocolTimeCheck.minutesElapsed = currentCalc - startCalc;
        if (protocolTimeCheck.minutesElapsed < 0) protocolTimeCheck.minutesElapsed += (24*60); // if current time was lower than start add 24 hours
          protocolTimeCheck.minutesToGo = stopCalc - currentCalc;
        if (protocolTimeCheck.minutesToGo < 0) protocolTimeCheck.minutesToGo += (24*60); // if current time was higher than stop add 24 hours      
      
    }
  }
  else		
  {
    if((currentCalc >= startCalc) && (currentCalc < stopCalc)) 
    {
      protocolTimeCheck.isWithin = TRUE;
      protocolTimeCheck.protocolNum = protocolNum;
      protocolTimeCheck.minutesElapsed = currentCalc - startCalc;
      protocolTimeCheck.minutesToGo = stopCalc - currentCalc;
    }
    
  }
        
  return &protocolTimeCheck;
}



/*********************************************************************
* CheckRecordTime
* takes the start time and duration and returns flag if time in range
*********************************************************************/

ProtocolTimeCheck_TypeDef *CheckProtocolTime(rtcTimeTypeDef *rtcTime)
{
  ProtocolTimeCheck_TypeDef *protocolTimeCheck;


  protocolTimeCheck = NULL;
  
  int slots = GetNumOfProtocolSlots();
  
  for(int i=0; i <slots; i++)   
  {
    protocolTimeCheck = IsWithinProtocolOnTime(rtcTime->hour, rtcTime->min, i);
    if (protocolTimeCheck->isWithin) break;
  }
     
	
  return protocolTimeCheck;
}








//fills array of start stop times that the quick time check function can use
void FillProtocolStartStopList(void)
{
   int endHour,endMin;
   int carry;
   int index;
   Protocol_UserDef_Start_Span_TypeDef *pStartStopItem;
   
   //GET START AND STOP TIMES
   int slots = GetNumOfProtocolSlots();
   
   for(index=0; index < slots; index++)
   {
    pStartStopItem = ReadProtocolStartSpanTime(index);
       
    endMin = pStartStopItem->startMin + pStartStopItem->spanMin;
    carry = 0;
    if (endMin > 59) 
    {
      endMin -= 60;
      carry = 1;
    }
    endHour = pStartStopItem->startHour + pStartStopItem->spanHour + carry;
    if (endHour > 23) endHour -= 24;
    startStopList[index].start_in_mins = ((int)pStartStopItem->startHour * 60) + (int)pStartStopItem->startMin;		//59minutes hex = 89
    startStopList[index].stop_in_mins = (endHour * 60) + endMin;      
   } 
   
   isStartStopListValid = TRUE;
} 



//Gets the next active start time
//uses: to set the next alarm period
Protocol_UserDef_Start_Span_TypeDef *GetNextActiveProtocolStartSpanTime(rtcTimeTypeDef * currentTime)
{
  int currentTimeInMinutes;
  int startTimeinMins;
  int previousCalc = INT32_MAX;
  int indexOfNextProtocol = 0;
  
  //make sure the start stop list is filled
  if (!isStartStopListValid) FillProtocolStartStopList();

  currentTimeInMinutes = ((uint16_t)currentTime->hour * 60) + (uint16_t)currentTime->min;	

  int slots = GetNumOfProtocolSlots();
  
  for(int i = 0; i < slots; ++i)
  {
    startTimeinMins = startStopList[i].start_in_mins;
    if(startTimeinMins <= currentTimeInMinutes) startTimeinMins +=  24*60;    //add 24 hours if below current time
     
     
    if ( startTimeinMins - currentTimeInMinutes < previousCalc)
    {
      previousCalc = startTimeinMins - currentTimeInMinutes;
      indexOfNextProtocol = i;
    }
  }
  
  return ReadProtocolStartSpanTime(indexOfNextProtocol);
}



ProtocolTimeCheck_TypeDef * CheckProtocolTimeQuick(rtcTimeTypeDef * currentTime)
{
    
  uint16_t currentTimeInMinutes;
  static ProtocolTimeCheck_TypeDef protocolTimeCheck;
  int index;
  
  protocolTimeCheck.isWithin = FALSE;
  
  //make sure the start stop list is filled
  if (!isStartStopListValid) FillProtocolStartStopList();
   
  currentTimeInMinutes = ((uint16_t)currentTime->hour * 60) + (uint16_t)currentTime->min;	
  int slots = GetNumOfProtocolSlots();
  
  for(index=0; index < slots; index++)
  {
    // decision depends on if the stop time is before or after the start time (if survey passes through midnight)
    if (startStopList[index].start_in_mins > startStopList[index].stop_in_mins)		// so survey passes across midnight
    {
      if ((currentTimeInMinutes >= startStopList[index].start_in_mins) || (currentTimeInMinutes < startStopList[index].stop_in_mins)) 
      {
        protocolTimeCheck.isWithin = TRUE;
        protocolTimeCheck.minutesElapsed = currentTimeInMinutes - startStopList[index].start_in_mins;
        if (protocolTimeCheck.minutesElapsed < 0) protocolTimeCheck.minutesElapsed += (24*60); // if current time was lower than start add 24 hours
          protocolTimeCheck.minutesToGo = startStopList[index].stop_in_mins - currentTimeInMinutes;
        if (protocolTimeCheck.minutesToGo < 0) protocolTimeCheck.minutesToGo += (24*60); // if current time was higher than stop add 24 hours
          protocolTimeCheck.protocolNum = index;
      }
    }
    else		
    {
      if((currentTimeInMinutes >= startStopList[index].start_in_mins) && (currentTimeInMinutes < startStopList[index].stop_in_mins)) 
      {
        protocolTimeCheck.isWithin = TRUE;
        protocolTimeCheck.minutesElapsed = currentTimeInMinutes - startStopList[index].start_in_mins;
        protocolTimeCheck.minutesToGo = startStopList[index].stop_in_mins - currentTimeInMinutes;
        protocolTimeCheck.protocolNum = index;
      }
    }
    
    if (protocolTimeCheck.isWithin) break;
  }
  
  protocolTimeCheck.secondsToGo = protocolTimeCheck.minutesToGo * 60;

  return &protocolTimeCheck;
}




InstructionType GetUserDefinedSamplingMode()
{
 
  if (!isUserDefSamplingModeLoaded) 
  {
    userDefinedSamplingMode = (InstructionType)EEPROM_ReadUserDefSamplingType();
    
    //if out of range
    if (userDefinedSamplingMode < MENUSAMPLINGOPTIONS_FIRST || userDefinedSamplingMode > MENUSAMPLINGOPTIONS_LAST)
    {
      userDefinedSamplingMode = MENUSAMPLINGOPTIONS_FIRST; 
      EEPROM_WriteUserDefSamplingType(userDefinedSamplingMode);
    
    }
    isUserDefSamplingModeLoaded = TRUE;
  }
  
  return userDefinedSamplingMode;
}




void IncUserDefSamplingMode(void)
{
  //get value from eeprom
  GetUserDefinedSamplingMode(); // gets value from eeprom  to store in the global var userDefinedSamplingMode 
  if (++userDefinedSamplingMode > MENUSAMPLINGOPTIONS_LAST) userDefinedSamplingMode = MENUSAMPLINGOPTIONS_FIRST;
  EEPROM_WriteUserDefSamplingType(userDefinedSamplingMode);
}
       
       
void DecUserDefSamplingMode(void)
{
  //get value from eeprom
  GetUserDefinedSamplingMode(); // gets value from eeprom  to store in the global var userDefinedSamplingMode 
  if (--userDefinedSamplingMode < MENUSAMPLINGOPTIONS_FIRST) userDefinedSamplingMode = MENUSAMPLINGOPTIONS_LAST;
  EEPROM_WriteUserDefSamplingType(userDefinedSamplingMode);
}
       
uint8_t *GetUserDefSamplingModeName()
{
  uint8_t  *optionItem[] = {"",  //not used "Reserved"
                             "    LOW ",
                             "    HIGH",
                             "    BAT "
                              };  
  InstructionType samplingMode;
  
  samplingMode = GetUserDefinedSamplingMode(); //loads the global userDefinedSamplingMode from eeprom
  return (uint8_t*)optionItem[samplingMode];
}










// Gets the next recording type in the protocol sequence
// - inputs  
//      - protocol index 
//
ProtocolSequence_TypeDef * GetNextProtocolSequence(int slotIndex)
{
  
  static ProtocolSequence_TypeDef protoSequence;
  static int lastProtocolSlotIndex= -1; // if this changes we can reset the sequence
  static int commandIndex;
  bool commandReady;
  Protocol_Struct_TypeDef *protocol;
  
  protocol = GetProtocolFromSlot(slotIndex);
  
  if (lastProtocolSlotIndex != slotIndex) commandIndex = 0; //reset command sequence if protocol changes
  lastProtocolSlotIndex = slotIndex;
  
  protoSequence.protocolName = protocol->name;
  do 
  {   
    commandReady = TRUE;  
    
    
    switch (protocol->commands[commandIndex].command)  //
    {
      case COMMAND_RECORD_USER_DEF:
        protoSequence.instruction = GetUserDefinedSamplingMode();   //if the userdefined sampling mode has't been loaded then do that
        break;
      case COMMAND_RECORD_LOW_8KHZ:
        protoSequence.instruction = RECORD_LOW_8KHZ;
        break;
      case COMMAND_RECORD_HIGH_32KHZ:
        protoSequence.instruction = RECORD_HIGH_32KHZ;
        break;
      case COMMAND_RECORD_BAT_176KHZ:
        protoSequence.instruction = RECORD_BAT_176KHZ;
        break;
      case COMMAND_SUSPEND:
        protoSequence.instruction = SUSPEND;
        break;
      case COMMAND_SUSPEND_RANDOM:
        protoSequence.instruction = SUSPEND_RANDOM;
        break;
      case COMMAND_RESET_SEQUENCE:
        commandIndex = 0; //we want it to reset to 0, 
        commandReady = FALSE; //loop again and get next command
        break;
      default: // error
        commandIndex = 0; //we want it to reset to 0, 
        commandReady = FALSE; //loop again and get next command
        break;
    }
    
    if (commandReady)
    {
      protoSequence.timeInSec = protocol->commands[commandIndex].timeInSec; //
      protoSequence.protocolNumber = slotIndex;
      
      if (++commandIndex >= PROTOCOL_NUM_OF_COMMANDS) commandIndex = 0; 
    }
    
  }
  while (!commandReady);
   
  return &protoSequence;
}



void WriteProtocolNameToLogFile(RecordSessionInit_TypeDef * session)
{
  static uint32_t index_flags = 0;
  uint8_t text_string[64];
  uint32_t bitmask = 1 << session->protocolNumber;
  
  if (bitmask == 0) return;
  if ((bitmask & index_flags) == 0)
  {
    rtcTimeTypeDef *time;
    rtcDateTypeDef *rtc_date;
    
    // if time was recently aquired from rtc
    time = RtcGetBufferedTime();
    rtc_date = RtcGetBufferedDate();
    
    sprintf((char*)text_string,"%02d/%02d/%02d %02d:%02d - Start Recording (%s)\r\n", rtc_date->date, rtc_date->month,rtc_date->year, time->hour, time->min,session->protocolName);
    WriteLogFileToCard(text_string, FALSE);
    index_flags |= bitmask;
  }
}




/////////////////////////////////////////EOF/////////////////////////////////////