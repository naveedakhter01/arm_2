
/* This file contains the functions for modifying the AR4 settings
*
* - only includes the functions for loading settings from SD card and NFC
* - all other settings are selected via the menu
*/ 


#include "settings.h"
#include "protocol.h"
#include "eeprom.h"
#include "FatSD.h"
#include "menu.h" //to get defines


#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ini.h"

void ConvertAsciiToSurveyFormat(char* str);

extern uint8_t  *stationItem[];

int const daysInMonth[] = {31,31,28,31,30,31,30,31,31,30,31,30,31}; // JFMAMJJASOND
#define DAY_OF_MONTH_CHK(DAY,MON) 		if(DAY>daysInMonth[MON]){DAY=1;} else if (DAY<1) {DAY=daysInMonth[MON];}



#undef __strdup
#undef strdup
#ifndef weak_alias
# define __strdup strdup
#endif
/* Duplicate S, returning an identical malloc'd string.  */
char *
__strdup (const char *s)
{
  size_t len = strlen (s) + 1;
  void *new = malloc (len);
  if (new == NULL)
    return NULL;
  return (char *) memcpy (new, s, len);
}
#ifdef libc_hidden_def
libc_hidden_def (__strdup)
#endif
#ifdef weak_alias
weak_alias (__strdup, strdup)
#endif


// coverts a string to uppercase
void strupr(char * s) {
  // Convert to upper case
  while (*s) {
    *s = toupper((unsigned char) *s);
    s++;
  }

}

// coverts a string to uppercase
void strlwr(char * s) {
  // Convert to upper case
  while (*s) {
    *s = tolower((unsigned char) *s);
    s++;
  }

}







static int lastNonce = -1;

  
  static int handler(void* user, const char* section, const char* name,
                     const char* value)
  {
      SettingsConfiguration* pconfig = (SettingsConfiguration*)user;
  
      #define MATCH_SECTION(s) strcmp(section, s) == 0
      #define MATCH_NAME(n) strcmp(name, n) == 0
      #define MATCH(s, n) strcmp(section, s) == 0 && strcmp(name, n) == 0
      
      int ret = 0;
      int recordNum = 0;
      
      //clock settings
      strupr((char*)section);
      strupr((char*)name);
      
      if (MATCH_SECTION("RTC") || MATCH_SECTION("CLOCK"))
      {
        if (MATCH_NAME("TIME")) 
        {
          int h = -1; //required
          int m = -1; //required
          int s = 0;
          //format scans for hour and min
          if (sscanf((const char*)value,"%2d%2d%2d",  &h, &m, &s) == 3 ||  //"10:12"
              sscanf(value,"%d:%d:%d", &h, &m, &s) == 3 ||
              sscanf(value,"%2d%2d", &h, &m) == 2 || 
              sscanf(value,"%d:%d", &h, &m) == 2 )
          {
            //assert
            if ( (h >= 0 && h < 24 ) &&
                (m >= 0 && m < 60 ) &&
                 (s >= 0 && s <60) ) 
            {
              pconfig->hour = (uint8_t)h;
              pconfig->min = (uint8_t)m;
              pconfig->sec = (uint8_t)s;
              pconfig->set_time=TRUE;
              ret = 1; // success
            }  
          }
        }        
        else if (MATCH_NAME("DATE")) 
        {
          int D = -1;
          int M = -1;
          int Y = -1;
          
          //format scans for hour and min
          if (sscanf((const char*)value,"%2d%2d%4d",  &D, &M, &Y) == 3 ||  
              sscanf((const char*)value,"%2d%2d%2d",  &D, &M, &Y) == 3 ||
              sscanf((const char*)value,"%d/%d/%d",  &D, &M, &Y) == 3 //
              )
          {
            //assert
            if ( (D> 0 && D <= 31 ) &&
                (M> 0 && M <= 12 ) &&
                 (Y> 0 && Y <= 255 || Y >=2000 & Y < 2255) ) 
            {
              
              DAY_OF_MONTH_CHK(D,M);
              
              pconfig->day = (uint8_t)D;
              pconfig->month = (uint8_t)M;
              pconfig->year =  Y >= 2000 ? (uint8_t)(Y-2000) : (uint8_t)Y;
              pconfig->set_date=TRUE;
              ret = 1; // success
            }  
          }  

        }         
      }
      // additional options           
      else if (MATCH_SECTION("GPS"))
      {
        
        if (MATCH_NAME("LOG")|| MATCH_NAME("LOGGING")) 
        {
          strupr((char*)value);
          if (strcmp((const char*)value,"OFF") == 0 ||
              strcmp((const char*)value,"0") == 0 ||
              strcmp((const char*)value,"FALSE") == 0) 
            { pconfig->gps_log = 0; ret = 1; }
          else if (strcmp((const char*)value,"ON") == 0 ||
                   strcmp((const char*)value,"1") == 0 ||
                   strcmp((const char*)value,"TRUE") == 0  ) 
            { pconfig->gps_log = 1; ret = 1; }          
          if (ret ==1 ) pconfig->set_gps = TRUE;
        }
        else if (MATCH_NAME("SYNC")|| MATCH_NAME("TIMESYNC")) 
        {
          strupr((char*)value);
          if (strcmp((const char*)value,"OFF") == 0 ||
              strcmp((const char*)value,"0") == 0 ||
              strcmp((const char*)value,"FALSE") == 0) 
            { pconfig->gps_sync = 0; ret = 1; }
          else if (strcmp((const char*)value,"ON") == 0 ||
                   strcmp((const char*)value,"1") == 0 ||
                   strcmp((const char*)value,"TRUE") == 0  ) 
            { pconfig->gps_sync = 1; ret = 1; }          
        }     
      }   
      // additional options           
      else if (MATCH_SECTION("TIER1"))
      {
        if (MATCH_NAME("STATION")) 
        {
          strupr((char*)value);
          for (int i=0;i<sizeof(stationItem)/ sizeof(char*) ;i++)
          {
            if (strcmp((const char*)value,(const char*)stationItem[i]) == 0) 
            {
               pconfig->station = i;
               pconfig->set_station = TRUE;
               ret = 1;
               break;
            }
          }
        }   
        else if (MATCH_NAME("SURVEY")) 
        {
          int i;
          strupr((char*)value);
          for (i=0;i<SURVEY_NAME_LENGTH;i++)
          {
            if (value[i] == 0) 
              break;
          }
          
          if (i > 0 && i <= SURVEY_NAME_LENGTH)
          {
            strncpy((char*)pconfig->survey,value,SURVEY_NAME_LENGTH);
            pconfig->set_survey = TRUE;
            ret = 1;
          }
        }
      }         
      //record slots    
      // STATION == RECORD
      else if (sscanf((const char*)section,"TIMESLOT%d",  &recordNum) == 1 ||
               sscanf((const char*)section,"RECORD%d",  &recordNum) == 1 ||
               MATCH_SECTION("TIMESLOT") || MATCH_SECTION("RECORD")
               )
      {
        int h = -1; //required
        int m = -1; //required

        if (recordNum == 0) recordNum = 1; // set to first slot if was not set
        if (recordNum > MAX_NUM_OF_PROTOCOL_SLOTS) return 0;
        
        
        if (MATCH_NAME("START")) 
        {

          //format scans for hour and min
          if (sscanf(value,"%2d%2d", &h, &m) == 2 || 
              sscanf(value,"%d:%d", &h, &m) == 2 )
          {
            //assert
            if ( (h >= 0 && h < 24 ) &&
                (m >= 0 && m < 60 )) 
            {
              pconfig->slot[recordNum-1].start_hour = (uint8_t)h;
              pconfig->slot[recordNum-1].start_min = (uint8_t)m;
              pconfig->slot[recordNum-1].set_flag = TRUE;
              pconfig->set_slot = TRUE;
              ret = 1; // success
            }  
          }
        }
        else if (MATCH_NAME("SPAN")) 
        {

          //format scans for hour and min
          if (sscanf(value,"%d:%d", &h, &m) == 2 || 
              sscanf(value,"%2d%2d", &h, &m) == 2  )
          {
            //assert
            if ( (h >= 0 && h < 24 ) &&
                (m >= 0 && m < 60 )) 
            {
              pconfig->slot[recordNum-1].span_hour = (uint8_t)h;
              pconfig->slot[recordNum-1].span_min = (uint8_t)m;
              pconfig->slot[recordNum-1].set_flag = TRUE;
              pconfig->set_slot = TRUE;
              ret = 1; // success
            }  
          }
        }
        else if (MATCH_NAME("PROTOCOL"))
        {
          int protocol_index = -1;
          if (strlen(value) <= PROTOCOL_NAME_LENGTH)
          {
            //make value always lower case except firt char
            strlwr((char*)value);
            ((char*)value)[0] =  toupper((unsigned char) value[0]); //make first char uppercase
            //we also have to make the next word's first cahr upper if it exists
            int i;
            for (i=0;i<strlen(value);i++) if (value[i] == ' ') break;
            if (i != strlen(value)) ((char*)value)[i+1] =  toupper((unsigned char) value[i+1]); //make second word's first char uppercase
              
            protocol_index =  GetIndexOfProtocolFromName((uint8_t*)value);
            
            if (protocol_index != -1)
            {
              pconfig->slot[recordNum-1].protocol = (uint8_t)protocol_index;
              pconfig->slot[recordNum-1].set_flag = TRUE;
              pconfig->set_slot = TRUE;
              ret = 1;
            }
          }
        }
      }
      else if (MATCH_SECTION("FILE"))
      {
        // determine flag to delete ini file after reading
        if (MATCH_NAME("DELETEAFTER") || MATCH_NAME("DELETE")) 
        {
          strupr((char*)value);
          if (strcmp((const char*)value,"OFF") == 0 ||
              strcmp((const char*)value,"0") == 0 ||
              strcmp((const char*)value,"FALSE") == 0) 
            { pconfig->del_ini = FALSE; ret = 1; }
          else if (strcmp((const char*)value,"ON") == 0 ||
                   strcmp((const char*)value,"1") == 0 ||
                   strcmp((const char*)value,"TRUE") == 0  ) 
            { pconfig->del_ini = TRUE; ret = 1; }          
        }   
      }
      
      
      
/*
      if (MATCH("protocol", "version")) {
          pconfig->version = atoi(value);
      }  else if (MATCH("protocol", "id")) {
          pconfig->id = atoi(value);    
      } else if (MATCH("user", "name")) {
          pconfig->name = strdup(value);
      } else if (MATCH("user", "email")) {
          pconfig->email = strdup(value);
      } else {
          return 0;   //unknown section/name, error
      }
      */
                 
      return ret;
  }



//Updates AR4 settings by parsing a block of data
//
// - data block from either SD or NFC



int ParseSettings(uint8_t* blockData)
{
  Protocol_UserDef_Start_Span_TypeDef  startSpanItem;
  int index = 0; //general purpose index
  rtcTimeTypeDef rtcTime;
  rtcDateTypeDef rtcDate;
  int slotsCount;
  uint8_t *str;

    //nfc_data = &profile.blockData;
    
    //check if nonce has not been used
    if (GetEntry16bit(blockData, NONCE) == lastNonce) 
    {
      return 1;  
    }


    
    //check if checksum is correct
    uint16_t checksum;
    uint16_t sum=0;
    
    checksum = GetEntry16bit(blockData,CHECKSUM);
    int startIndex = NONCE; //set time entry will be start index
    int endIndex = PAYLOAD_EOL;      // end of list will be used to mark end
    for (int i=startIndex; i< endIndex-startIndex ;i++) //sum the entire struct minus the checksum bytes
      sum += blockData[i];
    if (sum != checksum)     
    {
      return 2;  
    }
    

      //Set time if flag set
    
    if (GetEntryBool(blockData, SET_TIME) == TRUE)
    { 
      rtcTime.sec = 0;
      rtcTime.min = GetEntry8bit(blockData, TIME_MIN);
      rtcTime.hour = GetEntry8bit(blockData, TIME_HOUR);
      
      RtcSetTime(&rtcTime);
      
      rtcDate.date = GetEntry8bit(blockData, TIME_DAY);
      rtcDate.month = GetEntry8bit(blockData, TIME_MONTH);
      rtcDate.year = GetEntry8bit(blockData, TIME_YEAR);
      
      RtcSetDate(&rtcDate);
    }
    
    
  //set number of slots used
    slotsCount = GetEntry8bit(blockData, SLOTS_USED);
    
    int num = slotsCount < 2 ? 2 : slotsCount;
    num = num > 6 ? 6 : num;    
    SetNumOfProtocolSlots(num);

    //save protocols
    int i_slots;
    for (i_slots=0;i_slots<slotsCount;i_slots++)
    {
      // write start time and duration
      startSpanItem.startHour = GetEntry8bit(blockData,  TIME_SLOT_ITEMS_0_START_HOUR + (i_slots*TIME_SLOT_ITEMS_SIZE));
      startSpanItem.startMin = GetEntry8bit(blockData,  TIME_SLOT_ITEMS_0_START_MIN + (i_slots*TIME_SLOT_ITEMS_SIZE));
      
      startSpanItem.spanHour = GetEntry8bit(blockData,  TIME_SLOT_ITEMS_0_SPAN_HOUR + (i_slots*TIME_SLOT_ITEMS_SIZE));
      startSpanItem.spanMin = GetEntry8bit(blockData,  TIME_SLOT_ITEMS_0_SPAN_MIN + (i_slots*TIME_SLOT_ITEMS_SIZE));
      
      startSpanItem.startOffset = GetEntry8bit(blockData,  TIME_SLOT_ITEMS_0_START_OFFSET + (i_slots*TIME_SLOT_ITEMS_SIZE));
      startSpanItem.stopOffset = GetEntry8bit(blockData,   TIME_SLOT_ITEMS_0_STOP_OFFSET + (i_slots*TIME_SLOT_ITEMS_SIZE));
      
      WriteProtocolStartSpanTime(i_slots, &startSpanItem);
      
      //save protocol name
      
      index = GetIndexOfProtocolFromName( GetEntryString(blockData,TIME_SLOT_ITEMS_0_PROTOCOL_NAME + (i_slots*TIME_SLOT_ITEMS_SIZE), 12) );
      SetProtocolToSlot(i_slots, index);
    }
    // SET AT LEAST 2 DEFAULT TIMES
    //make sure that at least two time slots are availble if not then create two "default OFF" times
    for (;i_slots < 2;i_slots++)
    {
      startSpanItem.startHour = 0;
      startSpanItem.startMin = 0;
      
      startSpanItem.spanHour = 0;
      startSpanItem.spanMin = 0;
      
      startSpanItem.startOffset = 0;
      startSpanItem.stopOffset = 0;
      
      WriteProtocolStartSpanTime(i_slots, &startSpanItem);
      SetProtocolToSlot(i_slots, 0);
    }
    
    
    
    //write survey name
    str = GetEntryString(blockData,SURVEY_NAME,8);
    int strlength = strlen((char*)str);
    if (strlength > 0 && strlength < 8 )
    {
      //convert string values to custom encoding
      int i;
      for (i=0; i<strlength ;i++)
      {
        if (str[i] >= 0x30 && str[i] <= 0x39)   str[i] -= 0x30;
        else if (str[i] >= 0x41 && str[i] <= 0x5A)   str[i] -= (0x41 - 10); // offset will be start of 'A' char in new encoding scheme
        else str[i] = sizeof(charTable) -1; // insert termination char if nonvalid  char
      }    
      for (; i< (SURVEY_NAME_MAX_LENGTH-1);i++)
      {
        str[i] = sizeof(charTable) -1; // insert termination char if nonvalid  char
      }
      
      
      EEPROM_WriteSurveyName(str);    
     
    }
    //save station option
    EEPROM_WriteStation(GetEntry8bit(blockData, STATION_OPTION ) );// eg x = BIRX

    //save GPS options 
    EEPROM_WriteGpsOptions(GetEntry8bit(blockData, GPS_OPTION ));  //eg x = GPS + LOG
    
    //save random delay
    EEPROM_WriteRandomDelay(GetEntry8bit(blockData, RANDOM_DELAY_OPTION ));
    
    
    //save the nonce
    lastNonce = GetEntry16bit(blockData, NONCE);
    
    return 0;
         
}




/************************
* SETTINGS FROM SD CARD
* - read settings file from SD card 
*
*************************/


int LoadSettingsFromSDCard(void)
{  
//  F_FILE *file;
//  int ret;
//  int32_t  filesize; //size of settings file
//  rtcTimeTypeDef rtcTime;
//  rtcDateTypeDef rtcDate;
//  Protocol_UserDef_Start_Span_TypeDef  startSpanItem;
//  SettingsConfiguration config;
//  bool delete_ini = FALSE;
//  
//  
//  //clear config struct
//  memset((void*)&config,0,sizeof(SettingsConfiguration));
//
//  
//  /* 1st power on */
//  ret = _f_poweron();                     // power on sd card
//  
//  if (ret==F_ERR_NOTFORMATTED)
//  {
//    return 1;                       //not formatted
//  }  
//  
// 
//  //open dsp firmware file - thre are two possible files either just the arm main firmware, or the one with the bootloader.
//  if ( (filesize = (f_filelength(SETTINGS_FILENAME))) > 0 )
//  {  
//    file = f_open(SETTINGS_FILENAME, "r");
//  }   
//  
//  // check if valid
//  if (!file) 
//  {
//    ret=_f_poweroff();
//    return -1;
//  }             
//  //check if filesize != 0
//  
//  f_seek(file,0,F_SEEK_SET); // seek offset into file ignoring bootloader code
//  
//  //allocate mem
//  void *str = malloc (filesize);
//  if (str == NULL)
//    return 1;
//  
//  f_read(str,1,filesize,file);  
//  
//  
//  if (ini_parse_string((const char*)str, handler, &config) < 0)
//  {
//    //error handle
//    
//  }
//  else
//  {
//    // if we are updating any settings
//    if (config.set_time || config.set_date || config.set_station || config.set_survey || config.set_gps ||
//        config.set_slot)
//    {
//      
//      write_LCD_line1(" Card settings ");
//      write_LCD_line2("    loaded     ");
//    
//      // check if config values are valid then update AR4 
//      
//      
//      //if time settings
//      if (config.set_time)
//      {
//        rtcTime.sec = config.sec;
//        rtcTime.min = config.min;
//        rtcTime.hour = config.hour;
//         RtcSetTime(&rtcTime);
//      }
//      //if date settings
//      if (config.set_date)
//      {
//        rtcDate.date = config.day;
//        rtcDate.month = config.month;
//        rtcDate.year = config.year;
//        RtcSetDate(&rtcDate);
//      } 
//      //if station settings
//      if(config.set_station)
//      {
//        //save station option
//        EEPROM_WriteStation(config.station);// eg x = BIRX
//      }
//      //if gps settings
//      if (config.set_gps)
//      {
//        uint8_t gps_value = config.gps_log;
//        if (gps_value == 1 && config.gps_sync == 1) gps_value = 2; // apply sync value is logging on
//        
//         EEPROM_WriteGpsOptions(gps_value);  
//      }
//      // if survey name
//      if (config.set_survey)
//      {
//        //convert from ascii to custom coding
//        ConvertAsciiToSurveyFormat((char*)config.survey); //converts in place to custom format
//        EEPROM_WriteSurveyName(config.survey);
//      }
//      
//    
//      //determine how many slots used byt the highest allocated slot
//      // but always allow 2 slots 
//      int allocated_slots;
//      for (allocated_slots = MAX_NUM_OF_PROTOCOL_SLOTS-1; allocated_slots >= 2; allocated_slots--)
//      {
//        if (config.slot[allocated_slots].set_flag) break;
//      }
//      allocated_slots++; // not zero indexed
//      
//      SetNumOfProtocolSlots(allocated_slots);
//      
//      
//      // cycle through tstart times for slots
//      for (int i=0; i< MAX_NUM_OF_PROTOCOL_SLOTS;i++)
//      {
//        if (!config.slot[i].set_flag) continue;
//        
//        startSpanItem.startHour =  config.slot[i].start_hour ;
//        startSpanItem.startMin = config.slot[i].start_min % 5 == 0 ? config.slot[i].start_min : config.slot[i].start_min - (config.slot[i].start_min% 5);
//        
//        startSpanItem.spanHour = config.slot[i].span_hour;
//        startSpanItem.spanMin = config.slot[i].span_min % 5 == 0 ? config.slot[i].span_min : config.slot[i].span_min + (5 - (config.slot[i].span_min % 5));
//        
//        startSpanItem.startOffset = 0;
//        startSpanItem.stopOffset = 0;
//        
//        WriteProtocolStartSpanTime(i, &startSpanItem);
//        
//        //save protocol name
//        SetProtocolToSlot(i, config.slot[i].protocol);
//        
//      }
//      
//      //determine if delete ini file
//      delete_ini = config.del_ini;
//      
//      
//      delay_ms(2000);
//      clear_LCD();
//      
//    }
//
//    
//    
//    
//    
//  }
//  
//  
//  
// 
//  f_close(file);   
//
//  if (str) free((void*)str);
// 
//  
//  //delete the ini file if flag set
//  if (delete_ini)
//  {
//    ret = f_delete(SETTINGS_FILENAME);
//    if (ret) return ret;
//    
//    write_LCD_line1(" Deleted ini  ");
//    write_LCD_line2("     file     ");
//    
//    delay_ms(1000);
//    clear_LCD();
//  }
//  
//   
//  
//  /* simple restart */
//  ret=_f_poweroff();
//  if (ret) return ret;

  
  return 0;  
  
}












uint8_t GetEntry8bit(uint8_t* data, int index)
{
  return (uint8_t)(data[index]);  
} 
uint16_t GetEntry16bit(uint8_t* data, int index)
{
  return (uint16_t)(data[index] | (data[index+1] <<8) );  
}

bool GetEntryBool(uint8_t* data, int index)
{
  return  GetEntry8bit(data, index) != 0 ? TRUE : FALSE;
}

// set maxlength to 0 for no limit
uint8_t* GetEntryString(uint8_t* data, int index, int maxlength)
{
  static uint8_t string[64];
  int i = 0;
  
  
  if (maxlength <= 0) maxlength = 63;
  
  while(maxlength > 0)
  {
    if (data[index+i] == 0) break;
    string[i] = data[index+i];
    
    i++;
    maxlength--;
  }
  string[i] = 0;
  
  return string; 
}

// converts in place 
// str 
void ConvertAsciiToSurveyFormat(char* str)
{
      //convert string values to custom encoding
      int i;
      for (i=0; i<strlen(str) ;i++)
      {
        if (str[i] >= 0x30 && str[i] <= 0x39)   str[i] -= 0x30;
        else if (str[i] >= 0x41 && str[i] <= 0x5A)   str[i] -= (0x41 - 10); // offset will be start of 'A' char in new encoding scheme
        else str[i] = sizeof(charTable) -1; // insert termination char if nonvalid  char
      }    
      for (; i< (SURVEY_NAME_MAX_LENGTH-1);i++)
      {
        str[i] = sizeof(charTable) -1; // insert termination char if nonvalid  char
      }            
}



/************************************EOF************************************/




