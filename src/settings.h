





#ifndef SETTINGS_H
#define SETTINGS_H

#include "stm32f10x.h"
#include "protocol.h"


#define SETTINGS_FILENAME "settings.ini"  // file to modify settings, must match define in bootloader
#define SETTINGS_FILESIZE 136  // filesize of settings file


#define TIME_SLOT_ITEMS_SIZE (TIME_SLOT_ITEMS_1_START_HOUR-TIME_SLOT_ITEMS_0_START_HOUR)
#define PROTOCOL_NAME_MAX_LENGTH 12
#define SURVEY_NAME_MAX_LENGTH 8
#define PAYLOAD_SIZE PAYLOAD_EOL





// The Configuration from SD card
typedef struct
{
  bool set_flag; // 
  uint8_t start_hour;
  uint8_t start_min;
  
  uint8_t span_hour;
  uint8_t span_min;

  uint8_t start_offset;
  uint8_t stop_offset;
  uint8_t protocol;  // just the index which represents the name 
    
}TimeSlotConfiguration;

typedef struct
{
  bool del_ini; // if set, the ini file will be deleted after loading
  
  bool set_time;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  
  bool set_date;
  uint8_t day;
  uint8_t month;
  uint8_t year;

  bool set_station;
  uint8_t station;
  
  bool set_gps;
  uint8_t gps_log; // is on or off
  uint8_t  gps_sync;  // will sync time ( logging must be on)
  
  
  bool set_survey;
  uint8_t survey[SURVEY_NAME_MAX_LENGTH];

  //the start times for recorder
  bool set_slot; // indicates that at least one slot is to be set 
  TimeSlotConfiguration slot[MAX_NUM_OF_PROTOCOL_SLOTS];
  
  
}SettingsConfiguration;


typedef enum
{
  CHECKSUM = 0,  // 2bytes
  MARKER  = 2,  // 2bytes
    
  NONCE = 4,    // 1 byte
  SET_TIME = 5,
  TIME_HOUR = 6,
  TIME_MIN = 7,
  TIME_MONTH = 8,
  TIME_DAY = 9,
  TIME_YEAR = 10,
  
  
  STATION_OPTION = 11,
  GPS_OPTION = 12,
  RANDOM_DELAY_OPTION = 13,
  
  //max 7 chars (reserved 8 bytes)
  SURVEY_NAME = 14, // 8bytes
  
  RESERVED_0 = 22,
  RESERVED_1 = 23,
  RESERVED_2 = 24,
  RESERVED_3 = 25,
  RESERVED_4 = 26,
  
  SLOTS_USED = 27,
  
  
  //0 - each of theses are 18 bytes in length
  TIME_SLOT_ITEMS_0_START_HOUR = 28,    
  TIME_SLOT_ITEMS_0_START_MIN,
  TIME_SLOT_ITEMS_0_SPAN_HOUR,
  TIME_SLOT_ITEMS_0_SPAN_MIN,
  TIME_SLOT_ITEMS_0_START_OFFSET,
  TIME_SLOT_ITEMS_0_STOP_OFFSET,
  TIME_SLOT_ITEMS_0_PROTOCOL_NAME, // 12 byte length
  
  
  //1
  TIME_SLOT_ITEMS_1_START_HOUR = 46,    
  TIME_SLOT_ITEMS_1_START_MIN,
  TIME_SLOT_ITEMS_1_SPAN_HOUR,
  TIME_SLOT_ITEMS_1_SPAN_MIN,
  TIME_SLOT_ITEMS_1_START_OFFSET,
  TIME_SLOT_ITEMS_1_STOP_OFFSET,
  TIME_SLOT_ITEMS_1_PROTOCOL_NAME, // 12 byte length
  
  //2
  TIME_SLOT_ITEMS_2_START_HOUR = 64,    
  TIME_SLOT_ITEMS_2_START_MIN,
  TIME_SLOT_ITEMS_2_SPAN_HOUR,
  TIME_SLOT_ITEMS_2_SPAN_MIN,
  TIME_SLOT_ITEMS_2_START_OFFSET,
  TIME_SLOT_ITEMS_2_STOP_OFFSET,
  TIME_SLOT_ITEMS_2_PROTOCOL_NAME, // 12 byte length
  
  //3
  TIME_SLOT_ITEMS_3_START_HOUR = 82,    
  TIME_SLOT_ITEMS_3_START_MIN,
  TIME_SLOT_ITEMS_3_SPAN_HOUR,
  TIME_SLOT_ITEMS_3_SPAN_MIN,
  TIME_SLOT_ITEMS_3_START_OFFSET,
  TIME_SLOT_ITEMS_3_STOP_OFFSET,
  TIME_SLOT_ITEMS_3_PROTOCOL_NAME, // 12 byte length
  
  //4  
  TIME_SLOT_ITEMS_4_START_HOUR = 100,    
  TIME_SLOT_ITEMS_4_START_MIN,
  TIME_SLOT_ITEMS_4_SPAN_HOUR,
  TIME_SLOT_ITEMS_4_SPAN_MIN,
  TIME_SLOT_ITEMS_4_START_OFFSET,
  TIME_SLOT_ITEMS_4_STOP_OFFSET,
  TIME_SLOT_ITEMS_4_PROTOCOL_NAME, // 12 byte length
  
  //1
  TIME_SLOT_ITEMS_5_START_HOUR = 118,    
  TIME_SLOT_ITEMS_5_START_MIN,
  TIME_SLOT_ITEMS_5_SPAN_HOUR,
  TIME_SLOT_ITEMS_5_SPAN_MIN,
  TIME_SLOT_ITEMS_5_START_OFFSET,
  TIME_SLOT_ITEMS_5_STOP_OFFSET,
  TIME_SLOT_ITEMS_5_PROTOCOL_NAME, // 12 byte length
  
  
  PAYLOAD_EOL  = 136
} Ar4PayloadIndexes;








int ParseSettings(uint8_t* blockData);

int LoadSettingsFromSDCard(void);


uint8_t GetEntry8bit(uint8_t* data, int index);
uint16_t GetEntry16bit(uint8_t* data, int index);
bool GetEntryBool(uint8_t* data, int index);
uint8_t* GetEntryString(uint8_t* data, int index, int maxlength);




#endif