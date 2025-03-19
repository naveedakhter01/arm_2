/**************************************************************************
* File Name          : Eeprom.h
* Author             :  DOC Electronics Development Team
* Version            : V1.0.0
* Date               : 17/08/2009
* Description        : This file contains all the functions prototypes for the
*                      Eeprom driver.
***************************************************************************/

#ifndef __EEPROM_H
#define __EEPROM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "i2c_bitbang.h"
#include "protocol.h"
#include "log.h"


#define EEPROM_512


#ifdef EEPROM_512
#define	 EEPROM_SIZE 65536
#define	 EEPROM_PAGE_SIZE 128
#define	 EEPROM_NO_OF_PAGE EEPROM_SIZE/EEPROM_PAGE_SIZE
#else
#define	 EEPROM_SIZE 16384
#define	 EEPROM_PAGE_SIZE 64
#define	 EEPROM_NO_OF_PAGE 256
#endif
/* Variable Declaration ---------------------------------------------------------*/

#define EEPROM_WRITE_SUCCESS	0x01
#define EEPROM_WRITE_ERROR		0x02
#define EEPROM_READ_SUCCESS	0x03
#define EEPROM_READ_ERROR		0x04



//EEPROM offsets



#define EE_NUM_START_STOP_TIMES 6


#define EE_CONTRAST    7 //1 byte
#define EE_STATION     8 //(1byte)
#define EE_USER_DEF_SAMPLING_TYPE 9 //(1byte)
#define EE_ERROR_FLAG    10        //(1byte)
#define EE_TEST_FLAG     11//(2bytes)
#define EE_TEMP_SENSOR_ID   13 //(1 byte) temp sensor selected
#define EE_GPS_LOGGED 14 //(1 byte)
#define EE_RANDOM_DELAY 15 //(1 byte)
#define EE_GPS_FIRST_START 16 //(1 byte)
#define EE_GPS_OPTIONS 17 //(1 byte)

#define EE_USER_SELECT_PROTOCOL_BASE 18 //(6 byte - up to 6 protocols) 

#define EE_SURVEY_NAME   24 //(8bytes)

#define EE_START_TIME  32 //(12bytes) 6slots max*2
#define EE_DURATION    44 //(12bytes) 6slots max*2






#define EE_LOG_COUNT_ADDR ((uint16_t)510)       //(2 bytes)  //REMOVE NOT USED
#define EE_LOG_BASE_ADDRESS ((uint16_t)512)  // 



#define EE_PROTOCOL_MAX_START_SPAN_TIMES 6  // maximum start span times for up to 6 user defined protocols . we will ONLY USE 2 though
#define EE_PROTOCOL_START_SPAN_TIME_BASE 200  // where the user changable start stop times for the protocols are stored, 2 bytes start1, 2 bytes span1, 2 bytes 








/* Function Declaration--------------------------------------------------------- */

void I2C_Eeprom_Init(void);
uint8_t EEPROM_ReadByte (uint16_t addr);
void  EEPROM_WriteByte( uint16_t addr, uint8_t data);
void   EEPROM_WriteShort( uint16_t addr, uint16_t data);
uint16_t EEPROM_ReadShort (uint16_t addr);

uint8_t	I2C_EEPROM_Read();
//uint8_t EEPROM_ReadPage (int16_t addr, uint8_t length);
void EEPROM_ReadPage (uint16_t addr, uint8_t *dest, int length);
void EEPROM_WritePage (uint16_t addr, uint8_t *src, int length);
void EEPROM_WriteStartTime( uint8_t entry, uint8_t hour, uint8_t min);
void EEPROM_ReadStartTime(uint8_t entry, uint8_t *hour, uint8_t *min);
void EEPROM_WriteStopTime(uint8_t entry, uint8_t hour, uint8_t min);
void EEPROM_ReadStopTime(uint8_t entry, uint8_t *hour, uint8_t *min);
void EEPROM_ReadSurveyName(uint8_t *desData);
void EEPROM_WriteSurveyName(uint8_t *desData);

void EEPROM_WriteUserDefSamplingType(uint8_t sampling);
uint8_t EEPROM_ReadUserDefSamplingType();

uint8_t EEPROM_ReadErrorCode(void);
void EEPROM_WriteErrorCode(uint8_t code);


void EEPROM_WaitEepromStandbyState(void) ;
void I2C_EE_WaitEepromStandbyState(void)  ;




uint8_t EEPROM_ReadStation(void);
void EEPROM_WriteStation(uint8_t value);


uint8_t EEPROM_ReadTempSensorId(void);
void EEPROM_WriteTempSensorId(uint8_t value);


uint8_t EEPROM_ReadStation(void);
void EEPROM_WriteStation(uint8_t value);



int16_t EEPROM_ReadProtocolCount(void);
void EERPOM_WriteProtocolCount(int16_t count);
uint8_t *ReadProtocolListItem(int index);
void WriteProtocolListItem(int index, uint8_t *itemData);

uint8_t EEPROM_ReadStation(void);
void EEPROM_WriteStation(uint8_t value);

uint8_t EEPROM_ReadRandomDelay(void);
uint8_t EEPROM_WriteRandomDelay(uint8_t value);

//read and write the user defined start and span time to eeprom, used when protocols start and span times are not locked eg in manual mode 
void EEPROM_WriteProtocolUserDefinedStartSpanTime(int protocolIndex, Protocol_UserDef_Start_Span_TypeDef *startSpanItem);
Protocol_UserDef_Start_Span_TypeDef *EEPROM_ReadProtocolUserDefinedStartSpanTime(int protocolIndex);



void EEPROM_WriteSelectedProtocol(int protocolNum, int protocolListIndex);
uint8_t EEPROM_ReadSelectedProtocol(int protocolNum);



uint8_t EEPROM_ReadGpsOptions(void);
uint8_t EEPROM_WriteGpsOptions(uint8_t value);


uint8_t EEPROM_ReadContrast(void);
uint8_t EEPROM_WriteContrast(uint8_t value);


uint8_t EEPROM_ReadNumStartStopTimes(void);
uint8_t EEPROM_WriteNumStartStopTimes(uint8_t value);



#endif  /* __EEPROM_H_ */
