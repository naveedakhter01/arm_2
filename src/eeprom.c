/*******************************************************************************
* File Name          : Eeprom.c
* Author             : DOC Electronics R&D Team  
* Version            : V1.0.0
* Date               : 17/08/2009
* Description        : EEPROM driver source file.
*
*             Pin assignment:
*             ------------------------------------------
*             |  STM32F10x    |   STLM75     Pin       |
*             ------------------------------------------
*             | PB7/ SDA |   SDA         1        |
*             | PB6/ SCL |   SCL         2        |
*             | PB5/       |   OS/INT      3        |
*             | .             |   GND         4  (0V)  |
*             | .             |   GND         5  (0V)  |
*             | .             |   GND         6  (0V)  |
*             | .             |   GND         7  (0V)  |
*             | .             |   VDD         8  (5V)  |
*             ------------------------------------------
*
*******************************************************************************/

/* Includes -------------------------------------------------------------------*/

#include "eeprom.h"
#include "LCD.h"



/* Const Variables--------------------------------------------------------------*/
#define 	EEPROM_ADDR_RD			0xA1		/* 24LC128 I2C Eeprom*/
#define 	EEPROM_ADDR_WR			0xA0		/* 24LC128 I2C Eeprom*/
#define 	EEPROM_ADDR			0xA0		/* 24LC128 I2C Eeprom*/






//FSMC <-> I2C  Anti conflict delays
#define ENTRYDELAY  4
#define STOPDELAY   10

#define I2Cx    I2C2





/* Global variables -------------------------------------------------------------*/

uint8_t EEPROM_ReadByte(uint16_t addr)
{
	uint8_t data;

	i2c1_start();
        i2c1_wr(EEPROM_ADDR_WR);   //DIR= WR
	i2c1_wr((uint8_t)(addr>>8));
        i2c1_wr((uint8_t)(addr));
        
        i2c1_start();   //second start bit
        i2c1_wr(EEPROM_ADDR_RD);  //DIR = RD

	data = i2c1_rd(NACK);

        i2c1_stop();        
        return data;           
}


void EEPROM_WriteByte(uint16_t addr, uint8_t data)
{
   
	i2c1_start();
        i2c1_wr(EEPROM_ADDR_WR);   //DIR= WR
	
        i2c1_wr((uint8_t)(addr>>8));
        i2c1_wr((uint8_t)(addr));
        i2c1_wr(data);
        
        i2c1_stop();  
        EEPROM_WaitEepromStandbyState();
}

void EEPROM_WriteShort(uint16_t addr, uint16_t data)
{
   
	i2c1_start();
        i2c1_wr(EEPROM_ADDR_WR);   //DIR= WR
	
        i2c1_wr((uint8_t)(addr>>8));
        i2c1_wr((uint8_t)(addr));
        i2c1_wr((uint8_t)data);
        i2c1_wr( (uint8_t)(data >> 8) );
        i2c1_stop();  
        EEPROM_WaitEepromStandbyState();
}

uint16_t EEPROM_ReadShort(uint16_t addr)
{
	uint16_t data;

	i2c1_start();
        i2c1_wr(EEPROM_ADDR_WR);   //DIR= WR
	i2c1_wr((uint8_t)(addr>>8));
        i2c1_wr((uint8_t)(addr));
        
        i2c1_start();   //second start bit
        i2c1_wr(EEPROM_ADDR_RD);  //DIR = RD

	data = (uint16_t)i2c1_rd(ACK);
        data |= (uint16_t)i2c1_rd(NACK) << 8 ;
        
        i2c1_stop();        
        return data;           
}


void EEPROM_WritePage(uint16_t addr, uint8_t *src, int length)
{
	int count = 0;

	i2c1_start();
        i2c1_wr(EEPROM_ADDR_WR);   //DIR= WR
        i2c1_wr((uint8_t)(addr>>8));
        i2c1_wr((uint8_t)(addr));
	
	while ( count < length)
	{
		i2c1_wr(*src);
		src++;
		count++;
	}	
        i2c1_stop(); 

	EEPROM_WaitEepromStandbyState();
}





void EEPROM_ReadPage(uint16_t addr, uint8_t *dest, int length)
{
	i2c1_start();
        i2c1_wr(EEPROM_ADDR_WR);   //DIR= WR
        i2c1_wr((uint8_t)(addr>>8));
        i2c1_wr((uint8_t)(addr));
	
	i2c1_start();   //second start bit
        i2c1_wr(EEPROM_ADDR_RD);  //DIR = RD

        for (int i = 0; i < (length-1); i++)
        {
		*dest = i2c1_rd(ACK);
		dest++;
	}
	
	*dest = i2c1_rd(NACK);
        i2c1_stop(); 
}




void EEPROM_WriteStartTime(uint8_t entry, uint8_t hour, uint8_t min)
{
	uint16_t addr = EEPROM_PAGE_SIZE + EE_START_TIME + (entry*2);
	
        i2c1_start();
        i2c1_wr(EEPROM_ADDR_WR);   //DIR= WR
        i2c1_wr((uint8_t)(addr>>8));
        i2c1_wr((uint8_t)(addr));
        
	i2c1_wr (hour);
	i2c1_wr (min);
	i2c1_stop(); 
	EEPROM_WaitEepromStandbyState();
}	

void EEPROM_ReadStartTime(uint8_t entry, uint8_t *hour, uint8_t *min)
{
  uint16_t addr = EEPROM_PAGE_SIZE + EE_START_TIME + (entry*2);
  
  i2c1_start();
  i2c1_wr(EEPROM_ADDR_WR);   //DIR= WR
  i2c1_wr((uint8_t)(addr>>8));
  i2c1_wr((uint8_t)(addr));
  
  i2c1_start();   //second start bit
  i2c1_wr(EEPROM_ADDR_RD);  //DIR = RD
  
  *hour = i2c1_rd(ACK);
  *min = i2c1_rd(NACK);
  i2c1_stop(); 
}	

void EEPROM_WriteStopTime(uint8_t entry, uint8_t hour, uint8_t min)
{
  uint16_t addr = EEPROM_PAGE_SIZE + EE_DURATION + (entry*2);
  
  i2c1_start();
  i2c1_wr(EEPROM_ADDR_WR);   //DIR= WR
  i2c1_wr((uint8_t)(addr>>8));
  i2c1_wr((uint8_t)(addr));
  
  i2c1_wr(hour);
  i2c1_wr(min);
  i2c1_stop(); 
  EEPROM_WaitEepromStandbyState();
}	

void EEPROM_ReadStopTime(uint8_t entry, uint8_t *hour, uint8_t *min)
{
  uint16_t addr = EEPROM_PAGE_SIZE + EE_DURATION + (entry*2);
  
  i2c1_start();
  i2c1_wr(EEPROM_ADDR_WR);   //DIR= WR
  i2c1_wr((uint8_t)(addr>>8));
  i2c1_wr((uint8_t)(addr));
  
  i2c1_start();   //second start bit
  i2c1_wr(EEPROM_ADDR_RD);  //DIR = RD
  
  *hour = i2c1_rd(ACK);
  *min = i2c1_rd(NACK);
  i2c1_stop();
}	


/******************************************************************************
* Read ERROR CODE 
*
*******************************************************************************/
uint8_t EEPROM_ReadErrorCode(void)
{
        uint8_t code;
  
        uint16_t Addr = EEPROM_PAGE_SIZE + EE_ERROR_FLAG;

        i2c1_start();
        i2c1_wr(EEPROM_ADDR_WR);   //DIR= WR
        i2c1_wr((uint8_t)(Addr>>8));
        i2c1_wr((uint8_t)(Addr));
	
	i2c1_start();   //second start bit
        i2c1_wr(EEPROM_ADDR_RD);  //DIR = RD
	
        code = i2c1_rd(NACK); 
        i2c1_stop();

      return(code);
}

void EEPROM_WriteErrorCode(uint8_t code)
{
	//int count = 0;
	uint16_t addr = EEPROM_PAGE_SIZE + EE_ERROR_FLAG;
	

	 EEPROM_WriteByte((addr),code);

}	



void EEPROM_WriteUserDefSamplingType(uint8_t sampling)
{
  EEPROM_WriteByte(EE_USER_DEF_SAMPLING_TYPE, sampling);
}	

uint8_t EEPROM_ReadUserDefSamplingType()
{
  return EEPROM_ReadByte(EE_USER_DEF_SAMPLING_TYPE);
}















void EEPROM_WaitEepromStandbyState(void)      
{
   uint8_t ack;
   int timeout;
  ack = 1;
  
  timeout = 500;
  while((ack) && (timeout))
  {
    delay_us(10);
    i2c1_start();
    ack = i2c1_wr(EEPROM_ADDR_WR);   //DIR= WR
    timeout--;
  } 
  i2c1_stop();
}





void EEPROM_ReadSurveyName(uint8_t *desData)
{
	int i;
        uint16_t addr = EEPROM_PAGE_SIZE + EE_SURVEY_NAME;

        i2c1_start();
        i2c1_wr(EEPROM_ADDR_WR);   //DIR= WR
        i2c1_wr((uint8_t)(addr>>8));
        i2c1_wr((uint8_t)(addr));
	
        i2c1_start();   //second start bit
        i2c1_wr(EEPROM_ADDR_RD);  //DIR = RD
  
        for(i=0;i<7;i++)
        {
          *desData = i2c1_rd(ACK);
           //if (*DesData == 0x00) break;  
	    desData++;
        }        
        
        i2c1_rd(NACK);
        i2c1_stop();
}	


void EEPROM_WriteSurveyName(uint8_t *desData)
{
        uint16_t addr = EEPROM_PAGE_SIZE + EE_SURVEY_NAME;
	int i;

        for(i=0;i<MAGIC(7);i++)
        {
  	    EEPROM_WriteByte((addr+i),*desData);
            //if (*DesData == 0x00) break;
	    desData++;
        }
}	





// protocol num is the is the index of the user 
void EEPROM_WriteSelectedProtocol(int protocolNum, int protocolListIndex)
{
  EEPROM_WriteByte( EE_USER_SELECT_PROTOCOL_BASE + protocolNum, protocolListIndex);
}	


uint8_t EEPROM_ReadSelectedProtocol(int protocolNum)
{
  return EEPROM_ReadByte(EE_USER_SELECT_PROTOCOL_BASE + protocolNum);
}	






void EEPROM_WriteProtocolUserDefinedStartSpanTime(int protocolIndex, Protocol_UserDef_Start_Span_TypeDef *startSpanItem)
{
  
  int i;
  uint8_t *pStartSpanItem;
  
  pStartSpanItem = (uint8_t*)startSpanItem; 
  uint16_t addr = ( protocolIndex * sizeof(Protocol_UserDef_Start_Span_TypeDef) ) + EE_PROTOCOL_START_SPAN_TIME_BASE;     
  
  i2c1_start();
  i2c1_wr(EEPROM_ADDR_WR);   //DIR= WR
  i2c1_wr((uint8_t)(addr>>8));
  i2c1_wr((uint8_t)(addr));
  
  for (i= 0;i< sizeof(Protocol_UserDef_Start_Span_TypeDef);i++)
  {
    i2c1_wr(pStartSpanItem[i]);
  }
  i2c1_stop(); 
  EEPROM_WaitEepromStandbyState();
}	

Protocol_UserDef_Start_Span_TypeDef *EEPROM_ReadProtocolUserDefinedStartSpanTime(int protocolIndex)
{
  static Protocol_UserDef_Start_Span_TypeDef startSpanItem;
  uint8_t *pStartSpanItem;
  int i;
  
  pStartSpanItem = (uint8_t*)&startSpanItem;
  uint16_t addr = ( protocolIndex * sizeof(Protocol_UserDef_Start_Span_TypeDef) ) + EE_PROTOCOL_START_SPAN_TIME_BASE;    

  i2c1_start();
  i2c1_wr(EEPROM_ADDR_WR);   //DIR= WR
  i2c1_wr((uint8_t)(addr>>8));
  i2c1_wr((uint8_t)(addr));
  
  i2c1_start();   //second start bit
  i2c1_wr(EEPROM_ADDR_RD);  //DIR = RD
	  
  for (i= 0;i< sizeof(Protocol_UserDef_Start_Span_TypeDef)-1;i++)
  {
    pStartSpanItem[i] = i2c1_rd(ACK);
  }
  
  pStartSpanItem[i] = i2c1_rd(NACK);  // the last one needs to send NACK
  
  i2c1_stop();
  
  
  return &startSpanItem;
}	
                  
                  
/******************************************************************************
* Read HI/LO Sampling flag 
*
*******************************************************************************/
uint8_t EEPROM_ReadStation(void)
{
  return EEPROM_ReadByte(EEPROM_PAGE_SIZE + EE_STATION);

}

void EEPROM_WriteStation(uint8_t value)
{ 
  EEPROM_WriteByte(EEPROM_PAGE_SIZE + EE_STATION, value);
}	


/******************************************************************************
*get/set temp sensor id
*
* id for temp sensors
*******************************************************************************/
uint8_t EEPROM_ReadTempSensorId(void)
{
  return EEPROM_ReadByte(EEPROM_PAGE_SIZE + EE_TEMP_SENSOR_ID);

}

void EEPROM_WriteTempSensorId(uint8_t value)
{ 
  EEPROM_WriteByte(EEPROM_PAGE_SIZE + EE_TEMP_SENSOR_ID, value);
}	




/******************************************************************************
* Read HI/LO Sampling flag 
*
*******************************************************************************/
#define MAX_RANDOM_DELAY 60
uint8_t EEPROM_ReadRandomDelay(void)
{
  uint8_t val = EEPROM_ReadByte(EEPROM_PAGE_SIZE + EE_RANDOM_DELAY);
  if (val >= MAX_RANDOM_DELAY) val = 0;
  return val;

}

uint8_t EEPROM_WriteRandomDelay(uint8_t value)
{ 
  if (value == 255) value = MAX_RANDOM_DELAY-1;
  else if (value >= MAX_RANDOM_DELAY) value = 0;
  EEPROM_WriteByte(EEPROM_PAGE_SIZE + EE_RANDOM_DELAY, value);
  return value;
}	



/******************************************************************************
* Read GPS Options flag 
*
*******************************************************************************/
#define MAX_GPS_OPTIONS 3
uint8_t EEPROM_ReadGpsOptions(void)
{
  uint8_t val = EEPROM_ReadByte(EEPROM_PAGE_SIZE + EE_GPS_OPTIONS);
    if (val >= MAX_GPS_OPTIONS) val = 0;

  return val;

}

uint8_t EEPROM_WriteGpsOptions(uint8_t value)
{ 
  if (value == 255) value = MAX_GPS_OPTIONS-1;
  else if (value >= MAX_GPS_OPTIONS) value = 0;
  EEPROM_WriteByte(EEPROM_PAGE_SIZE + EE_GPS_OPTIONS, value);
  return value;
}	







/******************************************************************************
* Contrast
*
*******************************************************************************/
uint8_t EEPROM_ReadContrast(void)
{
  uint8_t val = EEPROM_ReadByte(EEPROM_PAGE_SIZE + EE_CONTRAST);
  if (val >= MAX_CONTRAST_VALUE) val = DEFAULT_CONTRAST_VALUE;
  return val;

}

uint8_t EEPROM_WriteContrast(uint8_t value)
{
  if (value == 255) value = DEFAULT_CONTRAST_VALUE;
  else if (value >= MAX_CONTRAST_VALUE) value = MAX_CONTRAST_VALUE;
  EEPROM_WriteByte(EEPROM_PAGE_SIZE + EE_CONTRAST, value);
  return value;
}	



/******************************************************************************
* NUMBER OF PROTOCOL START/STOPS TIMES
*
*******************************************************************************/
uint8_t EEPROM_ReadNumStartStopTimes(void)
{
  uint8_t val = EEPROM_ReadByte(EEPROM_PAGE_SIZE + EE_NUM_START_STOP_TIMES);
  if (val > NUM_OF_VIEWABLE_PROTOCOL_SLOTS) val = DEFAULT_NUM_PROTOCOL_SLOTS; //if slots are greater than 6
  if (val < DEFAULT_NUM_PROTOCOL_SLOTS) val = DEFAULT_NUM_PROTOCOL_SLOTS;  //if slots are less than 2
  return val;

}

uint8_t EEPROM_WriteNumStartStopTimes(uint8_t value)
{
  if (value > NUM_OF_VIEWABLE_PROTOCOL_SLOTS || value < DEFAULT_NUM_PROTOCOL_SLOTS ) value = DEFAULT_NUM_PROTOCOL_SLOTS; //range 2..6
  EEPROM_WriteByte(EEPROM_PAGE_SIZE + EE_NUM_START_STOP_TIMES, value);
  return value;
}	






/**************************************************EOF*********************************************/