/*******************************************************************************
* File Name          : Eeprom.c
* Author             : DOC Electronics R&D Team  
* Version            : V1.0.0
* Date               : 17/08/2009
* Description        : EEPROM driver source file.
*
*
*******************************************************************************/

/* Includes -------------------------------------------------------------------*/

#include "eeprom.h"










//FSMC <-> I2C  Anti conflict delays
#define ENTRYDELAY  4
#define STOPDELAY   10

#define I2Cx    I2C2


static void EEPROM_WaitEepromStandbyState(void);


/* Global variables -------------------------------------------------------------*/

uint8_t EEPROM_ReadByte (uint16_t addr)
{
  uint8_t data1;
  
  i2c1_start();
  i2c1_wr(EEPROM_ADDR_WR);   //DIR= WR
  i2c1_wr((uint8_t)(addr>>8));
  i2c1_wr((uint8_t)(addr));
  
  i2c1_start();   //second start bit
  i2c1_wr(EEPROM_ADDR_RD);  //DIR = RD
  
  data1 = i2c1_rd(NACK);
  
  i2c1_stop();        
  return data1;           
}


void   EEPROM_WriteByte( uint16_t addr, uint8_t data)
{ 
  i2c1_start();
  i2c1_wr(EEPROM_ADDR_WR);   //DIR= WR
  
  i2c1_wr((uint8_t)(addr>>8));
  i2c1_wr((uint8_t)(addr));
  i2c1_wr(data);
  
  i2c1_stop();  
  EEPROM_WaitEepromStandbyState();
}

void   EEPROM_WriteShort( uint16_t addr, uint16_t data)
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

uint16_t EEPROM_ReadShort (uint16_t addr)
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

//PROBLEM WITH THIS CODE - doesn't work in certain situations
void EEPROM_WritePage (uint16_t addr, uint8_t *src, int length)
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



/**************************************************EOF*********************************************/