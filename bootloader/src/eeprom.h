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


#define EEPROM_512

/* Const Variables--------------------------------------------------------------*/
#define 	EEPROM_ADDR_RD			0xA1		/* 24LC128 I2C Eeprom*/
#define 	EEPROM_ADDR_WR			0xA0		/* 24LC128 I2C Eeprom*/
#define 	EEPROM_ADDR			0xA0		/* 24LC128 I2C Eeprom*/

#define ACK       0x00
#define NACK      0x80



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



/* Function Declaration--------------------------------------------------------- */
uint8_t EEPROM_ReadByte (uint16_t addr);
void  EEPROM_WriteByte( uint16_t addr, uint8_t data);
void   EEPROM_WriteShort( uint16_t addr, uint16_t data);
uint16_t EEPROM_ReadShort (uint16_t addr);
void EEPROM_ReadPage (uint16_t addr, uint8_t *dest, int length);
void EEPROM_WritePage (uint16_t addr, uint8_t *src, int length);


#endif  /* __EEPROM_H_ */
