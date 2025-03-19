

#ifndef _NFC_H_
#define _NFC_H_


#include "stm32f10x.h"
#include "i2c_bitbang.h"



#define USING_MAILBOX    //uses mailbox as method of transfer

//reference value returned from IC will either be 0x24 or 0x26..
#define IC_REF_MASK0 0x24
#define IC_REF_MASK1 0x26



#define HOST_PUT_MSG  0x02
#define RF_PUT_MSG    0x04
#define HOST_MISS_MSG  0x10
#define RF_MISS_MSG    0x20
#define HOST_CURRENT_MSG  0x40
#define RF_CURRENT_MSG    0x80

typedef enum 
{
  
  MB_MODE = 0x000d,
  MB_WDG = 0x000E,
  IC_REF = 0x0017,
  I2C_PWD = 0x0900   //password access
  
  
}NfcSystemCommand;


typedef enum 
{
  I2C_SSO = 0x2004, //is security session open
  MB_CTRL = 0x2006,
  MB_LEN = 0x2007
  
  
  
}NfcDynamicCommand;










void Nfc_FastTransferMode(FunctionalState NewState);

void Nfc_SetStaticReg(NfcSystemCommand cmd, uint8_t data);
uint8_t Nfc_GetStaticReg(NfcSystemCommand cmd);

void Nfc_SetDynamicReg(NfcDynamicCommand cmd, uint8_t data);
uint8_t Nfc_GetDynamicReg(NfcDynamicCommand cmd);


void Nfc_WriteMailbox(uint8_t *src, int length);
void Nfc_ReadMailbox(uint8_t *dest, int length);


void Nfc_WriteMemory(uint8_t *src, int length);
void Nfc_ReadMmeory(uint8_t *dest, int length);


bool NfcTestBlockDataForMarker();

//bool NfcReadBlockData();

//void NfcWriteTestBlock(int nonce);

void Nfc_InitFastTransferMode();


bool Nfc_OpenSecuritySession();

bool NfcIsMessage(uint8_t messageType);
int NfcModifySettings();
int NfcUpdateSettings(uint8_t* message);


bool NfcIsIc();


#endif

/************************************EOF***************************************/