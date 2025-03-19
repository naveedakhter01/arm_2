



#ifndef DSP_PROGRAM_H
#define DSP_PROGRAM_H


#include "stm32f10x.h"
#include "record.h"



//Must match 
typedef enum {
	VERSION_ID_0 = 0, 	
	VERSION_ID_1, 	
	VERSION_ID_2, 		 
	VERSION_ID_3, 	
	SAMPLING_MODE, 	
	TEST_MODE,
        TOGGLE_READ  
} CommandIdType;


typedef enum
{
  RESEVERD = 0,
  LOW_SAMPLING,
  HIGH_SAMPLING,
  BAT_SAMPLING,
  NO_SAMPLING
} SamplingOptionType;


typedef struct
{
 uint8_t majorVersion;
 uint8_t minorVersion; 
 bool valid;
  
} DspVersion_typedef;



int boot_dsp(void);
void DspEEPROM_WritePage (uint16_t addr, uint8_t *srcData, uint8_t recordlength);
bool DspEEPROM_VerifyPage(uint16_t addr, uint8_t *comparebuf, uint8_t length);
void DspEEPROM_WaitEepromStandbyState(void);  

uint8_t DspEEPROM_ReadByte (uint16_t addr);
void   DspEEPROM_WriteByte( uint16_t addr, uint8_t data);



//bool DspReadCommand(uint8_t command, uint8_t *data);
uint8_t DspReadCommand(uint8_t command);
void DspWriteCommand(uint8_t command, uint8_t data);
bool SetDspSamplingMode(DspSamplingOption option);

DspVersion_typedef *GetDspVersion();
bool IsDspInCommandMode();
bool HasDspResponded();


bool WaitForDspEvent();
bool AttemptConnectDsp(void);



#endif //__DSP_PROGRAM_H








