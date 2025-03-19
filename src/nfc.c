

#include "nfc.h"
#include "protocol.h"
#include "eeprom.h"
#include "settings.h"
#include <stddef.h>



#define NFC_DEVICE "ST25DV64K"

#define NFC_DEVICE_I2C_ADDRESSING_BITS 0b1010    // msb 4bits used to address device
#define NFC_DEVICE_I2C_USER_MEMORY_BIT   0b1     // used to access user memory, mailbox or dynamic registers 
#define NFC_DEVICE_I2C_SYSTEM_BIT   0b0     // used to access system area
#define NFC_DEVICE_I2C_CHIP_ENABLE_BITS 0b11  // 

// and i2c address is formed by concatenating NFC_DEVICE_I2C_ADDRESSING_BITS + 
//                                            NFC_DEVICE_I2C_USER_MEMORY_BIT/NFC_DEVICE_I2C_SYSTEM_BIT +
//                                            NFC_DEVICE_I2C_CHIP_ENABLE_BITS +
//                                            I2C R/W bit (R=1, W=0)
// 
// eg a read from user memory will be 0b1010_1_11_1
// eg a write to system area will be 0b1010_0_11_0


#define NFC_DEVICE_I2C_USER_MEM_ADDR_WR  ((uint8_t)0xA6) //0b10100110
#define NFC_DEVICE_I2C_USER_MEM_ADDR_RD  ((uint8_t)0xA7)    // 0b10100111
#define NFC_DEVICE_I2C_SYSTEM_ADDR_WR    ((uint8_t)0xAE)    // 0b10101110
#define NFC_DEVICE_I2C_SYSTEM_ADDR_RD    ((uint8_t)0xAF)    //0b10101111 


#define NFC_DEVICE_I2C_USER_MEMORY_ADDR  ((uint16_t)0x0000)

// for mail box

//-- static regs
#define NFC_DEVICE_I2C_MB_MODE   ((uint16_t)0x000d)  // authorise fast transfer
#define NFC_DEVICE_I2C_MB_WDG   ((uint16_t)0x000e)  // mailbox watchdog timer

//-- dynamic regs
#define NFC_DEVICE_I2C_MB_CTRL_REG  ((uint16_t)0x2006)    // bit 0 enable/disable fast transfer mode
#define NFC_DEVICE_I2C_MAILBOX_ADDR  ((uint16_t)0x2008)


/*
MAILBOX 

Writing to mailbox first requires setting the fast transfer mode reg (bit0) to 1.
Note: This bit will need to be set back to 0 if writing to eeprom or system regs





*/

static bool isFirstCycle = TRUE;


typedef struct 
{
  
  int8_t startHour;
  int8_t startMin;
  
  int8_t spanHour;
  int8_t spanMin;
  int16_t startOffset;
  int16_t stopOffset;
  uint8_t protocolName[12];
  
  
} NfcTimeSlot_TypeDef;


/*
typedef struct 
{
  uint8_t nonce;
  bool isValid; //used to indicate data in struct is valid, should be set false after each read
  bool setTime; //if set the fllowing members are used
  int8_t hour;
  int8_t min;
  int8_t day;
  int8_t month;
  int8_t year;
  
  int slotsCount;
  NfcTimeSlot_TypeDef timeSlotItems[6];
  uint8_t surveyName[8]; //max 7 chars
  uint8_t stationOption;  // eg x = BIRX
  uint8_t gpsOptions; //eg x = GPS + LOG
  uint8_t randomDelay;
  

} NfcBlockData_TypeDef;




typedef struct 
{  
  uint16_t checksum;  
  uint16_t  marker;
  NfcBlockData_TypeDef blockData;
 
} NfcBlockDataPlusMarker_TypeDef;
*/



#define NFC_MEMORY_BLOCK_SIZE    64


#define NFC_DEVICE_IDENTIFIER    0b1010
#define NFC_CHIP_ENABLE_ADDRESS  0b11




void  NfcReadMemoryBlock(void)
{
  
  
  
}


void  NfcWriteMemoryBlock(void)
{
  
  
  
}

uint8_t senddata[] = {0,2,1,4,3,6,5,8,7,10,9};

void Nfc_InitFastTransferMode()
{

  volatile uint8_t reg1,reg2;
 volatile bool ret;
 int length;
  uint8_t recvdata[256];
  
  
  
  //check if security session is open
  
  
    //check if fast transfers have been enabled
  if ( (Nfc_GetStaticReg( MB_MODE) & 0x01) == 0)
  {
    //open security session
    ret = Nfc_OpenSecuritySession();
    
    //authorise fast transfer mode (static register)
    Nfc_SetStaticReg( MB_MODE, 0x01);
    
  }
  
  //read back to test reg has been written
  //reg1 = Nfc_GetStaticReg( MB_MODE);
  
  //set watchdog timer for mailbox
  //Nfc_SetDynamicReg(NFC_DEVICE_I2C_MB_WDG,0x01);
  
  //enable the mailbox
  Nfc_SetDynamicReg(MB_CTRL,0x01);
  

  
  
  
  //Nfc_WriteMailbox(senddata, sizeof(senddata));
  //reg2 = Nfc_GetDynamicReg(MB_CTRL);  
 // length = Nfc_GetDynamicReg(MB_LEN);

  
  
 // Nfc_ReadMailbox(recvdata, length+1);
  //reg1 = Nfc_GetDynamicReg(MB_CTRL);
}



void Nfc_FastTransferMode(FunctionalState NewState)
{
  if (NewState == ENABLE)        
       Nfc_SetDynamicReg(MB_CTRL, 0x01);
  else
       Nfc_SetDynamicReg(MB_CTRL, 0x00);
}  





void Nfc_WriteMailbox(uint8_t *src, int length)
{
	int count = 0;

        uint16_t addr = NFC_DEVICE_I2C_MAILBOX_ADDR ; //NFC_DEVICE_I2C_MAILBOX_ADDR;
        
	i2c1_start();
        i2c1_wr(NFC_DEVICE_I2C_USER_MEM_ADDR_WR);   //DIR= WR
        i2c1_wr((uint8_t)(addr>>8));
        i2c1_wr((uint8_t)(addr));
	
	while ( count < length)
	{
		i2c1_wr(src[count]);
		count++;
	}	
        i2c1_stop(); 

}


void Nfc_ReadMailbox(uint8_t *dest, int length)
{
        uint16_t addr = NFC_DEVICE_I2C_MAILBOX_ADDR; //NFC_DEVICE_I2C_MAILBOX_ADDR;
  
	i2c1_start();
        i2c1_wr(NFC_DEVICE_I2C_USER_MEM_ADDR_WR);   //DIR= WR
        i2c1_wr((uint8_t)(addr>>8));
        i2c1_wr((uint8_t)(addr));
	
	i2c1_start();   //second start bit
        i2c1_wr(NFC_DEVICE_I2C_USER_MEM_ADDR_RD);  //DIR = RD

        for (int i = 0; i < (length-1); i++)
        {
		*dest = i2c1_rd(ACK);
		dest++;
	}
	
	*dest = i2c1_rd(NACK);
        i2c1_stop(); 
        
 

}



void Nfc_WriteMemory(uint8_t *src, int length)
{
	int count = 0;

        uint16_t addr = NFC_DEVICE_I2C_USER_MEMORY_ADDR ;//NFC_DEVICE_I2C_MAILBOX_ADDR;
        
	i2c1_start();
        i2c1_wr(NFC_DEVICE_I2C_USER_MEM_ADDR_WR);   //DIR= WR
        i2c1_wr((uint8_t)(addr>>8));
        i2c1_wr((uint8_t)(addr));
	
	while ( count < length)
	{
		i2c1_wr(*src);
		src++;
		count++;
	}	
        i2c1_stop(); 

}


void Nfc_ReadStartMemory(uint8_t *dest, int length)
{
        uint16_t addr = NFC_DEVICE_I2C_USER_MEMORY_ADDR; //NFC_DEVICE_I2C_MAILBOX_ADDR;
  
	i2c1_start();
        i2c1_wr(NFC_DEVICE_I2C_USER_MEM_ADDR_WR);   // DIR= WR
        i2c1_wr((uint8_t)(addr>>8));
        i2c1_wr((uint8_t)(addr));
	
	i2c1_start();   //second start bit
        i2c1_wr(NFC_DEVICE_I2C_USER_MEM_ADDR_RD);  // DIR = RD

        for (int i = 0; i < (length-1); i++)
        {
		*dest = i2c1_rd(ACK);
		dest++;
	}
	
	*dest = i2c1_rd(NACK);
        i2c1_stop(); 
       
}


void Nfc_ReadMemory(uint8_t *dest, int length)
{
        uint16_t addr = NFC_DEVICE_I2C_USER_MEMORY_ADDR; //NFC_DEVICE_I2C_MAILBOX_ADDR;
  
	i2c1_start();
        i2c1_wr(NFC_DEVICE_I2C_USER_MEM_ADDR_WR);   // DIR= WR
        i2c1_wr((uint8_t)(addr>>8));
        i2c1_wr((uint8_t)(addr));
	
	i2c1_start();   //second start bit
        i2c1_wr(NFC_DEVICE_I2C_USER_MEM_ADDR_RD);  // DIR = RD

        for (int i = 0; i < (length-1); i++)
        {
		*dest = i2c1_rd(ACK);
		dest++;
	}
	
	*dest = i2c1_rd(NACK);
        i2c1_stop(); 
       
}


void Nfc_ClearMemory(int length)
{
	int count = 0;

        uint16_t addr = NFC_DEVICE_I2C_USER_MEMORY_ADDR ;//NFC_DEVICE_I2C_MAILBOX_ADDR;
        
	i2c1_start();
        i2c1_wr(NFC_DEVICE_I2C_USER_MEM_ADDR_WR);   //DIR= WR
        i2c1_wr((uint8_t)(addr>>8));
        i2c1_wr((uint8_t)(addr));
	
	while ( count < length)
	{
		i2c1_wr(0);
		count++;
	}	
        i2c1_stop(); 

}




void Nfc_SetStaticReg(NfcSystemCommand cmd, uint8_t data)
{
  int timeout = 1000;
	i2c1_start();
        i2c1_wr(NFC_DEVICE_I2C_SYSTEM_ADDR_WR);   //DIR= WR
	i2c1_wr((uint8_t)(cmd>>8));
        i2c1_wr((uint8_t)(cmd));
        
        i2c1_wr(data);
        
        i2c1_stop();  
        
        
        // since we are writing to static register, there will be a write delay required
        // poll for ack will deterimine when write period has finished
        while(timeout-- > 0)
        {
          i2c1_start();
          if (i2c1_wr(NFC_DEVICE_I2C_SYSTEM_ADDR_WR) == 0) break; //if ACK
          delay_us(100);
        }
        i2c1_stop(); 
  
} 

void Nfc_SetDynamicReg(NfcDynamicCommand cmd, uint8_t data)
{
	i2c1_start();
        i2c1_wr(NFC_DEVICE_I2C_USER_MEM_ADDR_WR);   //DIR= WR
	i2c1_wr((uint8_t)(cmd>>8));
        i2c1_wr((uint8_t)(cmd));
        
        i2c1_wr(data);
        
        i2c1_stop();  
        
        
  
}  


uint8_t Nfc_GetStaticReg(NfcSystemCommand cmd)
{
       uint8_t data;
       
	i2c1_start();
        i2c1_wr(NFC_DEVICE_I2C_SYSTEM_ADDR_WR);   //DIR = WR
	i2c1_wr((uint8_t)(cmd>>8));
        i2c1_wr((uint8_t)(cmd));
        
        i2c1_start();   //second start bit
        i2c1_wr(NFC_DEVICE_I2C_SYSTEM_ADDR_RD);  //DIR = RD

	data = i2c1_rd(NACK);

        i2c1_stop();        
        return data;   
  
}  



uint8_t Nfc_GetDynamicReg(NfcDynamicCommand cmd)
{
       uint8_t data;
       
	i2c1_start();
        i2c1_wr(NFC_DEVICE_I2C_USER_MEM_ADDR_WR);   //DIR = WR
	i2c1_wr((uint8_t)(cmd>>8));
        i2c1_wr((uint8_t)(cmd));
        
        i2c1_start();   //second start bit
        i2c1_wr(NFC_DEVICE_I2C_USER_MEM_ADDR_RD);  //DIR = RD

	data = i2c1_rd(NACK);

        i2c1_stop();        
        return data;   
  
}  

#define PASSWORD_LENGTH 8
#define PASSWORD_VALIDATION_CODE 0x09
uint8_t password[] = {0,0,0,0, 0,0,0,0};

bool Nfc_OpenSecuritySession()
{
  int count;
  volatile uint8_t reg;
  NfcSystemCommand addr = I2C_PWD; //password access address
  

  reg = Nfc_GetDynamicReg(I2C_SSO);
  
  if ( (reg & 0x01) == 0)
  {
    i2c1_start();
    i2c1_wr(NFC_DEVICE_I2C_SYSTEM_ADDR_WR);   // DIR= WR
    i2c1_wr((uint8_t)(addr>>8));
    i2c1_wr((uint8_t)(addr));
    
    //send password 1st time
    count = 0;
    while ( count < PASSWORD_LENGTH)
    {
            i2c1_wr(password[count]);
            count++;
    }	
    
    //send the validation code
    i2c1_wr(PASSWORD_VALIDATION_CODE);
    
    //send it again(required)
    count = 0;
    while ( count < PASSWORD_LENGTH)
    {
            i2c1_wr(password[count]);
            count++;
    }	  
    i2c1_stop(); 
    
    
    //wait for session to open
    count = 10; 
    while (count-- > 0)
    {
      reg = Nfc_GetDynamicReg(I2C_SSO);
      if (reg != 0) break;
      delay_ms(1);
    }
    
  }         
  
  return (bool)(reg != 0);
  
  
}






// check mailbox for message
// message types are... (can combine eg HOST_PUT_MSG  | RF_PUT_MSG  )
//    HOST_PUT_MSG  
//    RF_PUT_MSG    
//    HOST_MISS_MSG  
//    RF_MISS_MSG    
//    HOST_CURRENT_MSG  
//    RF_CURRENT_MSG   

bool NfcIsMessage(uint8_t messageType)
{
  uint8_t reg;
  reg = Nfc_GetDynamicReg(MB_CTRL);
  
  
  if ((reg & messageType) != 0) return TRUE;
  
  
  
  return FALSE;
}


// is the nfc IC present
bool NfcIsIc()
{
  uint8_t value;
  
  //check 5 times
  for (int i= 0;i<3;i++)
  {
    value = Nfc_GetStaticReg(IC_REF);
    if (value != IC_REF_MASK0 && value != IC_REF_MASK1) return FALSE;
    delay_ms(10);
  }
  
  return TRUE;
}



//number of time slots active
int NfcGetActiveTimeSlotsCount()
{
  
  return 2; //test 
  
}




uint8_t* NfcGetProtocolName(int timeSlot)
{
 return "HIGH"; 
}



void NfcWriteTestBlock(int nonce)
{
  
  uint16_t  sum;
  uint8_t  data[256];   
 
  
  data[MARKER] = 0x55;
  data[MARKER+1] = 0xaa;
  
  
  data[SET_TIME] = 1;
  
  data[TIME_HOUR] = 4;
  data[TIME_MIN] = 24;
  data[TIME_DAY] = 13;
  data[TIME_MONTH] = 7;
  data[TIME_YEAR] = 20;

  
  strncpy((char*)&data[SURVEY_NAME],"CAKE02",8);
  
  
  
  
  data[SLOTS_USED] = 5;
  data[STATION_OPTION] = 4; // eg x = "BIRX", "BIRA",  "BIRD", "BIRM", "BIRP"
  data[GPS_OPTION] = 2;   //eg x = GPS + LOG
  data[RANDOM_DELAY_OPTION] = 0;
  data[NONCE] = nonce;

  
    //0 - each of theses are 18 bytes in length
  data[TIME_SLOT_ITEMS_0_START_HOUR] = 5;    
  data[TIME_SLOT_ITEMS_0_START_MIN] = 23;
  data[TIME_SLOT_ITEMS_0_SPAN_HOUR] = 10;
  data[TIME_SLOT_ITEMS_0_SPAN_MIN] = 30;
  data[TIME_SLOT_ITEMS_0_START_OFFSET] = 0;
  data[TIME_SLOT_ITEMS_0_STOP_OFFSET] = 0;
  strncpy((char*)&data[TIME_SLOT_ITEMS_0_PROTOCOL_NAME],"High",12); 
  
  
  //1
  data[TIME_SLOT_ITEMS_1_START_HOUR] = 6;   
  data[TIME_SLOT_ITEMS_1_START_MIN] = 14;
  data[TIME_SLOT_ITEMS_1_SPAN_HOUR] = 3;
  data[TIME_SLOT_ITEMS_1_SPAN_MIN] = 15;
  data[TIME_SLOT_ITEMS_1_START_OFFSET] = 0;
  data[TIME_SLOT_ITEMS_1_STOP_OFFSET] = 0;
  strncpy((char*)&data[TIME_SLOT_ITEMS_1_PROTOCOL_NAME],"Low",12);
  
  //2
  data[TIME_SLOT_ITEMS_2_START_HOUR] =  14;   
  data[TIME_SLOT_ITEMS_2_START_MIN] = 55;
  data[TIME_SLOT_ITEMS_2_SPAN_HOUR] = 10;
  data[TIME_SLOT_ITEMS_2_SPAN_MIN] = 15;
  data[TIME_SLOT_ITEMS_2_START_OFFSET] = 0; 
  data[TIME_SLOT_ITEMS_2_STOP_OFFSET] = 0;
  strncpy((char*)&data[TIME_SLOT_ITEMS_2_PROTOCOL_NAME],"Tier1 Day",12);
  
  //3
  data[TIME_SLOT_ITEMS_3_START_HOUR] =  23;
  data[TIME_SLOT_ITEMS_3_START_MIN] = 55;
  data[TIME_SLOT_ITEMS_3_SPAN_HOUR] = 0;
  data[TIME_SLOT_ITEMS_3_SPAN_MIN] = 45;
  data[TIME_SLOT_ITEMS_3_START_OFFSET] = 0;
  data[TIME_SLOT_ITEMS_3_STOP_OFFSET] = 0;
  strncpy((char*)&data[TIME_SLOT_ITEMS_3_PROTOCOL_NAME],"Bat",12);
  
  //4
  data[TIME_SLOT_ITEMS_4_START_HOUR] = 0;
  data[TIME_SLOT_ITEMS_4_START_MIN] = 0;
  data[TIME_SLOT_ITEMS_4_SPAN_HOUR] = 0;
  data[TIME_SLOT_ITEMS_4_SPAN_MIN] = 0;
  data[TIME_SLOT_ITEMS_4_START_OFFSET] = 0;
  data[TIME_SLOT_ITEMS_4_STOP_OFFSET] = 0;
  strncpy((char*)&data[TIME_SLOT_ITEMS_4_PROTOCOL_NAME],"Tier1 Night",12);
  
  //1
  data[TIME_SLOT_ITEMS_5_START_HOUR] = 0;
  data[TIME_SLOT_ITEMS_5_START_MIN] = 0;
  data[TIME_SLOT_ITEMS_5_SPAN_HOUR] = 0;
  data[TIME_SLOT_ITEMS_5_SPAN_MIN] = 0;
  data[TIME_SLOT_ITEMS_5_START_OFFSET] = 0;
  data[TIME_SLOT_ITEMS_5_STOP_OFFSET] = 0;
  strncpy((char*)&data[TIME_SLOT_ITEMS_5_PROTOCOL_NAME],"Off",12);
  
  
  
  
  //calc 
  sum =0;
  //int memberOffset = offsetof(NfcBlockData_TypeDef,checksum );
  for (int i = NONCE; i < (PAYLOAD_SIZE-NONCE) ;i++) //sum the entire struct minus the checksum bytes    | sizeof(NfcBlockData_TypeDef)-4
      sum += data[i];
  data[CHECKSUM] = (uint8_t)sum;
  data[CHECKSUM+1] = (uint8_t)(sum>>8);
  

#ifdef USING_MAILBOX  
  Nfc_WriteMailbox(data, PAYLOAD_SIZE );
#else
  Nfc_WriteMemory(data, PAYLOAD_SIZE );
#endif  
  
}








int NfcModifySettings()
{
  uint8_t blockData[256];
  int length;

  
  int attempts;
  int ret =0;
  
  
  for (attempts=0;attempts<5;attempts++)
  {
    length = Nfc_GetDynamicReg(MB_LEN);
    length++; //inc by 1 to get true length
    Nfc_ReadMailbox(blockData, length);
    if (GetEntry16bit(blockData, MARKER) == 0xAA55)
    {
      ret = ParseSettings(blockData);
      if (ret == 0) break;
    }
      
    delay_ms(100);
  }
  
  
  
   return attempts*-1; 
}





bool NfcTestBlockDataForMarker()
{
  uint8_t packetData[PAYLOAD_EOL]; 
  
    
  
  Nfc_ReadMemory(packetData,MARKER+2); //only read out enough just test the marker is present

  if (GetEntry16bit(packetData, MARKER) == 0xAA55)
    return TRUE;
 
  
  isFirstCycle = FALSE;
  
  return FALSE;
}










//interface with nfc chip to read
/*
bool NfcReadBlockData()
{
  uint8_t packetData[PAYLOAD_EOL]; 
  static uint8_t lastNonce;
  Protocol_UserDef_Start_Span_TypeDef  startSpanItem;
  int index = 0; //general purpose index
  rtcTimeTypeDef rtcTime;
  rtcDateTypeDef rtcDate;
  int slotsCount;
  uint8_t *str;
  


  
  
  
 

 // for (int i=0;i<sizeof(NfcBlockData_TypeDef) - sizeof(uint16_t);i++)
  //  sum += ((uint8_t*)profile.blockdata)[i];
    
  Nfc_ReadMemory(packetData,PAYLOAD_EOL);
  
  
  
   //read via i2c 
  // copy sizeof (NfcBlockData_TypeDef)
  if (GetEntry16bit(packetData, MARKER) == 0xAA55)
  {   

    
    //nfc_data = &profile.blockData;
    
    //check if nonce has not been used
    if (GetEntry16bit(packetData, NONCE) == lastNonce) 
    {
      Nfc_ClearMemory(PAYLOAD_SIZE); 
      isFirstCycle = FALSE;
      return FALSE;  
    }
    //save the nonce
    lastNonce = GetEntry16bit(packetData, NONCE);

    
    //check if checksum is correct
    uint16_t checksum;
    uint16_t sum=0;
    
    checksum = GetEntry16bit(packetData,CHECKSUM);
    int startIndex = NONCE; //set time entry will be start index
    int endIndex = PAYLOAD_EOL;      // end of list will be used to mark end
    for (int i=startIndex; i< endIndex-startIndex ;i++) //sum the entire struct minus the checksum bytes
      sum += packetData[i];
    if (isFirstCycle == TRUE || sum != checksum)     
    {
      Nfc_ClearMemory(endIndex); 
      isFirstCycle = FALSE;
      return FALSE;  
    }
    

    
    
    //Set time if flag set
    
    if (GetEntryBool(packetData, SET_TIME) == TRUE)
    { 
      rtcTime.sec = 0;
      rtcTime.min = GetEntry8bit(packetData, TIME_MIN);
      rtcTime.hour = GetEntry8bit(packetData, TIME_HOUR);
      
      RtcSetTime(&rtcTime);
      
      rtcDate.date = GetEntry8bit(packetData, TIME_DAY);
      rtcDate.month = GetEntry8bit(packetData, TIME_MONTH);
      rtcDate.year = GetEntry8bit(packetData, TIME_YEAR);
      
      RtcSetDate(&rtcDate);
      
    }

    //set number of slots used
    slotsCount = GetEntry8bit(packetData, SLOTS_USED);
    
    int num = slotsCount < 2 ? 2 : slotsCount;
    num = num > 6 ? 6 : num;    
    SetNumOfProtocolSlots(num);

    //save protocols
    int i_slots;
    for (i_slots=0;i_slots<slotsCount;i_slots++)
    {
      // write start time and duration
      startSpanItem.startHour = GetEntry8bit(packetData,  TIME_SLOT_ITEMS_0_START_HOUR + (i_slots*TIME_SLOT_ITEMS_SIZE));
      startSpanItem.startMin = GetEntry8bit(packetData,  TIME_SLOT_ITEMS_0_START_MIN + (i_slots*TIME_SLOT_ITEMS_SIZE));
      
      startSpanItem.spanHour = GetEntry8bit(packetData,  TIME_SLOT_ITEMS_0_SPAN_HOUR + (i_slots*TIME_SLOT_ITEMS_SIZE));
      startSpanItem.spanMin = GetEntry8bit(packetData,  TIME_SLOT_ITEMS_0_SPAN_MIN + (i_slots*TIME_SLOT_ITEMS_SIZE));
      
      startSpanItem.startOffset = GetEntry8bit(packetData,  TIME_SLOT_ITEMS_0_START_OFFSET + (i_slots*TIME_SLOT_ITEMS_SIZE));
      startSpanItem.stopOffset = GetEntry8bit(packetData,   TIME_SLOT_ITEMS_0_STOP_OFFSET + (i_slots*TIME_SLOT_ITEMS_SIZE));
      
      WriteProtocolStartSpanTime(i_slots, &startSpanItem);
      
      //save protocol name
      
      index = GetIndexOfProtocolFromName( GetEntryString(packetData,TIME_SLOT_ITEMS_0_PROTOCOL_NAME + (i_slots*TIME_SLOT_ITEMS_SIZE), 12) );
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
    str = GetEntryString(packetData,SURVEY_NAME,8);
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
    EEPROM_WriteStation(GetEntry8bit(packetData, STATION_OPTION ) );// eg x = BIRX

    //save GPS options 
    EEPROM_WriteGpsOptions(GetEntry8bit(packetData, GPS_OPTION ));  //eg x = GPS + LOG
    
    //save random delay
    EEPROM_WriteRandomDelay(GetEntry8bit(packetData, RANDOM_DELAY_OPTION ));
    
   
    
    
    //clear memory once we have retrieved data 
    Nfc_ClearMemory(PAYLOAD_SIZE); 
    isFirstCycle = FALSE;
    return TRUE;
    
  }
        
        
  isFirstCycle = FALSE;
  
  return FALSE;
}

*/

//reads nfc data and if valid will update settings based on it
//
/*
bool NfcUpdateSettings()
{
  rtcTimeTypeDef rtcTime;
  rtcDateTypeDef rtcDate;
  NfcBlockData_TypeDef blockdata;
  
  blockdata = NfcReadBlockData();
  
  //data is valid so will will update settings
  if (blockdata.isValid)
  {
    //set time
    rtcTime.sec = 0;
    rtcTime.min = blockdata.min;
    rtcTime.hour = blockdata.hour;
    RtcSetTime(rtcTime);  // write new time
    
    //set date
    rtcDate.date = blockdata.day;
    rtcDate.month = blockdata.month;
    rtcDate.year = blockdata.year;
    RtcSetDate(rtcDate);  // write new date
    
    
    for (int i=0;i< blockdata.slotsCount;i++)
    {
    
      //update protocol
      //we need to get index of protocol from name
      int index = GetIndexOfProtocolFromName();
      if (index == -1) index = 0; // make index the "off" protocol   
      SaveProtocolSlot(slotIndex); //save to eepron new value
      
      
      
      //update start stop times (only applies to protocols that allow it
      PROTOCOL_USER_DEFINED_TIME
      
      
      
      
      
    }
  }

  
  int slotsCount;
  NfcTimeSlot_TypeDef timeSlotItems[6];
  uint8_t surveyName[8];
  int8_t stationName;  // eg x = BIRX
  int8_t gpsOptions; //eg x = GPS + LOG
  int8_t randomDelay;

}  
*/