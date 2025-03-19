/******************************************************************************
* @file    gps.c
* @author  DOC
* @version V1.0
* @date    10/07/2011
* @brief   gps m10372 module
******************************************************************************/

#include "gps.h"
#include "FatSD.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "eeprom.h"
#include "protocol.h"
#include "math.h"
#include "sysclk.h"




//#define USE_DUMMY_LOCATION  //uses a valid dummy gps location instead to allow testing indoors 
#define HDOP_THRESHOLD 15 //hdop must be under this for valid location 

#define DEFAULT_BAUDRATE BAUD_9600
#define ALTERNATIVE_BAUDRATE BAUD_4800


//math functions
double Deg2rad(double deg);
double Rad2deg(double rad); 
int ConvertToInt(char* string, int length);
bool IsTimedOut(void);
uint8_t IsChecksumValid(uint8_t* message);
void CreateFilename();

void GetGpsMessage(void);

typedef struct
{
  int16_t StartIndex;      //index pointer to message. 
  int16_t Count;           // message length 
  int8_t  MessageType;     //Type of message  
}NMEAResponse_TypeDef;


typedef struct
{
    bool isValidMessage; //unitl this is true all other variables invalid
    bool isStartChar;  //if startchar present in command
    bool messageDetect; // used to check if messages are coming, same as isValidMessage but is reset manualy
    uint8_t length; //length of command in bytes
} gpsMessageInfo_typedef;

#define MESSAGE_SIZE 128
#define ELEMENTS_SIZE 30
static uint8_t messageBuffer[MESSAGE_SIZE];
char*  messageElements[ELEMENTS_SIZE];
static gpsMessageInfo_typedef decodedMessageInfo = {.isValidMessage = FALSE, .isStartChar = FALSE, .length = 0};

static bool gpsStartedFlag = FALSE;


static uint32_t baudrate = DEFAULT_BAUDRATE;


#define TABLE_BUFFER_SIZE 8 //must be power of 2

uint8_t TxBuffer[64];
uint8_t TxCounter = 0;
uint8_t NbrOfDataToTransfer;

uint8_t *dummy_message = "$GPRMC,184050.84,A,3907.3839,N,12102.4772,W,00.0,000.0,080301,15,E*54\r";
//uint8_t *dummy_message = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r";

//uint8_t *dummy_message = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,1.9,545.4,M,46.9,M,,*46\r";




static GlobalPosition_TypeDef GlobalPosition;
static  RMCData_TypeDef RMCData;
//static ZDAData_TypeDef ZDAData;


static bool doSyncTime = FALSE;
static bool hasTimeSynced = FALSE;  
static int32_t timediff = 0;

/* Holds gps data */ 
//static struct gpsPositionData gpsData;
static GPS_Position_TypeDef gpsCurrentData; 
static GPS_Position_TypeDef gpsLastData; 



//setup array of gps coordinates
#define NUMBER_OF_GPS_SAMPLES 2
GpsCoordinates_TypeDef gpsCoorBuffer[NUMBER_OF_GPS_SAMPLES];                                   
int gpsCoorBufferCount;
bool haveGpsCoordinates = FALSE;  

void InitGpsCoordinateAveraging(void)
{
  //number of saved gps samples
  gpsCoorBufferCount =0;
  
  for (int i=0;i< NUMBER_OF_GPS_SAMPLES;i++)
  {
    gpsCoorBuffer[i].latitude = 0.0;
    gpsCoorBuffer[i].longitude = 0.0;
    gpsCoorBuffer[i].hdop = INT32_MAX;
    gpsCoorBuffer[i].northSouth[0] = 0;
    gpsCoorBuffer[i].eastWest[0] = 0;
    gpsCoorBuffer[i].latStr[0] = 0; 
    gpsCoorBuffer[i].longStr[0] = 0;
  }
}


//adds gps sample to averaging buffer
void AddGpsSample(GpsCoordinates_TypeDef gpsCoordinate)
{
  int i;
  int insert_index; //set out of range
  bool toInsert = FALSE;
  //determine where in buffer to place sample based on hdop value
  
  for(i = 0; i< NUMBER_OF_GPS_SAMPLES;i++)
  {
    if (gpsCoordinate.hdop <= gpsCoorBuffer[i].hdop)
    {  
      insert_index = i;
      toInsert = TRUE;
      break;
    }
  }
  
  if (toInsert) 
  {
    for(i = NUMBER_OF_GPS_SAMPLES-1; i > insert_index;i--)
    {
         gpsCoorBuffer[i] = gpsCoorBuffer[i-1];
    }
    
    gpsCoorBuffer[insert_index] = gpsCoordinate;
    
    if (gpsCoorBufferCount < NUMBER_OF_GPS_SAMPLES)  gpsCoorBufferCount++;
  }
  
}


//GpsCoordinates_TypeDef* GetAveragedGpsCoordinates(void)
//{
//  static GpsCoordinates_TypeDef gpsCoor = {.hdop = INT32_MAX, .latitude = 0.0, .longitude = 0.0};
//  int bufferCount = gpsCoorBufferCount; 
//  int downcount = bufferCount; 
//  int totalcountsums = 0;
//  double weightedMultiplier;
//  
//  //create the weighted values 
//  for (int i=0;i<downcount;i++) totalcountsums += i+1;
//  weightedMultiplier = (double)(1.0 / totalcountsums);
//  
//  for (int i=0; i < bufferCount;i++)
//  {
//    gpsCoor.latitude += gpsCoorBuffer[i].latitude * (weightedMultiplier * downcount);
//    gpsCoor.longitude += gpsCoorBuffer[i].longitude * (weightedMultiplier * downcount);
//    downcount--;
//  }
//  
//  return &gpsCoor;
//}



//
GpsCoordinates_TypeDef* GetGpsCoordinates(void)
{
  return &gpsCoorBuffer[0];
}




/***********************************************************************
* INIT GPS UART
*
*
*/

void GpsInit(void)
{
 NVIC_InitTypeDef NVIC_InitStructure;
 USART_InitTypeDef USART_InitStructure; 
 GPIO_InitTypeDef GPIO_InitStructure; 
 

 //init on off gpio pin
  GPIO_InitStructure.GPIO_Pin =  GPS_ON_OFF_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPS_ON_OFF_PORT, &GPIO_InitStructure);
  GPIO_WriteBit(GPS_ON_OFF_PORT, GPS_ON_OFF_PIN, Bit_RESET);
 
/* USARTx configuration ------------------------------------------------------*/
  /* USARTx configured as follow:
        - BaudRate = 9600 baud  
        - Word Length = 8 Bits
        - Two Stop Bit
        - Odd parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

  /* Enable UART clock */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 

  //GPIO_PinRemapConfig((uint32_t)0x001D2000, ENABLE);   //CAN remap check errata
  
  
  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin =  GPS_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPS_TX_PORT, &GPIO_InitStructure);
    
  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = GPS_RX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPS_RX_PORT, &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(USART2, &USART_InitStructure);
    
  /* Enable USART */
  USART_Cmd(USART2, ENABLE);


  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);   

//  /* Enable the EVAL_COM1 Transmoit interrupt: this interrupt is generated when the 
//     EVAL_COM1 transmit data register is empty */  
//  //USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

  /* Enable the EVAL_COM1 Receive interrupt: this interrupt is generated when the 
     EVAL_COM1 receive data register is not empty */
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 
  NbrOfDataToTransfer =0; //reset
  
  InitMessageBuffer();
}



void DisableGpsUSART(void)
{
  USART_ITConfig(USART2, USART_IT_RXNE, DISABLE); 
  USART_Cmd(USART2, DISABLE);
}

void EnableGpsUSART(void)
{
  USART_InitTypeDef USART_InitStructure; 
   
  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  /* USART configuration */
  USART_Init(USART2, &USART_InitStructure);
    
  /* Enable USART */
  USART_Cmd(USART2, ENABLE);
   
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 
  USART_Cmd(USART2, ENABLE);
}


//puts  io for gps module into the lowest power state 
void GpsIOLowPower(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;   
  
  GPIO_WriteBit(GPS_ON_OFF_PORT, GPS_ON_OFF_PIN, Bit_RESET);
  GPIO_InitStructure.GPIO_Pin =  GPS_ON_OFF_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPS_ON_OFF_PORT, &GPIO_InitStructure);
  GPIO_WriteBit(GPS_ON_OFF_PORT, GPS_ON_OFF_PIN, Bit_RESET);
 
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  
  GPIO_InitStructure.GPIO_Pin =  GPS_TX_PIN;
  GPIO_Init(GPS_TX_PORT, &GPIO_InitStructure);
    
  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = GPS_RX_PIN;
  GPIO_Init(GPS_RX_PORT, &GPIO_InitStructure);
  
  
  
  USART_Cmd(USART2, DISABLE);
  USART_ITConfig(USART2, USART_IT_RXNE, DISABLE); 
  
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE); 
  
  
}





void GPS_On(void)
{
   GPIO_WriteBit(GPS_ON_OFF_PORT, GPS_ON_OFF_PIN, Bit_SET);
}

void GPS_Off(void)
{
   GPIO_WriteBit(GPS_ON_OFF_PORT, GPS_ON_OFF_PIN, Bit_RESET);
}



bool StartGPSModule(void) //pulse on_off pin for ~100ms
{
  int gpsMessageTimeout;
  bool ret = TRUE;
 

  gpsMessageTimeout = 10;
  do 
  {
    GPS_On();
    delay_ms(210);
    GPS_Off();
    
    gpsMessageTimeout--;
  } while(!IsGpsModuleOnWithAutoBaud() && (gpsMessageTimeout > 0)); //check if messages are coming in from gps module 
  
  if(gpsMessageTimeout == 0)
  {
    ret = FALSE;
  }
  
  gpsStartedFlag = ret;
  
  // if we have started then enable zda messages
  //if (gpsStartedFlag)
 // {
  //  GpsEnableZdaMessage();
  //}
  
  
  return ret;
}

bool StopGPSModule(void) //pulse on_off pin for ~200ms
{
  int gpsMessageTimeout;
  //uint8_t text_string[256];
  bool ret = TRUE;
 
  gpsMessageTimeout = 10;
  do 
  { 
    GPS_On();
    delay_ms(210);
    GPS_Off();
    
    gpsMessageTimeout--;
  } while(IsGpsModuleOn() && (gpsMessageTimeout > 0)); //check if messages are coming in from gps module 
  
  if(gpsMessageTimeout == 0) ret = FALSE;
  
  gpsStartedFlag = ret;
  return ret;
}



bool IsGpsOn()
{
  return gpsStartedFlag;
}


//makes sure gps is off if acidentally turns on on power up
//gps connections must have been init first
//
// if data detected coming through then turn off
void TurnOffGpsIfActive(void)
{
  GetGpsMessage();
  if(decodedMessageInfo.isValidMessage) StopGPSModule();

  GpsIOLowPower();
  
}



void ReadToBuffer(void)
{
  uint8_t indata;
  indata = (uint8_t)(USART_ReceiveData(USART2));
  GPSFifoEnqueue(indata); 
}


void InitMessageBuffer(void)
{
  InitGpsCoordinateAveraging();  
  GPSFifoClear();
  decodedMessageInfo.isStartChar = FALSE;
  decodedMessageInfo.length = 0;
  decodedMessageInfo.isValidMessage = FALSE;
}

#define START_COMMAND_CHAR '$'
#define END_COMMAND_CHAR '\r'
void GetGpsMessage(void)
{
    uint8_t index;
    uint8_t data;
 
    if( (decodedMessageInfo.isValidMessage) || (decodedMessageInfo.length == MESSAGE_SIZE) ) //if last read returned valid command or buffer was full then reset states
    {
        decodedMessageInfo.isStartChar = FALSE; //if last call to this function ret with valid command then reset startchar flag
        decodedMessageInfo.length = 0; //if last call to this function ret iwith valid command then reset buffer index
        decodedMessageInfo.isValidMessage = FALSE;
    }
    for(index=decodedMessageInfo.length;index<MESSAGE_SIZE;index++)
    {
        if(GPSFifoCount() > 0) {
            data = GPSFifoDequeue();
            if(data == START_COMMAND_CHAR) {
                index = 0;
                decodedMessageInfo.isStartChar = TRUE;
        }
            messageBuffer[index] = data;
            if(data == END_COMMAND_CHAR) {
                index++;
                decodedMessageInfo.isValidMessage = TRUE;
                decodedMessageInfo.messageDetect = TRUE;
                break;
            }
        }
        else {
            break; // no more data to retrieve so exit
        }
    }
    decodedMessageInfo.length = index;  //save the length
}


void WriteFromBuffer(void)
{

   //delay_us(500);
  /* Write one byte to the transmit data register */
    USART_SendData(USART2, TxBuffer[TxCounter++]);

    if(TxCounter == NbrOfDataToTransfer)
    {
      /* Disable the Transmit interrupt */
      USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
      
      NbrOfDataToTransfer =0;
    }
}


/*
void GpsEnableZdaMessage()
{
  
  char query_str[] = "PSRF103,00,6,00,0";// "PSRF103,08,00,01,01";
  uint8_t checksum = 0;
  int i = 0;
  
  //calc checksum
  while (query_str[i] != 0)
  {
    checksum = (checksum & 0x7f) ^ query_str[i]; 
    i++;
  }
    
    
  //send message
  sprintf(TxBuffer,"$%s*%02X\r\n", query_str, checksum);
  TxCounter =0;
  NbrOfDataToTransfer = strlen(TxBuffer);
  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

}


*/

static char zda_message [] = {0xA0, 0xA2, 0x00, 0x0E, 0x88, 0x00, 0x00, 0x04, 
                              0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x0F, 0x02, 0x00, 0xA1, 0xB0, 0xB3};


/*
static char zda_message [] = {0xA0, 0xA2, 0x00, 0x18, 0x81, 0x02, 0x00, 0x01, 
                      0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 
                      0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 
                      0x00, 0x01, 0x12, 0xC0, 0x01, 0x5F, 0xB0, 0xB3};
*/


void GpsEnableZdaMessage()
{

  for (int i=0;i < sizeof(zda_message);i++)
  {
    TxBuffer[i] = zda_message[i];
  }
  
  TxCounter =0;
  NbrOfDataToTransfer = sizeof(zda_message);
  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}









void SplitMessageElements(char *pMessage)
{
      for(int i=0;i<sizeof(messageElements);i++)
      {
        *pMessage = 0; //replace comma with null
        pMessage++;
        messageElements[i] = pMessage; //save pointer
        pMessage = strpbrk(pMessage,",*");
        if(pMessage == NULL) break;
      }
}


void DecodeMessageTable(void)
{
  char protocolheader[12];
  char* pMessage;
  
  if(decodedMessageInfo.isValidMessage)
  {
    //make sure first char is '$'
    if(messageBuffer[0] != '$') return;
    //now check the checksum
    if(!IsChecksumValid(messageBuffer)) return;
    
    pMessage = strpbrk((char*)&messageBuffer[1],",");
    protocolheader[0] =0;
    strncat(protocolheader,(char*)&messageBuffer[1],(pMessage-(char*)&messageBuffer[1]));
    //protocolheader = strtok((char*)&NMEA_Message[1], ",");
   
    if(!strcmp(protocolheader,"GPGGA"))   //GLOBAL POSITION DATA
    {   
       SplitMessageElements(pMessage);
       strcpy(GlobalPosition.UTCTime,messageElements[0]); //UTC Time
       strcpy(GlobalPosition.Latitude,messageElements[1]); //Latitude
       strcpy(GlobalPosition.NorthSouth,messageElements[2]); //North/South indicator
       strcpy(GlobalPosition.Longitude,messageElements[3]); //Longitude
       strcpy(GlobalPosition.EastWest,messageElements[4]);//East/West indicator
       GlobalPosition.PositionFix = atoi(messageElements[5]);  //Position Fix indicator     
       GlobalPosition.SatellitesUsed = atoi(messageElements[6]);        
       GlobalPosition.HorizontalDilutionOfPosition = atoi(messageElements[7]); //Horizontal Dilution Of Position


       GpsCoordinates_TypeDef gpsCoordinate;
         
         //copy strings
       strcpy(gpsCoordinate.latStr, GlobalPosition.Latitude);
       strcpy(gpsCoordinate.longStr, GlobalPosition.Longitude);
       strcpy(gpsCoordinate.northSouth, GlobalPosition.NorthSouth);
       strcpy(gpsCoordinate.eastWest, GlobalPosition.EastWest);       
       
       
       
       //save the hdop value to gspData struct
       gpsCurrentData.hdop = GlobalPosition.HorizontalDilutionOfPosition;
       gpsCurrentData.latitude = ConvertToDecimalDegrees((uint8_t*)GlobalPosition.Latitude,(char)GlobalPosition.NorthSouth[0]);
       gpsCurrentData.longitude = ConvertToDecimalDegrees((uint8_t*)GlobalPosition.Longitude,(char)GlobalPosition.EastWest[0]);
       gpsCurrentData.satellites = GlobalPosition.SatellitesUsed;
       
       gpsCurrentData.isValid = (GlobalPosition.PositionFix == 1) ? TRUE:FALSE;   //check if fix is valid
       
       gpsCoordinate.latitude = gpsCurrentData.latitude;
       gpsCoordinate.longitude = gpsCurrentData.longitude;
       gpsCoordinate.hdop = gpsCurrentData.hdop;
       
       
       
       //save location to buffer
       if (gpsCurrentData.isValid && gpsCurrentData.hdop < HDOP_THRESHOLD)
       {

         
         AddGpsSample(gpsCoordinate);
       }
    }
    else if(!strcmp(protocolheader,"GPGLL"))    //GEOPOSITION DATA
    {   
//       SplitMessageElements(pMessage);
//       strcpy(GeoPosition.Latitude,messageElements[0]);//Latitude
//       strcpy(GeoPosition.NorthSouth,messageElements[1]);//North/South indicator
//       strcpy(GeoPosition.Longitude,messageElements[2]);//Longitude
//       strcpy(GeoPosition.EastWest,messageElements[3]);//East/West indicator
//       strcpy(GeoPosition.UTCTime,messageElements[4]);//UTC Time
//       strcpy(GeoPosition.Status,messageElements[5]); //Position Fix indicator      
//       strcpy(GeoPosition.Mode,messageElements[6]); //Satellites used       
                 
    }
    else if(!strcmp(protocolheader,"GPRMC"))    //RECOMMENDED MINIMUM SPECIFIC 
    {
       SplitMessageElements(pMessage); 
       strcpy(RMCData.UTCTime,messageElements[0]); //UTC Time
       strcpy(RMCData.Date,messageElements[8]);  //Date     
       //strcpy(RMCData.Status,messageElements[1]);  //Status indicator
       //strcpy(RMCData.Latitude,messageElements[2]);//Latitude
       //strcpy(RMCData.NorthSouth,messageElements[3]); //North/South indicator
       //strcpy(RMCData.Longitude,messageElements[4]);//Longitude
       //strcpy(RMCData.EastWest,messageElements[5]);//East/West indicator
       //strcpy(RMCData.Mode,messageElements[11]); //Mode used 
       
       //convert from degreesminutes to decimal degrees
       //gpsCurrentData.latitude = ConvertToDecimalDegrees((uint8_t*)RMCData.Latitude,(char)RMCData.NorthSouth[0]);
       //gpsCurrentData.longitude = ConvertToDecimalDegrees((uint8_t*)RMCData.Longitude,(char)RMCData.EastWest[0]);
      
       
       
       
       
       
       //copy time components
       gpsCurrentData.time.sec =  ConvertStringToInt(&RMCData.UTCTime[4],2);
       gpsCurrentData.time.min =  ConvertStringToInt(&RMCData.UTCTime[2],2);
       gpsCurrentData.time.hour = ConvertStringToInt(&RMCData.UTCTime[0],2);
       gpsCurrentData.time.day = ConvertStringToInt(&RMCData.Date[0],2);
       gpsCurrentData.time.month = ConvertStringToInt(&RMCData.Date[2],2);
       gpsCurrentData.time.year = ConvertStringToInt(&RMCData.Date[4],2);
       
       //gpsCurrentData.isValid = (!strcmp(RMCData.Status,"A")) ? TRUE:FALSE;   //check if fix is valid
       //gpsCurrentData.timeout = INIT_TIMEOUT; //reset timeout. used to detect if data is not been updated 
       
       
         if (!hasTimeSynced && doSyncTime && IsFix())
         {
            struct tm gps_time;
           
            
            gps_time.tm_sec = gpsCurrentData.time.sec;
            gps_time.tm_min =  gpsCurrentData.time.min;
            gps_time.tm_hour =  gpsCurrentData.time.hour;
            gps_time.tm_mday =  gpsCurrentData.time.day;
            gps_time.tm_mon =  gpsCurrentData.time.month -1;
            gps_time.tm_year =  gpsCurrentData.time.year + 100;
            gps_time.tm_isdst = -1;

            timediff = SyncToGpsTime(&gps_time);
            SetGpsAdjustOffset(timediff);
            hasTimeSynced = TRUE;
         }

    }
    /*
    else if(!strcmp(protocolheader,"GPZDA"))
    {
       SplitMessageElements(pMessage);
       strcpy(ZDAData.HrMinSec,messageElements[0]); //UTC Time
       strcpy(ZDAData.Day,messageElements[1]);  //Date     
       strcpy(ZDAData.Month,messageElements[2]);  //Date    
       strcpy(ZDAData.Year,messageElements[3]);  //Date    
       strcpy(ZDAData.LoacalZoneHours,messageElements[4]);
       strcpy(ZDAData.LocalZoneMins,messageElements[5]);
    }
    */
    
  }
}





void ProcessIncomingGpsMessages(void)
{
  GetGpsMessage();
  DecodeMessageTable();
}

//same as atoi but set length
int ConvertStringToInt(char* string, int length)
{
  char s[8];
  int i;
  
  if(length > 7) length = 7;
  for(i=0;i<length;i++)
  {
    if(*string == 0) break; 
    s[i] = *string;
    string++;
  }
  s[i] = 0; // set null
  
  return atoi(s);
}




/* Returns the structure that holds GPS DATA 
*
*
*/
GPS_Position_TypeDef * GetGPSData(void)
{
  return &gpsCurrentData; 
}



//// Returns true if gps fix found and 
//// hdop is within range
//bool IsValidFix(void)
//{
//  bool ret = FALSE;
//
//  if(!IsTimedOut())
//  {
//    if ((gpsCurrentData.isValid) &&
//       (gpsCurrentData.hdop < HDOP_THRESHOLD ))
//        ret = TRUE;
//  }
//  
//  
//  return ret;
//}



// Attempts to get the best fix possible by  true if gps fix found and 
// hdop is within range
//timer has to be set based on the frequency of calls, in this case every 5ms so 200 = 1 sec

bool IsValidFix(void)
{
  
  
  static int hdop_target = HDOP_THRESHOLD;
  static int timer = -1;
  bool ret = FALSE;
  
  if ((gpsCoorBufferCount > 0) &&
     (gpsCoorBuffer[0].hdop < hdop_target ))
  {
     hdop_target = gpsCurrentData.hdop;
     timer = 2500;
  }
  
  if (timer >= 0)
  {
    timer--;
    if (timer == 0) ret = TRUE;
  }
  return ret;
}



bool IsFix(void)
{
 return gpsCurrentData.isValid;
}


//check if gps data has timedout
// will only occur if datat from gps mpodule is stopped been sent
bool IsTimedOut(void)
{
  bool ret = FALSE;
  if(gpsCurrentData.timeout == 0) 
    ret = TRUE;
  else
    gpsCurrentData.timeout--;

  return ret;
}



uint8_t IsChecksumValid(uint8_t* pMessage)
{
  int i;
  uint8_t xorsum;
  uint32_t val;
  char* pchksum;
  pchksum = strchr((char*)pMessage,'*');
   if(pchksum == NULL) return 0;
  pchksum++; //point to checksum string in hex
  sscanf(pchksum, "%x", &val);  //read as hexidecimal
  xorsum =0;
    for(i=1;i<256;i++)   //init i= 1 to ignore '$' char
  {
     if((!pMessage[i]) || (pMessage[i] == '*')) break;
     //if((message[i] != '$') && (message[i] != ',') ) 
       xorsum ^= pMessage[i];
  }
  if (xorsum == (uint8_t)val) return 1;
  return 0;
}





/*******************************************************************************
*Converts from decimal minutes to decimal degrees
*Inputs: DegreesMinutes - in ascii
*       North South West East indicator, ascii character
*Outputs: DecimalDegrees - in ascii
* Also returns the position in double format
*******************************************************************************/
double ConvertToDecimalDegrees(const uint8_t* DegreesMinutes,char northEastSouthWest)
{
 int i;
 char degreesstring[6];
 float degrees;
 float minutes;
 double minutesdiv;
 double sum; 
   //first find the '.' char
  
  for(i=0;i<15;i++)
  {
    if(DegreesMinutes[i] == NULL) return 0.0;
    if(DegreesMinutes[i] == '.') break;
  }
  if(i > 6) return 0.0; //decimal point should be within first 6 char's
  
  i-=2; //subtract two to point to start of minutes
  sscanf((char*)&DegreesMinutes[i], "%f", &minutes);  //read as float the minutes from string
  minutesdiv = minutes / 60.0;
  
  
  
  degreesstring[0] = NULL; //set first char null
  strncat((char*)degreesstring,(char*)DegreesMinutes,i); //copy over only the degrees
  sscanf(degreesstring, "%f", &degrees);  //read as float the degrees from string
  
  sum = degrees+minutesdiv;
  if((northEastSouthWest == 'S') || (northEastSouthWest == 'W')) sum *=-1.0;
  
  //sprintf((char*)DecimalDegrees,"%f",sum);
  
  return sum;
}


//checks if we are receiving messages from the gps module
// tries 2 baudrates. the one that works will automatically be set
bool IsGpsModuleOnWithAutoBaud(void)
{
  int i;

  //try default baud rate
  DisableGpsUSART();
  InitMessageBuffer();
  baudrate = DEFAULT_BAUDRATE;
  EnableGpsUSART();
  
  for(i=0;i!=10;i++)
  {
    GetGpsMessage();
    if(decodedMessageInfo.isValidMessage)
    {
      return TRUE;
    }
      delay_ms(250);
  }
  
  //attempt alternative baud rate
  DisableGpsUSART();
  InitMessageBuffer();
  baudrate = ALTERNATIVE_BAUDRATE;
  EnableGpsUSART();
  
  for(i=0;i!=10;i++)
  {
    GetGpsMessage();
    if(decodedMessageInfo.isValidMessage)
    {
      return TRUE;
    }
      delay_ms(250);
  }    
  
  return FALSE;
}

//checks if we are receiving messages from the gps module
// tries 2 baudrates. the one that works will automatically be set
bool IsGpsModuleOn(void)
{
  int i;

  InitMessageBuffer();

  for(i=0;i!=10;i++)
  {
    GetGpsMessage();
    if(decodedMessageInfo.isValidMessage)
    {
      return TRUE;
    }
      delay_ms(250);
  }
  
  return FALSE;
}










//
bool IsReceivingGPSMessages(void)
{
  bool ret = TRUE;
  static int failedToReceiveCount = 0;
  
  if (!decodedMessageInfo.messageDetect) 
  {
    if (failedToReceiveCount > 10) { // can't get a message a number of times      
      ret = FALSE;
    }
    failedToReceiveCount++;
  }
  else
  {
    decodedMessageInfo.messageDetect = FALSE;
    failedToReceiveCount = 0;
  }

  return ret;
}


//Logs once the gps position to a file once
//sets flag in eeprom to mark that a gps position was recorded so as to prevent repeat logging, this flag gets reset upon user wake up
#define MAX_GPS_START_TRIES 3
void LogGps()
{
  
//  static bool isGpsOn = FALSE;
//  static bool hasLoggedGps = FALSE;
//  static int gpsStartCount = 0; //used to detect how many attempts 
//  GPS_Position_TypeDef *gpsPositionData;
//  uint8_t text_string[64];
//  int timeout;
//  
//  if (hasLoggedGps) return;  //if already logged then never log again while recorder is on
//  if (EEPROM_ReadGpsLogged() == SET) { hasLoggedGps = TRUE; return; } //if already logged previous session/day then stop
//  //detect if log file already written by checking eeprom flag
//  
//  SysTickInterrupt(ENABLE); //turn on systick int to allow delay_ms function
//  if (!isGpsOn)         //if not on then we init gps and start module
//  {
//    GpsInit();
//    
//    if(StartGPSModule())
//      isGpsOn = TRUE;
//    else
//    {
//      gpsStartCount++;
//      if (gpsStartCount >= MAX_GPS_START_TRIES) hasLoggedGps = TRUE;
//    } 
//    
//    DisableGpsUSART(); // disable the uart, we don't want the usart int triggering while recording
//
//  }
//  else  //GPS IS ON
//  {
//    InitMessageBuffer(); //clear messages in gps message buffer
//    
//#ifdef USE_DUMMY_LOCATION
//    uint8_t *str = dummy_message;
//    while (*str != 0)
//    {
//      GPSFifoEnqueue(*str);
//      str++;
//    }
//#else    
//    EnableGpsUSART(); // turn the interrupts back on 
//#endif
//    
//    timeout = 250 * 4; //4 seconds to get message
//    while (timeout-- > 0)
//    {
//      
//      delay_ms(5);
//       ProcessIncomingGpsMessages(); 
//       
//       if(IsValidFix())
//       {
//         gpsPositionData =  GetGPSData();   
//       //  sprintf(text_string,"Latitude= %f\r\nLongitude= %f\r\n\r\n", gpsPos.latitude,gpsPos.longitude);
//         
//         if (WriteLogFileToCard("log.txt", text_string) == 0)
//         { //success in writing log file
//           hasLoggedGps = TRUE;
//           EEPROM_SetGpsLogged(SET);  //mark as being logged in eeprom
//           SetGpsLoggedTries(GPSLOG_PASSED);
//           timeout = 0;
//         }
//         else
//         { //Failed to write
//           timeout = 0;
//           hasLoggedGps = TRUE; //only mark in volatile memory, will prevent re-logging this session 
//         }
//          
//       }
//       else
//        delay_ms(5);
//    }
//
//  }    
//  
//  
//  SysTickInterrupt(DISABLE);
//           
}


////////////////////////////////////////////////////////////////////////////////////////////
// LogGpsAtOnce()
// 
//
//
//starts gps module and waits for fix,
#define INIT_TIMEOUT_IN_SECS 200  // this time out is used to recieve a fix
#define TIMEOUT_UNIT 230 // one seconds worth of timeout , also based on delay value 
#define TIME_EXTENSION_UNIT 18   // in secs the amount of time given for '1' unit of extension 
#define MAX_MULTIPLICATION_VALUE 15
#define MAX_NUM_OF_EXTENSIONS 5 // 

void LogGpsAtOnce(bool syncTime)
{
  GpsCoordinates_TypeDef *gpsCoor;
  uint8_t text_string[64];
  int timeout;
  bool gotFix = FALSE;
  rtcTimeTypeDef *time;
  rtcDateTypeDef *rtc_date;
  //int timeExtendedCount = 0;
  int lowestHdop = INT32_MAX;
  int multiplier;
  int extensionCount;
  char limitChar[] = {' ','>'};  
  char signChar[] = {'+','-'};  
  
  
  doSyncTime = syncTime;

  
  SetSysClk8MHz();
  
#ifdef LOG_GPS_ONCE  
  if (IsGpsLogged()) {  return; } //if already logged previous session/day then stop
#endif
  
  SysTickInterrupt(ENABLE); //turn on systick int to allow delay_ms function
  GpsInit();
 
      
#ifdef USE_DUMMY_LOCATION
  
  DisableGpsUSART(); // disable the uart, we don't want the usart int triggering while recording
  InitMessageBuffer(); //clear messages in gps message buffer
  uint8_t *str = dummy_message;

  
  while (*str != 0)
  {
    GPSFifoEnqueue(*str);
    str++;
  }
#else     
  
  
  //try at default baud
  if(!StartGPSModule()) 
  { 
    AddToLog(LOG_GPS_FAIL_START, 0,0,0, NO_TIMESTAMP);
    return; //return if fail to start gps
  }
  
  
  
  
  AddToLog(LOG_GPS_STARTED, 0,0,0, NO_TIMESTAMP);  
  
  InitMessageBuffer(); //clear messages in gps message buffer
  EnableGpsUSART(); //turn the interrupts back on
#endif
  
  
   LED_Display(LED_ON);
   /////////////////////////////////////////////////////////////////////////////////////////
   //THIS FIRST LOOP JUST WAITS FOR ANY GPS FIX
  //if this is the first time starting
  // increase time to allow for receiving more info
  timeout = INIT_TIMEOUT_IN_SECS * TIMEOUT_UNIT; //360 seconds to get message
  if (IsGpsFirstStart()) timeout = timeout * 5; //extend time if first turn on to allow gps to collect data from sats
  while (timeout-- >= 0)
  {
    ResetWatchdog();
    ProcessIncomingGpsMessages();
    delay_ms(4);

    //exit if we have recieved any fix
      gpsCoor = GetGpsCoordinates();
      if (gpsCoorBufferCount > 0 &&
          gpsCoor->hdop < lowestHdop &&
            IsFix())
      {
        gotFix = TRUE;
        lowestHdop = gpsCoor->hdop;
        timeout = 0;
      }
    
  }
 
   /////////////////////////////////////////////////////////////////////////////////////////
   //THIS SECOND LOOP IS TO WAIT FOR MORE ACCURATE FIX
   // - this only runs if we get fix in first loop
  if (gotFix)
  {
    
    //first delay for a set time before calc timeout from hdop 
    timeout = 15 * TIMEOUT_UNIT; //~15secs delay
    while (timeout-- >= 0)
    {
      ResetWatchdog();
      ProcessIncomingGpsMessages();
      delay_ms(4);
    }
    
    //calc timeout from the hdop value recieved
      multiplier = lowestHdop < MAX_MULTIPLICATION_VALUE ? lowestHdop : MAX_MULTIPLICATION_VALUE; 
      if (--multiplier < 0) multiplier = 0;
      timeout =   (TIME_EXTENSION_UNIT * TIMEOUT_UNIT) * multiplier;
    
    extensionCount = MAX_NUM_OF_EXTENSIONS; //set the number of allowed extensions
    while (timeout-- >= 0)
    {
      ResetWatchdog();
      ProcessIncomingGpsMessages();
      delay_ms(4);
      
      //Does time need extending?.  on last timeout round check if hdop has changed
      if (timeout == 0) 
      {
        gpsCoor = GetGpsCoordinates();
        if (gpsCoorBufferCount > 0 && 
            gpsCoor->hdop < lowestHdop)
        {
          gotFix = TRUE;
          lowestHdop = gpsCoor->hdop;
  
          //calc new timeout value
          //timeout is calculated by multipling a constant "unit" by lowest hdop value then adding it to current timeout
          if (extensionCount > 0)
          {
            multiplier = lowestHdop < MAX_MULTIPLICATION_VALUE ? lowestHdop : MAX_MULTIPLICATION_VALUE; 
            if (--multiplier < 0) multiplier = 0;
            timeout +=   (TIME_EXTENSION_UNIT * TIMEOUT_UNIT) * multiplier;
            extensionCount--; 
          }
        }
      }
    }

    //NOW STORE GPS POSITION TO LOG
    //get time for log entry
    time = RtcGetTime();
    rtc_date = RtcGetDate();
  
    // if we have a fix then log to sd card
  
      gpsCoor =  GetGpsCoordinates();
      sprintf( (char*)text_string, "%02d/%02d/%02d %02d:%02d - GPS (lat,long): %f,%f\r\n", rtc_date->date, rtc_date->month,rtc_date->year, time->hour, time->min, gpsCoor->latitude, gpsCoor->longitude);
     
  //#ifdef LOG_GPS_ONCE   
    //    if (InsertTextToLogFile(text_string, FALSE) == 0)  //inserts message into start of log file
  //#else
        if (WriteLogFileToCard(text_string, FALSE) ==0 ) //appends message
 // #endif   
        {
          AddToLog(LOG_GPS_LOCATION_FIX, (uint8_t)gpsCoor->hdop,0,0, NO_TIMESTAMP);
        }
        else
        { //Failed to write
          AddToLog(LOG_GPS_FAIL_SAVE, 0,0,0, NO_TIMESTAMP);
        }
          
#ifdef DEBUG
        sprintf( (char*)text_string, "hdop = %d, ", gpsCoor->hdop);
        strncat((char*)text_string,gpsCoor->latStr,12); // copy lat string
        strncat((char*)text_string, gpsCoor->northSouth,2);
        strncat((char*)text_string,gpsCoor->longStr,12);
        strncat((char*)text_string,gpsCoor->eastWest,2);
        strcat((char*)text_string,"\r\n");
        WriteLogFileToCard(text_string, FALSE); //appends message
#endif  
        
      SetGpsLoggedTries(GPSLOG_PASSED); 
  }
  else
  {
    time = RtcGetTime();
    rtc_date = RtcGetDate();
    sprintf((char*)text_string,"%02d/%02d/%02d %02d:%02d - No GPS fix\r\n", rtc_date->date, rtc_date->month,rtc_date->year, time->hour, time->min);
    WriteLogFileToCard(text_string, FALSE);
    AddToLog(LOG_GPS_FAIL_FIX, 0,0,0, NO_TIMESTAMP);
    SetGpsLoggedTries(GPSLOG_FAILED); 
  }  
  
  
  //if gps was synced then log it
  if (hasTimeSynced)
  {
    int hour=0;
    int min = 0;
    int sec = 0;
    int sign = 0;
    int limit = 0;
    
    //save the diff to record, used to correct timestamps on filenames
    sign = timediff < 0 ? 1 : 0;
    
    timediff = abs(timediff);
    
    //set limit
    if (timediff > (3600 * 24)) { timediff = 3600 * 24; limit = 1;}

    //get hour 
    hour = timediff/3600;
    timediff %= 3600;
      
     //get min
    min = timediff/60;
    timediff %= 60;
      
    //get sec
    sec = timediff; 
     
    sprintf((char*)text_string, "GPS time synced - %c%c%02dh%02dm%02ds\r\n",limitChar[limit],signChar[sign],hour,min,sec);  

    if (WriteLogFileToCard(text_string, FALSE) == 0 )
    {
    }

  }
  
  
  
  LED_Display(LED_OFF);  
 
  //set flag to acknowledge that the gps has run once for an extended amount of time
  SetGpsFirstStart(GPS_NOT_FIRST_START);
  
  SysTickInterrupt(DISABLE);
}




const static double  pi  = 3.14159265358979323846;
double Distance(double lat1, double lon1, double lat2, double lon2)
{
    double theta, dist;
    
    theta = lon1 - lon2;
    dist = sin(Deg2rad(lat1)) * sin(Deg2rad(lat2)) + cos(Deg2rad(lat1)) * cos(Deg2rad(lat2)) * cos(Deg2rad(theta));
    dist = acos(dist);
    dist = Rad2deg(dist);
    //dist = dist * 60 * 1.1515;
    dist = dist * 111895.7696; //convert to meters
    return dist;
}

double Deg2rad(double deg) 
{
    return (deg * pi / 180);
}

double Rad2deg(double rad) 
{
        return (rad * 180 / pi);
}




#define DISTANCE_THRESHOLD 6.0    //in meters
bool IsLastCoordinateFarAwayEnough(void)
{
  double dist;
  bool ret = FALSE;
  //determine distance between two gps points
  dist = Distance(gpsLastData.latitude, gpsLastData.longitude, gpsCurrentData.latitude, gpsCurrentData.longitude); 
  
  if(dist > DISTANCE_THRESHOLD)
  {
    //save the gps  position
     gpsLastData = gpsCurrentData;
     ret = TRUE;
  }
  
  return ret;
}




/*****************************************************
*GPS LOGGED 
*
* used to indicate whether a gps position has been logged
*
*/


#define GPS_LOG_TRIES 5 //number of attempts to get fix, once per session
//#define GPS_LOG_TRIES 0xff //use for infinite tries
void SetGpsLoggedTries(GpsLogOptions options)
{ 
  uint8_t tries;
  
  switch(options)
  {
    case GPSLOG_RESET_TRIES:
      EEPROM_WriteByte(EE_GPS_LOGGED, GPS_LOG_TRIES);
      break;
    case GPSLOG_PASSED:
      EEPROM_WriteByte(EE_GPS_LOGGED, 0); // gps has passed so write count to 0 to stop recurring
      break;
    case GPSLOG_FAILED:
      tries = EEPROM_ReadByte(EE_GPS_LOGGED);
      if (tries != 0 && tries != 0xff) tries--;
      EEPROM_WriteByte(EE_GPS_LOGGED, tries);
      break;
    default:
      break;
  }
} 


bool IsGpsLogged(void)
{
  uint8_t tries = EEPROM_ReadByte(EE_GPS_LOGGED);
  return  tries == 0 ? TRUE: FALSE;
}



void SetGpsFirstStart(GpsFirstStartOptions options)
{
  EEPROM_WriteByte(EE_GPS_FIRST_START, options);
}

bool IsGpsFirstStart(void)
{
  uint8_t flag = EEPROM_ReadByte(EE_GPS_FIRST_START);
  return  flag == GPS_IS_FIRST_START ? TRUE: FALSE;
}




/////////////////////////////////////////////////////////
#define TIME_ZONE 12 //+ 12 hours
#define DAYLIGHT_SAVINGS_OFFSET 1// 1 hour +
#define HOUR_SECS 3600





//returns difference between last rtc time and gps time
int32_t SyncToGpsTime(struct tm *gps_time)
{
  time_t gps_ticks;
  time_t rtc_ticks;
  int32_t offset;
  double d;
  //int32_t diff;
  int32_t comp;
  //get current rtc time
  rtc_ticks = GetRtcTimeInTicks();
 
  gps_ticks = mktime(gps_time);
  

  
  //add time zone offset
  offset = TIME_ZONE * HOUR_SECS; //in secs   
  gps_ticks =  (time_t)(gps_ticks + offset);
  
  
  //compare with current rtc time  
  //check if closer to possible daylight savings time, if so sync
  comp = (int32_t)((gps_ticks + HOUR_SECS) - rtc_ticks);
  if ( abs(comp)  <= (HOUR_SECS/2) ) gps_ticks += HOUR_SECS; // if rtc time is withn +-30mins of what would be daylight savings time, then sync to that 
  
  //get difference -(after gps time adjusted for timezone)
  d = difftime(gps_ticks, rtc_ticks);  
  
  SetRtcTimeInTicks(gps_ticks);

  //timeinfo = localtime(&timestamp);
    
  return (int32_t)d;
}




bool IsGpsLogOptionEnabled()
{
  uint8_t option;
  
  option = EEPROM_ReadGpsOptions();
  
  
  return option >= 1 ? TRUE : FALSE;
}


bool IsGpsSyncOptionEnabled()
{
  uint8_t option;
  
  option = EEPROM_ReadGpsOptions();
   
  return option >= 2 ? TRUE : FALSE; 
}




/*******************************************************
* GPS RX FIFO ROUTINES
*
*
*/

#define FIFO_BUFFER_SIZE 512
uint8_t fifoBuffer[FIFO_BUFFER_SIZE];
int32_t fifoWriteIndex = 0;
int32_t fifoReadIndex = 0;
int32_t fifoCount = 0;



bool GPSFifoEnqueue(uint8_t data)
{
    bool status = FALSE;
    if(fifoCount < (FIFO_BUFFER_SIZE-1))  //if enough space in buffer then allow
    {
        fifoBuffer[fifoWriteIndex] = data; //save the data to FIFO
        if(++fifoWriteIndex >= FIFO_BUFFER_SIZE) fifoWriteIndex =0;
        fifoCount++;
        status = TRUE;
    }
    return status;
}


uint8_t GPSFifoDequeue(void)
{
    uint8_t data = 0;
    if(fifoCount > 0)
    {
        data = fifoBuffer[fifoReadIndex];
        if(++fifoReadIndex >= FIFO_BUFFER_SIZE) fifoReadIndex =0;
        fifoCount--;
    }
    return data;
}

void GPSFifoClear(void)
{
  fifoWriteIndex = 0;
  fifoReadIndex = 0;
  fifoCount = 0;
}


int32_t GPSFifoCount(void)
{
 return fifoCount; 
}

/*************************************EOF**************************************/