/*****************************************************************************
* File Name          : rtc.c
* Author               : DOC Electronics R&D Team  
* Version              : V1.0.0
* Date                 : 06/11/2009
* Description        : RTC driver source file.
*
*                      Pin assignment:
*             ------------------------------------------
*             |  STM32F10x    |   STLM75     Pin       |
*             ------------------------------------------
*             | PB7/ SDA      |   SDA         1        |
*             | PB6/ SCL      |   SCL         2        |
*             | PB5/          |   OS/INT      3        |
*             | .             |   GND         4  (0V)  |
*             | .             |   GND         5  (0V)  |
*             | .             |   GND         6  (0V)  |
*             | .             |   GND         7  (0V)  |
*             | .             |   VDD         8  (5V)  |
*             ------------------------------------------
*
*******************************************************************************/


/* Includes n-------------------------------------------------------------------*/
#include "main.h"
#include "rtc.h"



/* Global variables -------------------------------------------------------------*/



/* Const Variables--------------------------------------------------------------*/

#define ACK       0x00
#define NACK      0x80

#define RTC_ADDR_WR	0xDE
#define RTC_ADDR_RD	0xDF

//RTC chip Address
#define RTC_ADDRESS				0xDE   /* 0b1101000 (7 bit address) */
#define RTC_READ					0xD1

// Date and Time
#define REG_SEC					0x00
#define REG_MIN					0x01
#define REG_HOUR					0x02
#define REG_DATE					0X03
#define REG_MONTH					0X04
#define REG_YEAR					0x05
#define REG_DAYOFWEEK	     			0X06
#define HOUR24          0x80

// ALARM 1
#define	REG_ALRM1_SEC			0X0C
#define	REG_ALRM1_MIN			0X0D
#define	REG_ALRM1_HOUR			0X0E	
#define	REG_ALRM1_DY_DATE		0X0F

//ALARM 2
#define	REG_ALRM2_MIN			0X0B
#define	REG_ALRM2_HOUR			0X0C	
#define	REG_ALRM2_DY_DATE		0X0D

// Control & Status Reg
//#define	REG_CON					0x07
#define	REG_STATUS				0x0F

//__IO uint8_t	REG_CON = 0x07;
#define REG_CON 0x07

// RTC Alarm 
#define RTC_DISABLE_MATCH		0x80
#define RTC_SQW_8K192			0x10
#define RTC_SQW_DISABLE			0x04
#define RTC_ALARM1_ENABLE		0x01
#define RTC_ALARM_FLAG			0x01

#define ALARM_TIME_SET			0x80



static bool rtcPowerFailFlag;  


static rtcTimeTypeDef rtcTime;
static rtcDateTypeDef rtcDate;


static rtcTimeTypeDef rtcAlarmTime;



/* Functions-----------------------------------------------------------------*/

/*******************************************************************************
* Function Name  : Rtc_bin2bcd
* Description    : Converts an 8-bit binary to 8 bit BCD.
* Input          :  binary value (The input range must be from 0 to 99)
* Output         : None
* Return         : bcd value
*******************************************************************************/

uint8_t Rtc_bin2bcd(uint8_t bin_value)
{
  uint8_t bcd;
  
  bcd = (bin_value / 10) << 4;
  bcd |= bin_value % 10;
  return (bcd) ;
}


/*******************************************************************************
* Function Name  : Rtc_bcd2bin
* Description     : Converts an 8-bit BCD to 8 bit Binary.
* Input         	 :  BCD value (The input range must be from 0 to 99)
* Output          : None
* Return          : Binary value
*******************************************************************************/
uint8_t Rtc_bcd2bin(uint8_t bcd_value)
{
  return ( (bcd_value & 0x0F) + (((bcd_value & 0xF0)>>4) *10));
}




void RtcStart()
{
  //rtcTimeTypeDef *rtcTime;
  //rtcDateTypeDef *rtcDate;
  
  rtcPowerFailFlag = RtcRegRead(0x07) & 0x01 ? TRUE : FALSE; //check if rtc power failure occured
  RtcRegWrite(REG_CON, 0x10);  //enables WRTC - write enable bit
  
  //read rtc if all clock regs are 0 then write 0 to the day reg to start clocking 
  //rtcTime = RtcGetTime();
  //rtcDate = RtcGetDate();
  //if (rtcTime->sec == 0 && rtcTime->min == 0 && rtcTime->hour == 0 &&
  //    rtcDate->date == 0 && rtcDate->month == 0 && rtcDate->year)
  //{
    RtcRegWrite(REG_DAYOFWEEK, 0x00);  //write day of week register to start clocking rtc
  //}
}



bool HasRtcPowerFailed()
{
  return rtcPowerFailFlag;
}


/*******************************************************************************
* Function Name  : Rtc_SetTime
* Description     : Set the given hour,min and sec into RTC time registers.
* Input         	 :  rtc_time 
* Output          : None
* Return          : None
*******************************************************************************/
void RtcSetTime( rtcTimeTypeDef *rtcTime )
{
  rtcTime->hour = Rtc_bin2bcd (rtcTime->hour) ;
  rtcTime->min =  Rtc_bin2bcd (rtcTime->min);
  rtcTime->sec = Rtc_bin2bcd (rtcTime->sec) ;
  
  i2c1_start();
  i2c1_wr(RTC_ADDR_WR);
  
  i2c1_wr(REG_SEC);
  i2c1_wr( rtcTime->sec );
  i2c1_wr( rtcTime->min );		//Minutes
  i2c1_wr( rtcTime->hour | HOUR24);			//Hour
  i2c1_stop(); 
}


/*******************************************************************************
* Function Name  : Rtc_SetDate
* Description     : Set the given day,date,month and year into RTC date register.
* Input         	 :  rtc_date 
* Output          : None
* Return          : None
*******************************************************************************/
void RtcSetDate( rtcDateTypeDef *rtcDate )
{
  RtcRegWrite(REG_CON, 0x10); //enable writes
  
  i2c1_start();
  i2c1_wr(RTC_ADDR_WR);
  i2c1_wr(REG_DATE);		//Day
  i2c1_wr( Rtc_bin2bcd(rtcDate->date));
  i2c1_wr( Rtc_bin2bcd(rtcDate->month) );
  i2c1_wr( Rtc_bin2bcd(rtcDate->year));
  i2c1_wr( Rtc_bin2bcd(rtcDate->dayOfWeek));
  i2c1_stop(); 
}



/*******************************************************************************
* Function Name  : Rtc_SetAlarm
* Description     : Set the alarm hour,min,sec and day/date to RTC alarm1 register.
* Input         	 :  rtc_time, alarm date/day 
* Output          : None
* Return          : None
*******************************************************************************/

void RtcSetAlarm( rtcTimeTypeDef *rtcTime)
{
  RtcRegWrite(REG_CON, 0x10); //enable writes
  
  i2c1_start();
  i2c1_wr(RTC_ADDR_WR);
  i2c1_wr(REG_ALRM1_SEC);
  i2c1_wr( rtcTime->sec);			//alarm seconds diabled 	// alarm match  seconds
  i2c1_wr( rtcTime->min );				// alarm  match minutes
  i2c1_wr( rtcTime->hour );				// alarm match hours
  i2c1_wr(0); //date
  i2c1_wr(0); //month
  i2c1_wr(0); //day of week
  i2c1_stop();
  
  RtcRegWrite(0x08,0x40);		//Enable the Alarm
}
/*******************************************************************************
* Function Name  : Rtc_DisableAlarm
* Description     : disables the alarm .
* Input         	 : 
* Output          : None
* Return          : None
*******************************************************************************/

void RtcDisableAlarm(void)
{
  RtcRegWrite(0x08,0x00); //isnable the Alarm bit
}

/*******************************************************************************
* Function Name  : RTC_GetDate
* Description     : get day,date,month and year from RTC date register.
* Input         	 :   None
* Output          : rtc_date
* Return          : None
*******************************************************************************/
rtcDateTypeDef *RtcGetDate(void)
{
  i2c1_start();
  i2c1_wr(RTC_ADDR_WR);   //DIR= WR
  i2c1_wr(REG_DATE);
  
  i2c1_start();   //second start bit
  i2c1_wr(RTC_ADDR_RD);  //DIR = RD
  
  rtcDate.date = Rtc_bcd2bin( i2c1_rd(ACK) );
  rtcDate.month  = Rtc_bcd2bin( i2c1_rd(ACK) );
  rtcDate.year = Rtc_bcd2bin(i2c1_rd(ACK) );
  rtcDate.dayOfWeek = Rtc_bcd2bin( i2c1_rd(NACK) );
  
  return &rtcDate;
}

/*******************************************************************************
* Function Name  : RTC_GetTime
* Description     : Get hour,min and sec from RTC time registers.
* Input         	 :  None 
* Output          : rtc_time
* Return          : None
*******************************************************************************/

rtcTimeTypeDef *RtcGetTime(void)
{
  i2c1_start();
  i2c1_wr(RTC_ADDR_WR);   //DIR= WR
  i2c1_wr(REG_SEC);
  
  i2c1_start();   //second start bit
  i2c1_wr(RTC_ADDR_RD);  //DIR = RD
  
  rtcTime.sec = Rtc_bcd2bin( i2c1_rd(ACK) );
  rtcTime.min = Rtc_bcd2bin( i2c1_rd(ACK) );
  rtcTime.hour = Rtc_bcd2bin( i2c1_rd(NACK)  & ~HOUR24 );
  
  i2c1_stop();
  
  return &rtcTime;
}



/*******************************************************************************
* Function Name  : RtcGetAlarm
* Description     : Gets the alarm hour,min,sec of the RTC alarm1 register.
* Input           : None
* Output          : 
* Return          : None
*******************************************************************************/

rtcTimeTypeDef *RtcGetAlarm(void)
{
  i2c1_start();
  i2c1_wr(RTC_ADDR_WR);   //DIR= WR
  i2c1_wr(REG_ALRM1_SEC);
  
  i2c1_start();   //second start bit
  i2c1_wr(RTC_ADDR_RD);  //DIR = RD

  rtcTime.sec =  Rtc_bcd2bin( 0x7f & i2c1_rd(ACK));
  rtcTime.min = Rtc_bcd2bin( 0x7f & i2c1_rd(ACK));
  rtcTime.hour = Rtc_bcd2bin(0x7f & i2c1_rd(NACK));
  
  i2c1_stop();
  
  return &rtcAlarmTime;
}

//gets buffered rtc date without refreshing date from rtc
rtcDateTypeDef *RtcGetBufferedDate(void)
{
  return &rtcDate;
}

//gets buffered rtc time without refreshing time from rtc
rtcTimeTypeDef *RtcGetBufferedTime(void)
{
  return &rtcTime;
}


void RtcRegWrite(uint8_t RegName, uint16_t RegValue) //*
{
  i2c1_start();
  i2c1_wr(RTC_ADDR_WR);
  i2c1_wr(RegName);
  i2c1_wr(RegValue);
  i2c1_stop();
}

uint8_t RtcRegRead(uint8_t RegName) //*
{
  uint8_t	RegRead;
  
  i2c1_start();
  i2c1_wr(RTC_ADDR_WR);
  i2c1_wr(RegName);
  
  i2c1_start();
  i2c1_wr(RTC_ADDR_RD);
  
  RegRead = i2c1_rd(NACK);
  i2c1_stop();
  
  return RegRead;
}

void RtcClearAlarmFlag( void)
{
  uint8_t status;
  
  status = RtcRegRead(0x07);  //read status register
  status &= 0xFB;
  RtcRegWrite(0x07,status); //write modified status data
}	


uint8_t RtcGetAlarmFlag(void)
{
  uint8_t	RegData;
  RegData = RtcRegRead(0x07); //read register
  if (RegData & 0x04) return 1;
  return 0; 
}



uint8_t	RtcGetUser1Reg(void)
{
  return(RtcRegRead(0x12));
}

void RtcSetUser1Reg(uint8_t Byte)
{
  RtcRegWrite(0x12,Byte);	//
}


uint8_t	RtcGetUser2Reg(void)
{
  return(RtcRegRead(0x13));
}

void RtcSetUser2Reg(uint8_t Byte)
{
  RtcRegWrite(0x13,Byte);		//
}


//
////adds seconds on to current time and date in order to get a future time
//void ModifyTimeAndDateWithAddedSeconds(rtcTimeTypeDef *updatedTime ,rtcDateTypeDef *updatedDate, int addedSeconds)
//{
//  //struct tm _tm;
//  //volatile struct tm *p_tm;
//  //time_t rawtime;
//  uint32_t rawtime;
//  
//  //convert current time to time struct 
//  rawtime = date_time_to_epoch(updatedTime , updatedDate);
//  rawtime += addedSeconds; 
//  //add seconds to rawtime
//  //rawtime = mktime(&_tm);
//  //rawtime += addedSeconds;
//  //p_tm = localtime(&rawtime);
//  
//  //convert back from time struct to updsated time
//   epoch_to_date_time(updatedTime, updatedDate,rawtime );
//   
//}
//
//static unsigned short days[4][12] =
//{
//    {   0,  31,  60,  91, 121, 152, 182, 213, 244, 274, 305, 335},
//    { 366, 397, 425, 456, 486, 517, 547, 578, 609, 639, 670, 700},
//    { 731, 762, 790, 821, 851, 882, 912, 943, 974,1004,1035,1065},
//    {1096,1127,1155,1186,1216,1247,1277,1308,1339,1369,1400,1430},
//};
//
//
//unsigned int date_time_to_epoch(rtcTimeTypeDef *pTime, rtcDateTypeDef * pDate)
//{
//  uint32_t second = pTime->sec;  // 0-59
//  uint32_t minute = pTime->min;  // 0-59
//  uint32_t hour   = pTime->hour;    // 0-23
//  uint32_t day    = pDate->date-1;   // 0-30
//  uint32_t month  = pDate->month-1; // 0-11
//  uint32_t year   = pDate->year;    // 0-99
//  return (((year/4*(365*4+1)+days[year%4][month]+day)*24+hour)*60+minute)*60+second;
//}
//
//
//void epoch_to_date_time(rtcTimeTypeDef *pTime, rtcDateTypeDef * pDate,unsigned int epoch)
//{
//    pTime->sec = epoch%60; epoch /= 60;
//    pTime->min = epoch%60; epoch /= 60;
//    pTime->hour   = epoch%24; epoch /= 24;
//
//    unsigned int years = epoch/(365*4+1)*4; epoch %= 365*4+1;
//
//    unsigned int year;
//    for (year=3; year>0; year--)
//    {
//        if (epoch >= days[year][0])
//            break;
//    }
//
//    unsigned int month;
//    for (month=11; month>0; month--)
//    {
//        if (epoch >= days[year][month])
//            break;
//    }
//
//    pDate->year  = years+year;
//    pDate->month = month+1;
//    pDate->date   = epoch-days[year][month]+1;
//}
//
//
//
//


bool IsOnAlarmTime(void)
{
  rtcTimeTypeDef alarmTime;
 rtcTimeTypeDef rtcTime;

  alarmTime = *RtcGetAlarm(); 
  rtcTime = *RtcGetTime();
  
  if (alarmTime.hour == rtcTime.hour && alarmTime.min == rtcTime.min) return TRUE;
  
  return FALSE;
}





/****************************************************
* Internal RTC configuration 
*
*
*
*/

void Internal_RTC_Configuration(FunctionalState NewState, int interruptSeconds)
{
 NVIC_InitTypeDef NVIC_InitStructure;
 EXTI_InitTypeDef EXTI_InitStructure;
 
 if (NewState == ENABLE)
 {
    /* Enable PWR and BKP clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
  
    /* Allow access to BKP Domain */
    PWR_BackupAccessCmd(ENABLE);
  
    /* Reset Backup Domain */
    BKP_DeInit(); 
  
    /* Enable the LSI OSC */
    RCC_LSICmd(ENABLE);
    /* Wait till LSI is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
    {}
    /* Select the RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
  
    /* Enable RTC Clock */
    RCC_RTCCLKCmd(ENABLE);
  
    /* Wait for RTC registers synchronization */
    RTC_WaitForSynchro();
  
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
  
    /* Set RTC prescaler: set RTC period to 1sec */
    RTC_SetPrescaler(40000);
  
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
  
    RTC_SetCounter(0);
       /* Clear Interrupt pending bit */
    //RTC_ClearITPendingBit(RTC_FLAG_ALR);   
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask(); 
    RTC_SetAlarm(interruptSeconds);
      RTC_WaitForLastTask(); 
    /* Enable the RTC Second */
    RTC_ITConfig(RTC_IT_ALR, ENABLE);
  
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
  
    
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;   //EXTI_Mode_Interrupt     EXTI_Mode_Event
    EXTI_InitStructure.EXTI_Line = EXTI_Line17;  /*!< External interrupt line 17 Connected to the RTC Alarm event */        
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    
    /* Enable the RTC Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
 }
 else //disable 
 {
   RCC_LSICmd(DISABLE);
   PWR_BackupAccessCmd(DISABLE);
       /* Enable RTC Clock */
    RCC_RTCCLKCmd(DISABLE);
   
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;   //EXTI_Mode_Interrupt     EXTI_Mode_Event
    EXTI_InitStructure.EXTI_Line = EXTI_Line17;          
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    /* Enable the RTC Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);  
 }
  
  
  
}




time_t GetRtcTimeInTicks()
{
    //get rtc datetime
  rtcTimeTypeDef *pTime;
  rtcDateTypeDef *pDate;
  struct tm datetime = {0};
  
  pTime = RtcGetTime(); //update buffered date
  pDate = RtcGetDate(); //update buffered date

     
  datetime.tm_sec = pTime->sec;
  datetime.tm_min = pTime->min;
  datetime.tm_hour = pTime->hour;
  
  datetime.tm_mday = pDate->date;
  datetime.tm_mon = pDate->month-1;
  datetime.tm_year = pDate->year + 100;   //
  datetime.tm_isdst = -1;
  
  return mktime(&datetime);
}




void SetRtcTimeInTicks(time_t ticks)
{
  rtcTimeTypeDef rtcTime;
  rtcDateTypeDef rtcDate;
  struct tm *datetime;
  
  datetime = localtime(&ticks);

  rtcTime.sec = (int8_t)datetime->tm_sec;
  rtcTime.min = (int8_t)datetime->tm_min;
  rtcTime.hour = (int8_t)datetime->tm_hour;
  
  
  rtcDate.date =  (int8_t)datetime->tm_mday;
  rtcDate.month =  (int8_t)(datetime->tm_mon + 1);
  rtcDate.year = (int8_t)(datetime->tm_year - 100);   //
  
  RtcSetTime(&rtcTime);  // write new time
  RtcSetDate(&rtcDate);  //write new date
}






/*************************************EOF***************************************/