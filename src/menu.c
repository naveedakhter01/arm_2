/*******************************************************************************
* File Name         : Menu.c
* Author              : DOC Electronics
* Version             : V1.0.0
* Date                 : 14/09/2009
* Description        : 
********************************************************************************/

/* Includes -------------------------------------------------------------------*/
#include "main.h"
//#include "LCD.h"
#include "FatSD.h"
#include "target/target.h"
#include "rtc.h"
#include "menu.h"
#include "record.h"
#include "sysclk.h"
#include "pushbutton.h"
#include "eeprom.h"
#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x_it.h"
#include "battery.h"
#include "platform_config.h"
#include "spi.h"
#include "gps.h"
#include "temp_sensor.h"
#include "dsp_program.h"
#include "hardware.h"
#include "protocol.h"
#include "test.h"
#include "nfc.h"
/* Const Variables--------------------------------------------------------------*/



//#define TEST_MENU //used only to display test menu for hardware testers

#define DOWN	0
#define UP 		1	

#define INC		1
#define DEC		-1


#define UPDATEDELAY     500 // in msecs
#ifndef DEBUG
  #define TIMEOUT       20000 / UPDATEDELAY // ~20secs
#else
  #define TIMEOUT       1800000 / UPDATEDELAY // DEBUG TIMEOUT  = ~30 * 60secs (30mins)
#endif

//for RTC
int const days[13] = {31,31,28,31,30,31,30,31,31,30,31,30,31};
#define HR_CHK(HR)  			if(HR>23){ HR=0;} else if(HR<0) {HR=23;}	
#define MIN_SEC_CHK(MIN) 	if(MIN>59){MIN=0;} else if (MIN<0) {MIN=59;}
#define DATE_CHK(DATE,MON) 		if(DATE>days[MON]){DATE=1;} else if (DATE<1) {DATE=days[MON];}
#define MONTH_CHK(MON)		if(MON>12) {MON=1;} else if (MON<1) {MON=12;}
#define YEAR_CHK(YR)			if(YR>99) {YR=0;} else if (YR<0){YR=99;}


#define MIN_15_CHK(MIN) 	if(MIN>55){MIN=0;} else if (MIN<0) {MIN=55;}


#define ALPHA_CHK(CHR)          if(CHR < 0){CHR=0;} else if (CHR >= sizeof(CharTable)) {CHR=0;}

/* Global variables -------------------------------------------------------------*/




 
bool nfcAvailable = FALSE;

int preMenuCurPos;
int menuCurPos;
int dateAdjustFlag = 0;

bool scheduleFileFlag = FALSE;
static int updateTimer = 0;
static int timeout = TIMEOUT;
static bool timeoutEnabled = FALSE;

struct sMenuItem CurActiveMenu;


/***************************************DEFAULT*********************************/

struct sMenuItem sDebugMenu[] = {        { "Time: ",TimeDisplay, TIME }, 
					{ "Date:", DateDisplay, RTCDATE},
                                        //{ "", TimeAndDate, RTCDATE},
					//{ "Start Time:", StartTimeDisplay, START_TIME},
					//{ "Duration:", DurationDisplay, DUR_TIME},

                                       // { "Record: ",Record, RECORD },
                                       // { "",StartAndDuration, CODE },
   // protocol start times
                                        { "",ProtocolDisplay,  ENTRY1 },
                                        { "",StartAndDuration, ENTRY1 }, 
  
                                        { "",ProtocolDisplay,  ENTRY2 },
                                        { "",StartAndDuration, ENTRY2 }, 
   
 
                                        { "",ProtocolDisplay,  ENTRY3 },
                                        { "",StartAndDuration, ENTRY3 },

  
                                        { "",ProtocolDisplay,  ENTRY4 },
                                        { "",StartAndDuration, ENTRY4 },


                                        { "",ProtocolDisplay,  ENTRY5 },
                                        { "",StartAndDuration, ENTRY5 },
 
                                        { "",ProtocolDisplay,  ENTRY6 },
                                        { "",StartAndDuration, ENTRY6 },
  

                                        { "", GpsOptionsDisplay, GPS }, 
                                        { "", SurveyDisplay, SURVEY},   
                                        { "", StationDisplay, STATION},
                                        
                                        { "Random Delay",StartupRandomDelay, RANDOM_DELAY},
                                        //{ "Sampling:",Sampling, SAMP},
                                        { "Card: ",Test, TEST },
                                        { "Test: ",CodeTest, CODE },
                                        { "DSP Boot", DspBoot, DSP},
                                        { "Sleep Test", Sleep, SLEEP},
                                        { "Contrast:", Contrast, CONTRAST_LCD},
                                        { "NFC:", NFCTestDelay, NFC},
                                       // { "SPI Test: ",DMA_Start, DMA },
                                        //{ "Card: ",CardDisplay, CARD },
                                        { "Power Off", MainPowerOff, PWROFF},
                                       // { "Test: ",CodeTest, CODE },
                                        //{ "",LowBatteryAck, LOWBATTERY }    //gets user to ack when low battery shutdown occured
              			};

struct sMenuItem sTestMenu[] = {     {"Test: ",CodeTest, CODE },      // used purely for testing hardware,
                                        {"DSP Boot", DspBoot, DSP},
                                    { "Sleep Test", Sleep, SLEEP},
                                    { "Card: ",Test, TEST },
                                        { "Contrast:", Contrast, CONTRAST_LCD}
                                   
                                      
                                    //{ "Contrast:", Contrast, CONTRAST_LCD}              			
                                };


struct sMenuItem sMainMenu[] = {        { "Time: ",TimeDisplay, TIME },   
					{ "Date:", DateDisplay, RTCDATE},   
                                        { "",ProtocolDisplay, ENTRY1 },   
                                        { "",StartAndDuration, ENTRY1 },   
                                         { "",ProtocolDisplay, ENTRY2 },   
                                         { "",StartAndDuration, ENTRY2 },  
                                        { "",ProtocolDisplay,  ENTRY3 },
                                        { "",StartAndDuration, ENTRY3 },
                                         { "",ProtocolDisplay,  ENTRY4 },
                                         { "",StartAndDuration, ENTRY4 },
                                        { "",ProtocolDisplay,  ENTRY5 },
                                        { "",StartAndDuration, ENTRY5 },
                                         { "",ProtocolDisplay,  ENTRY6 },
                                         { "",StartAndDuration, ENTRY6 },                                        
                                        { "", GpsOptionsDisplay, GPS },   
                                        { "", SurveyDisplay, SURVEY},   
                                        { "", StationDisplay, STATION},  
                                        { "Card: ",Test, TEST },   
              			};

struct sMenuItem *pMenu;


static int maxMenuItems;  

/* Functions-------------------------------------------------------------------*/

void CheckForNfc();


/******************************************************************************* 
* Function Name  : Menu_Init()
* Description    : Initialize the menu
* Input           :  None
* Output         : None
* Return         : None
*******************************************************************************/

void Menu_Init(void)
{
  
#if defined TEST_MENU
    maxMenuItems = sizeof(sTestMenu) / sizeof(struct sMenuItem);
    pMenu = sTestMenu;
    timeoutEnabled = FALSE;
#elif defined DEBUG  
    maxMenuItems = sizeof(sDebugMenu) / sizeof(struct sMenuItem);
    pMenu = sDebugMenu;
    timeoutEnabled = TRUE;
#else
    maxMenuItems = sizeof(sMainMenu) / sizeof(struct sMenuItem);
    pMenu = sMainMenu;
    timeoutEnabled = TRUE;
#endif
  
  write_LCD_line1 ( pMenu[0].PageName ); 
  CurActiveMenu = pMenu[0];	
  preMenuCurPos =0;
  menuCurPos = 0;
  
  updateTimer = GetTicks();
  CurActiveMenu.pfMenuFunc(INIT);
  LCD_CursorOff();
  
  //determine if NFC IC is present
  nfcAvailable = NfcIsIc();
  
}

/*******************************************************************************
* Function Name  : Call_Update() 
* Description       : Call Update Status for current menu function every so many ms (UPDATEDELAY)
* Input               : None
* Output         	: None
* Return         	: None
*******************************************************************************/


void CallUpdate() 
{
  uint16_t tickvalue; 
  
   tickvalue = GetTicks();
    if ((uint16_t)(tickvalue- updateTimer) >= UPDATEDELAY) 
    {
      CurActiveMenu.pfMenuFunc(UPDATE);  // call current page functions UPDATE if it has one
      updateTimer = GetTicks();  //set new reference
       if (timeoutEnabled) timeout--; //decrement timeout value
    }
}


/*******************************************************************************
* Function Name  : RefreshMenu()
* Description       : Is continuously cycled  
* Input               : None
* Output         	: None
* Return         	: None
*******************************************************************************/

MenuExitCode RefreshMenu()      //returns timeout value
{
      uint16_t bttnOut;   // button flags
      bool skipFlag;
      
      MenuExitCode exitCode = NO_EXIT; //assume menu will keep running after exit
 
      if(menuCurPos != preMenuCurPos)   // if new page pending then set it up
      {
    
        CurActiveMenu.pfMenuFunc(CLOSE);   // call current page item's CLOSE event
        clear_LCD();
        
        do {   
          preMenuCurPos = menuCurPos;
   
          CurActiveMenu = pMenu[menuCurPos]; //load new page data
          // commented for EVAL 
           
          skipFlag = CurActiveMenu.pfMenuFunc(INIT);   //call page items INIT event
          if (skipFlag) {
            if ( ++menuCurPos >= maxMenuItems )   menuCurPos = 0;
          }
          else 
            write_LCD_line1 ( CurActiveMenu.PageName ); 
          
        } while (skipFlag);           
      }	
      
      bttnOut =GetButtonStatus();
      
      
      if((bttnOut) || (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9))) timeout = TIMEOUT; //reset timout value if button pushed of usb pluged in then     
      
      //If Page key pushed the next page 
      if( bttnOut & PAGE_KEY == PAGE_KEY) 
      {
        menuCurPos++;
        if ( menuCurPos >= maxMenuItems )
                    menuCurPos = 0;        
      }
      
      if (nfcAvailable)
        CheckForNfc();

      
      
      //call the battery check routine, shutdown if batteries low
       if (!IsPowerOk()) exitCode =  POWER_FAILED;      
              
       if( (bttnOut & PLUS_KEY) == PLUS_KEY) CurActiveMenu.pfMenuFunc(_PLUS);
       else
       if( (bttnOut & MINUS_KEY) == MINUS_KEY) CurActiveMenu.pfMenuFunc(_MINUS);
       else
       if( (bttnOut & NEXT_KEY) == NEXT_KEY) CurActiveMenu.pfMenuFunc(_NEXT);

       
       
       CallUpdate();
       
       
     
      // timeout occurs - call page items CLOSE event 
      if (timeout <=0)
      {
        CurActiveMenu.pfMenuFunc(CLOSE);
        exitCode = TIMED_OUT;
      }


      return(exitCode);
      
}




void CheckForNfc()
{
  static int checkForNfcCounter = 0;
  int ret;
  
      //check if nfc settings data is present
      if ( (checkForNfcCounter++ % 10000) == 0)
      {
        
#ifdef USING_MAILBOX        
        
      if (NfcIsMessage(RF_PUT_MSG) || NfcIsMessage(HOST_PUT_MSG)) //check if we have recieved message --NOTE sometimes if flag is set the payload may not be valid
      {
        ret = NfcModifySettings(); //gets payload and updates settings, will perform a number of attempts to read payload correctly
            if (ret <= 0)
            {
              write_LCD_line1("  Settings  "); 
              write_LCD_line2("  Updated   ");   
            }
            /*
            else if (ret < 0)
            {
              write_LCD_line1("  Updated   ");   
              write_LCD_line2("   ");
              LCD_out_hex((char)(ret*-1) );
              write_LCD("        ");
            }
            */
            else
            {
              write_LCD_line1("  Failed   ");
              write_LCD_line2("   ");
              LCD_out_hex((char)ret);
              //write_LCD_line2(" Try Again ");
              write_LCD("        ");
            }
            delay_ms(4000);
            clear_LCD();
            write_LCD_line1 ( CurActiveMenu.PageName ); 
            CurActiveMenu.pfMenuFunc(INIT);
      }  
      
#else
        if (NfcTestBlockDataForMarker())
        {
            write_LCD_line1(" Updating.. "); 
            write_LCD_line2("            "); 
            delay_ms(700);
            
            if (NfcReadBlockData())
            {
              write_LCD_line1("  Settings  "); 
              write_LCD_line2("  Updated   ");   
            }
            else
            {
              write_LCD_line1("    Try     "); 
              write_LCD_line2("   Again    "); 
            }
            delay_ms(2000);
            clear_LCD();
            write_LCD_line1 ( CurActiveMenu.PageName ); 
            CurActiveMenu.pfMenuFunc(INIT);
          }
#endif      
  
      }
  
}








/*******************************************************************************
* Function Name  : TIME DISPLAY
* Description    : show/modify the RTC time.
* Input          : None
* Output         : NOPE
* Return         : None
*******************************************************************************/
bool TimeDisplay (bACTION event)
{
  static uint8_t digitPosition = 0; // position of digit:0= nothing,1 = hour,2 = minute
  static FlagStatus blink = SET; // init to set = instant blink when next pushed
  rtcTimeTypeDef *rtcTime;
 
  switch (event)
  {

  case INIT:    // Called once when menu page first activated
        digitPosition = 0;
        //ChangeTime = RESET;
        TimeDisplay(UPDATE);  // functions calls itself with update event to initialise
       break;
  case UPDATE:  // Called periodically when menu page displayed

    LCD_Addr(0x8B);
      if((CheckRecordTime()) && (!scheduleFileFlag)) write_LCD("*"); // within time range and schedule file not present
      else write_LCD(" "); // not within time range
    
    
    {
      rtcTime = RtcGetTime();
      LCD_Addr(0xc2);  //CD_Addr(0xAA); 
      if((blink==SET) && (digitPosition == 1)) write_LCD("  ");
         else LCD_out_hex((char)Rtc_bin2bcd(rtcTime->hour));
         write_LCD(":");
      if((blink==SET) && (digitPosition == 2)) write_LCD("  ");
        else LCD_out_hex((char)Rtc_bin2bcd(rtcTime->min));
         write_LCD(":");
         LCD_out_hex((char)Rtc_bin2bcd(rtcTime->sec));


         blink ^= 1;    // toggle  the blink status  
    }
       
        break;

        
  case _NEXT:        //move to next digit
         blink = SET; // make sure to blink value instantly
         updateTimer += UPDATEDELAY;  // instant update upon return 
         
          if(++digitPosition > 2) //cycle through digits
          { 
           digitPosition = 0;
            //ChangeTime = RESET;
            updateTimer += UPDATEDELAY; 
          }
           // else ChangeTime = SET;
            break;
            
  case _PLUS:        //increment a value
        
          if(digitPosition) //if modifying hours or minutes
          {
            rtcTime = RtcGetTime(); //load time structure
           
            rtcTime->sec = 0; //reset seconds
            if(digitPosition == 1) 
            {
              rtcTime->hour += 1; //adjust hours
              HR_CHK(rtcTime->hour);
            }
            else  if(digitPosition == 2)
            {
              rtcTime->min += 1;  //adjust hours
              MIN_SEC_CHK(rtcTime->min)
            }
            
      
              RtcSetTime(rtcTime);  // write new time
              blink = RESET;           //don't blink when button pushed
              updateTimer = GetTicks(); // reset reference 
              TimeDisplay(UPDATE);  // function calls itself with UPDATE
          }
    
             break;
    
  case _MINUS:        //deccrement a value
        
          if(digitPosition) //if modifying hours or minutes
          {
            rtcTime = RtcGetTime(); //load time structure
           
            rtcTime->sec = 0; //reset seconds
            if(digitPosition == 1) 
            {
              rtcTime->hour -= 1;//adjust hours
              HR_CHK(rtcTime->hour);
            }
              
            else  if(digitPosition == 2)
            {
              rtcTime->min -= 1;  //adjust hours
              MIN_SEC_CHK(rtcTime->min)
            }
      
              RtcSetTime(rtcTime);  // write new time
              blink = RESET;           //don't blink when button pushed
              updateTimer = GetTicks(); // reset reference 
              TimeDisplay(UPDATE);  // function calls itself with UPDATE
          }    
        break;   
  }
  
  return FALSE;
  
}

/*******************************************************************************
* Function Name  : DATE DISPLAY
* Description    : show/modify the RTC date.
* Input          : None
* Output         : NOPE
* Return         : None
*******************************************************************************/


bool DateDisplay (bACTION event)
{
  static uint8_t digitPosition = 0; // position of digit:0= nothing,1 = hour,2 = minute
 // static FlagStatus ChangeDate = RESET; // set when clock values are being altered by user
 	//uint8_t timestring[10]	;
  static FlagStatus blink = SET; // init to set = instant blink when next pushed
  rtcDateTypeDef *rtcDate;      
        
  switch (event)
  {

  case INIT:    // Called once when menu page first activated
        digitPosition = 0;
        //ChangeDate = RESET;
        DateDisplay(UPDATE);  // functions calls itself with update event to initialise
       break;
  case UPDATE:  // Called periodically when menu page displayed

      rtcDate = RtcGetDate();
      
      //display if time within recording range
      

      
      
      LCD_Addr(0xC1);
      if((blink==SET) && (digitPosition == 1)) write_LCD("  ");
         else LCD_out_hex((char)Rtc_bin2bcd(rtcDate->date));
         write_LCD("/");
      if((blink==SET) && (digitPosition == 2)) write_LCD("  ");
        else LCD_out_hex((char)Rtc_bin2bcd(rtcDate->month));
         write_LCD("/");
      if((blink==SET) && (digitPosition == 3)) write_LCD("    ");
        
        else 
        {
          write_LCD("20");
          LCD_out_hex((char)Rtc_bin2bcd(rtcDate->year));
        }
          blink ^= 1;    // toggle  the blink status         
        break;
        
  case _NEXT:        //move to next digit
         blink = SET; // make sure to blink value instantly
         updateTimer += UPDATEDELAY;  // instant update upon return 
         
          if(++digitPosition > 3) //cycle through digits
          { 
            digitPosition = 0;
            //ChangeDate = RESET;
            updateTimer += UPDATEDELAY; 
          }
           //else ChangeDate = SET;
            break;
  case _PLUS:        //increment a value
        
          if(digitPosition) //if modifying date
          {
            rtcDate = RtcGetDate(); //load date structure           
            switch (digitPosition)
            {
            case 1:
              rtcDate->date += 1;//adjust days
              DATE_CHK(rtcDate->date, rtcDate->month);
             break;
            case 2:
              rtcDate->month += 1;  //adjust month
              MONTH_CHK(rtcDate->month)
              break;
            case 3:
              rtcDate->year += 1;  //adjust year
              YEAR_CHK(rtcDate->year)
              break;
            }      
              RtcSetDate(rtcDate);  // write new date
              blink = RESET;           //don't blink when button pushed
              updateTimer = GetTicks(); // reset reference 
              DateDisplay(UPDATE);  // function calls itself with UPDATE
           }    
           break;
    
  case _MINUS:        //decrement a value
        
          if(digitPosition) //if modifying date
          {
            rtcDate = RtcGetDate(); //load time structure        
            switch (digitPosition)
            {
            case 1:
              rtcDate->date -= 1;//adjust days
              DATE_CHK(rtcDate->date, rtcDate->month);
            break;
            case 2:
              rtcDate->month -= 1;  //adjust month
              MONTH_CHK(rtcDate->month)
            break;
            case 3:
              rtcDate->year -= 1;  //adjust year
              YEAR_CHK(rtcDate->year)
              break;
            }            
              RtcSetDate(rtcDate);  // write new date
              blink = RESET;           //don't blink when button pushed
              updateTimer = GetTicks(); // reset reference 
              DateDisplay(UPDATE);  // function calls itself with UPDATE
          }    
        break;            
  }
  
  return FALSE;
}

/*******************************************************************************
* Function Name  : STARTTIME DISPLAY
* Description    : show/modify the RTC time.
* Input          : None
* Output         : NOPE
* Return         : None
*******************************************************************************/


bool StartTimeDisplay (bACTION event)
{
  static uint8_t digitPosition = 0; // position of digit:0= nothing,1 = hour,2 = minute
  static FlagStatus blink = SET; // init to set = instant blink when next pushed
   static int8_t startMin;
   static int8_t startHour;
   static uint8_t entry =0;
   rtcTimeTypeDef rtcTime;
        
  switch (event)
  {
  case INIT:    // Called once when menu page first activated
        digitPosition = 0;
        StartTimeDisplay(UPDATE);  // functions calls itself with update event to initialise
       break;
  case UPDATE:  // Called periodically when menu page displayed

      EEPROM_ReadStartTime(entry, (uint8_t*)&startHour,(uint8_t*)&startMin);
      LCD_Addr(0xC3);
      if((blink==SET) && (digitPosition == 1)) write_LCD("  ");
         else LCD_out_hex((char)Rtc_bin2bcd(startHour));
         write_LCD(":");
      if((blink==SET) && (digitPosition == 2)) write_LCD("  ");
        else LCD_out_hex((char)Rtc_bin2bcd(startMin));
         blink ^= 1;    // toggle  the blink status         
        break;  

  case _NEXT:        //move to next digit
         blink = SET; // make sure to blink value instantly
         updateTimer += UPDATEDELAY;  // instant update upon return 
          if(++digitPosition > 2) //cycle through digits
          { 
            digitPosition = 0;
            updateTimer += UPDATEDELAY; 
          }      
          break;
  case _PLUS:        //increment a value
        
          if(digitPosition) //if modifying date
          {
            EEPROM_ReadStartTime(entry, (uint8_t*)&startHour,(uint8_t*)&startMin);         
            switch (digitPosition)
            {
            case 1:
              startHour += 1;//adjust days
              HR_CHK(startHour);
             break;
            case 2:
              startMin += 5;  //adjust month
              MIN_15_CHK(startMin);
              break;
            }      
              EEPROM_WriteStartTime(entry, startHour, startMin);
              rtcTime.sec = 0;
              rtcTime.min = Rtc_bin2bcd (startMin) | 0x80;
              rtcTime.hour = Rtc_bin2bcd (startHour) | 0x80;
              RtcClearAlarmFlag(); //clear RTC alarm
              RtcSetAlarm(&rtcTime);
              blink = RESET;           //don't blink when button pushed
              updateTimer = GetTicks(); // reset reference 
              StartTimeDisplay(UPDATE);  // function calls itself with UPDATE
           }    
           break;    
  case _MINUS:        //increment a value
        
          if(digitPosition) //if modifying date
          {
            EEPROM_ReadStartTime(entry, (uint8_t*)&startHour,(uint8_t*)&startMin);         
            switch (digitPosition)
            {
            case 1:
              startHour -= 1;//adjust days
              HR_CHK(startHour);
             break;
            case 2:
              startMin -= 5;  //adjust month
              MIN_15_CHK(startMin);
              break;
            }      
              EEPROM_WriteStartTime(entry, startHour, startMin);
              rtcTime.sec = 0;
              rtcTime.min = Rtc_bin2bcd (startMin) | 0x80;
              rtcTime.hour = Rtc_bin2bcd (startHour) | 0x80;
              RtcClearAlarmFlag(); //clear RTC alarm
              RtcSetAlarm(&rtcTime);
              blink = RESET;           //don't blink when button pushed
              updateTimer = GetTicks(); // reset reference 
              StartTimeDisplay(UPDATE);  // function calls itself with UPDATE
           }    
           break;            
          
         
  }
  return FALSE;
}

/*******************************************************************************
* Function Name  : STOPTIME DISPLAY
* Description    : show/modify the RTC time.
* Input          : None
* Output         : NOPE
* Return         : None
*******************************************************************************/


bool DurationDisplay (bACTION event)
{
  static uint8_t digitPosition = 0; // position of digit:0= nothing,1 = hour,2 = minute
  static FlagStatus blink = SET; // init to set = instant blink when next pushed
   static int8_t stopMin;
   static int8_t stopHour;     
   static uint8_t entry=0;     
  switch (event)
  {

  case INIT:    // Called once when menu page first activated
        digitPosition = 0;
        DurationDisplay(UPDATE);  // functions calls itself with update event to initialise
       break;
  case UPDATE:  // Called periodically when menu page displayed

      EEPROM_ReadStopTime(entry, (uint8_t*)&stopHour,(uint8_t*)&stopMin);
      LCD_Addr(0xC3);
      if((blink==SET) && (digitPosition == 1)) write_LCD("  ");
         else LCD_out_hex((char)Rtc_bin2bcd(stopHour));
         write_LCD(":");
      if((blink==SET) && (digitPosition == 2)) write_LCD("  ");
        else LCD_out_hex((char)Rtc_bin2bcd(stopMin));
        //write_LCD(":");
     // LCD_out_hex(0x00);
         blink ^= 1;    // toggle  the blink status         
        break;  

  case _NEXT:        //move to next digit
         blink = SET; // make sure to blink value instantly
         updateTimer += UPDATEDELAY;  // instant update upon return 
          if(++digitPosition > 2) //cycle through digits
          { 
            digitPosition = 0;
            updateTimer += UPDATEDELAY; 
          }      
          break;
  case _PLUS:        //increment a value
        
          if(digitPosition) //if modifying date
          {
            EEPROM_ReadStopTime(entry, (uint8_t*)&stopHour,(uint8_t*)&stopMin);         
            switch (digitPosition)
            {
            case 1:
              stopHour += 1;//adjust days
              HR_CHK(stopHour);
             break;
            case 2:
              stopMin += 5;  //adjust month
              MIN_15_CHK(stopMin);
              break;
            }      
              EEPROM_WriteStopTime(entry, stopHour, stopMin);
              blink = RESET;           //don't blink when button pushed
              updateTimer = GetTicks(); // reset reference 
              DurationDisplay(UPDATE);  // function calls itself with UPDATE
           }    
           break;    
  case _MINUS:        //increment a value
        
          if(digitPosition) //if modifying date
          {
            EEPROM_ReadStopTime(entry, (uint8_t*)&stopHour,(uint8_t*)&stopMin);         
            switch (digitPosition)
            {
            case 1:
              stopHour -= 1;//adjust days
              HR_CHK(stopHour);
             break;
            case 2:
              stopMin -= 5;  //adjust month
              MIN_15_CHK(stopMin);
              break;
            }      
              EEPROM_WriteStopTime(entry, stopHour, stopMin);
              blink = RESET;           //don't blink when button pushed
              updateTimer = GetTicks(); // reset reference 
              DurationDisplay(UPDATE);  // function calls itself with UPDATE
           }    
           break;            
  }
  return FALSE;
}



/*******************************************************************************
* Function Name  : Skip First Record
* Description    : Starts/stops record.
* Input          : None
* Output         : NOPE
* Return         : None
*******************************************************************************/

#define ACK       0x00
#define NACK      0x80

bool Sleep(bACTION event)
{
  
  switch (event)
  {
    case INIT:
      write_LCD_line2("-> to Sleep");
      break;
      
    case UPDATE:
      break;
      
    case _NEXT:
      
        InitBatteryMeasure();
        write_LCD_line2("battery");
        while(MonitorBattery())
        {
          delay_ms(200);
        }
        write_LCD_line2("bad");
        
        
      break;      
      
    case _PLUS:
      break;
         
    case _MINUS:  
      break;
      
    case CLOSE:
      break;
      
  }  
  return FALSE;
}

/*******************************************************************************
* Function Name  : contrast for lcd DOGM162W
* Description    : 
* Input          : None
* Output         : NOPE
* Return         : None
*******************************************************************************/

#define ACK       0x00
#define NACK      0x80

bool Contrast(bACTION event)
{
  int contrast;
  uint8_t s[8];
  
  switch (event)
  {
    case INIT:
      write_LCD_line2("+/- to Adj");
      contrast = LCD_ContrastAdjust(0); // no adjust
      LCD_Addr(0x8B);
      sprintf(s,"%02d",contrast);
      write_LCD(s);
      break;
      
    case UPDATE:
      break;
      
    case _NEXT: 
      break;      
      
    case _PLUS:
      contrast = LCD_ContrastAdjust(1);
      LCD_Addr(0x8B);
      sprintf(s,"%02d",contrast);
      write_LCD(s);
      break;
         
    case _MINUS:  
      contrast = LCD_ContrastAdjust(-1);
      LCD_Addr(0x8B);
      sprintf(s,"%02d",contrast);
      write_LCD(s);
      break;
      
    case CLOSE:
      break;
      
  }  
  return FALSE;
}



/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : None
* Output         : NOPE
* Return         : None
*******************************************************************************/
bool DspBoot(bACTION event)
{  
  int ret;
  static bool wasProblem = FALSE;
  
  switch (event)
  {
    case INIT:   
      if (!wasProblem) write_LCD_line2("-> to start");
      break;
      
    case UPDATE:
      break;
      
    case _NEXT:  
      write_LCD_line1("Updating... ");
      write_LCD_line2("            ");
      
      ret = boot_dsp();
      if (ret == 0)
      {
         write_LCD_line2("  All good  ");
         DSP_Reset(HOLD_IN_RESET); 
         delay_ms(10);
         DSP_Reset(RELEASE_RESET); 
      }
      else
      {
         write_LCD_line2("  Problem   ");
         wasProblem = TRUE; 
      }
      delay_ms(1500);
      
      write_LCD_line1(" DSP Boot   ");
      DspBoot(INIT);
      
      break;
    case _PLUS:
      break;
         
    case _MINUS:  
      break;
      
    case CLOSE:
      wasProblem = FALSE;
      break;
      
  }  
  return FALSE;
}

/*******************************************************************************
* Function Name  : get sample from analog board
* Description    : Starts/stops record.
* Input          : None
* Output         : NOPE
* Return         : None
*******************************************************************************/
bool Test(bACTION event)
{  
 const int MAXOPTION  = 2; 
//GPIO_InitTypeDef GPIO_InitStructure; 
//  uint32_t i;
 // uint16_t BatteryVal;
  static uint8_t  Option;
  static int8_t CardPrc;
  int ret;
   uint8_t  *OptionItem[] = {" Check Card ",
                             "Format Card ",
                             "Write Mailbox",
                             "TEST BATTERY",
                             " DELETE "};
    
  //uint8_t  *OptionItem[] = {"   BOOTLOADER   "}; 
  
  
  switch (event)
  {

  case INIT:
        Option =0;
        //break; //INIT event runs update routine by not breaking
  case UPDATE:
        //LCD_Addr(0xAB);
        write_LCD_line2(OptionItem[Option]);
        
    
    
        break;
  case _NEXT:  
           // perform function for selected option
           //clear_LCD();
           switch (Option)
           {
           case 0: //test card
                  //if no SD CARD in SLOT then exit card check function 
                 if (GPIO_ReadInputDataBit(SD_DETECT_PORT, SD_DETECT_PIN))
                   write_LCD_line2(" No SD Card ");
                 else  // test card with check
                 {
                   write_LCD_line2(" Checking.. ");
                   CardPrc = (int8_t)UsedCardSpace();
                   
                   if(CardPrc <0)
                   {
                     write_LCD_line2(" Card Error ");
                      //initialize fat system
                      f_mount(&fs, "", 0);
                   }
                   else if (CardPrc == 100)
                   {
                    write_LCD_line2("  100% Used  "); 
                   }
                   else
                   {
                     write_LCD_line2("  "); LCD_out_hex((char)Rtc_bin2bcd(CardPrc));  write_LCD("% Used  ");
                   }
                 }
                  delay_ms(2000);  //delay 2 secs  
             
                break;
                
                
           case 1: //FORMAT CARD
                  //if no SD CARD in SLOT then exit formating function 
                 if (GPIO_ReadInputDataBit(SD_DETECT_PORT, SD_DETECT_PIN))
                   write_LCD_line2(" No SD Card ");
                 else  // format card
                 {
                    ClearWatchdogInInterrupt(60*10);  //allow watchdog reset in systick int for X amount of seconds( here set to 10 minutes)
                    write_LCD_line2("Formatting..");
                    ret = FormatCard();// check status of card
                    if(ret != 0) write_LCD_line2(" Card Error ");
                    else   
                    {
                      write_LCD_line2(" Done....    ");
                       f_mount(&fs, "", 0);
                    }
                    ClearWatchdogInInterrupt(0);  //disable watchdog reset in systick int
                 }
                  delay_ms(2000);  //delay 2 secs  
             
                break;
           
                
           case 2: //write mailbox NFC
             write_LCD_line1("Write Mailbox");
             //NfcWriteTestBlock(0); 
             break;
                
           case 3: //test battery
               
               write_LCD_line1("Battery Test");
               LCD_Addr(0xC2);
               //CheckBattery(); //ignore first read
               // if (CheckBattery() < 0xB80)
                  //if low
                     write_LCD_line2("    LOW");
                 //else write_LCD_line2("    GOOD");
                
                
               
               
               delay_ms(2000);  //delay 2 secs                 break;   
               break;
           case 4:   //Delete
                 if (GPIO_ReadInputDataBit(SD_DETECT_PORT, SD_DETECT_PIN))
                 { 
                   write_LCD_line2(" No SD Card ");
                 }
                 else
                 {
                   write_LCD_line1("  Deleting  ");
                   if(DeleteCard())
                    {  
                        clear_LCD();
                        write_LCD_line1("Error ");  
                    }
                   
                 }
                delay_ms(2000);  //delay 2 secs                
               break;
           }
           clear_LCD();
           write_LCD_line1 ( CurActiveMenu.PageName ); 
           Test(UPDATE);  
        break;
        
  case _PLUS:
         Option++;
         if(Option >= MAXOPTION) Option =0;
         Test(UPDATE);  
        break;        
        
  case _MINUS:  
         Option--;
         if(Option == 0xff) Option = (MAXOPTION -1);
         Test(UPDATE);  
        break;        
        
        
        
  }
   
  return FALSE;
}


/*******************************************************************************
* Function Name  : MAIN POWER OFF
* Description    : Turns OFF Board.
* Input          : None
* Output         : NOPE
* Return         : None
*******************************************************************************/
bool MainPowerOff(bACTION event)
{  

   static uint8_t bxor =0;
  switch (event)
  {  
    
  case INIT: 
            break;
  case UPDATE:            //check status of power on flags and display

    
       LCD_Addr(0x8A);
      //DISPLAY TIMOUT VALUE
       //LCD_out_hex(Rtc_bin2bcd(*((uint8_t*)&Timeout+1)));
       LCD_out_hex(Rtc_bin2bcd(*(uint8_t*)&timeout));        
    
    
    LCD_Addr(0xC0);
          //check MCU flag and display
          if (GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_8))
                     write_LCD("MCU");
               else  write_LCD("   ");
               
           LCD_Addr(0xC4);
          //check MCU flag and display
          if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9))
                     write_LCD("USB");
               else  write_LCD("   ");               
               
          LCD_Addr(0xC8);     
          if (RtcGetAlarmFlag())
                      write_LCD("RTC");
               else  write_LCD("   ");              
               
               
               
           break;    
                        
    
//QUICK TEST OF POWER CONSUMPTION
    case _NEXT:  

         //Keep pwr on
      if(bxor) timeout=0;//GPIO_WriteBit(GPIOB,GPIO_Pin_8, Bit_SET);
      
      
      else timeout=0; //GPIO_WriteBit(GPIOB,GPIO_Pin_8, Bit_RESET);
        
          //will not run after this point unless usb or rtc flags set
          
      bxor ^=1;
      break;
  }
  return FALSE;
}




/*******************************************************************************
* Function Name  : SURVEY DISPLAY
* Description    :
* Input          : None
* Output         : NOPE
* Return         : None
*******************************************************************************/
bool SurveyDisplay(bACTION event)
{  
  static int digitPosition = 0; // position of digit:0= nothing,1 = hour,2 = minute
  // static FlagStatus ChangeTime = RESET; // set when clock values are being altered by user
  //uint8_t timestring[10]	;
  static FlagStatus blink = SET; // init to set = instant blink when next pushed
  static uint8_t surveyName[8]; //max id characters + 1
  int i,j; 
  int chr;
  static bool modified = FALSE; //if name change
  bool invalidCharFlag; 
   
  switch (event)
  {

    case INIT:
              //First check if this menu is enabled in current protocols      
               if (!HasAnyProtocolOption(PROTOCOL_SURVEY)) return TRUE; // skip this menu if not enabled in protocols
      
               write_LCD_line1("Survey Name:");
               
              //get survey string
               digitPosition = 0;
               modified = FALSE;
               for(i=0;i<7;i++) surveyName[i] =0;
               
               EEPROM_ReadSurveyName(surveyName);
               LCD_Addr(0xC0);
               
               invalidCharFlag=FALSE;
               for(i=0;i<7;i++) //check if all chars valid if not then init chars
               {
                 if(surveyName[i] >= sizeof(charTable)) {
                   surveyName[i] = 10; //'char 'A'
                   invalidCharFlag=TRUE;
                 }
               }
               if(invalidCharFlag) //if there was invalid char/s in survey name, save updated name
               {
                 EEPROM_WriteSurveyName(surveyName);
               }
               for(i=0;i<7;i++)
               {
                  write_LCD_char(charTable[surveyName[i]]);  //display all chars
                  //if(CharTable[SurveyName[i]] == ' ') break; 
               }
               //for(;i<7;i++) write_LCD(" "); //finish clearing chars
               
               //write_LCD_line2(SurveyName);
            //   for(i=0;i<7;i++)
            //   {
            //     ALPHA_CHK(SurveyName[i]);    
            //   }

              // EEPROM_WriteSurveyName(0, SurveyName);                    
        break;
                
  case UPDATE:
              {
                 //EEPROM_ReadSurveyName(0, SurveyName);
                LCD_Addr(0xC0);
                for(i=0;i<7;i++)
                {
                  
                 if((blink==SET) && ((digitPosition-1) == i))
                 {
                   if ((charTable[surveyName[digitPosition-1]] < 0x21) && ((digitPosition-1) == i)) 
                  {
                    write_LCD("_");
                    //if (SurveyName[i] == 0x00) { write_LCD(" "); break;}
                  }
                  else write_LCD(" ");               
                 }                   
                  else  
                  {
                    if (charTable[surveyName[i]] < 0x21) 
                    {
                      write_LCD(" ");
                      //break;
                    }else  write_LCD_char(charTable[surveyName[i]]);
                  }   
                 
                 //if(CharTable[SurveyName[i]] == ' ') break; 
                }
              }
               blink ^= 1;    // toggle  the blink status 
          break;
          
    case _NEXT:      
         blink = SET; // make sure to blink value instantly
         updateTimer += UPDATEDELAY;  // instant update upon return 
         /*
         for(i=0;i<7;i++) 
         {
           if (CharTable[SurveyName[i]] == ' ')  
           {
             SurveyName[i+1] = SurveyName[i];
             i++;
             break;
           }
         }
         */
         i=SURVEY_NAME_LENGTH;    
         if(charTable[surveyName[digitPosition-1]] =='_')
         {
           j = digitPosition-1;
           for(; j<i; j++)
           {
            surveyName[j] = sizeof(charTable)-1;
           }
          digitPosition = i+1; //make digitposition greater than string length 
         }
         else
         digitPosition++;
         
         if(digitPosition > i) //cycle through digits
          { 
            digitPosition = 0;
            //for (i=7;i>0;i--) if(SurveyName[i-1] < 0x21) SurveyName[i-1] =0; else break; 

            //ChangeTime = RESET;
            updateTimer += UPDATEDELAY; 
            blink = RESET;
            //save the survey name 
           EEPROM_WriteSurveyName(surveyName);
           modified = FALSE;
           SurveyDisplay(INIT); 
          }
            break;      
      
  case _PLUS:       //increment a value
        
          if(digitPosition) //if modifying date
          {
            //EEPROM_ReadStopTime(RecNum, (uint8_t*)&StopHour,(uint8_t*)&StopMin);         
            modified = TRUE;
            chr = surveyName[digitPosition-1];
            if(digitPosition < 2)
            {
              if(++chr >= sizeof(charTable)-1) chr=0;  //cannot access space char for first 6 digits
            }
            else
              if(++chr >= sizeof(charTable)) chr=0;  //only on last digit can it access the space char
            
            surveyName[digitPosition-1] = chr;        
            updateTimer = GetTicks(); // reset reference 
            blink = RESET; 
            SurveyDisplay(UPDATE);  // function calls itself with UPDATE
           }    
           break;         
      
   case  _MINUS:        //increment a value
        
          if(digitPosition) //if modifying date
          {
            //EEPROM_ReadStopTime(RecNum, (uint8_t*)&StopHour,(uint8_t*)&StopMin);         
            modified = TRUE;
            chr = surveyName[digitPosition-1];
            if(digitPosition < 2)
            {
             if(--chr < 0) chr = sizeof(charTable) -2; //cannot access space char for first 6 digits
            }
              else
             if(--chr < 0) chr = sizeof(charTable) -1; //only on last digit can it access the space char
            
            surveyName[digitPosition-1] = chr;
            updateTimer = GetTicks(); // reset reference 
            blink = RESET; 
            SurveyDisplay(UPDATE);  // function calls itself with UPDATE
           }    
           break;            
         
          
          
  case CLOSE: //called when PAGE closes allows clean up and saving 
        if(modified)
        {
           EEPROM_WriteSurveyName(surveyName);
           modified = FALSE;    
        }
       break;    
  }
  return FALSE;
}


/*******************************************************************************
* BATTERY VOLTAGE DISPLAY
*
*
*******************************************************************************/

bool Battery(bACTION event)
{
  uint16_t BatteryVal;
  
  
  switch (event)
   {
    case INIT:
     
    case UPDATE:
       LCD_Addr(0xc4);
       BatteryVal = CheckBattery();
       LCD_out_hex(*((uint8_t*)&BatteryVal+1));
       LCD_out_hex(*(uint8_t*)&BatteryVal);   
  
      break;

   } 
   return FALSE;
}

/*******************************************************************************
* LOW BATTERY ACKNOWLEDGE DISPLAY
* tells user when low battery shutdow occured
* also tells them to replace batteries
*******************************************************************************/

bool LowBatteryAck(bACTION event)
{

  static int32_t flash;
  
  switch (event)
   {
   case INIT:
          write_LCD_line1("Rec stopped ");
          write_LCD_line2("low battery ");   
          flash = 0;
        break;
  case UPDATE:  // Called periodically when menu page displayed
        if(flash++ > 5)
        {
          write_LCD_line1("Press PAGE  ");
          write_LCD_line2(" to Continue");
          if(flash >=8) flash = 0;
        }
        else
        {
          write_LCD_line1("Rec stopped ");
          write_LCD_line2("low battery ");
        }
        break;  
  case _NEXT:       
  case _PLUS:                
  case _MINUS:       
        menuCurPos++;
        if ( menuCurPos >= maxMenuItems )
        {
          menuCurPos = 0;
        }
       break;
  case CLOSE: //called when PAGE closes allows clean up and saving 
         if(timeout)
         {
           RtcSetUser2Reg(0); //clear flag that marks shutdown due to low battery only if it was done by push of button and not due to time out
           delay_ms(500);
         } 
       break;   
   } 
   return FALSE;
}




/*******************************************************************************
* TEST CODE
*
* run code to test 
*******************************************************************************/

bool CodeTest(bACTION event)
{
#ifdef TEST_MENU 
  const int  MAXOPTION = 3; //display only the first three options for test menu
#else  
  const int  MAXOPTION = 8;
#endif
//GPIO_InitTypeDef GPIO_InitStructure; 
//  uint32_t i;
 // uint16_t BatteryVal;
  static int  option;
  //uint8_t text_string[256];
  //double f = 34.9864534;
  //double g = 97.3394746;
  uint8_t *OptionItem[] = {" Periherals ",
                           " GPS        ",
                           " Recording  ",
                           " Dump Log   ",
                           " Flash Boot ",
                           " Flash DSP  ",
                           " Clear Log? ",
                           " test test? "
                          };
    
  switch (event)
  {

  case INIT:
      option =0;
  case UPDATE:
      write_LCD_line2(OptionItem[option]);
      break;
  case _NEXT:  
      // perform function for selected option
      //clear_LCD();  
      switch (option)
      {
        case 0: //test card
          HardwareTest();
          break;
        case 1: //test the gps
          GpsTest();
          break;
        case 2: //test recording high low and bat
          TestRecording();
          break;
        case 3: //save log file from eeprom to card
          write_LCD_line2 (" Saving...  ");  
          // first save to eeprom the log file
          SaveLogFileToEeprom();
          if (!SaveLogToCard())
          {  
            write_LCD_line2 (" Done       "); 
            option = MAXOPTION; // select the hidden option
          }          
          else
            write_LCD_line2 (" Failed     ");  
          delay_ms(1000);
          break; 
        case 4: //Flash bootloader
          write_LCD_line1 ("FlashingBoot");    
          write_LCD_line2 ("            ");  
          if (!FlashBootFile())
          {  
            write_LCD_line2 (" Done       "); 
          }          
          else
            write_LCD_line2 (" Failed     ");  
          delay_ms(1000);
          break;             
          
        case 5:
          write_LCD_line1("Flashing DSP");
          write_LCD_line2("            ");
          
          if (boot_dsp() == 0)
          {
            write_LCD_line2("  All good  ");
            DSP_Reset(HOLD_IN_RESET); 
            delay_ms(10);
            DSP_Reset(RELEASE_RESET); 
          }
          else
          {
            write_LCD_line2("  Failed   ");
            delay_ms(3000);
          }
          delay_ms(1500);
          break;
        case 6: //Hidden option to clear log file, set after saving log file
          write_LCD_line2 (" Clearing.. ");
          ClearLog();
          SaveLogFileToEeprom();
          write_LCD_line2 (" Done       ");
          delay_ms(500); 
          
        case 7: //generic test option
                 
         /* 
         sprintf((char*)text_string,"Latitude = %f\r\nLongitude = %f\r\n\r\n", f,g);
         
         if (WriteLogFileToCard("gps_log.txt", text_string) == 0)
         { //success in writing log file
           write_LCD_line2 (" Done       ");  
         }
         else
         {
           write_LCD_line2 (" Failed     ");  
         }
          */
         break;
        default:
          option = 0;
          break;
      }
      clear_LCD();
      write_LCD_line1 ( CurActiveMenu.PageName ); 
      CodeTest(UPDATE);  
      break;
        
  case _PLUS:
      option++;
      if(option >= MAXOPTION) option =0;
      CodeTest(UPDATE);  
      break;        
        
  case _MINUS:  
      option--;
      if(option < 0) option = (MAXOPTION -1);
      CodeTest(UPDATE);  
      break;        
  }
   
  return FALSE;
}


/*******************************************************************************
*
*
*******************************************************************************/

bool StartAndDuration(bACTION event)
{
  static int digitPosition = 0; // position of digit:0= nothing,1 = hour,2 = minute
  static FlagStatus blink = SET; // init to set = instant blink when next pushed
   static int entry;
   static bool isLocked = TRUE;
   static Protocol_UserDef_Start_Span_TypeDef startSpanItem;
   
//   rtc_time rtcTime;
        
  switch (event)
  {

  case INIT:    // Called once when menu page first activated
        digitPosition = 0;
        entry = CurActiveMenu.MenuType - ENTRY1; //to get the offset (all ENTRY DEFs must be in sequential order in enum)
        if (entry >= GetNumOfProtocolSlots()) return TRUE; //skip protocol greater thean max selected
        write_LCD_line1("Start");
        
        if (entry < 0)
        {
          write_LCD_line1("ERROR");
          entry =0;
        }
        else
        {
          if (entry >= 10)
            write_LCD_char(0x30 + ((entry+1) / 10));
                      
          write_LCD_char(0x30 + ((entry+1) % 10));
          write_LCD_char(':');
        }       

        write_LCD_line2("Span  :");
        //deterime if start and span times are changeable
        StartAndDuration(UPDATE);  // functions calls itself with update event to initialise
        isLocked = IsProtocolStartSpanLocked(entry);
        
        
       break;
  case UPDATE:  // Called periodically when menu page displayed
      
      startSpanItem = *ReadProtocolStartSpanTime(entry);

      LCD_Addr(0x87); //Display Start time
      if((startSpanItem.spanHour == 0) && (startSpanItem.spanMin == 0) && (!digitPosition))  write_LCD(" OFF ");
      else
       {       
          if((blink==SET) && (digitPosition == 1)) write_LCD("  ");
             else LCD_out_hex((char)Rtc_bin2bcd(startSpanItem.startHour));
             write_LCD(":");
          if((blink==SET) && (digitPosition == 2)) write_LCD("  ");
            else LCD_out_hex((char)Rtc_bin2bcd(startSpanItem.startMin));  
       }
     LCD_Addr(0xC7);  //Display Duration time
      if((blink==SET) && (digitPosition == 3)) write_LCD("  ");
         else LCD_out_hex((char)Rtc_bin2bcd(startSpanItem.spanHour));
         write_LCD(":");
      if((blink==SET) && (digitPosition == 4)) write_LCD("  ");
        else LCD_out_hex((char)Rtc_bin2bcd(startSpanItem.spanMin));
        //write_LCD(":");
     // LCD_out_hex(0x00);
         blink ^= 1;    // toggle  the blink status         
        break;  

  case _NEXT:        //move to next digit
         if (isLocked) break; 
         blink = SET; // make sure to blink value instantly
         updateTimer += UPDATEDELAY;  // instant update upon return 
          if(++digitPosition > 4) //cycle through digits
          { 
            digitPosition = 0;
            updateTimer += UPDATEDELAY; 
          }      
          //if((!StopHour) & (!StopMin) & ((DigitPosition == 1) |  (DigitPosition == 2)))
          //  DigitPosition =3;
          
          break;
  case _PLUS:        //increment a value
          switch (digitPosition)
          {
          case 1:
          case 2:  
                  {
                    startSpanItem = *ReadProtocolStartSpanTime(entry);
    
                    switch (digitPosition)
                    {
                    case 1:
                      startSpanItem.startHour += 1;//adjust days
                      HR_CHK(startSpanItem.startHour);
                     break;
                    case 2:
                      startSpanItem.startMin += 5;  //adjust month
                      MIN_15_CHK(startSpanItem.startMin);
                      break;
                    }      
                      WriteProtocolStartSpanTime(entry, &startSpanItem);
                      SetNextAlarm();
                      blink = RESET;           //don't blink when button pushed   
                      updateTimer = GetTicks(); // reset reference 
                      StartAndDuration(UPDATE);  // function calls itself with UPDATE    
                   } 
                  break;
                  
          case 3:
          case 4:
            
                    {
                      startSpanItem = *ReadProtocolStartSpanTime(entry);         
                      switch (digitPosition)
                      {
                      case 3:
                        startSpanItem.spanHour += 1;//adjust days
                        HR_CHK(startSpanItem.spanHour);
                       break;
                      case 4:
                        startSpanItem.spanMin += 5;  //adjust month
                        MIN_15_CHK(startSpanItem.spanMin);
                        break;
                      }      
                        WriteProtocolStartSpanTime(entry, &startSpanItem);
                        blink = RESET;           //don't blink when button pushed
                        updateTimer = GetTicks(); // reset reference 
                        StartAndDuration(UPDATE);  // function calls itself with UPDATE
                     }
                    break; 
          }
                     
         break;    
  case _MINUS:        //increment a value
          switch (digitPosition)
          {
          case 1:
          case 2:  
                  {
                    startSpanItem = *ReadProtocolStartSpanTime(entry);                
                    switch (digitPosition)
                    {
                    case 1:
                      startSpanItem.startHour -= 1;//adjust days
                      HR_CHK(startSpanItem.startHour);
                     break;
                    case 2:
                      startSpanItem.startMin -= 5;  //adjust month
                      MIN_15_CHK(startSpanItem.startMin);
                      break;
                    }      
                      WriteProtocolStartSpanTime(entry, &startSpanItem);
                      SetNextAlarm();
                      blink = RESET;           //don't blink when button pushed
                      updateTimer = GetTicks(); // reset reference 
                      StartAndDuration(UPDATE);  // function calls itself with UPDATE
                   } 
                  break;
                  
          case 3:
          case 4:
            
                    {
                      startSpanItem = *ReadProtocolStartSpanTime(entry);       
                      switch (digitPosition)
                      {
                      case 3:
                        startSpanItem.spanHour -= 1;//adjust days
                        HR_CHK(startSpanItem.spanHour);
                       break;
                      case 4:
                        startSpanItem.spanMin -= 5;  //adjust month
                        MIN_15_CHK(startSpanItem.spanMin);
                        break;
                      }    
                        WriteProtocolStartSpanTime(entry, &startSpanItem);
                        blink = RESET;           //don't blink when button pushed
                        updateTimer = GetTicks(); // reset reference 
                        StartAndDuration(UPDATE);  // function calls itself with UPDATE
                     }
                    break; 
          }
          
          
   
         break;           
  }
  return FALSE;
}


/*******************************************************************************
*
*
*******************************************************************************/

bool ProtocolDisplay(bACTION event)
{
   static int protocolSelectedIndex;
//   rtc_time rtcTime;
        
  switch (event)
  {

  case INIT:    // Called once when menu page first activated
    
       
        protocolSelectedIndex = CurActiveMenu.MenuType - ENTRY1;  // ENRTYx enum members must be in sequential order 
         if (protocolSelectedIndex >= GetNumOfProtocolSlots()) return TRUE; //skip protocol greater thean max selected
        if (protocolSelectedIndex < 0)
        {
         write_LCD_line1("ERROR");
          protocolSelectedIndex = 0; 
        }
        else
        {
          write_LCD_line1("Protocol");
          if (protocolSelectedIndex>=10)
            write_LCD_char(0x30 + ((protocolSelectedIndex+1)/10) );
          write_LCD_char(0x30 + ((protocolSelectedIndex+1)%10) );
          
          write_LCD_char(':');
        
        }

        
        write_LCD_line2(GetNameOfProtocol(protocolSelectedIndex, TRUE));
       
        break;
  case UPDATE:  // Called periodically when menu page displayed     
      break;  
  
  case _NEXT:        //move to next digit
      break;
          
  case _PLUS: 
    DecProtocolInSlot(protocolSelectedIndex);
    write_LCD_line2(GetNameOfProtocol(protocolSelectedIndex, TRUE));
    break;
         
  case _MINUS:
    IncProtocolInSlot(protocolSelectedIndex);
    write_LCD_line2(GetNameOfProtocol(protocolSelectedIndex, TRUE));
    break; 
  
  case CLOSE:  
    //we're going to check if all start stop times have been set to off
    // -if so then we can set the default start stop time slots back to 2
     if (protocolSelectedIndex == (GetNumOfProtocolSlots()-1) )  // if last entry then check if all start stop times have been set to off
     {
       bool allOff = TRUE;
       
       //if available slots are equal to default then skip this check
       if (GetNumOfProtocolSlots() == DEFAULT_NUM_PROTOCOL_SLOTS) break;
       
       
       for (int i=0;i < protocolSelectedIndex+1;i++)
       {
         if ( GetIndexOfProtocolFromName(GetNameOfProtocol(i,FALSE))  != PROTOCOL_OFF_INDEX) 
         {
           allOff = FALSE;
           break;
         }
       }
       // if all times are set to off then we can reset 
       if (allOff)
        SetNumOfProtocolSlots(DEFAULT_NUM_PROTOCOL_SLOTS);
     }
      break;  
  default:
    break;
  }
  
  return FALSE;
}


/*******************************************************************************
* SAMPLING DISPLAY
*
*
*******************************************************************************/
bool Sampling(bACTION event)
{

  switch (event)
  {
  
    case INIT:
      write_LCD_line2(GetUserDefSamplingModeName());
      break;     
    case UPDATE:
      break;
    
    case _PLUS:
      DecUserDefSamplingMode();
      write_LCD_line2(GetUserDefSamplingModeName());
      break;
         
    case _MINUS:
      IncUserDefSamplingMode();
      write_LCD_line2(GetUserDefSamplingModeName());
      break;
  } 
  return FALSE;
   
}

/*******************************************************************************
* STATION DISPLAY
*
*
*******************************************************************************/
bool StationDisplay(bACTION event)
{
  
  static int  option;

   switch (event)
   {
   case INIT:
             if (!HasAnyProtocolOption(PROTOCOL_STATION)) return TRUE;
             write_LCD_line1("Station:");
             option = EEPROM_ReadStation();
             if(( option >= 5) || (option < 0))
                {
                  option = 0;
                  EEPROM_WriteStation(option);
                } 
               LCD_Addr(0xc4);
               write_LCD(stationItem[option]);
              break;     
   case UPDATE:
            break;
            
   case _PLUS:
         if(--option <0) option = (5-1);
         EEPROM_WriteStation(option);
               LCD_Addr(0xc4);
               write_LCD(stationItem[option]);
         break;
         
   case _MINUS:
         if(++option >=5) option = 0;
         EEPROM_WriteStation(option);
               LCD_Addr(0xc4);
               write_LCD(stationItem[option]);
         break;
   }  
   return FALSE;
   
}


/*******************************************************************************
* STARTUP_RANDOM_DELAY
*
*
*******************************************************************************/

bool NFCTestDelay(bACTION event)
{
  static bool writeMem = FALSE; 
  static int downcount = 0;
  static int nonce = 1;

  switch (event)
  {
    case INIT:
      write_LCD_line2("press write");
      break;     
      
    case UPDATE:
      if (downcount > 0)
        downcount--;
      else
         write_LCD_line2("press write");
      break;
    

  case _NEXT:        //move to next digit
      write_LCD_line2("saved");
      downcount = 5;
      
      NfcWriteTestBlock(nonce++);
      break;
         

  } 
  return FALSE;
   
}

/*******************************************************************************
* STARTUP_RANDOM_DELAY
*
*
*******************************************************************************/

bool StartupRandomDelay(bACTION event)
{
  static int delay_value; 
  char str[16];
  switch (event)
  {
    case INIT:
      delay_value = EEPROM_ReadRandomDelay();
      sprintf(str,"   00-%02umin ", delay_value);
      write_LCD_line2((uint8_t*)str);
      break;     
    case UPDATE:
      break;
    
    case _PLUS:
      delay_value = EEPROM_WriteRandomDelay(++delay_value);
      sprintf(str,"   00-%02umin ", delay_value);
      write_LCD_line2((uint8_t*)str);
      break;
         
    case _MINUS:
      delay_value = EEPROM_WriteRandomDelay(--delay_value);
      sprintf(str,"   00-%02umin ", delay_value);
      write_LCD_line2((uint8_t*)str);
      break;
  } 
  return FALSE;
   
}








uint8_t* GetSurveyName(void)
{
  int i;
  uint8_t surveyChars[7]; //max id characters
  static uint8_t surveyName[8]; //max id characters  
  //read survey name chars from eeprom
  
  EEPROM_ReadSurveyName(surveyChars);
  
  for(i=0;i<7;i++)
  {
    if( charTable[surveyChars[i]] == '_') break;
    surveyName[i] = charTable[surveyChars[i]];
  }
  surveyName[i] = 0; //null
  
  return surveyName;
}


uint8_t* GetStationID(void)
{
  return stationItem[EEPROM_ReadStation()];
}


/*******************************************************************************
*
*
*******************************************************************************/

bool GpsOptionsDisplay(bACTION event)
{      
  static uint8_t option = 0;
    uint8_t *optionStrings[] = {"    OFF     ",
                                "  LOG ONLY  ",
                                " LOG & SYNC "
                          };
  
  
  switch (event)
  {
    case INIT:    // Called once when menu page first activated
          write_LCD_line1("Gps Log:");     
          option =  EEPROM_ReadGpsOptions();
          write_LCD_line2(optionStrings[option]);
          break;
    case UPDATE:  // Called periodically when menu page displayed     
        break;  
    
    case _NEXT:        //move to next digit
        break;
            
    case _PLUS: 
      option = EEPROM_WriteGpsOptions(++option);  
      write_LCD_line2(optionStrings[option]);
      break;
           
    case _MINUS:
      option = EEPROM_WriteGpsOptions(--option); 
      write_LCD_line2(optionStrings[option]);
      break;    
      
    default:
      break;
  }
  
  return FALSE;
}



/*******************************************EOF********************************/