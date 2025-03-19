/*******************************************************************************
* File Name         : Menu.c
* Author              : Raje K, Nick P, DOC Electronics Research and Development Team
* Version             : V1.0.0
* Date                 : 14/09/2009
* Description        : This file includes the menu navigation driver for the
						STM3210E-EVAL 
********************************************************************************/

/* Includes -------------------------------------------------------------------*/
#include "Main.h"
//#include "LCD.h"
#include "FatSD.h"
#include "target/target.h"
#include "rtc.h"
#include "menu.h"
#include "usb_lib.h"
#include "adc_AD7680.h"
#include "record.h"
#include "playback.h"
#include "fsmc_sram.h"
#include "sysclk.h"
#include "pushbutton.h"
#include "bootloader.h"
#include "eeprom.h"
#include <stdio.h>
#include "stm32f10x_it.h"
#include "battery.h"

/* Const Variables--------------------------------------------------------------*/


#define DOWN	0
#define UP 		1	

#define INC		1
#define DEC		-1


#define UPDATEDELAY     500 // in msecs
#define TIMEOUT         20000 / UPDATEDELAY// ~20secs

//for RTC
int const days[13] = {31,31,28,31,30,31,30,31,31,30,31,30,31};
#define HR_CHK(HR)  			if(HR>23){ HR=0;} else if(HR<0) {HR=23;}	
#define MIN_SEC_CHK(MIN) 	if(MIN>59){MIN=0;} else if (MIN<0) {MIN=59;}
#define DATE_CHK(DATE,MON) 		if(DATE>days[MON]){DATE=1;} else if (DATE<1) {DATE=days[MON];}
#define MONTH_CHK(MON)		if(MON>12) {MON=1;} else if (MON<1) {MON=12;}
#define YEAR_CHK(YR)			if(YR>99) {YR=0;} else if (YR<0){YR=99;}


#define MIN_15_CHK(MIN) 	if(MIN>55){MIN=0;} else if (MIN<0) {MIN=55;}


#define ALPHA_CHK(CHR)          if(CHR < 0x21){CHR=0;} else if (CHR > 0x7F) {CHR=0;}

/* Global variables -------------------------------------------------------------*/

int8_t PreMenuCurPos;
int8_t MenuCurPos;
int8_t DateAdjustFlag = 0;
uint16_t batterycounter =60000;



static __IO uint32_t SELStatus = 0;
static uint16_t UpdateTimer = 0;
static int16_t Timeout = TIMEOUT;

struct sMenuItem CurActiveMenu;
/*
struct sMenuItem sMainMenu[] = { 	{ "Time: ",TimeDisplay, TIME }, 
					{ "Date:", DateDisplay, RTCDATE},
					{ "Start Time:", StartTimeDisplay, START_TIME},
					{ "Stop Time:", StopTimeDisplay, STOP_TIME},
					{ "Survey Name:", SurveyNameDisplay, SURVEY},
					{ "GPS Location", GPSLocationDisplay, GPS}
					//{ "USB", USB_Start, USB},
                                        //{ "Play Wavefile", WaveFile_Play, WAV}
              			};
*/

//#define IN_FIELD        //use different menu/setup for field version -DEFINED IN PREPROCESSOR

#ifdef IN_FIELD
#define MAX_MENUITEM	5
struct sMenuItem sMainMenu[] = {        { "Time: ",TimeDisplay, TIME }, 
					{ "Date:", DateDisplay, RTCDATE},
					{ "Start Time:", StartTimeDisplay, START_TIME},
					{ "Duration:", DurationDisplay, DUR_TIME},
                                        { "Sampling Rate:",Sampling, SAMP},
                                        { "Bootloader: ",Boot, BOOT }, //needs to be last on list
                                        
                                        
              			};
#else
#define MAX_MENUITEM	7
struct sMenuItem sMainMenu[] = {        { "Time: ",TimeDisplay, TIME }, 
					{ "Date:", DateDisplay, RTCDATE},
					{ "Start Time:", StartTimeDisplay, START_TIME},
					{ "Duration:", DurationDisplay, DUR_TIME},
                                       // { "Survey Name:", SurveyDisplay, SURVEY},
                                       // { "Record: ",Record, RECORD },                                        
                                        { "Code Test: ",CodeTest, CODE },
                                       // { "Playback: ",USB_Start, USB },
                                        { "Sampling Rate:",Sampling, SAMP},
                                         //{ "Test: ",Test, TEST },
                                       // { "SPI Test: ",DMA_Start, DMA },
                                       // { "Syncing: ",Syncing, SYNC },
                                        { "Power Off", MainPowerOff, PWROFF},
                                        { "Bootloader: ",Boot, BOOT }
                                       
              			};
#endif



/* Functions-------------------------------------------------------------------*/

/******************************************************************************* 
* Function Name  : Menu_Init()
* Description    : Initialize the menu
* Input           :  None
* Output         : None
* Return         : None
*******************************************************************************/

void Menu_Init(void)
{

  uint8_t BootFile=0;
  //check to see if boot file on card
  
     //turn on SD card;
     GPIO_WriteBit(GPIOE,GPIO_Pin_2, Bit_RESET);
     //delay_ms(20);
     // f_init();
       /* first get binary file from sd card and load into external sram*/
      delay_ms(50);
     BootFile = CheckBootFile();
     delay_ms(20);
          //turn off card
     	//GPIO_WriteBit(GPIOE,GPIO_Pin_2, Bit_SET);  
  
       //if a file on card, then go to bootloader menu option
        // menu option for bootloading is unreachable when changing pages 
       if(BootFile) 
       {
	write_LCD_line1 ( sMainMenu[MAX_MENUITEM].PageName ); 
	CurActiveMenu = sMainMenu[MAX_MENUITEM];			
        //CurActiveMenu.pfMenuFunc();
	PreMenuCurPos =MAX_MENUITEM;
        MenuCurPos = MAX_MENUITEM;        
                 
       }
       else //goto default option
       {
  
	write_LCD_line1 ( sMainMenu[0].PageName ); 
	CurActiveMenu = sMainMenu[0];			
        //CurActiveMenu.pfMenuFunc();
	PreMenuCurPos =0;
        MenuCurPos = 0;
       }
       
        UpdateTimer = GetTicks();
        CurActiveMenu.pfMenuFunc(INIT);  
        LCD_CursorOff();
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
    if ((uint16_t)(tickvalue- UpdateTimer) >= UPDATEDELAY) 
    {
      
      CurActiveMenu.pfMenuFunc(UPDATE);  // call current page functions UPDATE if it has one
      UpdateTimer = GetTicks();  //set new reference
       Timeout--; //decrement timeout value
      
    }

}


void BattCheck(void)
{
    //Battery check -- shutdown if too low  
     
               batterycounter++; // adds 1 every loop
               if(batterycounter > 62000)
                  {
                      //shutdown 
                    clear_LCD(); 
                    write_LCD_line1("Battery low");
                    delay_ms(3500);
                    clear_LCD();                                        
                    batterycounter =0;
                    NVIC_SystemReset(); //reset MCU
                  }
               else
               if(batterycounter > 60600) 
                 {                        
                   if(GetBatteryADC() > 0xB20) 
                   {
                    //Battery off
                     GPIO_WriteBit(GPIOC,GPIO_Pin_5, Bit_RESET);                     
                     batterycounter =0; 
                   }
                 }
               else  
               if(batterycounter > 60000) // if low then shutdown
               {
                   //Battery on
                     GPIO_WriteBit(GPIOC,GPIO_Pin_5, Bit_SET); 
                            
               }    
}

/*******************************************************************************
* Function Name  : RefreshMenu()
* Description       : Is continuously cycled  
* Input               : None
* Output         	: None
* Return         	: None
*******************************************************************************/

int16_t RefreshMenu()      //returns timeout value
{
      uint16_t BttnOut;   // button flags

      if(MenuCurPos != PreMenuCurPos)   // if new page pending then set it up
      {

        CurActiveMenu.pfMenuFunc(CLOSE);   // call current page item's CLOSE event
        PreMenuCurPos = MenuCurPos;

	CurActiveMenu = sMainMenu[MenuCurPos]; //load new page data
	// commented for EVAL 
	clear_LCD();
	write_LCD_line1 ( CurActiveMenu.PageName ); 
        CurActiveMenu.pfMenuFunc(INIT);   // call page items INIT event
	//sMainMenu[currentpos -1].pfMenuFunc();
      }	
      
      BttnOut =GetButtonStatus();
      
      
      if((BttnOut) || (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9))) Timeout = TIMEOUT; //reset timout value if button pushed of usb pluged in then 
      
     //If Page key pushed the next page 
      if( BttnOut & PAGE_KEY == PAGE_KEY) 
          {
           // DateAdjustFlag = 0;
      	    //Timer1( ENABLE );
	    MenuCurPos++;
            if ( MenuCurPos >= MAX_MENUITEM )
			MenuCurPos = 0;        
          }
      
      //call the battery check routine, shutdown if batteries low
         BattCheck();
       
              
       if( (BttnOut & PLUS_KEY) == PLUS_KEY) CurActiveMenu.pfMenuFunc(_PLUS);
       else
       if( (BttnOut & MINUS_KEY) == MINUS_KEY) CurActiveMenu.pfMenuFunc(_MINUS);
       else
       if( (BttnOut & NEXT_KEY) == NEXT_KEY) CurActiveMenu.pfMenuFunc(_NEXT);
     
      
       CallUpdate();
     
      // timeout occurs - call page items CLOSE event 
      if (Timeout <=0) CurActiveMenu.pfMenuFunc(CLOSE);
      
      
      return(Timeout);
      
}








/*******************************************************************************
* Function Name  : SurveyNameDisplay()
* Description       : Display the Survey name 
* Input               : None
* Output         	: None
* Return         	: None
*******************************************************************************/
/*
void SurveyNameDisplay(void)
{

	uint8_t timestring[15];
	uint8_t *SurveyName;
	struct sRtcTime rtctime;

	rtctime.Hour = 12;
	rtctime.Min = 20;
	rtctime.Sec = 0;
	
	SurveyName = (uint8_t *) malloc (20);
//	EEPROM_ReadSurveyName (0, SurveyName );
	
	sprintf ( timestring, "   %s", "Survey_1709" );

	write_LCD_line2( timestring );
	LCD_CursorOff();
}
*/


/*******************************************************************************
* Function Name  : GPSLocationDisplay()
* Description       : Display the GPS location
* Input               : None
* Output         	: None
* Return         	: None
*******************************************************************************/

/*
void GPSLocationDisplay(void)
{

	uint8_t timestring[10]	;
	struct sRtcTime rtctime;

	rtctime.Hour = 12;
	rtctime.Min = 20;
	rtctime.Sec = 0;

	sprintf ( timestring, "   %02d", 1 );

	write_LCD_line2(timestring);
	LCD_CursorOff();
}

*/

/*******************************************************************************
* Function Name  : USB_Start()
* Description       : Initialise the USB HID device
* Input               : None
* Output         	: None
* Return         	: None
*******************************************************************************/

void USB_Start(bACTION event)
{
	// static uint8_t timestring[15];
	static uint8_t PlaybackState = 0;
        
        
  switch (event)
  { 
  case INIT:
    
    PlaybackState =0;
    
    break;
  case UPDATE:
         
    if(PlaybackState)
    {
      //reset if playback finished
      if(CheckEndPlayback()) 
      { 
        PlaybackState = 0; 
        break;
      }
       //check if buffers need updating
        if(CheckUpdateFlag()) 
        {          
                  	//SD pwr on
	    GPIO_WriteBit(GPIOE,GPIO_Pin_2, Bit_RESET);
            delay_ms(2);
              UpdatePlaybackBuffers();
                  //sd power off
            GPIO_WriteBit(GPIOE,GPIO_Pin_2, Bit_SET);     
        }
    }
    
        break;    
   case _NEXT:  

/*
	// Disble the JoyStick interrupts
	IntExtOnOffConfig(DISABLE);
	// Enable and GPIOD clock
        { 
        // USBCLK = PLLCLK 
          RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
         // Enable USB clock 
         RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
        }
        USB_Init();
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
	IntExtOnOffConfig(ENABLE);        
        write_LCD_line2("  USB started...");
  */
    
    
    
    if (PlaybackState)
    {
      
      PlaybackState =0;
     DisablePlayBackTIMint();
    }
    
    else
    {    
       PlaybackState =1;
           
        	//SD pwr on
	GPIO_WriteBit(GPIOE,GPIO_Pin_2, Bit_RESET);
        delay_ms(2);
        f_init();
        PlaybackInit();

        //sd power off
        GPIO_WriteBit(GPIOE,GPIO_Pin_2, Bit_SET);     
    
        
        Playback_TIM_init_36MHz();
    }   
    break; 


  case CLOSE:
        DisablePlayBackTIMint();
        break;

    
  }     
       
}

/*******************************************************************************
* Function Name  : WaveFile_Play()
* Description       : Play the wavefile from SD Card
* Input               : None
* Output         	: None
* Return         	: None
*******************************************************************************/

/*
void WaveFile_Play(void)
{
  	uint8_t WaveStr[15] = "Playing....";
	uint8_t tempRegRead = 0;

	// Disble the JoyStick interrupts
	IntExtOnOffConfig(DISABLE);
        
        

	write_LCD_line2(WaveStr);
	LCD_CursorOff();

	IntExtOnOffConfig(ENABLE);
  
}

*/

/*******************************************************************************
* Function Name  : Set_SELStatus
* Description    : Sets the SELStatus variable.
* Input          : None
* Output         : SELStatus
* Return         : None
*******************************************************************************/
void Set_SELStatus(void)
{
  SELStatus = 1;
}




/*******************************************************************************
* Function Name  : TIME DISPLAY
* Description    : show/modify the RTC time.
* Input          : None
* Output         : NOPE
* Return         : None
*******************************************************************************/


void TimeDisplay (bACTION event)
{
  
  static uint8_t DigitPosition = 0; // position of digit:0= nothing,1 = hour,2 = minute
 // static FlagStatus ChangeTime = RESET; // set when clock values are being altered by user
 	//uint8_t timestring[10]	;
  static FlagStatus Blink = SET; // init to set = instant blink when next pushed
   rtc_time rtcTime;
        
        
  switch (event)
  {

  case INIT:    // Called once when menu page first activated
        DigitPosition = 0;
        //ChangeTime = RESET;
        TimeDisplay(UPDATE);  // functions calls itself with update event to initialise
       break;
  case UPDATE:  // Called periodically when menu page displayed

    LCD_Addr(0x8B);
      if(CheckRecordTime()) write_LCD("*"); // within time range
      else write_LCD(" "); // not within time range
    
    
    {
      RTC_GetTime ( &rtcTime );
      LCD_Addr(0xAC);
      if((Blink==SET) && (DigitPosition == 1)) write_LCD("  ");
         else LCD_out_hex((char)Rtc_bin2bcd(rtcTime.hour));
         write_LCD(":");
      if((Blink==SET) && (DigitPosition == 2)) write_LCD("  ");
        else LCD_out_hex((char)Rtc_bin2bcd(rtcTime.min));
         write_LCD(":");
         LCD_out_hex((char)Rtc_bin2bcd(rtcTime.sec));


         Blink ^= 1;    // toggle  the blink status  
    }
       
        break;

        
  case _NEXT:        //move to next digit
         Blink = SET; // make sure to blink value instantly
         UpdateTimer += UPDATEDELAY;  // instant update upon return 
         
          if(++DigitPosition > 2) //cycle through digits
          { 
            DigitPosition = 0;
            //ChangeTime = RESET;
            UpdateTimer += UPDATEDELAY; 
          }
           // else ChangeTime = SET;
   
            break;
            
  case _PLUS:        //increment a value
        
          if(DigitPosition) //if modifying hours or minutes
          {
            RTC_GetTime ( &rtcTime ); //load time structure
           
            rtcTime.sec = 0; //reset seconds
            if(DigitPosition == 1) 
            {
              rtcTime.hour += 1;//adjust hours
              HR_CHK(rtcTime.hour);
            }
            else  if(DigitPosition == 2)
            {
              rtcTime.min += 1;  //adjust hours
              MIN_SEC_CHK(rtcTime.min)
            }
            
      
              Rtc_SetTime(rtcTime);  // write new time
              Blink = RESET;           //don't blink when button pushed
              UpdateTimer = GetTicks(); // reset reference 
              TimeDisplay(UPDATE);  // function calls itself with UPDATE
          }
    
             break;
    
  case _MINUS:        //deccrement a value
        
          if(DigitPosition) //if modifying hours or minutes
          {
            RTC_GetTime ( &rtcTime ); //load time structure
           
            rtcTime.sec = 0; //reset seconds
            if(DigitPosition == 1) 
            {
              rtcTime.hour -= 1;//adjust hours
              HR_CHK(rtcTime.hour);
            }
              
            else  if(DigitPosition == 2)
            {
              rtcTime.min -= 1;  //adjust hours
              MIN_SEC_CHK(rtcTime.min)
            }
      
              Rtc_SetTime(rtcTime);  // write new time
              Blink = RESET;           //don't blink when button pushed
              UpdateTimer = GetTicks(); // reset reference 
              TimeDisplay(UPDATE);  // function calls itself with UPDATE
          }    
        break;

        
  }
  
}

/*******************************************************************************
* Function Name  : DATE DISPLAY
* Description    : show/modify the RTC date.
* Input          : None
* Output         : NOPE
* Return         : None
*******************************************************************************/


void DateDisplay (bACTION event)
{
  static uint8_t DigitPosition = 0; // position of digit:0= nothing,1 = hour,2 = minute
 // static FlagStatus ChangeDate = RESET; // set when clock values are being altered by user
 	//uint8_t timestring[10]	;
  static FlagStatus Blink = SET; // init to set = instant blink when next pushed
   rtc_date rtcDate;
        
        
  switch (event)
  {

  case INIT:    // Called once when menu page first activated
        DigitPosition = 0;
        //ChangeDate = RESET;
        DateDisplay(UPDATE);  // functions calls itself with update event to initialise
       break;
  case UPDATE:  // Called periodically when menu page displayed

      RTC_GetDate ( &rtcDate );
      
      //display if time within recording range
      

      
      
      LCD_Addr(0xAB);
      if((Blink==SET) && (DigitPosition == 1)) write_LCD("  ");
         else LCD_out_hex((char)Rtc_bin2bcd(rtcDate.date));
         write_LCD("/");
      if((Blink==SET) && (DigitPosition == 2)) write_LCD("  ");
        else LCD_out_hex((char)Rtc_bin2bcd(rtcDate.month));
         write_LCD("/");
      if((Blink==SET) && (DigitPosition == 3)) write_LCD("    ");
        
        else 
        {
          write_LCD("20");
          LCD_out_hex((char)Rtc_bin2bcd(rtcDate.year));
        }
          Blink ^= 1;    // toggle  the blink status         
        break;
        
  case _NEXT:        //move to next digit
         Blink = SET; // make sure to blink value instantly
         UpdateTimer += UPDATEDELAY;  // instant update upon return 
         
          if(++DigitPosition > 3) //cycle through digits
          { 
            DigitPosition = 0;
            //ChangeDate = RESET;
            UpdateTimer += UPDATEDELAY; 
          }
           //else ChangeDate = SET;
   
            break;
  case _PLUS:        //increment a value
        
          if(DigitPosition) //if modifying date
          {
            RTC_GetDate ( &rtcDate ); //load date structure           
            switch (DigitPosition)
            {
            case 1:
              rtcDate.date += 1;//adjust days
              DATE_CHK(rtcDate.date, rtcDate.month);
             break;
            case 2:
              rtcDate.month += 1;  //adjust month
              MONTH_CHK(rtcDate.month)
              break;
            case 3:
              rtcDate.year += 1;  //adjust year
              YEAR_CHK(rtcDate.year)
              break;
            }      
              Rtc_SetDate(rtcDate);  // write new date
              Blink = RESET;           //don't blink when button pushed
              UpdateTimer = GetTicks(); // reset reference 
              DateDisplay(UPDATE);  // function calls itself with UPDATE
           }    
           break;
    
  case _MINUS:        //decrement a value
        
          if(DigitPosition) //if modifying date
          {
            RTC_GetDate ( &rtcDate ); //load time structure        
            switch (DigitPosition)
            {
            case 1:
              rtcDate.date -= 1;//adjust days
              DATE_CHK(rtcDate.date, rtcDate.month);
            break;
            case 2:
              rtcDate.month -= 1;  //adjust month
              MONTH_CHK(rtcDate.month)
            break;
            case 3:
              rtcDate.year -= 1;  //adjust year
              YEAR_CHK(rtcDate.year)
              break;
            }            
              Rtc_SetDate(rtcDate);  // write new date
              Blink = RESET;           //don't blink when button pushed
              UpdateTimer = GetTicks(); // reset reference 
              DateDisplay(UPDATE);  // function calls itself with UPDATE
          }    
        break;            
  }
}

/*******************************************************************************
* Function Name  : STARTTIME DISPLAY
* Description    : show/modify the RTC time.
* Input          : None
* Output         : NOPE
* Return         : None
*******************************************************************************/


void StartTimeDisplay (bACTION event)
{
  static uint8_t DigitPosition = 0; // position of digit:0= nothing,1 = hour,2 = minute
  static FlagStatus Blink = SET; // init to set = instant blink when next pushed
   static int8_t StartMin;
   static int8_t StartHour;
   static uint8_t RecNum;   
   rtc_time rtcTime;
        
  switch (event)
  {

  case INIT:    // Called once when menu page first activated
        DigitPosition = 0;
        RecNum = 0;
        StartTimeDisplay(UPDATE);  // functions calls itself with update event to initialise
       break;
  case UPDATE:  // Called periodically when menu page displayed

      EEPROM_ReadStartTime(RecNum, (uint8_t*)&StartHour,(uint8_t*)&StartMin);
      LCD_Addr(0xAD);
      if((Blink==SET) && (DigitPosition == 1)) write_LCD("  ");
         else LCD_out_hex((char)Rtc_bin2bcd(StartHour));
         write_LCD(":");
      if((Blink==SET) && (DigitPosition == 2)) write_LCD("  ");
        else LCD_out_hex((char)Rtc_bin2bcd(StartMin));
        //write_LCD(":");
     // LCD_out_hex(0x00);
         Blink ^= 1;    // toggle  the blink status         
        break;  

  case _NEXT:        //move to next digit
         Blink = SET; // make sure to blink value instantly
         UpdateTimer += UPDATEDELAY;  // instant update upon return 
          if(++DigitPosition > 2) //cycle through digits
          { 
            DigitPosition = 0;
            UpdateTimer += UPDATEDELAY; 
          }      
          break;
  case _PLUS:        //increment a value
        
          if(DigitPosition) //if modifying date
          {
            EEPROM_ReadStartTime(RecNum, (uint8_t*)&StartHour,(uint8_t*)&StartMin);         
            switch (DigitPosition)
            {
            case 1:
              StartHour += 1;//adjust days
              HR_CHK(StartHour);
             break;
            case 2:
              StartMin += 5;  //adjust month
              MIN_15_CHK(StartMin);
              break;
            }      
              EEPROM_WriteStartTime(RecNum, StartHour, StartMin);
              rtcTime.sec = 0;
              rtcTime.min = Rtc_bin2bcd (StartMin) | 0x80;
              rtcTime.hour = Rtc_bin2bcd (StartHour) | 0x80;
              Rtc_ClearAlarmFlag(); //clear RTC alarm
              Rtc_SetAlarm (rtcTime,0x00,0x00);
              Blink = RESET;           //don't blink when button pushed
              UpdateTimer = GetTicks(); // reset reference 
              StartTimeDisplay(UPDATE);  // function calls itself with UPDATE
           }    
           break;    
  case _MINUS:        //increment a value
        
          if(DigitPosition) //if modifying date
          {
            EEPROM_ReadStartTime(RecNum, (uint8_t*)&StartHour,(uint8_t*)&StartMin);         
            switch (DigitPosition)
            {
            case 1:
              StartHour -= 1;//adjust days
              HR_CHK(StartHour);
             break;
            case 2:
              StartMin -= 5;  //adjust month
              MIN_15_CHK(StartMin);
              break;
            }      
              EEPROM_WriteStartTime(RecNum, StartHour, StartMin);
              rtcTime.sec = 0;
              rtcTime.min = Rtc_bin2bcd (StartMin) | 0x80;
              rtcTime.hour = Rtc_bin2bcd (StartHour) | 0x80;
              Rtc_ClearAlarmFlag(); //clear RTC alarm
              Rtc_SetAlarm (rtcTime,0x00,0x00);
              Blink = RESET;           //don't blink when button pushed
              UpdateTimer = GetTicks(); // reset reference 
              StartTimeDisplay(UPDATE);  // function calls itself with UPDATE
           }    
           break;            
          
         
  }
}

/*******************************************************************************
* Function Name  : STOPTIME DISPLAY
* Description    : show/modify the RTC time.
* Input          : None
* Output         : NOPE
* Return         : None
*******************************************************************************/


void DurationDisplay (bACTION event)
{
  static uint8_t DigitPosition = 0; // position of digit:0= nothing,1 = hour,2 = minute
  static FlagStatus Blink = SET; // init to set = instant blink when next pushed
   static int8_t StopMin;
   static int8_t StopHour;
   static uint8_t RecNum;     
        
  switch (event)
  {

  case INIT:    // Called once when menu page first activated
        DigitPosition = 0;
        RecNum = 0;
        DurationDisplay(UPDATE);  // functions calls itself with update event to initialise
       break;
  case UPDATE:  // Called periodically when menu page displayed

      EEPROM_ReadStopTime(RecNum, (uint8_t*)&StopHour,(uint8_t*)&StopMin);
      LCD_Addr(0xAD);
      if((Blink==SET) && (DigitPosition == 1)) write_LCD("  ");
         else LCD_out_hex((char)Rtc_bin2bcd(StopHour));
         write_LCD(":");
      if((Blink==SET) && (DigitPosition == 2)) write_LCD("  ");
        else LCD_out_hex((char)Rtc_bin2bcd(StopMin));
        //write_LCD(":");
     // LCD_out_hex(0x00);
         Blink ^= 1;    // toggle  the blink status         
        break;  

  case _NEXT:        //move to next digit
         Blink = SET; // make sure to blink value instantly
         UpdateTimer += UPDATEDELAY;  // instant update upon return 
          if(++DigitPosition > 2) //cycle through digits
          { 
            DigitPosition = 0;
            UpdateTimer += UPDATEDELAY; 
          }      
          break;
  case _PLUS:        //increment a value
        
          if(DigitPosition) //if modifying date
          {
            EEPROM_ReadStopTime(RecNum, (uint8_t*)&StopHour,(uint8_t*)&StopMin);         
            switch (DigitPosition)
            {
            case 1:
              StopHour += 1;//adjust days
              HR_CHK(StopHour);
             break;
            case 2:
              StopMin += 5;  //adjust month
              MIN_15_CHK(StopMin);
              break;
            }      
              EEPROM_WriteStopTime(RecNum, StopHour, StopMin);
              Blink = RESET;           //don't blink when button pushed
              UpdateTimer = GetTicks(); // reset reference 
              DurationDisplay(UPDATE);  // function calls itself with UPDATE
           }    
           break;    
  case _MINUS:        //increment a value
        
          if(DigitPosition) //if modifying date
          {
            EEPROM_ReadStopTime(RecNum, (uint8_t*)&StopHour,(uint8_t*)&StopMin);         
            switch (DigitPosition)
            {
            case 1:
              StopHour -= 1;//adjust days
              HR_CHK(StopHour);
             break;
            case 2:
              StopMin -= 5;  //adjust month
              MIN_15_CHK(StopMin);
              break;
            }      
              EEPROM_WriteStopTime(RecNum, StopHour, StopMin);
              Blink = RESET;           //don't blink when button pushed
              UpdateTimer = GetTicks(); // reset reference 
              DurationDisplay(UPDATE);  // function calls itself with UPDATE
           }    
           break;            
  }
}

/*******************************************************************************
* Function Name  : Record
* Description    : Starts/stops record.
* Input          : None
* Output         : NOPE
* Return         : None
*******************************************************************************/
void Record(bACTION event)
{     

}



/*******************************************************************************
* Function Name  : get sample from analog board
* Description    : Starts/stops record.
* Input          : None
* Output         : NOPE
* Return         : None
*******************************************************************************/
void Test(bACTION event)
{  
#define MAXOPTION 4 
GPIO_InitTypeDef GPIO_InitStructure; 
  uint32_t i;
  uint16_t BatteryVal;
  static uint8_t  Option;
  int ret;
   uint8_t  *OptionItem[] = {"   TEST  CARD   ",
                             "  FORMAT  CARD  ",
                             "  TEST BATTERY  ",
                             "   BOOTLOADER   "};
    
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
           clear_LCD();
           switch (Option)
           {
           case 0: //test card
                  //if no SD CARD in SLOT then exit card check function 
                 if (GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_11))
                   write_LCD_line1("  No SD Card");
                 else  // test card with check
                 {
                   //SD pwr on
	            GPIO_WriteBit(GPIOE,GPIO_Pin_2, Bit_RESET);
                    delay_ms(2);
                    f_init();
        
                    ret = CheckCard();// check status of card
                    if(ret==1) write_LCD_line1(" Not Formatted");
                    else if(ret==2) write_LCD_line1("  Card Error");
                    else if(ret==3) write_LCD_line1("  Card Full");
                    else  if(!ret) write_LCD_line1("  Card OK");

                    //sd power off
                    GPIO_WriteBit(GPIOE,GPIO_Pin_2, Bit_SET);
                 }
                  delay_ms(2000);  //delay 2 secs  
             
                break;
                
                
           case 1: //FORMAT CARD
                  //if no SD CARD in SLOT then exit formating function 
                 if (GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_11))
                   write_LCD_line1("  No SD Card");
                 else  // format card
                 {
                   //SD pwr on
	            GPIO_WriteBit(GPIOE,GPIO_Pin_2, Bit_RESET);
                    delay_ms(2);
                    f_init();
                    SetResetIWDGflag(1);  //allow watchdog reset in systick int
                    write_LCD_line1(" Formatting...");
                    ret = FormatCard();// check status of card
                    if(ret==2) write_LCD_line1("  Card Error");
                    else  if(!ret) write_LCD_line1(" Card Formatted");
                    SetResetIWDGflag(0);  //disable watchdog reset in systick int
                    //sd power off
                    GPIO_WriteBit(GPIOE,GPIO_Pin_2, Bit_SET);
                 }
                  delay_ms(2000);  //delay 2 secs  
             
                break;
                
           case 2: //test battery
               
               write_LCD_line1("Battery Test:");
               LCD_Addr(0xAC);
               CheckBattery(); //ignore first read
                if (CheckBattery() < 0xB80)
                  //if low
                     write_LCD_line2("      LOW");
                 else write_LCD_line2("      GOOD");
                
                
               
               
               delay_ms(2000);  //delay 2 secs                 break;   
               break;
           case 3:   //BOOTLOADER
                 //check if battery voltage high enough to reprogram
                   //Battery on
                     GPIO_WriteBit(GPIOC,GPIO_Pin_5, Bit_SET);                      
                      EnableBatteryADC();
                      delay_us(1000);
                      GetBatteryADC();
                      if(GetBatteryADC() < 0xB00)  //voltage must be higher than this to reprogram
                      {
                        clear_LCD();
                        write_LCD_line1("Battery Voltage");
                        write_LCD_line2("   Too Low");
                        delay_ms(2000);  //delay 2 secs 
                        DisableBatteryADC();
                        break;
                      }
                      DisableBatteryADC();
                   //Battery off
                     GPIO_WriteBit(GPIOC,GPIO_Pin_5, Bit_RESET);                           
             
                 write_LCD_line1("REPROGRAMMING");
                 if (f_bootloader()) 
                 //enable int 
                  {  
                      clear_LCD();
                      write_LCD_line1("  No Firmware");  
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
   
  
}


/*******************************************************************************
* Function Name  : MAIN POWER OFF
* Description    : Turns OFF Board.
* Input          : None
* Output         : NOPE
* Return         : None
*******************************************************************************/
void MainPowerOff(bACTION event)
{  

   static uint8_t bxor =0;
  switch (event)
  {  
    
  case INIT:   
  case UPDATE:            //check status of power on flags and display

    
       LCD_Addr(0x8C);
      //DISPLAY TIMOUT VALUE
       LCD_out_hex(Rtc_bin2bcd(*((uint8_t*)&Timeout+1)));
       LCD_out_hex(Rtc_bin2bcd(*(uint8_t*)&Timeout));        
    
    
    LCD_Addr(0xA9);
          //check MCU flag and display
          if (GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_8))
                     write_LCD("MCU");
               else  write_LCD("   ");
               
           LCD_Addr(0xAD);
          //check MCU flag and display
          if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9))
                     write_LCD("USB");
               else  write_LCD("   ");               
               
          LCD_Addr(0xB1);     
          if (Rtc_GetAlarmFlag())
                      write_LCD("RTC");
               else  write_LCD("   ");              
               
               
               
           break;    
                        
    
//QUICK TEST OF POWER CONSUMPTION
    case _NEXT:  

         //Keep pwr on
      if(bxor) Timeout=0;//GPIO_WriteBit(GPIOB,GPIO_Pin_8, Bit_SET);
      
      
      else Timeout=0; //GPIO_WriteBit(GPIOB,GPIO_Pin_8, Bit_RESET);
        
          //will not run after this point unless usb or rtc flags set
          
      bxor ^=1;
      break;
  }
  
  
  
  
  
}




/*******************************************************************************
* Function Name  : MAIN POWER OFF
* Description    : Turns OFF Board.
* Input          : None
* Output         : NOPE
* Return         : None
*******************************************************************************/
void SurveyDisplay(bACTION event)
{  
  static uint8_t DigitPosition = 0; // position of digit:0= nothing,1 = hour,2 = minute
 // static FlagStatus ChangeTime = RESET; // set when clock values are being altered by user
 	//uint8_t timestring[10]	;
  static FlagStatus Blink = SET; // init to set = instant blink when next pushed
    static uint8_t SurveyName[17];
           uint8_t i; 
           uint8_t chr;
    static uint8_t delmode=0;
    static uint8_t modified=0; //if name change
   
  switch (event)
  {

    case INIT:
              //get survey string
               DigitPosition =0;
               modified =0;
               delmode =0;
               for(i=0;i<16;i++) SurveyName[i] =0;
               
               EEPROM_ReadSurveyName(0, SurveyName);
               
               write_LCD_line2(SurveyName);
               
               for(i=0;i<16;i++)
               {
                 ALPHA_CHK(SurveyName[i]); 
                 
               }
               SurveyName[16] =0;
               
              // EEPROM_WriteSurveyName(0, SurveyName);                    
        break;
                
  case UPDATE:
    
              {
                 //EEPROM_ReadSurveyName(0, SurveyName);
                LCD_Addr(0xA8);
                for(i=0;i<16;i++)
                {
                 if((Blink==SET) && ((DigitPosition-1) == i))
                 {
                  if ((SurveyName[DigitPosition-1] < 0x21) && ((DigitPosition-1) == i)) 
                  {
                    if(delmode) write_LCD_char(0xff); else write_LCD("_");
                    //if (SurveyName[i] == 0x00) { write_LCD(" "); break;}
                  } else  if(delmode) write_LCD_char(0xff); else write_LCD(" ");
                 }                   
                  else  
                  {
                    if (SurveyName[i] == 0x00) 
                    {
                      write_LCD(" ");
                      //break;
                    }else  write_LCD_char(SurveyName[i]);
                  }                 
                }
              }
               Blink ^= 1;    // toggle  the blink status
                            
                 
          break;
          
    case _NEXT:      
         Blink = SET; // make sure to blink value instantly
         UpdateTimer += UPDATEDELAY;  // instant update upon return 
         
         for(i=0;i<16;i++) 
         {
           if (SurveyName[i] == 0x00)  break;            
         }
                 
         i++;
         DigitPosition++;
         
         
         
         //if((SurveyName[DigitPosition-1] != 0) && (SurveyName[DigitPosition] == 0))i++;
          if(DigitPosition > i) //cycle through digits
          { 
            DigitPosition = 0;
            delmode =0;
            
            for (i=16;i>0;i--) if(SurveyName[i-1] < 0x21) SurveyName[i-1] =0; else break; 

            
            
            //ChangeTime = RESET;
            UpdateTimer += UPDATEDELAY; 
            Blink = RESET;
            //save the survey name 
           EEPROM_WriteSurveyName(0, SurveyName);
           modified =0;
          }
          else if(delmode)

                 SurveyName[DigitPosition-2] = 0x20;
                            

            
            
           // else ChangeTime = SET;
   
            break;      
      
   case _MINUS:        //increment a value
        
          if(DigitPosition) //if modifying date
          {
            //EEPROM_ReadStopTime(RecNum, (uint8_t*)&StopHour,(uint8_t*)&StopMin);         
            modified = 1;
            chr = SurveyName[DigitPosition-1];
            if ((chr==0) || (chr==' ')) chr = 'A';
            //if (chr<'A') chr = 'A';
            else chr++;
            delmode = 0;
            if (chr > 'Z') if(SurveyName[DigitPosition] >= 0x20) chr = 0x20; else chr = 0;
            SurveyName[DigitPosition-1] = chr;
            if(chr <0x21) 
             delmode =1;  //set to delete mode           
            
            if(delmode) Blink=SET; else Blink = RESET;           //don't blink when button pushed
              UpdateTimer = GetTicks(); // reset reference 
              SurveyDisplay(UPDATE);  // function calls itself with UPDATE
           }    
           break;         
      
   case _PLUS:        //increment a value
        
          if(DigitPosition) //if modifying date
          {
            //EEPROM_ReadStopTime(RecNum, (uint8_t*)&StopHour,(uint8_t*)&StopMin);         
            modified = 1;
            chr = SurveyName[DigitPosition-1];
            if ((chr==0) || (chr==' ')) chr = 'Z';
            else chr--;
            delmode = 0;
            
            if (chr < 'A') if(SurveyName[DigitPosition] >= 0x20) chr = 0x20; else chr = 0;


            SurveyName[DigitPosition-1] = chr;
            if(chr <0x21) delmode =1;  //set to delete mode
            
                          
            if(delmode) Blink=SET; else Blink = RESET;            //don't blink when button pushed
              UpdateTimer = GetTicks(); // reset reference 
              SurveyDisplay(UPDATE);  // function calls itself with UPDATE
           }    
           break;            
         
  case CLOSE: //called when PAGE closes allows clean up and saving 
        if(modified)
        {
           EEPROM_WriteSurveyName(0, SurveyName);
           modified =0;    
        }
    

       break;    
  }
  
  
  
  
  
  
}


/*******************************************************************************
* BATTERY VOLTAGE DISPLAY
*
*
*******************************************************************************/

void Battery(bACTION event)
{
  uint16_t BatteryVal;
  
  
  switch (event)
   {
  
   case UPDATE:
               LCD_Addr(0xAC);
               BatteryVal = CheckBattery();
               LCD_out_hex(*((uint8_t*)&BatteryVal+1));
               LCD_out_hex(*(uint8_t*)&BatteryVal);   
  
            break;
  
  
  
   } 
  
}

/*******************************************************************************
* BOOTLOADER CODE
*
* runs if boot file detected on card 
*******************************************************************************/

void Boot(bACTION event)
{
  uint8_t BootFile=0;
  
   switch (event)
  {

  case INIT:

  case UPDATE:
        //LCD_Addr(0xAB);
    write_LCD_line2("Press "); write_LCD_char(0x7e); write_LCD(" to start");
           
        break;
  case _NEXT:  
           // perform function for selected option
           clear_LCD();
             
               
              //BOOTLOADER
                 //check if battery voltage high enough to reprogram
                   //Battery on
                     GPIO_WriteBit(GPIOC,GPIO_Pin_5, Bit_SET);                      
                      EnableBatteryADC();
                      delay_us(1000);
                      GetBatteryADC();
                      if(GetBatteryADC() < 0xB20)  //voltage must be higher than this to reprogram
                      {
                        clear_LCD();
                        write_LCD_line1("Battery Voltage");
                        write_LCD_line2("   Too Low");
                        delay_ms(2000);  //delay 2 secs 
                        DisableBatteryADC();
                        break;
                      }
                      DisableBatteryADC();
                   //Battery off
                   //turn on SD card;
                   //GPIO_WriteBit(GPIOE,GPIO_Pin_2, Bit_RESET);
                   //delay_ms(2);
                   //f_init();
                   /* first get binary file from sd card and load into external sram*/
                  BootFile = CheckBootFile();                  
                 write_LCD_line1("REPROGRAMMING");
                 
                if(BootFile ==1)  //For ARM
                { 
                 if (f_bootloader()) 
                 //enable int 
                  {  
                      clear_LCD();
                      write_LCD_line1("  No Firmware");  
                  }
                delay_ms(2000);  //delay 2 secs                
                }
                else 
                  if(BootFile ==2) //For DSP
                  {
                    //BootDSP();
                    
                  }
                

           clear_LCD();
           GPIO_WriteBit(GPIOE,GPIO_Pin_2, Bit_SET);
           write_LCD_line1 ( CurActiveMenu.PageName ); 
           Test(UPDATE);  
        break;
        
     
  } 
  
  
  
  
  
  
  
}

/*******************************************************************************
* TEST CODE
*
* run code to test 
*******************************************************************************/

void CodeTest(bACTION event)
{
   
  int iri;
  
  
  switch (event)
   {
     
  
   case _NEXT:          
    
         
                _f_createdir();
                
                delay_ms(2000);
            
                //UpdateActiveBuffer(0x90000);
     
       
     
          break;
   } 
  
}








/*******************************************************************************
* SYNC MENU
*
* used to enable syncing for recording 
*******************************************************************************/

void Syncing(bACTION event)
{

  static int8_t  Option;
   uint8_t  *OptionItem[] = {"  Sync Off",
                             "  Sync On "};  

  switch (event)
   {
   case INIT:
             Option = EEPROM_ReadSyncFlag(0);
             if(Option & 0xfe)
                {
                  Option = 0;
                  EEPROM_WriteSyncFlag(0,Option);
                } 
             
               write_LCD_line2(OptionItem[Option]);
             
              break;     
   case UPDATE:

            break;
            
            
   case _PLUS:
     
         Option--;
         Option &= 0x01;
         EEPROM_WriteSyncFlag(0,Option);
         write_LCD_line2(OptionItem[Option]);
         break;
         
   case _MINUS:
     
         Option++;
         Option &= 0x01;
         EEPROM_WriteSyncFlag(0,Option);
         write_LCD_line2(OptionItem[Option]);
         break;
   }
  
}



/*******************************************************************************
* SAMPLING DISPLAY
*
*
*******************************************************************************/

void Sampling(bACTION event)
{
  
  static uint8_t  Option;
   uint8_t  *OptionItem[] = {"",
                             "     LOW ",
                             "     HIGH"};  
  
   switch (event)
   {
  
   case INIT:
             Option = EEPROM_ReadHiLoSamplingFlag(0);
             if((Option-1)>1)
                {
                  Option = 1;
                  EEPROM_WriteHiLoSamplingFlag(0,Option);
                } 
             
               write_LCD_line2(OptionItem[Option]);
             
              break;     
   case UPDATE:

            break;
            
            
   case _PLUS:
     
         Option--;
         if(!Option) Option = 2;
         EEPROM_WriteHiLoSamplingFlag(0,Option);
         write_LCD_line2(OptionItem[Option]);
         break;
         
   case _MINUS:
     
         Option++;
         if(Option >2) Option = 1;
         EEPROM_WriteHiLoSamplingFlag(0,Option);
         write_LCD_line2(OptionItem[Option]);
         break;
  
   }  
  
  
  
  
  
  
  
  
}




/******************************************************************************
*
*
******************************************************************************/

void DMA_Start(bACTION event)
{
  DMA_InitTypeDef  DMA_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
   uint8_t HiLoSampling;  // sampling frequency select low =1, High = 2 
   uint8_t ClkWait=0;
  static uint16_t  TestBuf[128];
  
    switch (event)
   {
  
   case INIT: 
  
  
          break;
  
  
          
          
   case _NEXT:       
      /* Enable the GPIO_LED Clock*/
       RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOA, ENABLE);  
       RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
      
         //determine to sample at hi or low freq
      HiLoSampling = EEPROM_ReadHiLoSamplingFlag(0);       
       
      /* Configure the SPI CS on pin */
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_Init(GPIOA, &GPIO_InitStructure);   
      
    //set CS to High
      GPIO_WriteBit(GPIOA, GPIO_Pin_7, Bit_SET);
      
      
        /* Setup up a interupt for checking if out of sync*/
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);         
        EXTI_ClearITPendingBit(EXTI_Line9);     
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Line = EXTI_Line9;         
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        EXTI_Init(&EXTI_InitStructure);      
        /* Connect Button EXTI Line to Button GPIO Pin */
        GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource9);       
      
      
    //wait until spi line high          
      /* Configure SPI_MASTER pins: MISO, SCK  */
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
      GPIO_Init(GPIOA, &GPIO_InitStructure);         
      
      
    //wait for Interrupt if sync enabled  
     if(EEPROM_ReadSyncFlag(0))
     {
        /* Setup up a interupt for sync*/
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);         
        /* Clear the the EXTI line 0,1,2 and 3 interrupt pending bit */
        EXTI_ClearITPendingBit(EXTI_Line8);     
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Line = EXTI_Line8;         
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        EXTI_Init(&EXTI_InitStructure);      
        /* Connect Button EXTI Line to Button GPIO Pin */
        GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource8);      
     
      
        
        
        while(!GetSyncFlag())
          {
          
                IWDG->KR =  ((uint16_t)0xAAAA); //KR_KEY_Reload;  //WatchDog reload
          }
     }
      
      

      uint8_t i;                      //at a spi clk of 288khz, 5 loops will miss clk, <10 won't
              GPIOC->BSRR = GPIO_Pin_7; 

              
        
      if (HiLoSampling ==1) ClkWait =25; else  ClkWait =10;       
      
      while(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5));  //wait for low signal                
      for(i=0;i<ClkWait;i++)
      {
        
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5)==0) i=0;  
        
      }
               GPIOC->BRR = GPIO_Pin_7; 
      
      
      
      
      /* Configure SPI_MASTER pins: MOSI, SCK  */
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 |GPIO_Pin_7;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
      GPIO_Init(GPIOA, &GPIO_InitStructure);    

      
      /* SPI_SLAVE_Rx_DMA_Channel configuration ---------------------------------------------*/
     /*
          DMA_DeInit(DMA1_Channel2);  //SPI_SLAVE_Rx_DMA_Channel
          DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)0x4001300c;
          DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)GetBufPtr(); //start of ext sram
          DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
          DMA_InitStructure.DMA_BufferSize = 256;
          DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
          DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
          DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
          DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
          DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
          DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
          DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
          DMA_Init(DMA1_Channel2, &DMA_InitStructure);      
     */
          

      
      
      
      
  /* SPI_SLAVE configuration ------------------------------------------------------*/
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);  


	/* Enable the  gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
       //  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure); 
        
        
        
        
   //Enable the SPI interrupt
  SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
  /* Enable SPI_SLAVE Rx request */
  //SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
  /* Enable SPI_SLAVE */
  SPI_Cmd(SPI1, ENABLE);
  
  SPI1->DR = 0xA569;

  //Enable the DMA interrupt
  //DMA_ITConfig(DMA1_Channel2, DMA_IT_TC |DMA_IT_HT, ENABLE);
 
  
  /* Enable DMA1 Channel2 */
  //DMA_Cmd(DMA1_Channel2, ENABLE);
  
  SPI_NSSInternalSoftwareConfig(SPI1,SPI_NSSInternalSoft_Reset);

     
   //turn off power to LCD
         GPIO_WriteBit(GPIOG,GPIO_Pin_11,Bit_RESET);   
         
   //make sure battery checking disabled
         DisableBatteryADC();

              //turn off USB
          //USB->CNTR |= 0x0001;
          SetCNTR(GetCNTR() | (uint16_t)0x0002);
	/* Disable the USB_LP_CAN_RX0 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);
	/* Disable the USB_HP_CAN_TX Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);                          
          RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, DISABLE);
         //turn off lED
         GPIO_WriteBit(GPIOC,GPIO_Pin_7,Bit_RESET);       
         
     //sets filename based on date and time recording started
      SetFileName(); 
      //ExitRecordFlag = RESET;      
      
      switch(HiLoSampling) //select frequency type
     {
     case 1:      
            RecInit(8000); // initialise variables used in recording
            SetSampleCounter(100000000); //set the interval         
         
  
            //change PCLK
            RCC_PCLK1Config(RCC_HCLK_Div1);
            //set 8Mhz Clk Rate
            RCC_HCLKConfig(RCC_SYSCLK_Div4);   
            SetClkHSE();       // set sysclk to 2MHz     , HCLK = 4MHz  , PCLK1 = 4MHz , PCLK2 = 4MHz 
             break;
     case 2:           
             RecInit(32000); // initialise variables used in recording
            SetSampleCounter(100000000); //set the interval  
            //change PCLK
            RCC_PCLK1Config(RCC_HCLK_Div1);
            //set 8Mhz Clk Rate
            RCC_HCLKConfig(RCC_SYSCLK_Div1);     
            SetClkHSE();       // set sysclk to 8MHz     , HCLK = 4MHz  , PCLK1 = 4MHz , PCLK2 = 4MHz                         
            break;
     }
            
             // Flash 0 wait state - no need for wait states when clock speed low 
             FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
             FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_0;  
    
      
             
             
      //disable systick int
     SysTick->CTRL = (1 << SYSTICK_CLKSOURCE) | (0<<SYSTICK_ENABLE) | (0<<SYSTICK_TICKINT);   
   
     GPIO_WriteBit(GPIOE,GPIO_Pin_2, Bit_RESET);
     // for testing 
     //PWR_EnterSTANDBYMode();
     
     
  //uint8_t inByte;
  while(1)
  {
    
    
             //if READY to write Buffer to SD card
              if (GetSDWriteFlag() == Bit_SET)
              {
                GPIOC->BSRR = GPIO_Pin_7; 
                    // Flash 2 wait state - when clock speed increase add wait states
                FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
                FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;  
                //SwitchFastClock   //Switch to 72Mhz clock rate
                RCC_PCLK1Config(RCC_HCLK_Div4);  //set t
            
                SetClkPLL();       // set sysclk to 8MHz     , HCLK = 4MHz  , PCLK1 = 4MHz , PCLK2 = 4MHz 
                  RCC_HCLKConfig(RCC_SYSCLK_Div1); //set to 72Mhz                 

                SetSDWriteFlag(Bit_RESET);
                //led on
               // GPIO_WriteBit(GPIOC, GPIO_Pin_6, Bit_SET);           
                                
        	//SD pwr on
	        //GPIO_WriteBit(GPIOE,GPIO_Pin_2, Bit_RESET);
                //SDCardOn();
                                
                
                delay_us(1000);  //so as not to use systick
                  
                f_init();
                //do it       
                SaveWAVFile();

               // check if within record time -if not then exit
                delay_us(5000);
                        	//SD pwr off
	       // GPIO_WriteBit(GPIOE,GPIO_Pin_2, Bit_SET);
               // SDCardOff();
                                   
                //GPIO_WriteBit(GPIOE,GPIO_Pin_2, Bit_SET); 
                //led off
               // GPIO_WriteBit(GPIOC, GPIO_Pin_6, Bit_RESET);
                
                //__WFI();  //WAIT FOR INTERRUPT BEFORE Switching Back Clocks
                
                  switch(HiLoSampling) //select frequency type
                   {
                    case 1:                   
                        //Switch Slow Clock
                        RCC_HCLKConfig(RCC_SYSCLK_Div4); //set to 2Mhz                 
                        SetClkHSE();
                        RCC_PCLK1Config(RCC_HCLK_Div1);  //set tim2 
                        break;
                    case 2:      
                        //Switch Slow Clock
                        RCC_HCLKConfig(RCC_SYSCLK_Div1); //set to 8Mhz                 
                        SetClkHSE();
                        RCC_PCLK1Config(RCC_HCLK_Div1);  //set tim2   
                        break;
                   }
                        
                      // Flash 0 wait state - no need for wait states when clock speed low
                    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
                     FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_0; 
                     GPIOC->BRR = GPIO_Pin_7; 
                      
               }      
    
    
    
    IWDG->KR =  ((uint16_t)0xAAAA); //KR_KEY_Reload;  //WatchDog reload    
    
    
    //__WFI();
    
  }
          break;
          
   }
  
  
  
  
}












/*******************************************EOF********************************/