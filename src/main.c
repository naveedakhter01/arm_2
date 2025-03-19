/**
  ******************************************************************************
  * @file    main.c 
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/


#include "main.h"
#include "FatSD.h"
#include "sysclk.h"
#include "menu.h"
#include "rtc.h"
#include "i2c_bitbang.h"
#include "record.h"
#include "eeprom.h"
#include "LCD.h"
#include "platform_config.h"
#include "battery.h"
#include "stm32f10x_it.h"
#include "spi.h"
#include "temp_sensor.h"
#include "gps.h"
#include "dsp_program.h"
#include "hardware.h"
#include "log.h"
#include "protocol.h"
#include "pushbutton.h"
#include "stdio.h"
#include "random.h"
#include "settings.h"
#include "../stm32f10xreg.h"


#include "nfc.h"
#include <time.h>

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup SysTick
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define SOFTWARE_MAJOR_VERSION ((uint16_t) 1)
                                        // .
#define SOFTWARE_MINOR_VERSION ((uint16_t) 6)

#define SOFTWARE_MINORMINOR_VERSION ((uint16_t) 6) //for small changes

#define HARDWARE_VERSION ((uint8_t) 'A')



/* Private function prototypes -----------------------------------------------*/
bool run_record(void);
uint8_t InitRecord(SamplingOptionType samplingOption, uint16_t* StartCalc,uint16_t* StopCalc, uint8_t PlaybackFlag);
void run_menu(void);
void menu_GPIO_config(void);
void NVIC_Configuration(void);
void SetUnusedIO(void);


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
ADC_InitTypeDef ADC_InitStructure;
SPI_InitTypeDef SPI_InitStructure; 
bool wasResetByWatchdog = FALSE;
  
static int  watchdogExtendCounter = 0;

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  bool menuRunFlag = FALSE;
  bool runRecord = TRUE;
  //TimeOffset_TypeDef offsets;
  //struct tm _tm;
  
  // these variables are modified directly by the bootloader
  volatile uint32_t *bootloader_page_pushed = BOOTLOADER_PAGE_DETECT;
  volatile uint32_t *bootloader_page_instruct = BOOTLOADER_PAGE_INSTRUCT;
  volatile uint32_t *bootloader_flash_instruct = BOOTLOADER_FLASH_INSTRUCT;

  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,
     initialize the PLL and update the SystemFrequency variable. */
  
  delay_us(1000);

  /* Clk for micro is set to 8MHz start up */
  SystemInit();

  /* IF SYSTEM CLK IS CHANGED THEN "CCLK_KHZ" WILL NEED TO BE CHANGED IN mmci.c */
  
  //SETUP THE WATCHDOG TIMER
  /* Check if the system has resumed from IWDG reset */
  if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
  {
    wasResetByWatchdog = TRUE;
    RCC_ClearFlag();
  }  
  
  NVIC_Configuration();

  PowerCntrl(ENABLE); //KEEP POWER LOCKED ON ASAP
  
  //setup
  SetUnusedIO();
  
  
  
  hcc_u32 tmp;
  
  RCC_AHBENR |= BIT10 | BIT2; /* Enable SDIO + DMA2 clock */
  
  
  /* GPIOA, GPIOB, SPI1 clocks */
  RCC_APB2ENR |=  RCC_APB2_PERIPH_GPIOC | RCC_APB2_PERIPH_GPIOD |
                         RCC_APB2_PERIPH_SPI1 | RCC_APB2_PERIPH_AFIO ;
  RCC_AHBENR |=  RCC_AHBENR_PERIPH_DMA2EN;

  tmp = GPIOx_CRL(GPIO_C_BASE);
  tmp &= ~(GPIO_MODE_OUT_OD << (PORT12+2));
  tmp |= (GPIO_SPEED_50MHz  | (GPIO_MODE_OUT_PP <<2)) << (PORT12); /* PC12 */
  GPIOx_CRH(GPIO_C_BASE) = tmp;
  
  SD_Init();
  LED_Init();
 
    
  //SD_Detect_Init();
  DSP_IO_Init();
  Dsp_RunCommandMode(ENABLE);
  PushButton_Init();
  i2c1_init();
  i2c2_init();
  TempSensorInit();
  GpsIOLowPower();
  InitBatteryMeasure();
  //GpsInit(); this enables usart as well
    
  IntExtOnOffConfig(ENABLE);  
  
  SysTickInterrupt(ENABLE);  //allows delay_ms to be used
  
#if BOOTLOADER  // if bootloader attached then get bool that the bootloader stored in end of memory
  // if theres a file to flash bootloader 
  if ( *bootloader_flash_instruct == BOOTLOADER_FLASH_CODEWORD )   //this var is set by the bootloader, bootloader checks if there's a file to flash bootloader and marks in memory
  {
    *bootloader_flash_instruct = 0;
    LCD_On();
    FlashBootloader();
    LCD_Off();
  }
  menuRunFlag |= *bootloader_page_pushed; //
  if (!GPIO_ReadInputDataBit(PAGE_BUTTON_PORT, PAGE_BUTTON_PIN)) menuRunFlag |= TRUE; 
#else
  //if button 1 low then goto menu 
  if (!GPIO_ReadInputDataBit(PAGE_BUTTON_PORT, PAGE_BUTTON_PIN)) menuRunFlag = TRUE; 
#endif  
  
  if (wasResetByWatchdog) {
    SaveLogFileToEeprom(); // hopefully we can still save the file to eeprom as it's memory is not initialised
    AddToLog(LOG_WATCHDOG_RESET, 0,0,0, NO_TIMESTAMP);   
  }
  CopyLogFileFromEeprom();
  AddToLog(LOG_UNIT_START, (uint8_t)menuRunFlag,0,0, TIMESTAMP); 
  


  ResetWatchdog(); //watchdog reload
  f_mount(&fs, "", 0);
  //_f_createdir();
  LED_Flash();  //F_Init LED indication
  

  runRecord = TRUE; // assume we've gonna record
  //RUN MENU
  //menuRunFlag=1;
  if(menuRunFlag) 
  {
    run_menu();
  }
  else
  {
    //wait set amount of time for dsp to start then wait for response, we don't need to do this if running menu as that's enough of a delay
    // if page button pushed then ignore delay and don't reord instead 
    // well also test if gps is sending data incase it acidentally turned on 
      GpsInit();
      if (!Delay_Dsp_Start()) 
        runRecord = FALSE;
      else
        TurnOffGpsIfActive();
  }
 
  WatchDogInit(LONG); //enable watch dog if recording  
  
  // if dsp responds
  if(RtcGetAlarmFlag()) 
    RtcClearAlarmFlag(); //clear RTC alarm
    

  //attempt to connect to dsp, if fails will reset dsp and try couple of times
  if (runRecord && AttemptConnectDsp())
  {

    //if battery low flag is not set
    if(!IsSeriousError())
    {
      //set 
      SetNextAlarm();  // we set the next alarm before hand incase something happens during recording 
      
      //check if within time range
      if( (CheckProtocolTime(RtcGetBufferedTime()))->isWithin )        //if in recording range then 
      {
        //tet if gps is running

        
        if (run_record()) //start low power recording
        { // if exit due to page button  pushed
          SaveLogFileToEeprom();
          delay_ms(50);
          *bootloader_page_instruct = BOOTLOADER_INSTRUCTION_CODEWORD; // tell bootloader to set the page button flag
          NVIC_SystemReset(); //reset MCU  if button push was cause of record exit
        }
      }
      
      //make sure alarm is cleared and set to next starttime
      if(RtcGetAlarmFlag()) 
        RtcClearAlarmFlag(); //clear RTC alarm
      SetNextAlarm();
    }
    else
    {
      AddToLog(LOG_PREVENT_RECORD, RtcGetUser2Reg(),0,0, NO_TIMESTAMP);
    }
   
  }
  else
  {
      MarkErrorRtc(MARK_MINOR_ERROR);  // dsp failed to communicate
      AddToLog(LOG_DSP_FAIL_COMMS, 0,0,0, NO_TIMESTAMP);
  }
  
  //decide here whether to reset and try again or turn off completely
  
  AddToLog(LOG_SHUTDOWN, 0,0,0, TIMESTAMP);
  SaveLogFileToEeprom();
  delay_us(20000);
  

  //SHUTDOWN  - DEVICE Should shutdown during this part  
  PowerCntrl(DISABLE);
  
  for (int i =0; i< MAGIC(1000) ;i++)
  {
    ResetWatchdog();
    delay_ms(10);
    if (IsPagebuttonPushed()) break;
  }

  NVIC_SystemReset();  // in case it doesn't shutdown then reset and hopefully reprocess
  
}      




/*******************************************
*RUN MENU
*runs if button pressed
*
*******************************************/
 
void run_menu(void)
{ 
  
  volatile uint32_t *bootloader_settings_update_instruct = BOOTLOADER_SETTINGS_INSTRUCT; // use to signal to main program to update settings from card
  
  //struct tm datetime;
  MenuExitCode exitCode = NO_EXIT;
  AddToLog(LOG_MENU_ENTER, 0,0,0, TIMESTAMP);
 /* Initialize Menu() */
  ResetWatchdog(); 
  RtcStart(); 
  LED_Flash(); //RTC initialise indication
  ResetWatchdog();
  LCD_On();   
  LED_Flash();  //LCD initialise indication
  SetGpsLoggedTries(GPSLOG_RESET_TRIES); // reset number of attemps to get gps location
  SetGpsFirstStart(GPS_IS_FIRST_START); // reset the first start flag so that the gps is on for longer
  
  DspVersion_typedef *dsp_version;
  uint8_t versionString[14];

  
  
  
//  uint8_t   Inbox[16];
//  uint8_t   Outbox[16];
  
  
  //IsValidLogFile();
  //TestRename();
  //GpsEnableZdaMessage();
  
  //Display Version Number

  
  
  
  ResetWatchdog();
   
  LED_Flash();//DSP initialise indication  

  clear_LCD();
  write_LCD_line1("ALATO AR5 ");
  
  // Dsiplay main program version
  sprintf((char*)versionString, "  ARM v%u.%u%u ", SOFTWARE_MAJOR_VERSION, SOFTWARE_MINOR_VERSION,SOFTWARE_MINORMINOR_VERSION);
  write_LCD_line2(versionString);  
  delay_ms(1200);

#ifdef DEBUG  
  //display bootloader version
  sprintf((char*)versionString, "  BOOT v%u.%u  ", *BOOTLOADER_MAJOR_VERSION, *BOOTLOADER_MINOR_VERSION);
  write_LCD_line2(versionString);  
  delay_ms(1100);
#else
  delay_ms(1100); //don't display boot version in release, but still need time for dsp to start
#endif
  
  //Display DSP Version
  WaitForDspEvent();
  
  dsp_version = GetDspVersion();
if (dsp_version->valid)
    sprintf((char*)versionString, "  DSP v%u.%u  " , dsp_version->majorVersion, dsp_version->minorVersion);
  else
    sprintf((char*)versionString, "  DSP vX.X  " );  //if dsp version cannot be retrieved
  write_LCD_line2(versionString);  
  delay_ms(1000);   
  
  LED_Display(LED_ON);      
       
  
//  for (int i=0;i<32;i++)
//    Inbox[i] = i+32;
   
//  uint8_t status;
  
  //status = Nfc_GetDynamicCommand(0x2006);
  //Nfc_FastTransferMode(ENABLE);
  //status = Nfc_GetDynamicCommand(0x2006);
  
 // Nfc_WriteMailbox(Inbox, 32);
//  status = Nfc_GetDynamicCommand(0x2006);
  
 // Nfc_ReadMailbox(Outbox, 32);
  
 // Nfc_WriteMemory(Inbox, 32);  
//  Nfc_ReadMemory(Outbox, 32);    
  
  
  ClearErrorMark();   
  clear_LCD();  
  
//check for bootloader file ARM/DSP
  Nfc_InitFastTransferMode();
  
#if BOOTLOADER  //if a settings update file is present then copy the data into the nfc ic
  if ( *bootloader_settings_update_instruct == BOOTLOADER_SETTINGS_CODEWORD )   
  {
    *bootloader_settings_update_instruct = 0;
    LoadSettingsFromSDCard();
  }
#endif 

  
  
  Menu_Init();      

  //Run Interrupts
  while (exitCode == NO_EXIT)
  {
     exitCode = RefreshMenu();
     ResetWatchdog();
  }  
  
  
  
  if (exitCode == POWER_FAILED)
  {
     write_LCD_line1(" Power Fail ");
     write_LCD_line2("            ");
     delay_ms(2000); 
     MarkErrorRtc(MARK_BATTERY_LOW);
  }
  
  LCD_Off(); 
  AddToLog(LOG_MENU_EXIT, 0,0,0, TIMESTAMP); 
}

/**************************************************
*RUN RECORD - main recording routine
*is run when rtc alarm is triggered or within duration period
*should use as little power as possible
**************************************************/
// returns true if button push was cause of exit
bool run_record(void)
{
  static RecordSessionInit_TypeDef *nextSessionInit;
  bool isWithinTimeRange = TRUE;
  bool forceGpsLog = FALSE; //override user gps settings
  SessionExitCode_Typedef defaultCode = {.error = NO_ERROR, .isButtonPushed = FALSE, .powerOk = TRUE};
  SessionExitCode_Typedef *exitCode = &defaultCode;
  static int8_t previousHour = -1; //used to store the hour value from rtc, set to -1 so as to always return true first equality test
  
  
  AddToLog(LOG_RECORDING_START, 0,0,0, TIMESTAMP);
  //turn off power to LCD
  LCD_Off();
  


  
  if (!IsSDCardDetected()) 
  {
    AddToLog(LOG_NO_SD_CARD, 0,0,0, NO_TIMESTAMP);
    return FALSE; //if no card
  }
  
  LED_Display(LED_ON);   //led on DEBUGGING ONLY   

  delay_ms(20);

  //loads up arrays with all the start and stop times
  FillProtocolStartStopList();  //fills the startstoplist with start stop times in minutes
  
  //f_init();
  
  InitBatteryMeasure(); //used to measure battery voltage
  
  ResetWatchdog();
     
  LED_Display( LED_ON);
  
    
  //MARK TIME TICKS
  SetBaseTimePoint();
  
  
  OpenSDCard(); 

  //saves the date string used to create folders
  SetFolderNameByDate();
  WatchdogExtendTimer(0);   //disable wd extension
  
  LED_Display(LED_OFF);
  
  ClearSessions();
  SysTickInterrupt(DISABLE);
  
  ReducePowerOnce(); //disables unused peripherals/io_pins to reduce power

  RandomizeSeed();  // randomise seed from time values
  //LogUserMessage(USERLOG_START);
    
  //first check time and init recordTimeCheck
  HasPageButtonTriggered(); // this is just to clear the flag
  
  while (exitCode->error == NO_ERROR &&
         !(exitCode->isButtonPushed |= HasPageButtonTriggered())   &&
         (exitCode->powerOk = IsPowerOk()) &&
         (exitCode->powerOk = MonitorBattery()) &&  // is battery below threshold
         isWithinTimeRange ) //we break out when we get an end session flag 
  {
    
   // Get the next "instruction" based on the time and protocol
    nextSessionInit = GetNextRecordingInit();
    forceGpsLog |= nextSessionInit->forceGpsLog;
    
    //write protocol name to log file - do only once
    //directory is root
    //WriteProtocolNameToLogFile(nextSessionInit);
    
    //DETERMINE WHAT RECORDING MODE TO RUN BASED ON INSTRUCTION
    //will first change dsp mode if protocol is new of different from last ones 
    switch (nextSessionInit->instruction) 
    {
      //Either go directly to low sampling record or change dsp mode first
      case CHANGE_LOW_SAMPLING:
        
        ChangeSamplingMode(nextSessionInit);
        ChangeDirectory(nextSessionInit->protocolName);
        WriteProtocolNameToLogFile(nextSessionInit);
          // flash led once every 10 buffer writes = 10 secs for low sampling
        SetLedDisplayNthCount(10);
        nextSessionInit->instruction = RECORD_LOW_8KHZ;
      case RECORD_LOW_8KHZ:
        exitCode = AudioRecordSessionRun(nextSessionInit); //If either the low or high recording
        break;
        
      //Either go directly to high sampling record or change dsp mode first
      case CHANGE_HIGH_SAMPLING:
        ChangeSamplingMode(nextSessionInit);
        ChangeDirectory(nextSessionInit->protocolName); //
        WriteProtocolNameToLogFile(nextSessionInit);
        SetLedDisplayNthCount(40); // once every 40 buffer writes = 10 secs for high sampling
        nextSessionInit->instruction = RECORD_HIGH_32KHZ;
      case RECORD_HIGH_32KHZ:
        exitCode = AudioRecordSessionRun(nextSessionInit); //If either the low or high recording
        break;
    
      //Either go directly to bat sampling record or change dsp mode first
      case CHANGE_BAT_SAMPLING:
        ChangeSamplingMode(nextSessionInit);
        ChangeDirectory(nextSessionInit->protocolName);
        WriteProtocolNameToLogFile(nextSessionInit);
        SetLedDisplayNthCount(1); // once every second for bat
        nextSessionInit->instruction = RECORD_BAT_176KHZ;
      case RECORD_BAT_176KHZ:
        exitCode = BatRecordSessionRun(nextSessionInit);
        break;

      //stops all recording for a set period of time
      case CHANGE_SUSPEND:
        //ChangeSamplingMode(nextSessionInit);
        nextSessionInit->instruction = SUSPEND;
      case SUSPEND:
        exitCode = SuspendSessionRun(nextSessionInit);
        break;
      //stops all recording for a random period of time
      case CHANGE_SUSPEND_RANDOM:
        //ChangeSamplingMode(nextSessionInit);
        nextSessionInit->instruction = SUSPEND_RANDOM;
      case SUSPEND_RANDOM:
        exitCode = SuspendSessionRun(nextSessionInit);
        break;
      case END_RECORDING:
        isWithinTimeRange = FALSE;
        continue;
      default:
        isWithinTimeRange = FALSE;
        continue;
    }
    
    //decide if to turn on gps logging, get a gps position or stop altogether
    //if (HasProtocolOption(nextSessionInit->protocolNumber, PROTOCOL_GPS))
    //LogGps(); //logs once the gps position
    
    //logs temp once per hour
    if (HasProtocolOption(nextSessionInit->protocolNumber, PROTOCOL_TEMP))
    {
      rtcTimeTypeDef *time;
  
      time = RtcGetBufferedTime(); // if time was recently aquired from rtc
      if (time->hour != previousHour)
      {  
        ResetWatchdog();
        LogTemp();
        previousHour = time->hour;
      }
    }
    
    
  }
  
  
  
  
  AddToLog(LOG_RECORDING_TERMINATED, (uint8_t)exitCode->isButtonPushed,0,0, TIMESTAMP);

  SysTickInterrupt(ENABLE);
  DisableRecordingSpi();   //disable dsp dma int+'s + spi +
  
  LogUserMessage(USERLOG_STOP);

  
  
  //perform a rename of all files to correct timestamp
  //if (1) BulkRenameFiles();
 // if (IsGpsSyncOptionEnabled()) BulkRenameFiles();
  
  //do something if there was an error
  if (exitCode->error != NO_ERROR)
  {
    //mark log file
    AddToLog(LOG_ERROR_CODE, exitCode->error,0,0,NO_TIMESTAMP);
    //mark in rtc's battery backed register
    if (exitCode->error ==  FAIL_CARD_FULL)
    {
      LogUserMessage(USERLOG_CARD_FULL);
      MarkErrorRtc(MARK_CARD_FULL);  //will cause recorder to shutdown apon restart
    }    
    else 
      MarkErrorRtc(MARK_MINOR_ERROR); //will allow a few retries before shutting down completely.
  }
  else //log gps if exited without error or power failure or button pushed
  {
    if (IsGpsLogOptionEnabled() || forceGpsLog)
      if (exitCode->powerOk && !exitCode->isButtonPushed) LogGpsAtOnce(IsGpsSyncOptionEnabled());
  }

  ResetWatchdog();
  
  if (IsGpsSyncOptionEnabled()) BulkRenameFiles();
  
  
  if (!exitCode->powerOk) //BATTERY FAILURE
  {   
    AddToLog(LOG_BATTERY_LOW, 0, 0, 0, NO_TIMESTAMP);
    MarkErrorRtc(MARK_BATTERY_LOW); //will cause recorder to shutdown apon restart
    LogUserMessage(USERLOG_BATTERY_LOW);
  }
  
  CloseSdCard();  
  return exitCode->isButtonPushed;
}




/*****************************************************
* Initialize IO for running menu
*
*****************************************************/

void menu_GPIO_config(void)
{

  /* Enable PWR_ON */;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOE 
                          | RCC_APB2Periph_GPIOF| RCC_APB2Periph_AFIO | RCC_APB2Periph_ADC1 , ENABLE);  
 
  //SETUP OUTPUTS
  /* Configure the PWR_ON pin */
  GPIO_InitStructure.GPIO_Pin = POWER_CNTRL_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(POWER_CNTRL_PORT, &GPIO_InitStructure);


  //SETUP INPUTS
  /* Configure SD Card detect pin as input floating */
  GPIO_InitStructure.GPIO_Pin = SD_DETECT_PIN ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(SD_DETECT_PORT, &GPIO_InitStructure); 
  

   //SETUP INPUTS + PULL UP
  /* Configure Button pin as input floating */
  GPIO_InitStructure.GPIO_Pin = PAGE_BUTTON_PIN | NEXT_BUTTON_PIN | PLUS_BUTTON_PIN | MINUS_BUTTON_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(ALL_BUTTON_PORT, &GPIO_InitStructure);  
 
  
  /* Connect Button EXTI Line to Button GPIO Pin */
  GPIO_EXTILineConfig(PAGE_BUTTON_PORT_SOURCE, PAGE_BUTTON_PIN_SOURCE);
  GPIO_EXTILineConfig(NEXT_BUTTON_PORT_SOURCE, NEXT_BUTTON_PIN_SOURCE);  
  GPIO_EXTILineConfig(PLUS_BUTTON_PORT_SOURCE, PLUS_BUTTON_PIN_SOURCE);  
  GPIO_EXTILineConfig(MINUS_BUTTON_PORT_SOURCE, MINUS_BUTTON_PIN_SOURCE);  
                
   /* Configure the GPIO_C port */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; 
  GPIO_Init(GPIOC, &GPIO_InitStructure);  
  GPIO_ResetBits(GPIOC, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);               

}



//sets all unsed io to known state
void SetUnusedIO(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  /* Configure the GPIO_C port */ 
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOC, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
}



/**
  * @brief  Configure the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{

#ifdef BOOTLOADER
	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable(((uint32_t)0x08010000), 0x00);
#else
        NVIC_SetVectorTable(((uint32_t)0x08000000), 0x00);
        
#endif
	/* Set the Vector Table base address at 0x08000000 */
	//NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x00);
               
	/* Configure the Priority Group to 2 bits */
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}




void IntExtOnOffConfig(FunctionalState NewState) //
{
  EXTI_InitTypeDef EXTI_InitStructure;

  /* Initializes the EXTI_InitStructure */
  //EXTI_StructInit(&EXTI_InitStructure);

  /* Disable the EXTI line 0,1,2 and 3 on falling edge */
  if(NewState == DISABLE)
  {
    EXTI_InitStructure.EXTI_Line = PAGE_BUTTON_EXTI_LINE|NEXT_BUTTON_EXTI_LINE|PLUS_BUTTON_EXTI_LINE|MINUS_BUTTON_EXTI_LINE;          
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);    
  }
  /* Enable the EXTI line 0,1,2 and 3 on falling edge */
  else
  {
    /* Clear the the EXTI line 0,1,2 and 3 interrupt pending bit */
    EXTI_ClearITPendingBit(PAGE_BUTTON_EXTI_LINE|NEXT_BUTTON_EXTI_LINE|PLUS_BUTTON_EXTI_LINE|MINUS_BUTTON_EXTI_LINE);     

    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Line = PAGE_BUTTON_EXTI_LINE|NEXT_BUTTON_EXTI_LINE|PLUS_BUTTON_EXTI_LINE|MINUS_BUTTON_EXTI_LINE;         
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
 
  }

}


void WatchdogExtendTimer(int ticks)
{
  static bool doneOnce = FALSE;
    TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
    TIM_OCInitTypeDef         TIM_OCInitStructure; 
    NVIC_InitTypeDef          NVIC_InitStructure;
     

    
    if (!doneOnce)
    {    
      uint16_t TimebasePrescaler = 24999; //25 
      uint16_t TIMPrescaler;
      
      TIMPrescaler = 200;  //FAST clock @ 36Mhz
      //TIMPrescaler = 15;  //slow clock @ 1Mhz
      
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);   
      
      //reset periph
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM4, ENABLE);
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM4, DISABLE);
      
      
        /* Time base configuration */
      TIM_TimeBaseStructure.TIM_Period = TimebasePrescaler;
      TIM_TimeBaseStructure.TIM_Prescaler = 0;
      TIM_TimeBaseStructure.TIM_ClockDivision = 0;
      TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
      TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
      
      /* Prescaler configuration */
      TIM_PrescalerConfig(TIM4, TIMPrescaler, TIM_PSCReloadMode_Immediate);
      
      /* Output Compare Timing Mode configuration: Channel1 */
      TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
      TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
      TIM_OCInitStructure.TIM_Pulse = TimebasePrescaler;
      TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
      TIM_OC1Init(TIM4, &TIM_OCInitStructure);
      TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Disable);
      
      /* Enable the TIM4 gloabal Interrupt */
      NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 8;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);   
      
      doneOnce = TRUE;
    }
    
    
    watchdogExtendCounter = ticks;
    
    /* TIM IT enable */
    TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE); 
    /* TIM4 enable counter */
    TIM_Cmd(TIM4, ENABLE);   
 
}

int GetWatchdogExtendCount(void)
{
  int value = watchdogExtendCounter--;
  return value;
}



void print_hex(char byte, char* p)
{
   char digit;
   
   digit = (((byte & 0xf0) >> 4) <= 9) ? (((byte & 0xf0) >> 4) + 0x30) : (((byte & 0xf0) >> 4) + 0x37);
   *p = digit;
     p++;
   digit = ((byte & 0x0f) <= 9) ? ((byte & 0x0f) + 0x30) : ((byte & 0x0f) + 0x37);
   *p = digit;     
  
}
      

//this marks error into the battery backed up register off the rtc
//
// - unknown errors are one point after a certain number recorder shuts down compelety
// - card full is 6 points ( instant turn off)
// - battery low is 7 points (instant turn off)
void MarkErrorRtc(MarkError_Typedef error)
{
  MarkError_Typedef lastError;

  if (error > STOP_MARK) 
  {
    RtcSetUser2Reg((uint8_t)error);
  }
  else
  {
    lastError = (MarkError_Typedef)RtcGetUser2Reg();
    lastError++;
    RtcSetUser2Reg((uint8_t)lastError);
  }
}

bool IsSeriousError(void)
{
  if ( RtcGetUser2Reg() >= STOP_MARK) return TRUE;
  return FALSE;
}

void ClearErrorMark(void)
{
  RtcSetUser2Reg(0);
}




#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
