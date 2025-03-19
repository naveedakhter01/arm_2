
#include "test.h"

#include "rtc.h"
#include "pushbutton.h"
#include "eeprom.h"
#include "gps.h"
#include "temp_sensor.h"
#include "battery.h"
#include "hardware.h"
#include "stdio.h"
#include "FatSD.h"
#include "sysclk.h"
#include "record.h"
#include "dsp_program.h"


void BatteryTest(void);
void EepromTest(void);
void RtcTest(void);
void ButtonTest(void);
bool GpsTest(void);
bool SdCardTest(void);
void TempSensorTest(void);
void BatteryTest(void);



void DisplayTempSensorId(uint8_t sensor_id);


void EepromTest(void)
{
  int i; 
          //test eeprom by writing different values to specific addresses 
  write_LCD_line1("DATA EEPROM ");
  
  i=MAGIC(2);  // 2 write/reads
  
  EEPROM_WriteByte(EE_TEST_FLAG, 0xAA);
  
  if(EEPROM_ReadByte (EE_TEST_FLAG) == 0xAA) i--;
  EEPROM_WriteByte(EE_TEST_FLAG, 0x55);
  if(EEPROM_ReadByte (EE_TEST_FLAG) == 0x55) i--;
  if (i==0) write_LCD_line2("   Passed   "); else { write_LCD_line2("   Failed   "); delay_ms(1000);}
  delay_ms(1200);
  IWDG_ReloadCounter(); //watchdog reload
}

#define DSP_EEPROM_LAST_ADDRESS   ((uint16_t) UINT16_MAX)
void DspEepromTest(void)
{
  int i; 
          //test eeprom by writing different values to specific addresses 
  write_LCD_line1(" DSP EEPROM  ");
  
  i = 2;  // 2 write/reads
  
  DspEEPROM_WriteByte( DSP_EEPROM_LAST_ADDRESS, 0xAA);
  if(DspEEPROM_ReadByte (DSP_EEPROM_LAST_ADDRESS) == 0xAA) i--;
  DspEEPROM_WriteByte( DSP_EEPROM_LAST_ADDRESS, 0x55);
  if(DspEEPROM_ReadByte (DSP_EEPROM_LAST_ADDRESS) == 0x55) i--;
  if (i==0) write_LCD_line2("   Passed   "); else {write_LCD_line2("   Failed   "); delay_ms(1000);}
  delay_ms(1500);
  IWDG_ReloadCounter(); //watchdog reload
}


void RtcTest(void)
{
  rtcTimeTypeDef rtcTime;
  rtcDateTypeDef rtcDate;
  
  //test rtc by writing time & date and checking alarm      
  write_LCD_line1("    RTC     ");
  write_LCD_line2("            ");
  
  if(HasRtcPowerFailed()) 
  {
    delay_ms(1500);
    write_LCD_line1(" Is RTC Bat ");
    write_LCD_line2("  Inserted? ");
  }
  
  rtcTime.sec = 0; 
  rtcTime.min = 0;
  rtcTime.hour= 12;
  RtcSetTime(&rtcTime);  //write time
  rtcDate.dayOfWeek = 0;
  rtcDate.date = 1;
  rtcDate.month = 1;
  rtcDate.year = 11;
  RtcSetDate(&rtcDate);   //write date
  rtcTime.sec = 1| 0x80;
  rtcTime.min = 0;
  rtcTime.hour = 0;
  RtcClearAlarmFlag(); //clear RTC alarm
  RtcSetAlarm(&rtcTime);  //set alarm secs ahead
    
  delay_ms(1500);
  if(RtcGetAlarmFlag())
  {
    //Turn pwr off, if rtc int on then pwr should still stay on
    GPIO_WriteBit(POWER_CNTRL_PORT,POWER_CNTRL_PIN, Bit_RESET); 
    delay_ms(1000);
    GPIO_WriteBit(POWER_CNTRL_PORT,POWER_CNTRL_PIN, Bit_SET); 
    write_LCD_line1(" RTC Passed ");
  }
  else{
  write_LCD_line1(" RTC Failed ");
  }
  
  write_LCD_line2("            ");
  delay_ms(1500);   
  rtcTime.sec = 0;
  rtcTime.min = 0;
  rtcTime.hour = 0;
  RtcClearAlarmFlag(); //clear RTC alarm
  RtcSetAlarm(&rtcTime);  // clear alarm settings
  
  IWDG_ReloadCounter(); //watchdog reload
  
}

void ButtonTest(void)
{
  uint8_t keymask,lastkeymask;
  int i,j;
  
  //test Buttons  
  write_LCD_line1("Test Switch");
  write_LCD_line2("            ");
  
  lastkeymask = 0x0f;
  for(i=0;i<20;i++)
  {
    LCD_Addr(0xAC);
    keymask = 0x0f;
    if(!GPIO_ReadInputDataBit(PAGE_BUTTON_PORT, PAGE_BUTTON_PIN)) keymask &= 0x0E;
    if(!GPIO_ReadInputDataBit(NEXT_BUTTON_PORT, NEXT_BUTTON_PIN)) keymask &= 0x0D;
    if(!GPIO_ReadInputDataBit(PLUS_BUTTON_PORT, PLUS_BUTTON_PIN)) keymask &= 0x0B;
    if(!GPIO_ReadInputDataBit(MINUS_BUTTON_PORT, MINUS_BUTTON_PIN)) keymask &= 0x07;
    
    for(j=0;j<4;j++)
    {
      if(!(keymask & (1 << j)))
      {
        write_LCD_char('0');
        if(lastkeymask != keymask) 
        {
          IWDG_ReloadCounter(); //watchdog reload
          i = 0; //reset timer
        }
      } 
      else write_LCD_char('1');
    }
    
    lastkeymask = keymask;
    delay_ms(100);
  }
}

bool GpsTest(void)
{
  bool ret = FALSE;
    GPS_Position_TypeDef *gpsPostionData;
  
  uint8_t s[20];
  
  
  GpsInit();
  
  
  write_LCD_line1(" Turning on ");
  write_LCD_line2("     GPS    ");
  if(StartGPSModule())
  {
    ret = TRUE;
    write_LCD_line1("   GPS on   ");
    write_LCD_line2("            ");
    delay_ms(500);
    clear_LCD();
    while(!AnyButtonPushed()) 
    {
      
      //display positions to lcd
      ProcessIncomingGpsMessages();
      gpsPostionData =  GetGPSData();
      
      sprintf((char*)s,"%2.6f   ",gpsPostionData->latitude);
      write_LCD_line1(s);
      sprintf((char*)s,"%3.6f   ",gpsPostionData->longitude);
      write_LCD_line2(s);
      sprintf((char*)s,"%2u   ",gpsPostionData->satellites);
      LCD_Addr(0x8A); //Display Start time
      write_LCD(s);
      sprintf((char*)s,"%2u   ",gpsPostionData->hdop);
      LCD_Addr(0xB2); //Display Start time
      write_LCD(s);  
      if (IsValidFix()) 
      {
        LCD_Addr(0xB3); //Display Start time
        write_LCD("v");
      }
      ResetWatchdog(); //watchdog reload
    }
    write_LCD_line1("Turning off ");
    write_LCD_line2("     GPS    ");
    StopGPSModule();
  }
  else
  {
    write_LCD_line1(" Failed to  ");  
    write_LCD_line2(" Turn On    "); 
  }
  delay_ms(1500);

  
  GpsIOLowPower();
  return ret;
}






void TempSensorTest(void)
{
  int i= 0;
  uint16_t tempVal;
  float *f;
  uint8_t tempString[16]; 
  uint8_t sensor_id;


  TempSensorADCState(ENABLE);
  TempSensorPower(ENABLE);
  delay_us(300);
  
  // assume temp sensor undefined 
  // get a couple of conversions
  tempVal = TempSensorValue();
  delay_us(1000);
  tempVal = TempSensorValue();
  
  
  //AUTO temp select
  //sensor_id = GetMostLikelySensorId(tempVal);
  //EEPROM_WriteTempSensorId(sensor_id);
  sensor_id = GetTempSensorId();
  DisplayTempSensorId(sensor_id);
  
  
  while(!IsPagebuttonPushed() && !IsNextbuttonPushed()) 
  {
    if ( (i++ & 0x0F) == 0)
    {
      LCD_Addr(0xC6);
      tempVal = TempSensorValue();
      
      f = (float*)TempConvertValue2(tempVal, sensor_id);
      sprintf((char*)tempString,"%2.1f   ",*f);
      write_LCD(tempString);
      
    }
    delay_ms(15);
    if (IsPlusbuttonPushed() || IsMinusbuttonPushed())
    {
      sensor_id++;
      SetTempSensorId(sensor_id);
      sensor_id = GetTempSensorId();
      DisplayTempSensorId(sensor_id);
    }
  }   
      
  
  TempSensorADCState(DISABLE);
  TempSensorPower(DISABLE);
}




void DisplayTempSensorId(uint8_t sensor_id)
{
  uint8_t *temp_string;
  
  temp_string = GetSensorIdName(sensor_id);
  write_LCD_line1(" Sensor Set    ");
  write_LCD_line2("   ");
  write_LCD(temp_string);
  write_LCD("    ");
  delay_ms(700);
  clear_LCD();

  write_LCD_line1("Temp (");
  write_LCD(temp_string);
  write_LCD(")");
  
  write_LCD_line2("            "); 
}



void BatteryTest(void)
{  
  float batval;
  uint8_t valStr[8];
  
  InitBatteryMeasure();

  BatteryMeasurementConfig(ENABLE);
  write_LCD_line1("Battery Test   ");
  write_LCD_line2("            ");

  GetBatteryMeasurementFloat();
  batval = GetBatteryMeasurementFloat();
  
  sprintf((char*)valStr,"%2.2f",batval);
  
  write_LCD_line2("   ");
  write_LCD(valStr);
  write_LCD("   ");
  BatteryMeasurementConfig(DISABLE);
  
  
  delay_ms(2500);
  
}





bool SdCardTest(void)
{
  int cardPrc;
  bool ret = FALSE;
  write_LCD_line1("  Sd Card   ");
  write_LCD_line2("            ");
  delay_ms(500);
  
  // first test if sd card is inserted
  if (!IsSDCardDetected()) // if not 
  {
    write_LCD_line2("Not inserted");
  }
  else
  {
    //check that sd card is ok
    cardPrc = UsedCardSpace();
    if(cardPrc >=0)
    {
      write_LCD_line2("   Passed   ");
      ret = TRUE;
    }
    else
      write_LCD_line2("   Failed   ");
  }
  delay_ms(1500);
  
  return ret;
}


//tests low high and bat recording
#define TEST_RECORDING_TIME_IN_SECS 5
void TestRecording()
{
  
  RecordSessionInit_TypeDef  sessionInit;
  SessionExitCode_Typedef defaultCode = {.error = NO_ERROR, .isButtonPushed = FALSE, .powerOk = TRUE};
  SessionExitCode_Typedef *exitCode = &defaultCode;
  
  
  //check if card inserted
  
    // first test if sd card is inserted
  if (!IsSDCardDetected()) // if not 
  {
    write_LCD_line2("  No Card   ");
    delay_ms(1500);
    return; 
  }
  
  write_LCD_line1(" Recording  ");
  write_LCD_line2("   Test     ");
  
  
  if (AttemptConnectDsp())
  {
    LED_Flash(); 
    LED_Flash(); 
    LED_Flash(); 
    LED_Flash(); 
  }
  
  OpenSDCard(); 
  CreateTestDirectory();
  SetLedDisplayNthCount(1); // led on once every dma buffer save
  ClearSessions(); //clear sessions 
  
  //SysTickInterrupt(DISABLE);
  //first check time and init recordTimeCheck
  HasPageButtonTriggered(); // this is just to clear the flag 
  while (exitCode->error == NO_ERROR && 
         exitCode->isButtonPushed == FALSE) //we break out when we get an end session flag 
  {

    
        //LOW RECORDING
        write_LCD_line1(" Recording  ");
        write_LCD_line2("   Low      ");     
        sessionInit.instruction = CHANGE_LOW_SAMPLING;
        sessionInit.startTime = *RtcGetTime();  //start time that instruction is run. maybe actual or estimated time
        sessionInit.startDate = *RtcGetDate();  //start date that instruction is run maybe actual or estimated date
        sessionInit.protocolNumber = PROTOCOL_TEST;  // protocol  that decided this session
        sessionInit.sessionTimeInSecs = TEST_RECORDING_TIME_IN_SECS;  // time of session      
        ChangeSamplingMode(&sessionInit);
        sessionInit.instruction = RECORD_LOW_8KHZ;
        exitCode = AudioRecordSessionRun(&sessionInit); // If either the low or high recording
        
        if ((exitCode->error != NO_ERROR) || exitCode->isButtonPushed) continue;
        
        //HIGH RECORDING
        write_LCD_line1(" Recording  ");
        write_LCD_line2("   High     "); 
        sessionInit.instruction = CHANGE_HIGH_SAMPLING;
        sessionInit.startTime = *RtcGetTime();  //start time that instruction is run. maybe actual or estimated time
        sessionInit.startDate = *RtcGetDate();  //start date that instruction is run maybe actual or estimated date
        sessionInit.protocolNumber = PROTOCOL_TEST;  // protocol  that decided this session
        sessionInit.sessionTimeInSecs = TEST_RECORDING_TIME_IN_SECS;  // time of session        
        ChangeSamplingMode(&sessionInit);
        sessionInit.instruction = RECORD_HIGH_32KHZ;
        exitCode = AudioRecordSessionRun(&sessionInit); // If either the low or high recording
  
        if ((exitCode->error != NO_ERROR) || exitCode->isButtonPushed) continue;
        
        //BAT RECORDING
        write_LCD_line1("Waiting for ");
        write_LCD_line2("bat trigger "); 
        sessionInit.instruction = CHANGE_BAT_SAMPLING;
        sessionInit.startTime = *RtcGetTime();  //start time that instruction is run. maybe actual or estimated time
        sessionInit.startDate = *RtcGetDate();  //start date that instruction is run maybe actual or estimated date
        sessionInit.protocolNumber = PROTOCOL_TEST;  // protocol  that decided this session
        sessionInit.sessionTimeInSecs = 2;  // time of session 5 second but won't end until it gets at least one trigger  or 15 second      
        ChangeSamplingMode(&sessionInit);
        sessionInit.instruction = RECORD_BAT_176KHZ;
        exitCode = BatRecordSessionRun(&sessionInit);
        
        
        break;
        
  }          

  DisableRecordingSpi();   //disable dsp dma int's + spi +
  Dsp_RunCommandMode(ENABLE); //stick dsp back into command mode to change settings
  SetSysClk8MHz();
  //SysTickInterrupt(ENABLE);
  
  CloseSdCard();
  
  if (exitCode->error != NO_ERROR) 
  {
    write_LCD_line2(GetErrorString(exitCode->error));
    delay_ms(1500);
  }

}



void TestDspComms(void)
{
  write_LCD_line1(" DSP comms  ");
  write_LCD_line2(" testing... ");
  if (IsDspInCommandMode() &&
      DSP_IsEvent(LEVEL))   //dsp event line should be high
    write_LCD_line2("     OK     ");
  else
    write_LCD_line2("   FAILED   ");
  delay_ms(1500);
}


void HardwareTest(void)
{
  EepromTest();  //writes values to eeprom and reads them back
  DspEepromTest();
  RtcTest();   //writes time to rtc chacks alarm triggeres correctly, checks power fail flag is set indicating missing backup battery
  //ButtonTest(); // checks buttons functioning
  TempSensorTest();
  BatteryTest();
 
  TestDspComms();
}











