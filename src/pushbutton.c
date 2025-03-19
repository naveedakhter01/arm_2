/*****************************************************************************
* pushbutton.c 
*
* 20/10/09
*
*For Sound Recorder Board
*
*  *Handles debouncing of all buttons on individual basis
*  *Creates a recurring output when button held down
*
*  
*  - modify values in the PUSHBUTTON SETUP STRUCTURE
*  - change the MAX_BTTN ITEMS to reflect number of items in structure
*  - create a GetTicks() function that returns a unsigned  16bit counter value based on 1msec increments (setup systick)
*  - In interrupt handler for button call SetButtonFlagFunc(BttnValue), where bttnValue reflects the button pushed
*
*****************************************************************************/


#include "pushbutton.h"
#include "delay.h"



#define   PUSHTIME    10  //10ms after push before checking if valid push 
#define   RELEASETIME    30  //wait 30ms after release until it can be triggered again 
#define   HOLDTIME    400 // determines the length of time button needs to be held down to enter "HELD" mode
#define   REPEATTIME   130 // when button held down , enable recurring trigger of this button every set msecs


  
#define MAX_BTTN_ITEMS     4          // b name      b pin         b port         b value      flag  status debnce_timer

/* PUSHBUTTON SETUP STRUCTURE */
struct sBttnItem sBttnList[] = { 	{ "PAGE",  PAGE_BUTTON_PIN,  PAGE_BUTTON_PORT,  PAGE_KEY,  RESET, IDLE,   0}, 
					{ "PLUS",  PLUS_BUTTON_PIN,  PLUS_BUTTON_PORT,  PLUS_KEY,  RESET, IDLE,   0},
                                        { "MINUS", MINUS_BUTTON_PIN, MINUS_BUTTON_PORT, MINUS_KEY, RESET, IDLE,   0},
                                        { "NEXT",  NEXT_BUTTON_PIN,  NEXT_BUTTON_PORT,  NEXT_KEY,  RESET, IDLE,   0}
              			};




/**************************************
* PushButton_Init
* 
* - initialise the gpio for the pushbuttons
* - initialise the interrupts for the pushbuttons
**************************************/
void PushButton_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure; 
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(PAGE_BUTTON_CLK | NEXT_BUTTON_CLK | PLUS_BUTTON_CLK | MINUS_BUTTON_CLK , ENABLE);    
   
     //SETUP INPUTS + PULL UP
  /* Configure Button pin as input floating */
  GPIO_InitStructure.GPIO_Pin = PAGE_BUTTON_PIN | NEXT_BUTTON_PIN | PLUS_BUTTON_PIN | MINUS_BUTTON_PIN ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(ALL_BUTTON_PORT, &GPIO_InitStructure);  
 
        /* Connect Button EXTI Line to Button GPIO Pin */
  GPIO_EXTILineConfig(PAGE_BUTTON_PORT_SOURCE, PAGE_BUTTON_PIN_SOURCE);
  GPIO_EXTILineConfig(NEXT_BUTTON_PORT_SOURCE, NEXT_BUTTON_PIN_SOURCE);  
  GPIO_EXTILineConfig(PLUS_BUTTON_PORT_SOURCE, PLUS_BUTTON_PIN_SOURCE);  
  GPIO_EXTILineConfig(MINUS_BUTTON_PORT_SOURCE, MINUS_BUTTON_PIN_SOURCE);       
  

  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
  
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;   
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;   
  NVIC_Init(&NVIC_InitStructure);
  
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;   
  NVIC_Init(&NVIC_InitStructure);  
  
}







/**************************************
*Gets the status of the buttons 
* Handles debouncing of buttons 
*
*
* returns status of button after debounce
*
**************************************/
uint16_t GetButtonStatus()
{
     uint16_t StatusOut = 0; // determined status of buttons loaded here 
     uint8_t i;
     
   for(i=0; i< MAX_BTTN_ITEMS ; i++)
    {
             
        
      switch (sBttnList[i].BttnSTATE)
      {
        
      case IDLE:   // default button in high state, when pushed wait set msecs after last int to trigger
   
            if(sBttnList[i].BttnFlag)  // if the button pushed (flag set)
              {
               if ((uint16_t)(GetTicks() -  sBttnList[i].BttnTimer) >= PUSHTIME) // check condition after debounce period
               {
                if (!GPIO_ReadInputDataBit(sBttnList[i].BttnGPIOport, sBttnList[i].BttnGPIOpin)) 
                {
                  sBttnList[i].BttnSTATE = PUSHED;
                  StatusOut |= sBttnList[i].BttnValue;
                }
                else sBttnList[i].BttnSTATE = RELEASED;
                
                sBttnList[i].BttnTimer = GetTicks();  // set new debounce period 
                sBttnList[i].BttnFlag = RESET; //reset flag   
               }  
              }
              break;
        
      case PUSHED: //checks if button held down or released 
              if ((uint16_t)(GetTicks() -  sBttnList[i].BttnTimer) >= HOLDTIME) // check condition after debounce period
               {
                sBttnList[i].BttnSTATE = HELD; //HELD;  // put into held mode if timer expired
                sBttnList[i].BttnTimer = GetTicks();  // set new debounce period 
                }   
             // if(PageF) {//if button trigger again then reset  
              // PageSTATE = RELEASED; 
               sBttnList[i].BttnFlag = RESET;
              //}
                
              if (GPIO_ReadInputDataBit(sBttnList[i].BttnGPIOport, sBttnList[i].BttnGPIOpin)) sBttnList[i].BttnSTATE = RELEASED; //go to Released state as soon a button HIGH
              break;      
       case RELEASED:  //  when released wait set msecs after last int to trigger
               if ((uint16_t)(GetTicks() -  sBttnList[i].BttnTimer) >= RELEASETIME) // check condition after debounce period
               {
                sBttnList[i].BttnSTATE = IDLE;  // put into held mode if timer expired
               sBttnList[i].BttnFlag = RESET;
               }         
               
               if(sBttnList[i].BttnFlag) {//if button trigger again then reset timer 
               sBttnList[i].BttnTimer = GetTicks(); 
               sBttnList[i].BttnFlag = RESET;
              }                 
               
               
         
            break;
      case HELD:
               if ((uint16_t)(GetTicks() -  sBttnList[i].BttnTimer) >= REPEATTIME) // check condition after debounce period
               {

                sBttnList[i].BttnTimer = GetTicks();
                StatusOut |= sBttnList[i].BttnValue;
               }          

               if(sBttnList[i].BttnFlag) {//if button trigger again then reset  
               sBttnList[i].BttnSTATE = RELEASED; 
               sBttnList[i].BttnFlag = RESET;
              }            
              if (GPIO_ReadInputDataBit(sBttnList[i].BttnGPIOport, sBttnList[i].BttnGPIOpin)) sBttnList[i].BttnSTATE = RELEASED; //go to Released state as soon a button HIGH        
            break;
      }      
      
    
  
       
    }
  
  
  

  
  
  return(StatusOut);
}





/**************************************************
* Is called from interrupt routine  
* sets flags for individual buttons
*************************************************/


void SetButtonFlagFunc(uint16_t intBttnValue)
{
  
   uint8_t i;
  
  
  for(i=0; i< MAX_BTTN_ITEMS;i++)
  {
    
   if (sBttnList[i].BttnValue = intBttnValue)
   {
       sBttnList[i].BttnFlag = SET;
      sBttnList[i].BttnTimer = GetTicks(); //get systick to use for debouncing 
    }
  }
}



uint8_t GetKeyMask(void)
{
 uint8_t keymask;
    keymask = 0x0f;
    if(!GPIO_ReadInputDataBit(PAGE_BUTTON_PORT,PAGE_BUTTON_PIN)) keymask &=0x0E;
    if(!GPIO_ReadInputDataBit(NEXT_BUTTON_PORT,NEXT_BUTTON_PIN)) keymask &=0x0D;
    if(!GPIO_ReadInputDataBit(PLUS_BUTTON_PORT,PLUS_BUTTON_PIN)) keymask &=0x0B;
    if(!GPIO_ReadInputDataBit(MINUS_BUTTON_PORT,MINUS_BUTTON_PIN)) keymask &=0x07;
    return keymask;
}



volatile bool pageButtonPushed = FALSE;

void SetPageButtonPushed(void)
{
  pageButtonPushed = TRUE;
}

bool HasPageButtonTriggered(void)
{
  bool ret;
  
  ret = pageButtonPushed;
  pageButtonPushed = FALSE;
  return ret;
}


//polled button test
bool AnyButtonPushed(void)
{
  bool pushed = FALSE;

  static int lastStates[MAX_BTTN_ITEMS];
  int currentStates[MAX_BTTN_ITEMS];
  
  currentStates[0] = GPIO_ReadInputDataBit(PAGE_BUTTON_PORT,PAGE_BUTTON_PIN); 
  currentStates[1] = GPIO_ReadInputDataBit(NEXT_BUTTON_PORT,NEXT_BUTTON_PIN);
  currentStates[2] = GPIO_ReadInputDataBit(PLUS_BUTTON_PORT,PLUS_BUTTON_PIN);
  currentStates[3] = GPIO_ReadInputDataBit(MINUS_BUTTON_PORT,MINUS_BUTTON_PIN);

  if (currentStates[0]==0 && lastStates[0] !=0) pushed |=TRUE;  
  if (currentStates[1]==0 && lastStates[1] !=0) pushed |=TRUE;  
  if (currentStates[2]==0 && lastStates[2] !=0) pushed |=TRUE;  
  if (currentStates[3]==0 && lastStates[3] !=0) pushed |=TRUE;  

  lastStates[0] = currentStates[0]; 
  lastStates[1] = currentStates[1];
  lastStates[2] = currentStates[2];
  lastStates[3] = currentStates[3];
  
  return pushed;
}


//legacy
bool IsPagebuttonPushed(void)
{
  return (GPIO_ReadInputDataBit(PAGE_BUTTON_PORT,PAGE_BUTTON_PIN) != 0) ?  FALSE: TRUE;
}




bool IsNextbuttonPushed(void)
{
  static bool last_state;
  bool current_state;
  bool ret;
  
  current_state = (GPIO_ReadInputDataBit(NEXT_BUTTON_PORT,NEXT_BUTTON_PIN) == 0);
 
  ret =  !last_state && current_state; 
  last_state = current_state;
  delay_ms(5);
  return ret;
}


bool IsPlusbuttonPushed(void)
{
  static bool last_state;
  bool current_state;
  bool ret;
  
  current_state = (GPIO_ReadInputDataBit(PLUS_BUTTON_PORT,PLUS_BUTTON_PIN) == 0);
 
  ret =  !last_state && current_state; 
  last_state = current_state;
  delay_ms(5);
  return ret;
}


bool IsMinusbuttonPushed(void)
{
  static bool last_state;
  bool current_state;
  bool ret;
  
  current_state = (GPIO_ReadInputDataBit(MINUS_BUTTON_PORT,MINUS_BUTTON_PIN) == 0);
 
  ret =  !last_state && current_state; 
  last_state = current_state;
  delay_ms(5);
  return ret;
}






/*********************************EOF*************************************/

















