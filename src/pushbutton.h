/*****************************************************************************
* pushbutton.h
*
* 20/10/09
*****************************************************************************/

#ifndef PUSHBUTTON_H
#define PUSHBUTTON_H

#include "stm32f10x.h"
#include "stm32_eval.h"


//Buttons setup for sound recorder board , up to 16


#define PAGE_KEY     0x0001
#define PLUS_KEY     0x0002
#define MINUS_KEY    0x0004
#define NEXT_KEY     0x0008   
   
#define BUTTONn                   4
#define ALL_BUTTON_PORT           GPIOC
/**
 * @brief PAGE push-button - KEY0
 */
#define PAGE_BUTTON_PORT          GPIOC
#define PAGE_BUTTON_CLK           RCC_APB2Periph_GPIOC
#define PAGE_BUTTON_PIN           GPIO_Pin_0
#define PAGE_BUTTON_EXTI_LINE     EXTI_Line0
#define PAGE_BUTTON_PORT_SOURCE   GPIO_PortSourceGPIOC
#define PAGE_BUTTON_PIN_SOURCE    GPIO_PinSource0
#define PAGE_BUTTON_IRQn          EXTI0_IRQn 
/**
 * @brief NEXT push-button
 */
#define NEXT_BUTTON_PORT          GPIOC
#define NEXT_BUTTON_CLK           RCC_APB2Periph_GPIOC
#define NEXT_BUTTON_PIN           GPIO_Pin_1
#define NEXT_BUTTON_EXTI_LINE     EXTI_Line1
#define NEXT_BUTTON_PORT_SOURCE   GPIO_PortSourceGPIOC
#define NEXT_BUTTON_PIN_SOURCE    GPIO_PinSource1
#define NEXT_BUTTON_IRQn          EXTI1_IRQn 
/**
 * @brief PLUS push-button
 */
#define PLUS_BUTTON_PORT          GPIOC
#define PLUS_BUTTON_CLK           RCC_APB2Periph_GPIOC
#define PLUS_BUTTON_PIN           GPIO_Pin_2
#define PLUS_BUTTON_EXTI_LINE     EXTI_Line2
#define PLUS_BUTTON_PORT_SOURCE   GPIO_PortSourceGPIOC
#define PLUS_BUTTON_PIN_SOURCE    GPIO_PinSource2
#define PLUS_BUTTON_IRQn          EXTI2_IRQn 
/**
 * @brief MINUS push-button
 */
#define  MINUS_BUTTON_PORT          GPIOC
#define  MINUS_BUTTON_CLK           RCC_APB2Periph_GPIOC
#define  MINUS_BUTTON_PIN           GPIO_Pin_3
#define  MINUS_BUTTON_EXTI_LINE     EXTI_Line3
#define  MINUS_BUTTON_PORT_SOURCE   GPIO_PortSourceGPIOC
#define  MINUS_BUTTON_PIN_SOURCE    GPIO_PinSource3
#define  MINUS_BUTTON_IRQn          EXTI3_IRQn 



typedef enum
{
		IDLE = 0,
		PUSHED,
                RELEASED,
		HELD
}bSTATE;	





struct sBttnItem
{
   uint8_t  *BttnName;
   uint16_t BttnGPIOpin;
   GPIO_TypeDef* BttnGPIOport;
   uint16_t BttnValue;  // value to define this button
   FlagStatus BttnFlag;
   bSTATE BttnSTATE;
   uint16_t BttnTimer;  //used for debouncing   
} ;


void PushButton_Init(void);

uint16_t GetButtonStatus();
void SetButtonFlagFunc(uint16_t intBttnValue);
uint8_t GetKeyMask(void);
void SetPageButtonPushed(void);

bool HasPageButtonTriggered(void);

bool AnyButtonPushed(void);


bool IsPagebuttonPushed(void);


bool IsNextbuttonPushed(void);
bool IsPlusbuttonPushed(void);
bool IsMinusbuttonPushed(void);

#endif