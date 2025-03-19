/**
  ******************************************************************************
*
*  DEFINITIONS FOR THE BUTTONS AND LED(S) oON THE ACOUSTIC MONITORING BOARD
*
*

/* Includes ------------------------------------------------------------------*/
#include "stm32_am.h"


GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT, LED2_GPIO_PORT, LED3_GPIO_PORT,
                                 LED4_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LED1_GPIO_PIN, LED2_GPIO_PIN, LED3_GPIO_PIN,
                                 LED4_GPIO_PIN};
const uint32_t GPIO_CLK[LEDn] = {LED1_GPIO_CLK, LED2_GPIO_CLK, LED3_GPIO_CLK,
                                 LED4_GPIO_CLK};




 GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {PAGE_BUTTON_PORT, PLUS_BUTTON_PORT, 
                                       MINUS_BUTTON_PORT, NEXT_BUTTON_PORT}; 

 const uint16_t BUTTON_PIN[BUTTONn] = {PAGE_BUTTON_PIN, PLUS_BUTTON_PIN, 
                                       MINUS_BUTTON_PIN, NEXT_BUTTON_PIN}; 

 const uint32_t BUTTON_CLK[BUTTONn] = {PAGE_BUTTON_CLK, PLUS_BUTTON_CLK,
                                       MINUS_BUTTON_CLK, NEXT_BUTTON_CLK};

 const uint16_t BUTTON_EXTI_LINE[BUTTONn] = {PAGE_BUTTON_EXTI_LINE,
                                             PLUS_BUTTON_EXTI_LINE, 
                                             MINUS_BUTTON_EXTI_LINE,
                                             NEXT_BUTTON_EXTI_LINE};

 const uint16_t BUTTON_PORT_SOURCE[BUTTONn] = {PAGE_BUTTON_PORT_SOURCE,
                                               PLUS_BUTTON_PORT_SOURCE, 
                                               MINUS_BUTTON_PORT_SOURCE,
                                               NEXT_BUTTON_PORT_SOURCE};
								 
 const uint16_t BUTTON_PIN_SOURCE[BUTTONn] = {PAGE_BUTTON_PIN_SOURCE,
                                              PLUS_BUTTON_PIN_SOURCE, 
                                              MINUS_BUTTON_PIN_SOURCE,
                                              NEXT_BUTTON_PIN_SOURCE}; 

 const uint16_t BUTTON_IRQn[BUTTONn] = {PAGE_BUTTON_IRQn, PLUS_BUTTON_IRQn, 
                                        MINUS_IRQn, NEXT_IRQn};




/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
void STM_AM_LEDInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_APB2PeriphClockCmd(GPIO_CLK[Led], ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN[Led];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIO_PORT[Led], &GPIO_InitStructure);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4  
  * @retval None
  */
void STM_AM_LEDOn(Led_TypeDef Led)
{
  GPIO_PORT[Led]->BSRR = GPIO_PIN[Led]; 
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4 
  * @retval None
  */
void STM_AM_LEDOff(Led_TypeDef Led)
{
  GPIO_PORT[Led]->BRR = GPIO_PIN[Led];
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  * @retval None
  */
void STM_AM_LEDToggle(Led_TypeDef Led)
{
  GPIO_PORT[Led]->ODR ^= GPIO_PIN[Led];
}

/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter can be one of following parameters:
  *     @arg Button_PAGE: Joystick PAGE Push Button 
  *     @arg Button_PLUS: Joystick PLUS Push Button 
  *     @arg Button_MINUS: Joystick MINUS Push Button
  *     @arg Button_NEXT: Joystick NEXT Push Button
  * @param  Button_Mode: Specifies Button mode.
  *   This parameter can be one of following parameters:   
  *     @arg Mode_GPIO: Button will be used as simple IO 
  *     @arg Mode_EXTI: Button will be connected to EXTI line with interrupt
  *                     generation capability  
  * @retval None
  */
void STM_AM_PBInit(Button_TypeDef Button, Button_Mode_TypeDef Button_Mode)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable Button GPIO clock */
  RCC_APB2PeriphClockCmd(BUTTON_CLK[Button] | RCC_APB2Periph_AFIO, ENABLE);
  
  /* Configure Button pin as input floating */
  GPIO_InitStructure.GPIO_Pin = BUTTON_PIN[Button];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStructure);

  if (Button_Mode == Mode_EXTI)
  {
    /* Connect Button EXTI Line to Button GPIO Pin */
    GPIO_EXTILineConfig(BUTTON_PORT_SOURCE[Button], BUTTON_PIN_SOURCE[Button]);  

    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = BUTTON_EXTI_LINE[Button];
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;

    if(Button != Button_WAKEUP)
    {
      EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
    }
    else
    {
      EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    }
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = BUTTON_IRQn[Button];
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure); 
  }
}

/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter can be one of following parameters:    
  *     @arg Button_PAGE: Joystick PAGE Push Button 
  *     @arg Button_PLUS: Joystick PLUS Push Button 
  *     @arg Button_MINUS: Joystick MINUS Push Button
  *     @arg Button_NEXT: Joystick NEXT Push Button 
  * @retval The Button GPIO pin value.
  */
uint32_t STM_AM_PBGetState(Button_TypeDef Button)
{
  return GPIO_ReadInputDataBit(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}



/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/