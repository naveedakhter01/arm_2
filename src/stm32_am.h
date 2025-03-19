/**
  ******************************************************************************
  * @file    stm3210e_am.h
  * @brief   This file contains definitions for  Leds, push-buttons
  *          on acoustic monitoring board
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM3210E_EVAL_H
#define __STM3210E_EVAL_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//PWR control to main board
#define PWRon_GPIO_PORT              GPIOB
#define PWRon_GPIO_CLK               RCC_APB2Periph_GPIOB  
#define PWRon_GPIO_PIN               GPIO_Pin_8

//Battery measure control
#define BATon_GPIO_PORT              GPIOC
#define BATon_GPIO_CLK               RCC_APB2Periph_GPIOC  
#define BATon_GPIO_PIN               GPIO_Pin_5

//SD pwr control ACTIVE LOW
#define SDon_GPIO_PORT              GPIOE
#define SDon_GPIO_CLK               RCC_APB2Periph_GPIOE  
#define SDon_GPIO_PIN               GPIO_Pin_2

//USBpower detect
#define USBdet_GPIO_PORT              GPIOB
#define USBdet_GPIO_CLK               RCC_APB2Periph_GPIOB  
#define USBdet_GPIO_PIN               GPIO_Pin_9
#define USBdet_EXTI_LINE              EXTI_Line9
#define USBdet_PORT_SOURCE            GPIO_PortSourceGPIOB
#define USBdet_PIN_SOURCE             GPIO_PinSource9
#define USBdet_IRQn                   EXTI9_5_IRQn 

//I2C Interrupt
#define I2Cint_GPIO_PORT              GPIOB
#define I2Cint_GPIO_CLK               RCC_APB2Periph_GPIOB  
#define I2Cint_GPIO_PIN               GPIO_Pin_5
#define I2Cint_EXTI_LINE              EXTI_Line5
#define I2Cint_PORT_SOURCE            GPIO_PortSourceGPIOB
#define I2Cint_PIN_SOURCE             GPIO_PinSource5
#define I2Cint_IRQn                   EXTI9_5_IRQn 


// ALL LEDS set to same pin
#define LEDn                        4
#define LED1_GPIO_PORT              GPIOC
#define LED1_GPIO_CLK               RCC_APB2Periph_GPIOC  
#define LED1_GPIO_PIN               GPIO_Pin_7
  
#define LED2_GPIO_PORT              GPIOC
#define LED2_GPIO_CLK               RCC_APB2Periph_GPIOC  
#define LED2_GPIO_PIN               GPIO_Pin_7
  
#define LED3_GPIO_PORT              GPIOC
#define LED3_GPIO_CLK               RCC_APB2Periph_GPIOC  
#define LED3_GPIO_PIN               GPIO_Pin_7
  
#define LED4_GPIO_PORT              GPIOC
#define LED4_GPIO_CLK               RCC_APB2Periph_GPIOC  
#define LED4_GPIO_PIN               GPIO_Pin_7


  

#define BUTTONn                     4

/**
 * @brief PAGE push-button
 */
#define PAGE_BUTTON_PORT          GPIOC
#define PAGE_BUTTON_CLK           RCC_APB2Periph_GPIOC
#define PAGE_BUTTON_PIN           GPIO_Pin_0
#define PAGE_BUTTON_EXTI_LINE     EXTI_Line0
#define PAGE_BUTTON_PORT_SOURCE   GPIO_PortSourceGPIOC
#define PAGE_BUTTON_PIN_SOURCE    GPIO_PinSource0
#define PAGE_BUTTON_IRQn          EXTI0_IRQn 
/**
 * @brief PLUS push-button
 */
#define PLUS_BUTTON_PORT          GPIOC
#define PLUS_BUTTON_CLK           RCC_APB2Periph_GPIOC
#define PLUS_BUTTON_PIN           GPIO_Pin_1
#define PLUS_BUTTON_EXTI_LINE     EXTI_Line1
#define PLUS_BUTTON_PORT_SOURCE   GPIO_PortSourceGPIOC
#define PLUS_BUTTON_PIN_SOURCE    GPIO_PinSource1
#define PLUS_BUTTON_IRQn          EXTI1_IRQn
/**
 * @brief MINUS push-button
 */
#define MINUS_BUTTON_PORT             GPIOC
#define MINUS_BUTTON_CLK              RCC_APB2Periph_GPIOC
#define MINUS_BUTTON_PIN              GPIO_Pin_2
#define MINUS_BUTTON_EXTI_LINE        EXTI_Line2
#define MINUS_BUTTON_PORT_SOURCE      GPIO_PortSourceGPIOC
#define MINUS_BUTTON_PIN_SOURCE       GPIO_PinSource2
#define MINUS_BUTTON_IRQn             EXTI2_IRQn
/**
 * @brief NEXT Right push-button
 */
#define NEXT_BUTTON_PORT           GPIOC
#define NEXT_BUTTON_CLK            RCC_APB2Periph_GPIOC
#define NEXT_BUTTON_PIN            GPIO_Pin_3
#define NEXT_BUTTON_EXTI_LINE      EXTI_Line3
#define NEXT_BUTTON_PORT_SOURCE    GPIO_PortSourceGPIOC
#define NEXT_BUTTON_PIN_SOURCE     GPIO_PinSource3
#define NEXT_BUTTON_IRQn           EXTI3_IRQn




/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
