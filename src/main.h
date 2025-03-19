/**
  ******************************************************************************
  * @file SysTick/main.h 
  * @author  MCD Application Team
  * @version V3.1.0
  * @date    06/19/2009
  * @brief  Header for main.c module
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MAIN_H
#define MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32_eval.h"



/* Exported types ------------------------------------------------------------*/
typedef enum
{ B0 = 0x01,
  B1 = 0x02,
  B2 = 0x04,
  B3 = 0x08
}Bkey;

/* Exported types ------------------------------------------------------------*/
typedef enum
{ RECORDING = 0x01,
  MENU,
  STARTUP
}RECORDER_Mode;

typedef enum
{
  MARK_MINOR_ERROR = 1,   //possible recoverable glitch
  STOP_MARK = 5,
  MARK_CARD_FULL = 6,
  MARK_BATTERY_LOW = 7
} MarkError_Typedef;










typedef void (*pFunction)(void); 

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void SetButton(Bkey Button);
void SetExitRecordFlag(void);
void IntExtOnOffConfig(FunctionalState NewState);
void DisableRecInt(void);
uint16_t CheckBattery(void);
RECORDER_Mode GetCurMode(void);
void print_hex(char byte, char* p);
void Battery_TIM_init(void);
void AddBatVal(void);
void Battery_TIM_IRQ(FunctionalState NewState);
void FlashLED(void);
void WavFileRecording();
void BmpFileRecording();

void ResetWatchDog(void);
void MarkErrorRtc(MarkError_Typedef error);
bool IsSeriousError(void);

void ClearErrorMark(void);
void WatchdogExtendTimer(int ticks);
int GetWatchdogExtendCount();

#endif /* __MAIN_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
