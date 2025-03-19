


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DELAY_H
#define __DELAY_H


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"


/* Exported types ------------------------------------------------------------*/

typedef enum
{
    FAST_CLK =  0x00,
    NORMAL_CLK = 0x01,
    SLOW_CLK = 0x01,
    
}sCLOCK;	



/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


void delay_ms(uint32_t nTime);
void delay_us(uint32_t nTime);
void delay_sec(uint32_t nTime);
void TimingDelay_Decrement(void);
uint16_t GetTicks(void);
void DelayTimer_init(sCLOCK Clock);
void DelayTimer_Prescaler(sCLOCK Clock);
void FiveSecDelay_Decrement(void);
uint16_t Get5SecTicks(void);
void DisableLowPowerDelayTimer();






#endif

/*******************************************************************************/
