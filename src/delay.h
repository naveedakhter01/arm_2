
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


void delay_ms(uint32_t nTime);
void delay_us(uint32_t nTime);
void TimingDelay_Decrement(void);
uint16_t GetTicks(void);
bool Delay_Dsp_Start(void);
