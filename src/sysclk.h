/******************************************************
*
*  sysclk.h
*
* 
*******************************************************/

#ifndef SYSCLK_H
#define SYSCLK_H

#include "stm32f10x.h"






void SysTickInterrupt(FunctionalState state);

void SetClkHSE(void);
void SetClkPLL(void);


void SetSysClk8MHz(void);
void SetSysClk2MHz(void);


void SYSCLKConfig_STOP(void);

#endif //SYSCLK_H

