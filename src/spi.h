/******************************************************************************
* spi.h
* 
*******************************************************************************/




#ifndef SPI_H
#define SPI_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"






typedef enum {DMA_Off = 0, DMA_On = !DMA_Off} DMAState;



void SetSpiSlave(DMAState _DMAState);
void ResetSpiSlave(void);
uint16_t GetDSPVer(void);
void Spi_Dma_Init(void);





#endif 