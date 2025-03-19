/*******************************************************************************
*SPI INTERFACE 
* 12/2/10
*******************************************************************************/


#include "spi.h"
#include "stm32f10x_it.h"
#include "record.h"




void SetSpiSlave(DMAState _DMAState)
{

  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  
  /* Enable the GPIO_LED Clock*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);  
  
  /* Configure SPI_MASTER pins: MOSI, SCK  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  

  /* SPI_SLAVE configuration ------------------------------------------------------*/
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);    
  
  if(_DMAState)         //Initialise DMA if used
  {  
    Spi_Dma_Init();
    /* Enable SPI_SLAVE Rx request */
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);   
  }

  //SPI_NSSInternalSoftwareConfig(SPI2,SPI_NSSInternalSoft_Reset);  
  
  SPI_Cmd(SPI2, ENABLE);
}


void ResetSpiSlave(void)
{
  SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, DISABLE);   //Disable the SPI interrupt 
  DMA_ITConfig(DMA1_Channel4, DMA_IT_TC |DMA_IT_HT, DISABLE);  
  SPI_Cmd(SPI2, DISABLE); // Disable SPI_SLAVE 
  DMA_Cmd(DMA1_Channel4, DISABLE);  // Disable DMA1 Channel2 
}








void Spi_Dma_Init(void)
{    
    NVIC_InitTypeDef NVIC_InitStructure; 
    
    DMA_InitTypeDef  DMA_InitStructure;  
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);        //Initialise DMA if used    
    DMA_DeInit(DMA1_Channel4);  //SPI_SLAVE_Rx_DMA_Channel
    DMA_InitStructure.DMA_PeripheralBaseAddr = 0x4000380C;    //SPI_SLAVE_DR_Base   SPI2         
    DMA_InitStructure.DMA_MemoryBaseAddr = SAMPLE_BUFFER_ADDR; //start of ext sram
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = SAMPLE_BUFFER_SIZE/2; // size in 16bit words 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);    
    
    
    /* Enable the  gloabal Interrupt */  
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);   
    
        //Enable the DMA interrupt
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC |DMA_IT_HT, ENABLE);  
    /* Enable DMA1 Channel4 */
    DMA_Cmd(DMA1_Channel4, ENABLE);  
    
}    









/*******************************************************************************
* Retrieves the dsp version number from the spi port
*
*******************************************************************************/

uint16_t GetDSPVer(void)
{

    return 0;
}



/********************************************EOF***************************************/








