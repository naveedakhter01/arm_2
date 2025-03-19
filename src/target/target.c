/****************************************************************************
 *
 *            Copyright (c) 2003-2008 by HCC Embedded
 *
 * This software is copyrighted by and is the sole property of
 * HCC.  All rights, title, ownership, or other interests
 * in the software remain the property of HCC.  This
 * software may only be used in accordance with the corresponding
 * license agreement.  Any unauthorized use, duplication, transmission,
 * distribution, or disclosure of this software is expressly forbidden.
 *
 * This Copyright notice may not be removed or modified without prior
 * written consent of HCC.
 *
 * HCC reserves the right to modify this software without notice.
 *
 * HCC Embedded
 * Budapest 1133
 * Vaci Ut 110
 * Hungary
 *
 * Tel:  +36 (1) 450 1302
 * Fax:  +36 (1) 450 1303
 * http: www.hcc-embedded.com
 * email: info@hcc-embedded.com
 *
 ***************************************************************************/
#include "target.h"
#include "../stm32f10xreg.h"
#include "uart.h"


 void hw_init(void)
 {
 hcc_u32 tmp;

  RCC_APB2RSTR = 0x0u;
  RCC_APB1RSTR = 0x0u;
  RCC_AHBENR = BIT10 | BIT2; /* Enable SDIO + DMA2 clock */
  RCC_APB2ENR = 0x0u;
  RCC_APB1ENR = 0x0u;
  RCC_CR |= 0x1u;
  RCC_CFGR &= 0xF8FF0000u;
  RCC_CR &= 0xFEF6FFFFu;
  RCC_CR &= 0xFFFBFFFFu;
  RCC_CFGR &= 0xFF80FFFFu;
  RCC_CIR = 0x0u;
  RCC_CR &= CR_HSEON_RESET;
  RCC_CR &= CR_HSEBYP_RESET;
  RCC_CR |= CR_HSEON_SET;
  RCC_CFGR &= CFGR_HPRE_RESET_M;
  tmp = RCC_CFGR;
  tmp &= CFGR_PPRE2_RESET_M;
  tmp |= RCC_HCLK_DIV2 << 3;
  RCC_CFGR = tmp;
  tmp = RCC_CFGR;
  tmp &= CFGR_PPRE1_RESET_M;
  tmp |= RCC_HCLK_DIV2;
  RCC_CFGR = tmp;
  FLASH_ACR &= FLASH_LATENCY_M;
  FLASH_ACR |= FLASH_LATENCY_2;
  FLASH_ACR &= FLASH_PREFETCH_BUFFER_M;
  FLASH_ACR |= FLASH_PREFETCH_BUFFER_ENABLE;

  /*    PLL config.: PLLCLK = 8MHz * 9 = 72 MHz */
  tmp = RCC_CFGR;
  tmp &= CFGR_PLL_M;
  tmp |= RCC_PLLSRC_HSE_DIV1 | RCC_PLLMUL_9;

  RCC_CFGR = tmp;
  RCC_CR |=  CR_PLL_ON;
  while((CR_PLL_READY & RCC_CR) == 0)
  ;
  tmp = RCC_CFGR;
  tmp &= CFGR_SW_M;
  tmp |= RCC_SYSCLKSRC_PLLCLK;
  RCC_CFGR = tmp;
  while((RCC_CFGR & CFGR_SWS_M) != 0x08)
  ;
  
  /* GPIOA, GPIOB, SPI1 clocks */
  RCC_APB2ENR |= RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_GPIOC | RCC_APB2_PERIPH_GPIOD |
                         RCC_APB2_PERIPH_SPI1 | RCC_APB2_PERIPH_AFIO ;
  RCC_AHBENR |=  RCC_AHBENR_PERIPH_DMA2EN;

  tmp = GPIOx_CRL(GPIO_A_BASE);
  tmp &= ~( (GPIO_MODE_OUT_OD << (PORT5+2))
  		  | (GPIO_MODE_OUT_OD << (PORT6+2))
  		  | (GPIO_MODE_OUT_OD << (PORT7+2)));
  tmp |= (GPIO_SPEED_50MHz  | (GPIO_MODE_ALTF_PP <<2)) << (PORT5)  /* PA5 */
	   | (GPIO_SPEED_50MHz  | (GPIO_MODE_ALTF_PP <<2)) << (PORT6)  /* PA6 */
	   | (GPIO_SPEED_50MHz  | (GPIO_MODE_ALTF_PP <<2)) << (PORT7); /* PA7 */
  GPIOx_CRL(GPIO_A_BASE) = tmp;

  tmp = GPIOx_CRL(GPIO_C_BASE);
  tmp &= ~(GPIO_MODE_OUT_OD << (PORT12+2));
  tmp |= (GPIO_SPEED_50MHz  | (GPIO_MODE_OUT_PP <<2)) << (PORT12); /* PC12 */
  GPIOx_CRH(GPIO_C_BASE) = tmp;
  //uart_init(UART,115200);
  DMA_CCR(DMA_2,DMA_CH_4) = 0;
  DMA_CNDTR(DMA_2,DMA_CH_4) = 0;
  DMA_CPAR(DMA_2,DMA_CH_4) = 0;
  DMA_CNDTR(DMA_2,DMA_CH_4) = 512;
  DMA_CCR(DMA_2,DMA_CH_4) = 0;
  
}


void pup_on_off( hcc_u8 on )
{
  int i;

  RCC_APB2ENR |= RCC_APB2_PERIPH_GPIOB;

  i = GPIOx_CRH( GPIO_B_BASE );
  i &= 0xf0ffffff;              /* Pin 11, 12, 14, 15 */
  i |= ( GPIO_SPEED_50MHz  | ( GPIO_MODE_OUT_OD << 2 ) ) << ( PORT6 );
  GPIOx_CRH( GPIO_B_BASE ) = i;

  if( on )
    GPIOx_BSRR( GPIO_B_BASE ) = 1 << 30;
  else
    GPIOx_BSRR( GPIO_B_BASE ) = 1 << 14;
} /* pup_on_off( ) */







void SDHWInit(void)
{
  hcc_u32 tmp;
  
  RCC_AHBENR |= BIT10 | BIT2; /* Enable SDIO + DMA2 clock */
  
  
  /* GPIOA, GPIOB, SPI1 clocks */
  RCC_APB2ENR |=  RCC_APB2_PERIPH_GPIOC | RCC_APB2_PERIPH_GPIOD |
                         RCC_APB2_PERIPH_SPI1 | RCC_APB2_PERIPH_AFIO ;
  RCC_AHBENR |=  RCC_AHBENR_PERIPH_DMA2EN;

  tmp = GPIOx_CRL(GPIO_C_BASE);
  tmp &= ~(GPIO_MODE_OUT_OD << (PORT12+2));
  tmp |= (GPIO_SPEED_50MHz  | (GPIO_MODE_OUT_PP <<2)) << (PORT12); /* PC12 */
  GPIOx_CRH(GPIO_C_BASE) = tmp;
  
  DMA_CCR(DMA_2,DMA_CH_4) = 0;
  DMA_CNDTR(DMA_2,DMA_CH_4) = 0;
  DMA_CPAR(DMA_2,DMA_CH_4) = 0;
  DMA_CNDTR(DMA_2,DMA_CH_4) = 512;
  DMA_CCR(DMA_2,DMA_CH_4) = 0;   
  
  
}