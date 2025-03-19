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
#include "../hcc_types.h"
#include "../stm32f10xreg.h"
#include "uart.h"

#define FCLK 36000000

int uart_init (hcc_u32 usart, hcc_u32 baudrate)
{
  hcc_u32 fdiv;
  hcc_u32 idiv;
  hcc_u32 tmp;

  /* USART TX,RX */
  if (usart == USART_1)
  {
    tmp = GPIOx_CRH(GPIO_A_BASE);
    tmp &= ~( (GPIO_MODE_OUT_OD << (PORT9+2))
  		  | (GPIO_MODE_OUT_OD << (PORT10+2)));
    tmp |= (GPIO_SPEED_50MHz  | (GPIO_MODE_ALTF_PP <<2)) << (PORT9)  /* PA9 */
	     | (GPIO_SPEED_50MHz  | (GPIO_MODE_ALTF_PP <<2)) << (PORT10);  /* PA10 */
    GPIOx_CRH(GPIO_A_BASE) = tmp;
    RCC_APB2ENR |= RCC_APB2_PERIPH_USART1;
  }
  else if  (usart == USART_2)
  {
    AFIO_MAPR |=  USART2_REMAP;
    tmp = GPIOx_CRL(GPIO_D_BASE);
    tmp &= ~( (GPIO_MODE_OUT_OD << (PORT5+2))
  		  | (GPIO_MODE_OUT_OD << (PORT6+2)));
    tmp |= (GPIO_SPEED_50MHz  | (GPIO_MODE_ALTF_PP <<2)) << (PORT5)  /* PD5 */
	     | (GPIO_SPEED_50MHz  | (GPIO_MODE_ALTF_PP <<2)) << (PORT6);  /* PD6 */
    GPIOx_CRL(GPIO_D_BASE) = tmp;
    RCC_APB1ENR |= RCC_APB2_PERIPH_USART2;
  }

  /* Determine the integer part */
  idiv = ((0x19 * FCLK) / (0x04 * (baudrate)));
  tmp = (idiv / 0x64) << 0x04;

  /* Determine the fractional part */
  fdiv = idiv - (0x64 * (tmp >> 0x04));
  tmp |= ((((fdiv * 0x10) + 0x32) / 0x64)) & ((hcc_u8)0x0f);

  /* Write to USART BRR */
  USART_BRR(usart) = (hcc_u16)tmp;
  USART_CR1(usart) |= USART_TE | USART_RE | USART_UE;
  return 0;
}

hcc_u16 uart_getkey (hcc_u32 usart)
{
    if (USART_SR(usart)&USART_RXNE) return (hcc_u16)USART_DR(usart);
    return 0xff00;
}


void uart_send (hcc_u32 usart, hcc_u8 *BufferPtr, hcc_u32 Length)
{
    while (Length--)
    {
      while ((USART_SR(usart)&(USART_TXE))==0);
      USART_DR(usart) = *BufferPtr++;
    }
}

void uart_putchar (hcc_u32 usart, hcc_u8 ch)
{
    while ((USART_SR(usart)&(USART_TXE))==0);
    USART_DR(usart)=ch;
}



