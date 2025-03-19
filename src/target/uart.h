                        #ifndef __UART_H
#define __UART_H

#include "../hcc_types.h"
#include "../stm32f10xreg.h"

/* test output */

#define UART USART_1
 
int uart_init (hcc_u32 uart, hcc_u32 Baudrate);
hcc_u16 uart_getkey (hcc_u32 uart);
void uart_send (hcc_u32 uart, hcc_u8 *BufferPtr, hcc_u32 Length);
void uart_putchar (hcc_u32 uart, hcc_u8 ch);

#endif
