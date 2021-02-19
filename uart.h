/*
   This code is based on the BMP085 library by Stefan Sicklinger published on
   http://www.sicklinger.com
   
   I am redistributing this modified version under the GNU General Public
   License version 3
*/

#ifndef UART_H_INCLUDED
#define UART_H_INCLUDED

#include <stdio.h>

void uartSetup(void);
void uartSendChar(unsigned char c);
void uartSendString (char *s);
int uartPutChar(char c, FILE *stream);
unsigned char usart_receive( void );
void USART_flush(void);

#endif
