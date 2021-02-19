/*
   free to use for ATmega328
*/

//#define BAUD 19200UL
#define BAUD 9600UL
#define BRC ((F_CPU / 16 / BAUD) - 1)

#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include "uart.h"

static FILE stdoutUart = FDEV_SETUP_STREAM(uartPutChar, NULL, _FDEV_SETUP_WRITE);

void uartSetup(void)
{
    // Set baud rate
    UBRR0H = (BRC >> 8);
    UBRR0L = BRC;
    
    // Enable the USART Receiver and Transmitter
    UCSR0B |= (1 << RXEN0);
    UCSR0B |= (1 << TXEN0);
    
    // Enable the RX Complete Interrupt and the TX Complete Interrupt
    UCSR0B |= (1 << RXCIE0);
    //UCSR0B |= (1 << TXCIE0);
    
    // Set the Character Size to 8 bits. There is one stop bit, which is
    // the default setting.
    UCSR0C |= (0<<USBS0)|(1 << UCSZ01) | (1 << UCSZ00)|(0<<UPM00);
    
    stdout = &stdoutUart;
}


//void USART_init(unsigned int ubrr){
	/* set baud rate 103 for 9600baud*/
//	UBRR0H = (unsigned char)(ubrr>>8);
//	UBRR0L = (unsigned char)ubrr;
	/*enable receiver and transmitter, interrupts*/
//	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
	/* UMSEL = 0 - asyn */
	/*set frame format 8bit None 1 stopbit - default*/
//	UCSR0C = (0<<USBS0)|(1<<UCSZ01)|(1<<UCSZ00)|(0<<UPM00);
// }


void uartSendChar(unsigned char c)
{
    // Wait until able to send
    while (!(UCSR0A & (1 << UDRE0)))
    {
        ;
    }
    
    // Send character
    UDR0 = c;
}

void uartSendString (char *c)
{
    // While *s != '\0'
    while (*c)
    {
        uartSendChar(*c);
        c++;
    }
}

int uartPutChar(char c, FILE *stream)
{
    if (c == '\n')
    {
        uartPutChar('\r', stream);
    }
    
    // Wait until able to send
    while (!(UCSR0A & (1 << UDRE0)))
    {
        ;
    }
    
    UDR0 = c;
    
    return 0;
}

unsigned char usart_receive( void ){
	while ( !(UCSR0A & (1<<RXC0)) );
	return UDR0;
}

// cleaning USART buffer
void USART_flush(void){
	unsigned char dummy;
	while (UCSR0A & (1<<RXC0)) dummy = UDR0;
	dummy=dummy;
}
