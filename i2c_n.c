/*
   This code is based on the BMP085 library by Stefan Sicklinger 
*/

#ifndef F_CPU
#define F_CPU 16000000L
#endif

#include <avr/io.h>
#include <util/delay.h> 
#include <util/twi.h>
// see macros like TW_START, TW_MT_DATA_ACK,...https://www.nongnu.org/avr-libc/user-manual/group__util__twi.html
#include "i2c_n.h"

void i2cSetup()
{    
    // Set the Serial Clock Line prescalar
    //
    // TWPS1    TWPS0   Prescalar Value
    //     0        0                 1
    //     0        1                 4
    //     1        0                16
    //     1        1                64
    TWSR = ((0 << TWPS1) | (0 << TWPS0));
    
    // Set the frequency of the Serial Clock Line to 200 kHz
    // FCPU - 8MHz - 100KHz SCL, 16MHz - 200kHz
    // F_SCL = F_CPU / (16 + 2 * TWBR * (Prescalar Value))
	// TWBR = ((F_CPU/F_SCL)-16)/2 = ((16/0.1)-16)/2 = 144/2 = 72 - 100kHz at 16MHz
	// TWBR = ((F_CPU/F_SCL)-16)/2 = ((8/0.1)-16)/2 = 144/2 = 32 - 100kHz at 8 MHz
	    
    TWBR = 32;	// 100 kHz
}

void i2cStart(void)
{
    // Send start condition
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
}

uint8_t i2cStop(void)
{
    uint8_t i = 0; // Time out variable
    uint8_t errorCode = 0;
    
    // Transmit stop condition
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);

    // Wait until stop condition is executed and bus released
    while ((TWCR & (1 << TWSTO)) && (i < 90))
		{
			_delay_us(1);
			i++;
		}
    
    if (i > 88)
    {
        _delay_us(1);
        errorCode = 10;
    }
    
    return errorCode;
}

// check value of TWI Status Register. Mask prescaler bits.
// TW_STATUS from twi.h

int i2c_twst_rep(void)
{	uint8_t   twst;
// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_START) && (twst != TW_REP_START)) return 1; //non-zero indicates that so form of error 	
	return 0;	// normal termination
}

int i2c_twst_sla_ack(void)
{	uint8_t   twst;
// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;	
	return 0;
}

int i2c_twst_data_ack(void)
{	uint8_t   twst;
	twst = TW_STATUS & 0xF8;
	if( twst != TW_MT_DATA_ACK) return 1;
	
	return 0;
}

void i2cSendByte(uint8_t data)
{
    // Save data to the TWDR
    TWDR = data;
    
    // Begin send
    TWCR = (1 << TWINT) | (1 << TWEN);
}

void i2cReadByteACK(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	while(!(TWCR & (1<<TWINT)));

}

void i2cReadByteNOACK(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN);
	while(!(TWCR & (1<<TWINT)));

}

uint8_t i2cWaitForComplete(void)
{
    uint8_t i = 0; // Time out variable
    uint8_t errorCode = 0;

    // Wait for the i2c interface to complete the operation
    while ((!(TWCR & (1 << TWINT))) && (i < 90))
    {
        _delay_us(1);
        i++;
    }
    
    if (i > 89)
    {
        _delay_us(1);
        errorCode = 100;
    }
    
    return errorCode;
}

uint8_t i2cGetReceivedByte(void)
{
    return (uint8_t) TWDR;
}

uint8_t i2cCheckReturnCode(uint8_t expectedErrorCode)
{
    uint8_t errorCode = 0; // return 0 - like break command
    
    if ((TWSR & 0xF8) != expectedErrorCode)
    {
        errorCode = 10;
    }
        
    return errorCode;	// return 0 - break, return 10 next commands
}

uint8_t i2cCheckReturnCodeInv(uint8_t expectedErrorCode)
{
    uint8_t errorCode = 10; // return 0 - like break command
    
    if ((TWSR & 0xF8) != expectedErrorCode)
    {
        errorCode = 0;
    }
        
    return errorCode;	// return 0 - break, return 10 next commands
}
