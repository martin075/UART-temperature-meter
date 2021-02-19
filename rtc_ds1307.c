// ---------------------------------------------------------------------------
// I2C (TWI) ROUTINES for DS1307
// On the AVRmega series, PA4 is the data line (SDA) and PA5 is the clock (SCL
// The standard clock rate is 100 KHz, and set by I2C_Init. It depends on the AVR osc. freq.
#include <avr/io.h>
#include <stdio.h>

#include <util/delay.h> 
#include <avr/pgmspace.h>

#include <util/twi.h>
#include "i2c.h"


#define TimeFormat12			0x40	/* Define 12 hour format */
#define AMPM				0x20
#define DS1307_SECONDS  0x00
#define DS1307_CH	(1<<7)
#define DS1307_R 0xD1	
#define DS1307_W 0xD0

// at 16 MHz, the SCL frequency will be 16/(16+2(TWBR)), assuming prescalar of 0.
// so for 100KHz SCL, TWBR = ((F_CPU/F_SCL)-16)/2 = ((16/0.1)-16)/2 = 144/2 = 72.


uint8_t i2crtc_start(char write_address)					/* I2C start function */
{
	uint8_t status;									/* Declare variable */
	//TWCR = (1<<TWSTA)|(1<<TWEN)|(1<<TWINT);		/* Enable TWI, generate start condition and clear interrupt flag */
	i2cStart();
	//while (!(TWCR & (1<<TWINT)));					/* Wait until TWI finish its current job (start condition) */
	i2cWaitForComplete();
	
	//status = TWSR & 0xF8;							/* Read TWI status register with masking lower three bits */
	//if (status != 0x08)								/* Check weather start condition transmitted successfully or not? */
	//	return 0;									/* If not then return 0 to indicate start condition fail */
	i2cCheckReturnCodeInv(0x08);
	
	//TWDR = write_address;							/* If yes then write SLA+W in TWI data register */
	//TWCR = (1<<TWEN)|(1<<TWINT);					/* Enable TWI and clear interrupt flag */
	i2cSendByte(write_address);
	
	//while (!(TWCR & (1<<TWINT)));					/* Wait until TWI finish its current job (Write operation) */
	i2cWaitForComplete();
	
	status = TWSR & 0xF8;							/* Read TWI status register with masking lower three bits */	
	if (status == 0x18)								/* Check weather SLA+W transmitted & ack received or not? */
		return 1;									/* If yes then return 1 to indicate ack received i.e. ready to accept data byte */
	if (status == 0x20)								/* Check weather SLA+W transmitted & nack received or not? */
		return 2;									/* If yes then return 2 to indicate nack received i.e. device is busy */
	else
	return 3;										/* Else return 3 to indicate SLA+W failed */
}


uint8_t i2crtc_repeated_start(char read_address)				/* I2C repeated start function */
{
	uint8_t status;											/* Declare variable */
	TWCR = (1<<TWSTA)|(1<<TWEN)|(1<<TWINT);					/* Enable TWI, generate start condition and clear interrupt flag */
	while (!(TWCR & (1<<TWINT)));							/* Wait until TWI finish its current job (start condition) */
	status = TWSR & 0xF8;									/* Read TWI status register with masking lower three bits */
	if (status != 0x10)										/* Check weather repeated start condition transmitted successfully or not? */
	return 0;												/* If no then return 0 to indicate repeated start condition fail */
	TWDR = read_address;									/* If yes then write SLA+R in TWI data register */
	TWCR = (1<<TWEN)|(1<<TWINT);							/* Enable TWI and clear interrupt flag */
	while (!(TWCR & (1<<TWINT)));							/* Wait until TWI finish its current job (Write operation) */
	status = TWSR & 0xF8;									/* Read TWI status register with masking lower three bits */
	if (status == 0x40)										/* Check weather SLA+R transmitted & ack received or not? */
	return 1;												/* If yes then return 1 to indicate ack received */ 
	if (status == 0x20)										/* Check weather SLA+R transmitted & nack received or not? */
	return 2;												/* If yes then return 2 to indicate nack received i.e. device is busy */
	else
	return 3;												/* Else return 3 to indicate SLA+W failed */
}

void i2crtc_stop()												/* I2C stop function */
{
	TWCR=(1<<TWSTO)|(1<<TWINT)|(1<<TWEN);					/* Enable TWI, generate stop condition and clear interrupt flag */
	while(TWCR & (1<<TWSTO));								/* Wait until stop condition execution */ 
}

void i2crtc_start_wait(char write_address)						/* I2C start wait function */
{
	uint8_t status;											/* Declare variable */
	while (1)
	{
		TWCR = (1<<TWSTA)|(1<<TWEN)|(1<<TWINT);				/* Enable TWI, generate start condition and clear interrupt flag */
		while (!(TWCR & (1<<TWINT)));						/* Wait until TWI finish its current job (start condition) */
		status = TWSR & 0xF8;								/* Read TWI status register with masking lower three bits */
		if (status != 0x08)									/* Check weather start condition transmitted successfully or not? */
		continue;											/* If no then continue with start loop again */
		TWDR = write_address;								/* If yes then write SLA+W in TWI data register */
		TWCR = (1<<TWEN)|(1<<TWINT);						/* Enable TWI and clear interrupt flag */
		while (!(TWCR & (1<<TWINT)));						/* Wait until TWI finish its current job (Write operation) */
		status = TWSR & 0xF8;								/* Read TWI status register with masking lower three bits */
		if (status != 0x18 )								/* Check weather SLA+W transmitted & ack received or not? */
		{
			i2crtc_stop();										/* If not then generate stop condition */
			continue;										/* continue with start loop again */
		}
		break;												/* If yes then break loop */
	}
}

uint8_t i2crtc_write(char data)								/* I2C write function */
{
	uint8_t status;											/* Declare variable */
	TWDR = data;											/* Copy data in TWI data register */
	TWCR = (1<<TWEN)|(1<<TWINT);							/* Enable TWI and clear interrupt flag */
	while (!(TWCR & (1<<TWINT)));							/* Wait until TWI finish its current job (Write operation) */
	status = TWSR & 0xF8;									/* Read TWI status register with masking lower three bits */
	if (status == 0x28)										/* Check weather data transmitted & ack received or not? */
	return 0;												/* If yes then return 0 to indicate ack received */
	if (status == 0x30)										/* Check weather data transmitted & nack received or not? */
	return 1;												/* If yes then return 1 to indicate nack received */
	else
	return 2;												/* Else return 2 to indicate data transmission failed */
}

char i2crtc_read_ack()											/* I2C read ack function */
{
	TWCR=(1<<TWEN)|(1<<TWINT)|(1<<TWEA);					/* Enable TWI, generation of ack and clear interrupt flag */
	while (!(TWCR & (1<<TWINT)));							/* Wait until TWI finish its current job (read operation) */
	return TWDR;											/* Return received data */
}	

char i2crtc_read_nack()										/* I2C read nack function */
{
	TWCR=(1<<TWEN)|(1<<TWINT);								/* Enable TWI and clear interrupt flag */
	while (!(TWCR & (1<<TWINT)));							/* Wait until TWI finish its current job (read operation) */
	return TWDR;											/* Return received data */
}


// start oscillator RTC
//  At power-on, the CH bit of register 0x00 is set of 1
//  preventing the oscillator from running.  Therefore, at
//  init, we clear the CH bit

void _ds1307_start_oscillator()
{
	//	Clear the CH bit of the register at 0x00 to
	//	enable the oscillator
	i2crtc_start_wait(DS1307_W);
	i2crtc_write( DS1307_SECONDS );
	i2crtc_repeated_start(DS1307_R);
	
	// unsigned char temp = i2c_readNak();
	unsigned char temp;

	i2crtc_stop();

	temp &= 0b01111111;		//	clear the CH bit
	
	i2crtc_start_wait(DS1307_W);
	i2crtc_write( DS1307_SECONDS );
	i2crtc_write( temp );
	i2crtc_stop;
}

