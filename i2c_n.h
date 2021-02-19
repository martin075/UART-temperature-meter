/*
   This code is based on the BMP085 library by Stefan Sicklinger 
*/
/** defines the data direction (reading from I2C device) in i2c_start(),i2c_rep_start() */
#define I2C_READ    1

/** defines the data direction (writing to I2C device) in i2c_start(),i2c_rep_start() */
#define I2C_WRITE   0


void i2cSetup();
  /* Sets pullup and initializes bus speed  */

void i2cStart(void);
  /* Sends a start condition (sets TWSTA) */

uint8_t i2cStop(void);
  /* Sends a stop condition (sets TWSTO) */
// check value of TWI Status Register. Mask prescaler bits.
// TW_STATUS from twi.h 
int i2c_twst_rep(void);
int i2c_twst_sla_ack(void);
int i2c_twst_data_ack(void);
 

void i2cSendByte(uint8_t data);
     /* Sends a data byte */

void i2cReadByteACK(void);
     /* Reads a data byte and sends ACK when done (sets TWEA) */

void i2cReadByteNOACK(void);
     /* Reads a data byte and sends NOACK when done (does not set TWEA) */

uint8_t i2cGetReceivedByte(void);
     /* Returns the received data byte form the TWDR register */

uint8_t i2cWaitForComplete(void);
     /* Waits until the hardware sets the TWINT flag */

uint8_t i2cCheckReturnCode(uint8_t expectedErrorCode);
     /* Compares the i2c error code with an expected error code */


