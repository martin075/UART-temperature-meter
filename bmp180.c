/*
   This code is based on the BMP085 library by Stefan Sicklinger published on
   http://www.sicklinger.com
   
   I am redistributing this modified version under the GNU General Public
   License version 3
*/

#ifndef F_CPU
#define F_CPU 16000000L
#endif

#include <avr/io.h>
#include <util/delay.h> 
#include <util/twi.h>
// see macros like TW_START, TW_MT_DATA_ACK,...https://www.nongnu.org/avr-libc/user-manual/group__util__twi.html
#include <stdio.h>
#include <math.h>
#include "bmp180.h"
#include "i2c_n.h"

// Oversampling
#define OSS 3 // The oversampling setting works for 0, 1, 2, and 3

// Addresses
//     A7      A2 A1  W/R
// bin 1 1101 1 1 0/1 - 119dec
#define BMP085_R 0xEF
#define BMP085_W 0xEE

int16_t calibrationSigned[8];
int16_t calibrationUnsigned[3];

void bmpCalibrate(uint8_t* errorcode)
{
    if (*errorcode == 0)
    {
        calibrationSigned[0] = bmpReadShort(0xAA, errorcode); // ac1
    }
    
    if (*errorcode == 0)
    {
        calibrationSigned[1] = bmpReadShort(0xAC, errorcode); // ac2
    }
    
    if (*errorcode == 0)
    {
        calibrationSigned[2] = bmpReadShort(0xAE, errorcode); // ac3
    }
    
    if (*errorcode == 0)
    {
        calibrationUnsigned[0] = bmpReadShort(0xB0, errorcode); // ac4
    }
    
    if (*errorcode == 0)
    {
        calibrationUnsigned[1] = bmpReadShort(0xB2, errorcode); // ac5
    }
    
    if (*errorcode == 0)
    {
        calibrationUnsigned[2] = bmpReadShort(0xB4, errorcode); // ac6
    }
    
    if (*errorcode == 0)
    {
        calibrationSigned[3] = bmpReadShort(0xB6, errorcode); // b1
    }
    
    if (*errorcode == 0)
    {
        calibrationSigned[4] = bmpReadShort(0xB8, errorcode); // b2
    }
    
    if (*errorcode == 0)
    {
        calibrationSigned[5] = bmpReadShort(0xBA, errorcode); // mb
    }
    
    if (*errorcode == 0)
    {
        calibrationSigned[6] = bmpReadShort(0xBC, errorcode); // mc
    }
    
    if (*errorcode == 0)
    {
        calibrationSigned[7] = bmpReadShort(0xBE, errorcode); // md
    }
}

uint16_t bmpReadShort(uint8_t address, uint8_t* errorCode)
{
    uint16_t msb = 0;
    uint16_t lsb = 0;
    uint16_t data;
    
    // Send a start condition (set TWSTA)
    i2cStart();
    *errorCode = i2cWaitForComplete();
    
    if (*errorCode == 0)
    {
        *errorCode = i2cCheckReturnCode(TW_START);
    }
    //printf("Error code: %d\n", errorCode);

    // Tell slave to go in receive mode
    if (*errorCode == 0)
    {
        i2cSendByte(BMP085_W); // write 0xEE
        *errorCode = i2cWaitForComplete();
    }
    
    // ACKS
    if (*errorCode == 0)
    {
        *errorCode = i2cCheckReturnCode(TW_MT_SLA_ACK);
    }
    //printf("Error code: %d\n", errorCode);
    
    // Send read address to slave
    if (*errorCode == 0)
    {
        i2cSendByte(address); // write register address
        *errorCode = i2cWaitForComplete();
    }
    // ACKS
    if (*errorCode == 0)
    {
        *errorCode = i2cCheckReturnCode(TW_MT_DATA_ACK);
    }
    //printf("Error code: %d\n", errorCode);
    
    // Send Restart to Slave
    if (*errorCode == 0)
    {
        i2cStart();
        *errorCode = i2cWaitForComplete();
    }
    
    if (*errorCode == 0)
    {
        *errorCode = i2cCheckReturnCode(TW_REP_START);
    }
    //printf("Error code: %d\n", errorCode);

    // Tell slave to go in send mode
    if (*errorCode == 0)
    {
        i2cSendByte(BMP085_R); // read 0xEF
        *errorCode = i2cWaitForComplete();
    }
    
    // ACKS
    if (*errorCode == 0)
    {
        *errorCode = i2cCheckReturnCode(TW_MR_SLA_ACK);
    }
    //printf("Error code: %d\n", errorCode);

    //Get MSB from Slave
    if (*errorCode == 0)
    {
        i2cReadByteACK();
        *errorCode=i2cWaitForComplete();
    }
    //printf("Error code: %d\n", errorCode);
    
    if (*errorCode == 0)
    {
        msb = i2cGetReceivedByte(); // Get MSB result
        *errorCode = i2cWaitForComplete();
    }
    
    // ACKM
    if (*errorCode == 0)
    {
        *errorCode = i2cCheckReturnCode(TW_MR_DATA_ACK);
    }
    //printf("Error code: %d\n", errorCode);
    
    //Get LSB from Slave
    if (*errorCode == 0)
    {
        i2cReadByteNOACK();
        *errorCode = i2cWaitForComplete();
    }
    //printf("Error code: %d\n", errorCode);
    
    if (*errorCode == 0)
    {
        lsb = i2cGetReceivedByte(); // Get LSB result
        *errorCode = i2cWaitForComplete();
    }
    
    // NACKM
    if (*errorCode == 0)
    {
        *errorCode = i2cCheckReturnCode(TW_MR_DATA_NACK);
    }
    //printf("Error code: %d\n", errorCode);
    //printf("TWSR STATUS: %d\n", (TWSR & 0xF8));
    
    // Send stop on the bus
    if (*errorCode == 0)
    {
        *errorCode = i2cStop();
    }
    //printf("Error code: %d\n", errorCode);
    
    data = (msb << 8) + lsb;
    
    return data;
}

int32_t bmpReadUT(uint8_t* errorCode)
{
    int32_t temperature = 0;
    
    // Send a start condition
    i2cStart();
    *errorCode = i2cWaitForComplete();
    
    if (*errorCode == 0)
    {
        *errorCode = i2cCheckReturnCode(TW_START);
    }
        
    // Tell slave to go in receive mode
    if (*errorCode == 0)
    {
        i2cSendByte(BMP085_W); // Write 0xEE
        *errorCode = i2cWaitForComplete();
    }
    
    // ACKS
    if (*errorCode == 0)
    {
        *errorCode = i2cCheckReturnCode(TW_MT_SLA_ACK);
    }
    //printf("Error code: %d\n", errorCode);
    
    //Write control register address
    if (*errorCode == 0)
    {
        i2cSendByte(0xF4); // write 0xF4
        *errorCode=i2cWaitForComplete();
    }
    
    // ACKS
    if (*errorCode == 0)
    {
        *errorCode=i2cCheckReturnCode(TW_MT_DATA_ACK);
    }
    //printf("Error code: %d\n", errorCode);
    //printf("TWSR STATUS: %d\n", (TWSR & 0xF8));
    
    // Write control register value
    if (*errorCode == 0)
    {
        i2cSendByte(0x2E); // write 0x2E
        *errorCode=i2cWaitForComplete();
    }
    // ACKS
    if (*errorCode == 0)
    {
        *errorCode=i2cCheckReturnCode(TW_MT_DATA_ACK);
    }
    //printf("Error code: %d\n", errorCode);
    
    // Send a stop condition
    if (*errorCode == 0)
    {
        *errorCode=i2cStop();
    }
    //printf("Error code: %d\n", errorCode);
    
    _delay_ms(5); // Minimum time is 4.5 ms
    
    if (*errorCode == 0)
    {
        temperature = bmpReadShort(0xF6,errorCode);
    }
    
    return temperature;
}

int32_t bmpReadUP(uint8_t* errorCode)
{
    int32_t pressure = 0;
    
    // Send a start condition
    i2cStart();
    *errorCode=i2cWaitForComplete();
    
    if (*errorCode == 0)
    {
        *errorCode = i2cCheckReturnCode(TW_START);
    }
    //printf("Error code: %d\n", errorCode);
    
    // Tell slave to go in receive mode
    if (*errorCode == 0)
    {
        i2cSendByte(BMP085_W); // write 0xEE
        *errorCode = i2cWaitForComplete();
    }
    
    // ACKS
    if (*errorCode == 0)
    {
        *errorCode = i2cCheckReturnCode(TW_MT_SLA_ACK);
    }
    //printf("Error code: %d\n", errorCode);
    
    // Write control register address
    if (*errorCode == 0)
    {
        i2cSendByte(0xF4); // write 0xF4
        *errorCode = i2cWaitForComplete();
    }
    
    // ACKS
    if (*errorCode == 0)
    {
        *errorCode = i2cCheckReturnCode(TW_MT_DATA_ACK);
    }
    //printf("Error code: %d\n", errorCode);
    //printf("TWSR STATUS: %d \n", (TWSR & 0xF8));
    
    // Write control register value
    if (*errorCode == 0)
    {
        i2cSendByte(0x34 + (OSS << 6)); // write 0x34 + (OSS << 6)
        *errorCode = i2cWaitForComplete();
    }
    
    // ACKS
    if (*errorCode == 0)
    {
        *errorCode = i2cCheckReturnCode(TW_MT_DATA_ACK);
    }
    //printf("Error code: %d\n", errorCode);
    
    // Send a stop condition
    if (*errorCode == 0)
    {
        *errorCode = i2cStop();
    }
    //printf("Error code: %d\n", errorCode);
    
    //_delay_ms(16); // Minimum time is 15 ms
    _delay_ms(2 + (3 << OSS));
    
    if (*errorCode == 0)
    {
        pressure = bmpReadShort(0xF6, errorCode);
        pressure = pressure << 8;
        pressure = pressure >> (8 - OSS);
    }
    
    return pressure;
}

void bmpComputePressureAndTemperature(int32_t* temperature, int32_t* pressure,
                                      uint8_t* errorCode)
{
    int32_t ut = 0;
    int32_t up = 0;
    int32_t x1, x2, b5, b6, x3, b3, p;
    uint32_t b4, b7;
    int16_t ac1 = calibrationSigned[0];
    int16_t ac2 = calibrationSigned[1]; 
    int16_t ac3 = calibrationSigned[2]; 
    uint16_t ac4 = calibrationUnsigned[0];
    uint16_t ac5 = calibrationUnsigned[1];
    uint16_t ac6 = calibrationUnsigned[2];
    int16_t b1 = calibrationSigned[3]; 
    int16_t b2 = calibrationSigned[4];
    //int16_t mb = calibrationSigned[5];
    int16_t mc = calibrationSigned[6];
    int16_t md = calibrationSigned[7];
    
    if (*errorCode == 0)
    {
        ut = bmpReadUT(errorCode);
    }
    
    if (*errorCode == 0)
    {
        up = bmpReadUP(errorCode);
    }
    
    x1 = ((int32_t) ut - (int32_t) ac6) * (int32_t) ac5 >> 15;
    x2 = ((int32_t) mc << 11) / (x1 + md);
    b5 = x1 + x2;
    *temperature = (b5 + 8) >> 4;
    
    b6 = b5 - 4000;
    x1 = (b2 * ((b6 * b6) >> 12)) >> 11;
    x2 = (ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((((int32_t) ac1) * 4 + x3) << OSS) + 2) >> 2;
    x1 = (ac3 * b6) >> 13;
    x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
    b7 = ((uint32_t) (up - b3) * (50000 >> OSS));
    //p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
    
    if (b7 < 0x80000000)
    {
        p = (b7 << 1) / b4;
    }
    
    else
    { 
        p = (b7 / b4) << 1;
    }
    
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    *pressure = p + ((x1 + x2 + 3791) >> 4);
}

int32_t bmpComputeAltitude(int32_t pressure)
{
    float temp;
    int32_t altitude;
    
    temp = (float) pressure / 101325;
    temp = 1 - pow(temp, 0.19029);
    altitude = round(44330 * temp * 10);
    
    // Return altitude in dm
    return altitude;
}
