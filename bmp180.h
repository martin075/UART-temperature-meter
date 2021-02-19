/*
   This code is based on the BMP085 library by Stefan Sicklinger published on
   http://www.sicklinger.com
   
   I am redistributing this modified version under the GNU General Public
   License version 3
*/

#ifndef BMP_H_INCLUDED
#define BMP_H_INCLUDED

void bmpCalibrate(uint8_t* errorcode);
       /* Loads the calibration constants from the EEPROM on the BMP sensor */

uint16_t bmpReadShort(uint8_t address, uint8_t* errorcode);
                                                 /* Returns 16 bits of data */

int32_t bmpReadUT(uint8_t* errorCode);
                                   /* Returns a raw temperature measurement */

int32_t bmpReadUP(uint8_t* errorcode);
                                      /* Returns a raw pressure measurement */

void bmpComputePressureAndTemperature(int32_t* temperature, int32_t* pressure,
                                      uint8_t* errorCode);
                 /* Computes human-readable temperature and pressure values */

int32_t bmpComputeAltitude(int32_t pressure);
                               /* Computes altitude with an empirical model */

#endif
