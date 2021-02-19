/*
pcf8574 

*/


#define PCF8574_ADDRBASE (0x3F) //device base address 111 111

#define PCF8574_I2CINIT 1 //init i2c bus

#define PCF8574_MAXDEVICES 8 //max devices, depends on address (3 bit)
#define PCF8574_MAXPINS 8 //max pin per device


//pin status
volatile uint8_t pcf8574_pinstatus[PCF8574_MAXDEVICES];


//functions
void pcf8574_init();
extern int8_t pcf8574_getoutput(uint8_t deviceid);
extern int8_t pcf8574_getoutputpin(uint8_t deviceid, uint8_t pin);
extern int8_t pcf8574_setoutput(uint8_t deviceid, uint8_t data);
extern int8_t pcf8574_setoutputpins(uint8_t deviceid, uint8_t pinstart, uint8_t pinlength, int8_t data);
extern int8_t pcf8574_setoutputpin(uint8_t deviceid, uint8_t pin, uint8_t data);
extern int8_t pcf8574_setoutputpinhigh(uint8_t deviceid, uint8_t pin);
extern int8_t pcf8574_setoutputpinlow(uint8_t deviceid, uint8_t pin);
extern int8_t pcf8574_getinput(uint8_t deviceid);
extern int8_t pcf8574_getinputpin(uint8_t deviceid, uint8_t pin);

