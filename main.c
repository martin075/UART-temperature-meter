/*Platform:    ATmega328P, AVR 8-bit , 8MHz
 * ---------------------------------------------------------------
 * Description: 
 *	LCD		- I2C 0x27,0xA7
 *	tm1637 	- PD6,PD7
 *	RTC1307	- I2C 0x68,0xE8, CH bit in first use - REG 0 - bit7 must be set to 0
 *	BMP		- I2C 0x77,0xF7
 *	TIME/counter - LED light for LCD
*/

/* Includes ------------------------------------------------------- */
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#include "bmp180.h"
#include "uart.h"
#include "i2c_n.h"
#include "rtcds1307.h"
#include "lcdpcf8574/lcdpcf8574.h"	//i2c addr 3F 
#include "pcf8574_1.h"
#include "ds18b20.h"


// address DS1307 7-bit 1101000 R/W - 0x68
#define DS1307_R 0xD1	
#define DS1307_W 0xD0	//8-bit address
#define MAX_STRLEN 10
//for i2c bus
#define MAXDEVICEID 7

//global var
uint8_t second,minute,hour,day,date,month,year,rtc_present=1;
unsigned char chyba1=0,chyba2=0,chyba3=0,chyba4=0,chyba5=0,chyba6=0;
unsigned char ErrFlag=0;
uint8_t Device1Presence=1,Device2Presence=1,Device3Presence=1,Device4Presence=1,Device5Presence=1,Device6Presence=1;
int CurrentTemp1=0,CurrentTemp2=0,CurrentTemp3=0,CurrentTemp4=0,CurrentTemp5=0,CurrentTemp6=0;

uint8_t led = 0; //display ON
volatile uint8_t tick=0,once=1;

volatile char nastavenie[MAX_STRLEN];	//pouzite v v main
volatile char nova_sprava=0;

//  Function prototypes 
void rtc_read_clock(char read_clock_address);
void rtc_read_cal(char read_calendar_address);
void rtc_clock_write(char set_hour, char set_minute);
void rtc_hour_w(uint8_t set_hour);
void rtc_minute_w(uint8_t set_min);
void rtc_dow_w(uint8_t set_dow);
void rtc_year_w(uint8_t set_year);
void rtc_day_w(uint8_t set_date);
void rtc_month_w(uint8_t set_month);
void rtc_calendar_write(char set_date, char set_month, char set_year);
void ds18b20ports(void);
void ds18b20presence(void);
void ds18b20readtemperature(int snimac);
void ds18b20printuart(void);
void timer_led();
void timer_tick();
int porovnaj_retazec(char *str1, char *str2, int pocet_znakov);


ISR (TIMER1_OVF_vect){
	// citac/casovac1 - 16bit
  	// nastavenie poèiatoènej hodnoty poèítadla
  //TCNT1 = 34286;
  	//TCNT1 = 3036; // 8sekund, pre delicku 1024
  	TCNT1 = 57724;	// 1 sekunda pre 1024
 	//TCNT1 = 49910; //2 sec
	tick++;
	once = 1;
	// 	preteèenie registra TCNT1
	// 	preddelièku èítaèa/èasovaèa1 nastav na 256 (1/8MHz=0.125->0.125*256=32us)
	// 	1s/32us = 31250 impulzov,65536 – 31250 = 34286  
	//	1s/128us = 7812 impulzov, 65536 – 7812 = 57723
	//	2s/128us = 15625 imp, 65536 - 15625 = 49910
	//	5s/0.000128 = 39062, 65536 – 39062 = 26473 
	//	6s/128us =  46875,	65536 – 46875 = 18661
	//	8s/128us = 62500, 	65536 –62500 = 3036
	//	PORTD ^= (1 << PD7); //neguj PD7  
}  

ISR(USART_RX_vect){
	static int cnt=0,i;
	char prijaty_znak;
	volatile char prijem[MAX_STRLEN];	//docasna premenna
	prijaty_znak = UDR0;
	
	//if(znak == 'T' ){zac=1;cnt=0;}
	if( (prijaty_znak != '\n') && ( prijaty_znak != '\r') && (cnt<MAX_STRLEN-2) ){prijem[cnt]=prijaty_znak;cnt++;}
		else {
			// koniec spravy osetrit znakom \0
			prijem[cnt]= '\0';
			for(i=0;i<MAX_STRLEN;i++)
				nastavenie[i]=prijem[i];
			//strncpy(nastavenie,prijem,MAX_STRLEN); 
			nova_sprava=1;
			cnt=0;
		}

}
//-----------------------------------------------------------
int main(void)
{ 
	int32_t temperature = 0; 
	int32_t pressure = 0;
    uint8_t errorCode = 0,x;
  
    char *days[7]= {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
	//uint8_t i=0;
	
	char CharBuffer[21];
	//unsigned char c_ROM_code;


// init section------------
	//sei();
	uartSetup();
	i2cSetup();
	bmpCalibrate(&errorCode); 
	ds18b20ports();
	
	bmpComputePressureAndTemperature(&temperature, &pressure, &errorCode);
	
	printf("Global error code: %d,rtc %d\n", errorCode,rtc_present);
	printf("Temperature: %ld °C (in deciCelsius)\n", temperature);
    printf("Pressure: %ld Pa\n\n", pressure);
	
	ds18b20presence();
	ds18b20readtemperature(1);
	ds18b20readtemperature(2);
	ds18b20readtemperature(3);
	ds18b20readtemperature(4);
	ds18b20readtemperature(5);
	ds18b20readtemperature(6);
	ds18b20printuart();

	//init lcd
    lcd_init(LCD_DISP_ON);
    //lcd go home
    lcd_home();
	lcd_led(led); //set led
	
	lcd_clrscr();
    lcd_gotoxy( 0, 0);	//column stlpec, row  riadok
	lcd_puts_P( "lcd i2c RTC BMP");
	sprintf( CharBuffer, "9600_8_N_1");
	lcd_gotoxy( 0, 1);
	lcd_puts(CharBuffer);
	if(rtc_present==1){
			rtc_read_clock(0);	//read hours,min,sec.
			printf("cas: %02x:%02x:%x \n", (hour & 0b00011111),minute,second); //format hex
			rtc_read_cal(3);	// read date
			printf("%02x/%02x/%02x %3s\n",date,month,year,days[day]);
			}
		sprintf( CharBuffer, "%02x:%02x %02x/%02x/20%02x",(hour & 0b00011111),minute,date,month,year);
		lcd_gotoxy( 0, 3);	//column, row
		lcd_puts(CharBuffer);
	_delay_ms(2000);

	lcd_clrscr();
	lcd_gotoxy( 0, 0);	//column, row
	sprintf( CharBuffer, "Ti:%ld C P:%ld Pa",temperature,pressure);
	lcd_puts(CharBuffer);
	
		lcd_gotoxy( 0, 1);
		sprintf( CharBuffer, "T1:%3d C   T2:%3d C",CurrentTemp1,CurrentTemp2);
		lcd_puts(CharBuffer);
		lcd_gotoxy( 0, 2);
		sprintf( CharBuffer, "T3:%3d C   T4:%3d C",CurrentTemp3,CurrentTemp4);
		lcd_puts(CharBuffer);
		lcd_gotoxy( 0, 3);
		sprintf( CharBuffer, "T5:%3d C   T6:%3d C",CurrentTemp5,CurrentTemp6);
		lcd_puts(CharBuffer);
	timer_tick();
	_delay_ms(100);
	
	

while(1)
 {
 
  switch(tick)
	{
	// vypis tlaku a teploty BMP180 - displej
	case 1:{if(once == 1){printf("error code: %d\n", errorCode);
				lcd_gotoxy( 0, 0);	//column, row
				sprintf( CharBuffer, "Ti:%ld C P:%ld Pa",temperature,pressure);
				lcd_puts(CharBuffer);//printf("tick time: %d\n", tick);
				once = 0;}
			} break;
	case 2:{if(once == 1){led = led;once = 0;printf("led status: %d %d\n", led,tick); } }break;
	// zhasnutie displeja
	case 3:{if(once == 1){led = led;once = 0;printf("led status: %d %d\n", led,tick); } }break;
	case 4:{if(once == 1){led = led;once = 0;printf("led status: %d %d\n", led,tick); } }break;
	// vycitavanie snimacov DS18b20
	case 5:{if(once == 1)ds18b20readtemperature(1); once = 0;} break;
	case 6:{if(once == 1){led = led;once = 0;printf("led status: %d %d\n", led,tick); } }break;
	//case 8:{if(once == 1)once = 0;} break;
	case 7:{if(once == 1)ds18b20readtemperature(2); once = 0;} break;
	//case 10:{if(once == 1)once = 0;} break;
	case 9:{if(once == 1)ds18b20readtemperature(3); once = 0;} break;
	case 11:{if(once == 1)ds18b20readtemperature(4); once = 0;} break;
	case 12:{if(once == 1){led = led;once = 0;} } break;
	// zhasnutie/rozsvietenie displeja
	case 13:{if(once == 1){led = !led;lcd_led(led);once = 0; printf("led status: %d %d\n", led,tick); } }break;
	case 14:{if(once == 1){led = led;once = 0;printf("led status: %d %d\n", led,tick); } }break;
	// vypis casu
	case 15:{if(once == 1){
				if(rtc_present==1){
					printf("Global error code: %d,rtc %d\n", errorCode,rtc_present);
					rtc_read_clock(0);	//read hours,min,sec.
					printf("cas: %02x:%02x:%x \n", (hour & 0b00011111),minute,second); //format hex
					rtc_read_cal(3);	// read date
					printf("%02x/%02x/%02x %3s\n",date,month,year,days[day]);
					//sprintf( CharBuffer, "%2x:%2x",(hour & 0b00011111),minute);
					sprintf( CharBuffer, "cas%02x:%02x %02x/%02x/20%02x",(hour & 0b00011111),minute,date,month,year);
					lcd_gotoxy( 0, 0);	//column, row
					lcd_puts(CharBuffer);//printf("tick time: %d\n", tick);
					once = 0;}}}	break;
	case 16:{if(once == 1){led = led;once = 0;printf("led status: %d %d\n", led,tick); } }break;
	case 17:{if(once == 1)ds18b20readtemperature(5); once = 0;} break;
	case 19:{if(once == 1)ds18b20readtemperature(6); once = 0;} break;
	//case 18:{if(once == 1)printf("tick time: %d\n", tick);once = 0;} break;
	// vypis teploty - UART
	case 21: {if(once == 1){ds18b20printuart();} once =0;}break;
	// zhasnutie/rozsvietenie displeja
	case 22:{if(once == 1){led = led;once = 0;printf("led status: %d %d\n", led,tick); } }break;
	case 23:{if(once == 1){led = !led;lcd_led(led);once = 0; printf("led status: %d %d\n", led,tick); } }break; 
	case 24:{if(once == 1){led = led;once = 0;printf("led status: %d %d\n", led,tick); } }break;
	// vypocet a vypis tlak a teplota BMP - UART
	case 25:{if(once == 1){bmpComputePressureAndTemperature(&temperature, &pressure, &errorCode);
				    printf("Temperature: %ld°C(in deciCelsius)\nPressure: %ld Pa\nAltitude: %ld dm\n", temperature,pressure,bmpComputeAltitude(pressure));}
					once = 0;
					} break;
	case 28:{if(once == 1){led = led;once = 0;printf("led status: %d %d\n", led,tick); } }break;
	// vypis teplota DS18b20 displej
	case 29:{if(once == 1){
				lcd_gotoxy( 0, 1);
				sprintf( CharBuffer, "T1:%3d C   T2:%3d C",CurrentTemp1,CurrentTemp2);
				lcd_puts(CharBuffer);
				lcd_gotoxy( 0, 2);
				sprintf( CharBuffer, "T3:%3d C   T4:%3d C",CurrentTemp3,CurrentTemp4);
				lcd_puts(CharBuffer);
				lcd_gotoxy( 0, 3);
				sprintf( CharBuffer, "T5:%3d C   T6:%3d C",CurrentTemp5,CurrentTemp6);
				lcd_puts(CharBuffer);//printf("tick time: %d\n", tick);
				once =0;
				} 
		 }break;

	case 30:{tick = 0;} break;
	default :{once = once;}break;
	}
	
	if(nova_sprava){
		if(porovnaj_retazec(nastavenie,"Timeh",5)==0){
		//if(strncmp(nastavenie,"Timehm",MAX_STRLEN)==0){
			if(sscanf_P(nastavenie,PSTR("Timeh%d"),&x)){
				if((x >= 0) && (x <= 24)){rtc_hour_w(x);printf("Nastavujem hodiny:%d\n\r",x);}	// posli odpoved	
					else printf("nespravny tvar\r\n");
					x=0;
					}
				} 
		else if(porovnaj_retazec(nastavenie,"Timem",5)==0){
				if(sscanf_P(nastavenie,PSTR("Timem%d"),&x)){
					if((x >= 0) && (x <= 59)){rtc_minute_w(x);printf("Nastavujem minuty:%d\n\r",x);}
						else printf("nespravny tvar\r\n");
						x=0;
					}
				}
		else if(strncmp(nastavenie,"DateW",5)==0){
				if(sscanf_P(nastavenie,PSTR("DateW%d"),&x)){
					if((x >= 1) && (x <= 7)){rtc_dow_w(x);printf("Nastavujem den:%d\n\r",x);}
						else printf("nespravny tvar\r\n");
						x=0;
					}
			}
		else if(strncmp(nastavenie,"DateD",5)==0){
				if(sscanf_P(nastavenie,PSTR("DateD%d"),&x)){
					if((x >= 1) && (x <= 31)){rtc_day_w(x);printf("Nastavujem datum:%d\n\r",x);}
						else printf("nespravny tvar\r\n");
						x=0;
					}
			}
		else if(strncmp(nastavenie,"DateM",5)==0){
			if(sscanf_P(nastavenie,PSTR("DateM%d"),&x)){
				if((x >= 1) && (x <= 12)){rtc_month_w(x);printf("Nastavujem mesiac:%d\n\r",x);}
					else printf("nespravny tvar\r\n");
						x=0;
					}
			}
		else if(strncmp(nastavenie,"DateY",5)==0){
			if(sscanf_P(nastavenie,PSTR("DateY%d"),&x)){
				if((x >= 0) && (x <= 99)){rtc_year_w(x);printf("Nastavujem rok:%d\n\r",x);}
					else printf("nespravny tvar\r\n");
						x=0;
					}
			}
		else if(strncmp(nastavenie,"help",4)==0){
			printf("help:Timeh,Timem,DateD,DateW,DateM,DateY, Year max2099, Time02 - 2 hours, Timem45 - 45 minutes\n\r");}
		else printf("neznamy prikaz\r\n");
		
		nova_sprava=0;
		}
	
	
 }	// end of while

} //end of main

//-------------------------------------------------------------------
int porovnaj_retazec(char *str1, char *str2, int pocet_znakov)
{	int i,rovnaky=0;
	
	for(i=0;i<pocet_znakov;i++){
		if(*str1++ == *str2++)rovnaky=0;
			else rovnaky = 1;
	}

return(rovnaky) ;
}

void timer_led()
{	
	//TCCR1B |= (1 << CS12);  //preddelièka 256 (32us)
	TCCR1B |= (1 << CS12) | (1 << CS10); //preddelièka 1024 (128us)    
    TIMSK1 |= (1 << TOIE1);  // prerušenie pri preteèení TCNT1     
    OSCCAL = 0xA1;    // nastavenie kalibracneho bajtu interneho RC oscilatora
}


void timer_tick()
{
	TCCR1B |= (1 << CS12) | (1 << CS10); //preddelièka 1024 (128us)    
    TIMSK1 |= (1 << TOIE1);  // prerušenie pri preteèení TCNT1     
    OSCCAL = 0xA1;    // nastavenie kalibracneho bajtu interneho RC oscilatora
}



void rtc_read_clock(char read_clock_address)
{
	rtc_present=i2crtc_start(DS1307_W);		/* Start I2C communication with RTC */
	i2crtc_write(read_clock_address);		/* Write address to read */
	i2crtc_repeated_start(DS1307_R);		/* Repeated start with device read address */

	second = i2crtc_read_ack();				/* Read second */
	minute = i2crtc_read_ack();				/* Read minute */
	hour = i2crtc_read_nack();				/* Read hour with Nack */
	i2crtc_stop();							/* Stop i2C communication */
}


void rtc_read_cal(char read_calendar_address)
{
	i2crtc_start(DS1307_W);
	i2crtc_write(read_calendar_address);
	i2crtc_repeated_start(DS1307_R);

	day = i2crtc_read_ack();				/* Read day */ 
	date = i2crtc_read_ack();				/* Read date */
	month = i2crtc_read_ack();				/* Read month */
	year = i2crtc_read_nack(); 				/* Read the year with Nack */
	i2crtc_stop();							/* Stop i2C communication */
}

void rtc_clock_write(char set_hour, char set_minute)
{
	// you need convert input data - bcd decimal to hex 	
	char bcd_hour,bcd_minute;
	bcd_hour = (((set_hour/10) *16) + (set_hour % 10));
	bcd_minute = (((set_minute/10)*16) + (set_minute % 10));
		
	i2crtc_start(DS1307_W);			// Start I2C communication with RTC 
	i2crtc_write(1);				// Write on 0 location for second value, 1 for minute 
	//i2crtc_write(5);				// Write second value on 00 location 
	i2crtc_write(bcd_minute);		// Write minute value on 01 location 
	i2crtc_write(bcd_hour);			// Write hour value on 02 location 
	i2crtc_stop();					// Stop I2C communication 
}

void rtc_hour_w(uint8_t set_hour)
{
	char bcd_hour;
	bcd_hour = (((set_hour/10) *16) + (set_hour % 10));
			
	i2crtc_start(DS1307_W);			// Start I2C communication with RTC 
	i2crtc_write(2);				// Write on 0 location for second value, 1 for minute 
	//i2crtc_write(5);				// Write second value on 00 location 
	i2crtc_write(bcd_hour);			// Write hour value on 02 location 
	i2crtc_stop();
}

void rtc_minute_w(uint8_t set_min)
{
	char bcd_minute;
	bcd_minute = (((set_min/10)*16) + (set_min % 10));
			
	i2crtc_start(DS1307_W);			// Start I2C communication with RTC 
	i2crtc_write(1);				// Write on 0 location for second value, 1 for minute 
	
	i2crtc_write(bcd_minute);			// Write value on 01 location 
	i2crtc_stop();
}

void rtc_dow_w(uint8_t set_dow)
{
	char bcd_dow;
	bcd_dow = (((set_dow/10)*16) + (set_dow % 10));
			
	i2crtc_start(DS1307_W);			// Start I2C communication with RTC 
	i2crtc_write(3);				// Write on 3 location for second value, 3 day of week 
	
	i2crtc_write(bcd_dow);			// Write value on 01 location 
	i2crtc_stop();
}

void rtc_day_w(uint8_t set_date)
{
	char bcd_date;
	bcd_date = (((set_date/10) *16) + (set_date % 10));
	i2crtc_start(DS1307_W);			// Start I2C communication with RTC 
	i2crtc_write(4);				// Write on 3 location for day value 
	i2crtc_write(bcd_date);			// Write date value on 04 location 
	i2crtc_stop();					// Stop I2C communication 
}

void rtc_month_w(uint8_t set_month)
{
	char bcd_month;
	bcd_month = (((set_month/10) *16) + (set_month % 10));
	i2crtc_start(DS1307_W);			// Start I2C communication with RTC 
	i2crtc_write(5);				// Write on 5 location for  value 
	i2crtc_write(bcd_month);			// Write value on location 
	i2crtc_stop();					// Stop I2C communication 
}

void rtc_year_w(uint8_t set_year)
{
	char bcd_year;
	
	bcd_year = (((set_year/10) *16) + (set_year % 10));
	i2crtc_start(DS1307_W);			// Start I2C communication with RTC 
	i2crtc_write(6);				// Write on 3 location for day value 
	i2crtc_write(bcd_year);			// Write year value on 06 location 
	i2crtc_stop();					// Stop I2C communication 
}

//void rtc_calendar_write(char set_day, char set_date, char set_month, char set_year)	// function for calendar 
void rtc_calendar_write(char set_date, char set_month, char set_year)
{
	char bcd_date,bcd_month,bcd_year;
	//bcd_day = (((set_day/10) *16) + (set_day % 10));
	bcd_date = (((set_date/10) *16) + (set_date % 10));
	bcd_month = (((set_month/10) *16) + (set_month % 10));
	bcd_year = (((set_year/10) *16) + (set_year % 10));
	
	i2crtc_start(DS1307_W);			// Start I2C communication with RTC 
	i2crtc_write(4);				// Write on 3 location for day value 
	//i2crtc_write(bcd_day);			// Write day value on 03 location 
	i2crtc_write(bcd_date);			// Write date value on 04 location 
	i2crtc_write(bcd_month);		// Write month value on 05 location 
	i2crtc_write(bcd_year);			// Write year value on 06 location 
	i2crtc_stop();					// Stop I2C communication 
}

void ds18b20ports(void)
{
// Port C initialization DS18b20 //
	DDRC &= ~(1 << PC0);   // input
    PORTC &= ~(1 << PC0);   
	DDRC &= ~(1 << PC1);   
    PORTC &= ~(1 << PC1);   
	DDRC &= ~(1 << PC2);   
    PORTC &= ~(1 << PC2);   
	DDRC &= ~(1 << PC3);   
    PORTC &= ~(1 << PC3);
	DDRD &= ~(1 << PD3);   
    PORTD &= ~(1 << PD3);   
	DDRD &= ~(1 << PD4);   
    PORTD &= ~(1 << PD4);  
}

void ds18b20presence(void)
{
 	int j;
	Device1Presence = ds18b20_reset(&PORTC,PC0);
	if(Device1Presence)
			{chyba1=0;}
		else { chyba1=1;}
	_delay_ms(100);
	//---------------------------------
	Device2Presence = ds18b20_reset(&PORTC,PC1);
	if(Device2Presence)
		{chyba2=0;}
		else { chyba2=1;}
	_delay_ms(100);
	//---------------------------------
	Device3Presence = ds18b20_reset(&PORTC,PC2);
	if(Device3Presence)
			{chyba3=0;}
		else { chyba3=1;}
	_delay_ms(100);
	//---------------------------------
	Device4Presence = ds18b20_reset(&PORTC,PC3);
	if(Device4Presence)
			{chyba4=0;}
		else { chyba4=1;}
	//---------------------------------
	Device5Presence = ds18b20_reset(&PORTD,PD3);
	if(Device5Presence)
			{chyba5=0;}
		else {  chyba5=1;}
	//---------------------------------
	Device6Presence = ds18b20_reset(&PORTD,PD4);
	if(Device6Presence)
			{chyba6=0;}
		else { chyba6=1;}
	_delay_ms(100);

	// ----------read print ROM code----------------
	
	if(chyba1==0)
		{	
			read_ROM_CODE(&PORTC,PC0);
			printf("T1:");
			for(j=0;j<8;j++)
				{
					printf(" %x", ROM_code[j]);
				}
				printf("\n");
		}
	if(chyba2==0)
		{	
			read_ROM_CODE(&PORTC,PC1);
			printf("T2:");
			for(j=0;j<8;j++)
				{
					printf(" %x", ROM_code[j]);
				}
				printf("\n");
		}
		
	if(chyba3==0)
		{
			read_ROM_CODE(&PORTC,PC2);
			printf("T3:");
			for(j=0;j<8;j++)
				{
					printf(" %x",ROM_code[j]);
				}
				printf("\n");
		}
			
	if(chyba4==0)
		{
			read_ROM_CODE(&PORTC,PC3);
			printf("T4:");
			for(j=0;j<8;j++)
				{
					printf(" %x",ROM_code[j]);
				}
				printf("\n");
		}
	
	if(chyba5==0)
		{
		read_ROM_CODE(&PORTD,PD3);
		printf("T5:");
		for(j=0;j<8;j++)
				{
					printf(" %x",ROM_code[j]);
				}
				printf("\n");
		}

	
	if(chyba6==0)
		{
		read_ROM_CODE(&PORTD,PD4);
		printf("T6:");
		for(j=0;j<8;j++)
				{
					printf(" %x",ROM_code[j]);
				} 
				printf("\n");
		}
	

}


void ds18b20readtemperature(int snimac)
{
	switch (snimac){
		
		case 1: {if(chyba1==0)CurrentTemp1 = ds18b20_gettemp(&PORTC,PC0); // decicelsius
			break;}
			//----------------------------------------------------	
		case 2:	{if(chyba2==0){CurrentTemp2 = ds18b20_gettemp(&PORTC,PC1);	//decicelsius
			}break;}
			//----------------------------------------------------	
		case 3: {if(chyba3==0){CurrentTemp3 = ds18b20_gettemp(&PORTC,PC2);	//decicelsius
				}break;}
			//----------------------------------------------------	
		case 4: {if(chyba4==0){CurrentTemp4 = ds18b20_gettemp(&PORTC,PC3);	//decicelsius
			}break;}
			//----------------------------------------------------			
		case 5:	{if(chyba5==0){CurrentTemp5 = ds18b20_gettemp(&PORTD,PD3);	//decicelsius
			}break;}
			//----------------------------------------------------			
		case 6:	{if(chyba6==0){CurrentTemp6 = ds18b20_gettemp(&PORTD,PD4);	//decicelsius
			}break;}
	}

}

void ds18b20printuart(void)
{
	if(chyba1==0)printf("temp1: %d °C (deci celsius)\n", CurrentTemp1);
	if(chyba2==0)printf("temp2: %d °C (deci celsius)\n", CurrentTemp2);
	if(chyba3==0)printf("temp3: %d °C (deci celsius)\n", CurrentTemp3);
	if(chyba4==0)printf("temp4: %d °C (deci celsius)\n", CurrentTemp4);
	if(chyba5==0)printf("temp5: %d °C (deci celsius)\n", CurrentTemp5);
	if(chyba6==0)printf("temp6: %d °C (deci celsius)\n", CurrentTemp6);
}
