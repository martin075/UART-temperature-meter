//--------------------------------------------------------------
#define TLAC_ON ((PIND & (1 << PD2)) == 0)

volatile uint8_t BTNBLOCK=0;
volatile uint8_t BUTTON=0;

ISR(TIMER1_COMPA_vect){

 if ((TLAC_ON)&&(!BTNBLOCK)) { BUTTON=1; BTNBLOCK=1; }
 if ((!TLAC_ON)&&(BTNBLOCK)) { BUTTON=0; BTNBLOCK=0; }        
}

void main()
{
	while(1)
	if (BUTTON){
	  BUTTON=0;
	 nejaka akcia ;

	 }
}
//---------------------------------------------------------------


alebo

if(RA1==1)
{_delay_ms(5);
if(RA1==1)
{
.....
}
}

alebo - kym nepusti tlacidlo

if(RA1==1)
{_delay_ms(5);
if(RA1==1)
{while(RA1);
_delay_ms(2);
.....
}
}
