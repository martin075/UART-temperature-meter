// for atmega328/168 8MHz
// http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html

// TIMER 0 for interrupt frequency 1000 Hz:
cli(); // stop interrupts
TCCR0A = 0; // set entire TCCR0A register to 0
TCCR0B = 0; // same for TCCR0B
TCNT0  = 0; // initialize counter value to 0
// set compare match register for 1000 Hz increments
OCR0A = 124; // = 8000000 / (64 * 1000) - 1 (must be <256)
// turn on CTC mode
TCCR0B |= (1 << WGM01);
// Set CS02, CS01 and CS00 bits for 64 prescaler
TCCR0B |= (0 << CS02) | (1 << CS01) | (1 << CS00);
// enable timer compare interrupt
TIMSK0 |= (1 << OCIE0A);
sei(); // allow interrupts


// TIMER 1 for interrupt frequency 1000 Hz:
cli(); // stop interrupts
TCCR1A = 0; // set entire TCCR1A register to 0
TCCR1B = 0; // same for TCCR1B
TCNT1  = 0; // initialize counter value to 0
// set compare match register for 1000 Hz increments
OCR1A = 7999; // = 8000000 / (1 * 1000) - 1 (must be <65536)
// turn on CTC mode
TCCR1B |= (1 << WGM12);
// Set CS12, CS11 and CS10 bits for 1 prescaler
TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
// enable timer compare interrupt
TIMSK1 |= (1 << OCIE1A);
sei(); // allow interrupts


// TIMER 2 for interrupt frequency 1000 Hz:
cli(); // stop interrupts
TCCR2A = 0; // set entire TCCR2A register to 0
TCCR2B = 0; // same for TCCR2B
TCNT2  = 0; // initialize counter value to 0
// set compare match register for 1000 Hz increments
OCR2A = 249; // = 8000000 / (32 * 1000) - 1 (must be <256)
// turn on CTC mode
TCCR2B |= (1 << WGM21);
// Set CS22, CS21 and CS20 bits for 32 prescaler
TCCR2B |= (0 << CS22) | (1 << CS21) | (1 << CS20);
// enable timer compare interrupt
TIMSK2 |= (1 << OCIE2A);
sei(); // allow interrupts



ISR(TIMER0_COMPA_vect){
   //interrupt commands for TIMER 0 here
}


ISR(TIMER1_COMPA_vect){
   //interrupt commands for TIMER 1 here
}


ISR(TIMER2_COMPA_vect){
   //interrupt commands for TIMER 2 here
}
