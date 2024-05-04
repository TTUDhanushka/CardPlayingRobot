/*

*/

#include "RcServo.h"

void Init_ISR(){
  TCCR4A = 0;

  TCCR4A = 0;

  // Timer Interrupt Mask Register
  TIMSK4 |= (1 << OCIE4A);  // Timer compare interrupt

  // Timer Interrupt Flag Register
  TIFR4 |= (0 << TOV4);

  // Clock source
  TCCR4B |= (1 << CS40);    // Clk / 1 from prescaler
  TCCR4B |= (1 << WGM42);   //| (1 << WGM13)

  // // Set pin toggle for Channel A of timer 1 and 2
  // TCCR1A |= (1 << COM1A0);     // normal operation - external pins disconnected
  // TCCR2A |= (1 << COM2A0);

  uint16_t reg_value = (uint16_t) (F_OSC4 / (2 * TIMER4_PRE_SCALER * F_PULSE4)) - 1;

  OCR4A = reg_value;             // This will generate 250kHz clock
}

ISR(TIMER4_OVF_vect) 
{
  // Clear the interrupt flag.
  TIFR4 |= (0 << TOV4);
}

ISR(TIMER4_COMPA_vect) 
{
  cli();

  // Count the number of ticks.
  //_tickCount++;

  TCNT4 = 0;
  sei();
}

// --------------------------- End of static member functions.

RcServo::Servo()
{
  Init_ISR();
}

void RcServo::attach(uint8_t pin){
  pinMode(pin, OUTPUT);
  servo_pin = pin;
}

void RcServo::setServo(uint16_t position){
  
}