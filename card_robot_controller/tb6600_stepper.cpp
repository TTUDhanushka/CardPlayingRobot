/*

*/

#include "tb6600_stepper.h"

uint16_t set_speed_value;
volatile double _tickCount = 0;
uint8_t servo_pin;

void Init_ISR(){
  cli();

  // Clear Timer Control Register
  TCCR2A = 0;
  TCCR2B = 0;

  // Timer Interrupt Mask Register
  TIMSK2 |= (1 << OCIE2A);  // Timer compare interrupt

  // Timer Interrupt Flag Register
  TIFR2 |= (0 << TOV2);

  // Clock source
  TCCR2B |= (1 << CS21);  // Clk / 8 from prescaler
  TCCR2B |= (1 << WGM22); //| (1 << WGM13)

  // // Set pin toggle for Channel A of timer 1 and 2
  // TCCR1A |= (1 << COM1A0);     
  // TCCR2A |= (1 << COM2A0);       // normal operation - external pins disconnected

  uint16_t reg_value = (uint16_t) (F_OSC / (16 * F_PULSE)) - 1;

  OCR2A = reg_value;                // This will generate 250kHz clock

  sei();
}

ISR(TIMER2_OVF_vect) 
{
  // Clear the interrupt flag.
  TIFR2 |= (0 << TOV2);
}

ISR(TIMER2_COMPA_vect) 
{
  cli();

  // Count the number of ticks.
  _tickCount++;

  if (_tickCount >= set_speed_value){
    digitalWrite(servo_pin, !digitalRead(servo_pin));
    _tickCount = 0;
  }

  TCNT2 = 0;
  sei();
}

// --------------------------- End of static member functions.

Stepper::Stepper()
{
  Init_ISR();
}

void Stepper::attach(uint8_t pin){
  pinMode(pin, OUTPUT);
  servo_pin = pin;
}

void Stepper::set_speed(uint16_t speed){
  set_speed_value = speed;
}
