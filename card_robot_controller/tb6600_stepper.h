/*
tb6600_stepper.h 

Runs using Timer 2
*/

#ifndef TB6600_STEPPER_H
#define TB6600_STEPPER_H

#include <Arduino.h>

#define F_OSC 16000000         // Clock of the arduino
#define F_PULSE 250000         // 4us pulses
#define TIMER4_PRE_SCALER 8

class Stepper
{
  public:
  Stepper();
  void attach(uint8_t pin);
  void set_speed(uint16_t position);
  
  private:

};

#endif