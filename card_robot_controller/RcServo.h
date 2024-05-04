/*
RcServo.h 

Runs using Timer 4
*/

#ifndef RCSERVO_H
#define RCSERVO_H

#include <Arduino.h>

#define F_OSC4 16000000         // Clock of the arduino
#define F_PULSE4 250000         // 4us pulses
#define TIMER4_PRE_SCALER 1

#define SERVO_MAX 1500          //   1.5 milliseconds
#define SERVO_MIN 500           //   0.5 milliseconds

#define DEAD_TIME 20000         //   Dead time between pulses.

class RcServo
{
  public:
  Servo();
  void attach(uint8_t pin);
  void setServo(uint16_t position);
  
  private:
  uint8_t servo_pin;
};

#endif