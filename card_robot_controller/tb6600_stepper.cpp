/*

*/

#include "tb6600_stepper.h"

volatile double _tickCount = 0;

uint8_t stepperCount = 0;

static stepper_t steppers[MAX_STEPPERS];

void StepperHandler();

void Init_ISR() {
  cli();

  // Clear Timer Control Register
  TCCR2A = 0;
  TCCR2B = 0;

  // Timer Interrupt Mask Register
  TIMSK2 |= (1 << OCIE2A);  // Timer compare interrupt

  // Timer Interrupt Flag Register
  TIFR2 |= (0 << TOV2);

  // Clock source
  TCCR2B |= (1 << CS22);   // Clk / 64 from prescaler
  TCCR2B |= (1 << WGM22);  //| (1 << WGM13)

  // // Set pin toggle for Channel A of timer 1 and 2
  // TCCR1A |= (1 << COM1A0);
 // TCCR2A |= (1 << COM2A0);         // normal operation - external pins disconnected

  uint32_t reg_value = (uint32_t)(F_OSC / (16 * F_PULSE)) - 1;

  OCR2A = (uint16_t)reg_value;  // This will generate 250kHz clock

  sei();
}

ISR(TIMER2_OVF_vect) {
  // Clear the interrupt flag.
  TIFR2 |= (0 << TOV2);
}

ISR(TIMER2_COMPA_vect) {
  cli();

  StepperHandler();

  //TCNT2 = 0;
  sei();
}

void StepperHandler() {
  // Setting the speed by varying output pulse duration with counting. OCR2A register value stays constant.
  for (int n = 0; n < stepperCount; n++) {
    // steppers[n].tickCount++;

    // if (steppers[n].tickCount >= steppers[n].pulses) {
    digitalWrite(steppers[n].stepper_pin, !digitalRead(steppers[n].stepper_pin));
    //   steppers[n].tickCount = 0;
    // }


  }
}

// --------------------------- End of static member functions.

Stepper::Stepper() {

  if (stepperCount < MAX_STEPPERS) {
    this->stepperIndex = stepperCount++;

  } else {
    this->stepperIndex = INVALID_STEPPER;
  }

  Init_ISR();
}

void Stepper::attach(uint8_t pin) {
  pinMode(pin, OUTPUT);

  steppers[this->stepperIndex].stepper_pin = pin;

}

void Stepper::home_axis(bool homing_sensor){
  if (homing_sensor){
    // TODO Check the travelled distance variable.
    steppers[this->stepperIndex].actual_position = 0;
  }
}

void Stepper::set_speed(uint16_t speed) {
  // steppers[this->stepperIndex].pulses = speed;

  // Vary OCR2A register value and set external pin toggle upon OC2A match
  float speed_param = (float)speed / 10;
  float pulse_freq = (float)((speed_param * 80) / 3); // 1600 is micro stepping and 60 is to calculate per second.
  double reg_value = (double)((1000000 / pulse_freq) - 1);
  OCR2A = (uint16_t)reg_value;

}

void Stepper::move_absolute(uint16_t target_position) {
  
}

void Stepper::move_relative(uint16_t target_position) {
}