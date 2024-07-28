/*
tb6600_stepper.h 

Runs using Timer 2
*/

#ifndef TB6600_STEPPER_H
#define TB6600_STEPPER_H

#include <Arduino.h>

#define F_OSC 16000000        // Clock of the arduino
#define F_PULSE 400           // 
#define TIMER3_PRE_SCALER 64  // Change to 8 -> 64

#define MAX_STEPPERS 5
#define INVALID_STEPPER 255

#define DEFAULT_RPM 6

void Init_ISR();
void StepperHandler();

typedef enum {
  forward = false,
  reverse = true
} Direction;

typedef struct {
  // Hardware pins
  uint8_t stepper_pin;
  uint8_t direction_pin;

  // Stepper turning direction.
  volatile Direction turn_direction;

  volatile float encoder_scale;
  volatile uint16_t actual_position;
  volatile uint16_t pulses;
  volatile uint16_t tickCount;
} stepper_t;

class Stepper {
  public:
    Stepper();
    void attach(uint8_t pin);
    void set_speed(uint16_t speed);
    void move_absolute(uint16_t target_position);
    void move_relative(uint16_t target_position);
    void home_axis(bool homing_sensor);

  private:
    uint8_t stepperIndex;
};

#endif