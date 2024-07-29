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

#define MAX_STEPPERS 2
#define INVALID_STEPPER 255

#define DEFAULT_RPM 6

void Init_Timer3_ISR();
void Init_Timer4_ISR();
void StepperHandler(int stepperIndex);

typedef enum {
  forward = false,
  reverse = true
} Direction;

typedef enum {
  stop = false,
  run = true
} State;

typedef struct {
  // Hardware pins
  volatile uint8_t pulsePin;
  volatile uint8_t directionPin;

  // Stepper turning direction.
  volatile Direction turnDirection;
  volatile State motorState;
  volatile float encoderScale;
  volatile uint16_t actualPosition;
  volatile uint16_t pulses;
  volatile uint16_t tickCount;
} stepper_t;

class Stepper {
  public:
    Stepper();
    void attach(uint8_t pulseOutPin, uint8_t directionOutPin);
    void stop();
    void setRpm(uint16_t speed);
    void setEncoderScaleFactor(float encoderScaleFactor);
    void move_absolute(uint16_t target_position);
    void move_relative(uint16_t target_position);
    void home_axis(bool homing_sensor);

  private:
    uint8_t stepperIndex;
};

#endif