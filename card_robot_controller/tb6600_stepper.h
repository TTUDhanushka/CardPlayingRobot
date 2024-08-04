/*
tb6600_stepper.h 

Runs using Timer 2
*/

#ifndef TB6600_STEPPER_H
#define TB6600_STEPPER_H

#include <Arduino.h>

#define F_OSC 16000000        // Clock of the arduino
#define F_PULSE 400           // 
// #define TIMER3_PRE_SCALER 64  // Change to 8 -> 64

#define MAX_STEPPERS 3
#define INVALID_STEPPER 255

#define PULSES_PER_REV 1600

// #define DEFAULT_RPM 6

void Init_Timer3_ISR();
void Init_Timer4_ISR();
void StepperHandler(int stepperIndex);

typedef enum {
  forward = true,
  reverse = false
} Direction;

typedef enum {
  stop = 0,
  run = 1,
  error = 2
} State;

typedef enum {
  velocity = 0,
  position = 1
} OpMode;

typedef enum{
  timer3 = 0,
  timer4 = 1,
  timer5 = 2
} Timers;

typedef struct {
  // Hardware pins
  volatile uint8_t pulsePin;
  volatile uint8_t directionPin;
  Timers timer;

  // Stepper turning direction.
  volatile Direction turnDirection;
  volatile OpMode modeOfOperation;
  volatile State motorState;
  volatile uint8_t teethCount;
  volatile uint16_t actualPosition;
  volatile uint16_t pulses;
  volatile uint32_t targetTickCount;
  volatile uint32_t currentTickCount;
} stepper_t;

class Stepper {
  public:
    Stepper();
    void attach(uint8_t pulseOutPin, uint8_t directionOutPin);
    void stop();
    void setRpm(float rpm);
    void setPulleyTeethCount(uint8_t teethCount);
    void move_absolute(int target_position, float rpm);
    void move_relative(uint16_t target_position);
    void home_axis(bool homing_sensor);

  private:
    uint8_t stepperIndex;
};

#endif