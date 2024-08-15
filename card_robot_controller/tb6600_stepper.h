/*
tb6600_stepper.h 

Runs using Timer 2
*/

#ifndef TB6600_STEPPER_H
#define TB6600_STEPPER_H

#include <Arduino.h>

#define F_OSC 16000000        // Clock of the arduino
#define F_PULSE 4687          // Should generate 1 rpm signal 

#define MAX_STEPPERS 3
#define INVALID_STEPPER 255

#define PULSES_PER_REV 1600


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
  position = 1, 
  cyclic_position = 2,
  homing = 4
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

  volatile bool homedStatus;

  // Hardware timer
  Timers timer;

  // Status variables
  volatile Direction turnDirection;
  volatile OpMode modeOfOperation;
  volatile State motorState;
  volatile uint8_t teethCount;
  volatile uint16_t actualPosition;         // In mm.
  volatile double rpm;
  volatile uint32_t targetTickCount;
  volatile uint32_t currentTickCount;
} stepper_t;

class Stepper {
  public:
    Stepper();
    void attach(uint8_t pulseOutPin, uint8_t directionOutPin);
    void stop();
    void setRpm(double rpm, Direction direction);
    void setPulleyTeethCount(uint8_t teethCount);
    void move_absolute(int target_position, double rpm);
    void move_relative(uint16_t target_position);
    void home_axis(uint8_t homing_sensor_input);
    bool isHomed(void);

  private:
    uint8_t stepperIndex;
};

#endif