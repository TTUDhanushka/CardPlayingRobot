/*
  This library uses Timer 3, 4 & 5 for stepper motor control depending on the availability.
*/

#include "tb6600_stepper.h"

volatile double _tickCount = 0;

uint8_t stepperCount = 0;
volatile uint16_t speedSetValue = 0;

static stepper_t steppers[MAX_STEPPERS];

uint16_t ocr_reg_table[300] = {15000, 11718, 5858, 3905, 2929, 2343, 1952, 1673, 1464, 1301, 4687, 4260, 3905, 3605, 3347, 3124, 2929, 2756, 2603, 2466, 2343, 2231,
                                2130, 2037, 1952, 1874, 1802, 1735, 1673, 1615, 1562, 1511, 1464, 1419, 1378, 1338, 1301, 1266, 1233, 1201, 1171, 1142, 1115, 1089, 1064, 
                                1041, 1018, 996, 976, 956, 937, 918, 900, 883, 867, 851, 836, 821, 807, 793, 780, 767, 755, 743, 731, 720, 709, 699, 688, 678, 669, 659, 
                                650, 641, 632, 624, 616, 608, 600, 592, 585, 578, 571, 564, 557, 550, 544, 538, 532, 526, 520, 514, 509, 503, 498, 492, 487, 482, 477, 
                                472, 468, 463, 459, 454, 450, 445, 441, 437, 433, 429, 425, 421, 418, 414, 410, 407,  403, 396, 393, 390, 386, 383, 380, 377, 374, 371, 
                                368, 365, 362, 360, 357,  354, 351, 349, 346, 344, 341, 339, 336, 334, 331, 329, 327, 325, 322, 320, 318, 316, 314, 311, 309, 307, 305,
                                303, 301, 299, 298, 296, 294, 292, 290, 288, 287, 285, 283, 281, 280, 278, 276, 275, 273, 272, 270, 268, 267, 265, 264, 262, 261, 259, 
                                258, 257, 255, 254, 252, 251, 250, 248, 247, 246, 244, 243, 242, 241, 239, 238, 237, 236, 235, 233, 232, 231, 230, 229, 228, 227, 225,
                                224, 223, 222, 221, 220, 219, 218, 217, 216, 215, 214, 213, 212, 211, 210, 209, 208, 207, 206, 205, 205, 204, 203, 202, 201, 200, 199,
                                198, 198, 197, 196, 195, 194, 194, 193, 192, 191, 190, 190, 189, 188, 187, 186, 186, 185, 184, 184, 186, 182, 181, 181, 180, 179, 179,
                                178, 177, 177, 176, 175, 175, 174, 173, 173, 172, 171, 171, 170, 169, 169, 168, 168, 167, 166, 166, 165, 165, 164, 163, 163, 162, 162, 
                                161, 161, 160, 160, 159, 158, 158, 158, 157, 157, 156, 155};

void Init_Timer3_ISR() {
  cli();

  // Clear Timer Control Register
  TCCR3A = 0;
  TCCR3B = 0;
  
  TCNT3 = 0;

  // Timer Interrupt Flag Register
  TIFR3 |= (0 << TOV3);

  // Clock source
  TCCR3B |= (1 << CS31) | (1 << CS30);   // Clk / 64 from prescaler
  TCCR3B |= (1 << WGM32);  //| (1 << WGM13)

  // uint32_t reg_value = (uint32_t)(F_OSC / (16 * F_PULSE)) - 1;
  OCR3A = 155;        // Initializing to a default value

  // Timer Interrupt Mask Register
  TIMSK3 |= (1 << OCIE3A);  // Timer compare interrupt
  TIMSK3 |= (1 << TOIE3);  // Timer overflow interrupt

  sei();
}

void Init_Timer4_ISR() {
  cli();

  // Clear Timer4 Control Register
  TCCR4A = 0;
  TCCR4B = 0;
  
  TCNT4 = 0;

  // Timer4 Interrupt Flag Register
  TIFR4 |= (0 << TOV4);

  // Clock source
  TCCR4B |= (1 << CS41) | (1 << CS40);   // Clk / 64 from prescaler
  TCCR4B |= (1 << WGM42);  //

  // uint32_t reg_value = (uint32_t)(F_OSC / (16 * F_PULSE)) - 1;
  OCR4A = 155;        // Initializing to a default value

  // Timer Interrupt Mask Register
  TIMSK4 |= (1 << OCIE4A);  // Timer compare interrupt
  TIMSK4 |= (1 << TOIE4);  // Timer overflow interrupt

  sei();
}

void Init_Timer5_ISR() {
  cli();

  // Clear Timer5 Control Register
  TCCR5A = 0;
  TCCR5B = 0;
  
  TCNT5 = 0;

  // Timer5 Interrupt Flag Register
  TIFR5 |= (0 << TOV5);

  // Clock source
  TCCR5B |= (1 << CS51) | (1 << CS50);   // Clk / 64 from prescaler
  TCCR5B |= (1 << WGM52);  //

  // uint32_t reg_value = (uint32_t)(F_OSC / (16 * F_PULSE)) - 1;
  OCR5A = 155;        // Initializing to a default value

  // Timer Interrupt Mask Register
  TIMSK5 |= (1 << OCIE5A);  // Timer compare interrupt
  TIMSK5 |= (1 << TOIE5);  // Timer overflow interrupt

  sei();
}

ISR(TIMER3_OVF_vect) {
  // Clear the interrupt flag.
  TIFR3 |= (0 << TOV3);
}

ISR(TIMER4_OVF_vect) {
  // Clear the interrupt flag.
  TIFR4 |= (0 << TOV4);
}

ISR(TIMER5_OVF_vect) {
  // Clear the interrupt flag.
  TIFR5 |= (0 << TOV5);
}

ISR(TIMER3_COMPA_vect) {
  cli();

  StepperHandler(0);            // Index 0 always use Timer 3.

  sei();
}

ISR(TIMER4_COMPA_vect) {

  cli();

  StepperHandler(1);            // Index 1 always use Timer 4.

  sei();
}

ISR(TIMER5_COMPA_vect) {

  cli();

  StepperHandler(2);            // Index 1 always use Timer 4.

  sei();
}

void update_timer_register(Timers timerId, float speed){

  // Speed (rpm) multiplied by 10 to set range 0 - 299 OCR registry array indices.
  int speedToFreqIndx = (int)speed * 10;

  switch(timerId){
    case Timers::timer3:

      if (speedToFreqIndx < 10){

        // Clear registry bits.
        TCCR3B |= (0 << CS32) |(0 << CS31) | (0 << CS30);

        TCCR3B |= (1 << CS32);   // Clk / 256 from prescaler 

      }
      else{
        
        // Clear registry bits.
        TCCR3B |= (0 << CS32) |(0 << CS31) | (0 << CS30);

        TCCR3B |= (1 << CS31) | (1 << CS30);   // Clk / 64 from prescaler

      }

      OCR3A = ocr_reg_table[speedToFreqIndx]; 

    break;

    case Timers::timer4:

      if (speedToFreqIndx < 10){

        // Clear registry bits.
        TCCR4B |= (0 << CS42) |(0 << CS41) | (0 << CS40);

        TCCR4B |= (1 << CS42);   // Clk / 256 from prescaler 

      }
      else{
        
        // Clear registry bits.
        TCCR4B |= (0 << CS42) |(0 << CS41) | (0 << CS40);

        TCCR4B |= (1 << CS41) | (1 << CS40);   // Clk / 64 from prescaler

      }

      OCR4A = ocr_reg_table[speedToFreqIndx]; 
    break;

    case Timers::timer5:

      if (speedToFreqIndx < 10){

        // Clear registry bits.
        TCCR5B |= (0 << CS52) |(0 << CS51) | (0 << CS50);

        TCCR5B |= (1 << CS52);   // Clk / 256 from prescaler 

      }
      else{
        
        // Clear registry bits.
        TCCR5B |= (0 << CS52) |(0 << CS51) | (0 << CS50);

        TCCR5B |= (1 << CS51) | (1 << CS50);   // Clk / 64 from prescaler

      }

      OCR5A = ocr_reg_table[speedToFreqIndx]; 
    break;
  }
}

void StepperHandler(int stepperIndex) {

  // Toggle the pin
  if (steppers[stepperIndex].motorState == State::stop){
    digitalWrite(steppers[stepperIndex].pulsePin, LOW);
  }
  else if (steppers[stepperIndex].motorState == State::run){
    if(steppers[stepperIndex].modeOfOperation == OpMode::velocity){
      // Set OCRnA register
      if(steppers[stepperIndex].timer == Timers:: timer3){
        OCR3A = ocr_reg_table[steppers[stepperIndex].pulses];
        TCNT3 = 0;
      } 
      else if(steppers[stepperIndex].timer == Timers:: timer4){
        OCR4A = ocr_reg_table[steppers[stepperIndex].pulses];
        TCNT4 = 0;
      }
      else if(steppers[stepperIndex].timer == Timers:: timer5){
        OCR5A = ocr_reg_table[steppers[stepperIndex].pulses];
        TCNT5 = 0;
      }

      digitalWrite(steppers[stepperIndex].pulsePin, !digitalRead(steppers[stepperIndex].pulsePin));

      if (steppers[stepperIndex].turnDirection == Direction::forward){
        digitalWrite(steppers[stepperIndex].directionPin, HIGH);
      }
      else{
        digitalWrite(steppers[stepperIndex].directionPin, LOW);
      }

    }
    else if(steppers[stepperIndex].modeOfOperation == OpMode::position){

      // Set OCRnA register
      if(steppers[stepperIndex].timer == Timers:: timer3){
        OCR3A = ocr_reg_table[steppers[stepperIndex].pulses];
        TCNT3 = 0;
      } 
      else if(steppers[stepperIndex].timer == Timers:: timer4){
        OCR4A = ocr_reg_table[steppers[stepperIndex].pulses];
        TCNT4 = 0;
      }
      else if(steppers[stepperIndex].timer == Timers:: timer5){
        OCR5A = ocr_reg_table[steppers[stepperIndex].pulses];
        TCNT5 = 0;
      }

      if (steppers[stepperIndex].targetTickCount > steppers[stepperIndex].currentTickCount){
        steppers[stepperIndex].motorState == State::run;
        steppers[stepperIndex].currentTickCount += 1;
        digitalWrite(steppers[stepperIndex].pulsePin, !digitalRead(steppers[stepperIndex].pulsePin));
      }
      else{
        steppers[stepperIndex].motorState == State::stop;
      }

    }


  }

}

// --------------------------- End of static member functions.

Stepper::Stepper() {

  if (stepperCount < MAX_STEPPERS) {
    this->stepperIndex = stepperCount++;

  } else {
    this->stepperIndex = INVALID_STEPPER;
  }

}

void Stepper::attach(uint8_t pulseOutPin, uint8_t directionOutPin) {
  pinMode(pulseOutPin, OUTPUT);

  steppers[this->stepperIndex].pulsePin = pulseOutPin;
  steppers[this->stepperIndex].directionPin = directionOutPin;

  // Set the motor to run state
  steppers[this->stepperIndex].motorState = State::run;

  if (this->stepperIndex == 0){
    steppers[this->stepperIndex].timer = Timers::timer3;
    Init_Timer3_ISR();
  }
  else if (this->stepperIndex == 1){
    steppers[this->stepperIndex].timer = Timers::timer4;
    Init_Timer4_ISR();
  }
  else if (this->stepperIndex == 2){
    steppers[this->stepperIndex].timer = Timers::timer5;
    Init_Timer5_ISR();
  }
  else{
    Serial.print("Only supports 3 stepper motors.\n");
  }

}

void Stepper::setPulleyTeethCount(uint8_t teethCount){
  steppers[this->stepperIndex].teethCount = teethCount;
}

void Stepper::home_axis(bool homing_sensor){

  if (homing_sensor){
    // TODO Check the travelled distance variable.
    steppers[this->stepperIndex].actualPosition = 0;
  }

}

void Stepper::setRpm(float rpm, Direction direction) {

  // Set the motor to run state
  if (steppers[this->stepperIndex].motorState != State::run){
    steppers[this->stepperIndex].motorState = State::run;
  }

  steppers[this->stepperIndex].pulses = rpm;
  steppers[this->stepperIndex].turnDirection = direction;
  steppers[this->stepperIndex].modeOfOperation = OpMode::velocity;
}

/*
Function: stop

Abruptly stop
*/
void Stepper::stop(){
  
  steppers[stepperIndex].motorState = State::stop; 

}


/*
Function: Move absolute 

param target_position: (int) Target absolute positioin refering to the origin. The motor needs to be homed.
param rpm: (float) Reference rpm 
*/
void Stepper::move_absolute(int target_position, float rpm) {

  steppers[this->stepperIndex].modeOfOperation = OpMode::position;

  if(steppers[this->stepperIndex].teethCount > 0){

    // Distance = tooth pitch x number of teeth
    double distancePerRev = 2 * steppers[this->stepperIndex].teethCount;

    double targetDistance = target_position - steppers[this->stepperIndex].actualPosition;
    double pulsesRequired = abs((2 * targetDistance * PULSES_PER_REV) / distancePerRev);          // Because timer counts both rising and falling edges.

    if (targetDistance > 0){
      steppers[this->stepperIndex].turnDirection = Direction::forward;

    }
    else{
      steppers[this->stepperIndex].turnDirection = Direction::reverse;

    }

    steppers[this->stepperIndex].targetTickCount = (uint32_t)pulsesRequired;

    // Reference rpm.
    steppers[this->stepperIndex].pulses = rpm;
  }

}

/*
Function: Move Relative
*/
void Stepper::move_relative(uint16_t target_position) {
  
}

void home_axis(uint8_t homing_sensor_input){
  bool sensor_status = digitalRead(homing_sensor_input);

  if (sensor_status){
    steppers[this->stepperIndex].motorState = State::stop;
  }
}
