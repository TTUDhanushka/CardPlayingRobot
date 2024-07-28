/*

*/

#include "tb6600_stepper.h"

volatile double _tickCount = 0;

uint8_t stepperCount = 0;
uint16_t speedSetValue = 0;

static stepper_t steppers[MAX_STEPPERS];

void StepperHandler();

uint16_t ocr_reg_table[300] = {50000, 46874, 23437, 15624, 11718, 9374, 7812, 6695, 5858, 5207, 4687, 4260, 3905, 3605, 3347, 3124, 2929, 2756, 2603, 2466, 2343, 2231,
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

void Init_ISR() {
  noInterrupts();

  // Clear Timer Control Register
  TCCR3A = 0;
  TCCR3B = 0;
  
  TCNT3 = 0;

  // // Timer Interrupt Flag Register
  // TIFR3 |= (0 << TOV3);

  // Clock source
  TCCR3B |= (1 << CS31) | (1 << CS30);   // Clk / 64 from prescaler
  TCCR3B |= (1 << WGM32);  //| (1 << WGM13)

  // uint32_t reg_value = (uint32_t)(F_OSC / (16 * F_PULSE)) - 1;

  // OCR2A = (uint16_t)reg_value;  // This will generate 250kHz clock
  OCR3A = 249;

  // Timer Interrupt Mask Register
  TIMSK3 |= (1 << OCIE3A);  // Timer compare interrupt
  // TIMSK3 |= (1 << TOIE3);  // Timer overflow interrupt

  interrupts();
}

// ISR(TIMER3_OVF_vect) {
//   // Clear the interrupt flag.
//   TIFR3 |= (0 << TOV3);
// }

ISR(TIMER3_COMPA_vect) {
  cli();

  StepperHandler();
  TCNT3 = 0;
  OCR3A = ocr_reg_table[speedSetValue];


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

  // // Vary OCR2A register value and set external pin toggle upon OC2A match
  // float speed_param = (float)speed / 10;
  // float pulse_freq = (float)((speed_param * 80) / 3); // 1600 is micro stepping and 60 is to calculate per second.
  // double reg_value = (double)((1000000 / pulse_freq) - 1);
  // OCR2A = (uint16_t)reg_value;

  speedSetValue = speed;

}

void Stepper::move_absolute(uint16_t target_position) {
  
}

void Stepper::move_relative(uint16_t target_position) {
}