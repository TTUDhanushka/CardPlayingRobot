/*
Author:       Dhanushka Liyanage
Description:  Card robot (cartesian robot) controller.
              Read ROS serial input commands and run stepper motors.
Date:         2023-11-01
Updated:      2024-04-30
*/

#include "tb6600_stepper.h"
// #include <Servo.h>

// Globale variables
volatile bool x_pos_lim_state = true, x_neg_lim_state = true;
volatile bool y_pos_lim_state = true, y_neg_lim_state = true;

// volatile double _tickCount = 0;
volatile uint16_t dividend = 10000;
double pulseCount = 0;
volatile bool xHomed = false;
volatile bool yHomed = false;
volatile bool homingStatus = false;

// Servo grabber_servo; 

// IO Pin mappings
//------------------------------------------------------------------------------
// const float SERIAL_BAUD = 19200;

// LImit switches attached pins.
const byte X_POS_LIM_SW = 19, X_NEG_LIM_SW = 18;
const byte Y_POS_LIM_SW = 3, Y_NEG_LIM_SW = 2;

// Stepper motor pins
const byte ENABLE_MOTOR_A = 52, ENABLE_MOTOR_B = 48;
const byte DIR_MOTOR_A = 50, DIR_MOTOR_B = 46;
const byte PULSE_MOTOR_A = 10, PULSE_MOTOR_B = 11;
// const byte ATTACHER = 6, ARM = 7;

const uint16_t MAX_PPS = 5000, MIN_PPS = 400;
const uint16_t MAX_RPM = 750, MIN_RPM = 0;

enum RobotState { Idle, 
                  Initializing, 
                  Running, 
                  ShuttingDown, 
                  Error
                };
RobotState robotCurrentState = Idle;

typedef struct{
  int x;
  int y;
} xy_point_t;

typedef struct{
  int x;
  int y;
} step_point_t;

// Function prototypes
//void setup_uart0_rx_interrupts();
void setup_io_ports();

void homeRobot(uint8_t x_axis_home_switch, uint8_t y_axis_home_switch);
bool isHomed();
void setHomingStatus();
void resetHomingStatus();

// void x_pos_lim_interrupt();

bool WriteToSerialPort(unsigned char data);
step_point_t xyToStepperAxes(xy_point_t coord);

Stepper right_stepper, left_stepper;

void setup_io_ports(){
  // Status indicator
  pinMode(LED_BUILTIN, OUTPUT);

  //Setup limit switches
  pinMode(X_POS_LIM_SW, INPUT_PULLUP);
  pinMode(X_NEG_LIM_SW, INPUT_PULLUP);
  pinMode(Y_POS_LIM_SW, INPUT_PULLUP);
  pinMode(Y_POS_LIM_SW, INPUT_PULLUP);

  // Set digital pins as outputs to enable motors
  pinMode(ENABLE_MOTOR_A, OUTPUT);
  pinMode(ENABLE_MOTOR_B, OUTPUT);

  // Setup direction outputs
  pinMode(DIR_MOTOR_A, OUTPUT);
  pinMode(DIR_MOTOR_B, OUTPUT);

  // Setup eletro magnet for grabbing
  // pinMode(ATTACHER, OUTPUT);


}

// void setup_uart0_rx_interrupts(){

//   float ubrrValue = (F_OSC / (16 * SERIAL_BAUD)) - 1;

//   noInterrupts();

//   UCSR0B |= bit(RXCIE0); 

//   // Enable receiver
//   UCSR0B |= bit(RXEN0); 

//   // Enable transmitter
//   UCSR0B |= bit(TXEN0); 

//   // Set baudrate registers
//   UBRR0 = round(ubrrValue); 

//   // Enable interrupts
//   interrupts();
// }

void setup()
{
  Serial.begin(9600);

  setup_io_ports();

  //setup_uart0_rx_interrupts();          // UART - Serial communication interrupt setup.

  // Input interrupts for limit switches.
  attachInterrupt(digitalPinToInterrupt(Y_POS_LIM_SW), y_pos_lim_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Y_NEG_LIM_SW), y_neg_lim_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(X_POS_LIM_SW), x_pos_lim_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(X_NEG_LIM_SW), x_neg_lim_interrupt, CHANGE);

  right_stepper.attach(PULSE_MOTOR_A, DIR_MOTOR_A);
  left_stepper.attach(PULSE_MOTOR_B, DIR_MOTOR_B);


  right_stepper.setPulleyTeethCount(60);
  left_stepper.setPulleyTeethCount(60);

  right_stepper.invert(true);
  left_stepper.invert(false);
  // grabber_servo.attach(7);

}

void loop(){

  // State machine
  switch(robotCurrentState){

    case Idle:
      
      Serial.println("Idle state");

      // Do nothing and switch off motors.
      right_stepper.stop(); 
      left_stepper.stop();  
      
      // Once command comes from the UI, this part will be changed
      robotCurrentState = RobotState::Initializing;
        
      delay(200);

      break;

    case Initializing:

      // Serial.println("Homing the robot");

      homeRobot(X_POS_LIM_SW, Y_POS_LIM_SW);;

      if (isHomed()){
        robotCurrentState = RobotState::Running;
      }

      delay(200);

      break;

    case Running:
      // Serial.println("Robot is ready for action");
      
      // //---------------------- Motor speed increasing --------------------------------------
      // for(int n = 0; n < 299; n++){
      //   right_stepper.setRpm(n/10, Direction::forward);
      //   // left_stepper.setRpm(n);  

      //    delay(50);
      // }
      
      // for(int n = 299; n > 0; n--){
      //   right_stepper.setRpm(n/10, Direction::forward);
      //   delay(50);
      // }

      // for(int n = 0; n < 299; n++){
      //   right_stepper.setRpm(n/10, Direction::reverse);
      //   // left_stepper.setRpm(n);  

      //    delay(50);
      // }

          // for(int n = 299; n > 0; n--){
      //   right_stepper.setRpm(n/10, Direction::reverse);
      //   delay(50);
      // }
        // right_stepper.setRpm(1, Direction::reverse);        // Can't handle floating points below 1.0
        delay(50);



        if (!right_stepper.isBusy()){
          xy_point_t target;
          target.x = 0;
          target.y = 120;

          step_point_t motorDistances = xyToStepperAxes(target);

          right_stepper.move_relative(motorDistances.y, 160);
          left_stepper.move_relative(motorDistances.x, 160);

        }



// ------------------ ABsolute position control
        //right_stepper.move_absolute(240, 250);
        // left_stepper.move_absolute(180, 50);  

      //grabber_servo.write(150);

      // digitalWrite(ATTACHER, HIGH);
    break;

    case Error:
      // In case of error stop everything.


    break;

  }
  

  // -------------- For diagnosis.------------------------------------------

  // if (!x_neg_lim_state){
  //   digitalWrite(LED_BUILTIN, HIGH);
  //   delay(100);
  // }
  // else{
  //   digitalWrite(LED_BUILTIN, LOW);
  //   delay(100);
  // }

}

void homeRobot(uint8_t x_axis_home_switch, uint8_t y_axis_home_switch){

  // First check the X axis and move to the home position.
  bool x_sensor = digitalRead(x_axis_home_switch);
  bool y_sensor = digitalRead(y_axis_home_switch);

  if (x_sensor){
    xHomed = false;
  }
  else{
    xHomed = true;
  }

  if (y_sensor){
    yHomed = false;
  }
  else{
    yHomed = true;
  }

  // Second check the Y axis and move to the home position.
  static uint8_t homingAxis = 0;

  switch(homingAxis){
    case 0:
      if (!xHomed){
        // This moves in X - direction
        right_stepper.setRpm(20, Direction::forward);
        left_stepper.setRpm(20, Direction::reverse);
      }
      else{
        right_stepper.stop();
        left_stepper.stop();

        homingAxis++;
      }
    break;

    case 1:
      if (!yHomed){
        // This moves in Y - direction
        right_stepper.setRpm(20, Direction::reverse);
        left_stepper.setRpm(20, Direction::reverse);
      }
      else{
        right_stepper.stop();
        left_stepper.stop();

        homingAxis++;
      }

    break;

    default:

      right_stepper.stop();
      left_stepper.stop();

    break;

  }

  if (homingAxis > 1){
    setHomingStatus();
  }

}

bool isHomed(){
  return homingStatus;
}

void setHomingStatus(){
  homingStatus = true;
}

void resetHomingStatus(){
  homingStatus = false;
}

step_point_t xyToStepperAxes(xy_point_t coord){

  double transformationMatrix[2][2] = {{1.0, -1.0}, {1.0, 1.0}};

  step_point_t out;

  out.x = (double) (transformationMatrix[0][0] * coord.x + transformationMatrix[0][1] * coord.y);
  out.y = (double) (transformationMatrix[1][0] * coord.x + transformationMatrix[1][1] * coord.y);

  return out;
}

// X-axis positive  direction limit switch interrupt.
void x_pos_lim_interrupt(){
  Serial.println("x pos lim");
  x_pos_lim_state = digitalRead(X_POS_LIM_SW);
}

void x_neg_lim_interrupt(){
  Serial.println("x neg lim");
  x_neg_lim_state = digitalRead(X_NEG_LIM_SW);
}

// X-axis positive  direction limit switch interrupt.
void y_pos_lim_interrupt(){
  Serial.println("y pos lim");
  y_pos_lim_state = digitalRead(Y_POS_LIM_SW);
}

void y_neg_lim_interrupt(){
  Serial.println("y neg lim");
  y_neg_lim_state = digitalRead(Y_NEG_LIM_SW);
}

// bool WriteToSerialPort(unsigned char data){
//   while (!(UCSR0A &= bit(UDRE0))) 
//   {
//     ;
//   }
  
//   UDR0 = data;
  
//   return true;
// }

// // UART0 receive interrupt service routine.
// ISR(USART0_RX_vect){

// // Do something
  
// }
