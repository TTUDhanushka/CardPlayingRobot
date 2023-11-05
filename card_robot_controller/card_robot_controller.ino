/*
Author:       Dhanushka Liyanage
Description:  Card robot (cartesian robot) controller.
              Read ROS serial input commands and run stepper motors.
Date:         2023-11-01
*/

// Globale variables
volatile bool x_pos_lim_state = false, x_neg_lim_state = false;
volatile bool y_pos_lim_state = false, y_neg_lim_state = false;

// Constants
const int SERIAL_BAUD = 19200;
const byte X_POS_LIM_SW = 2, X_NEG_LIM_SW = 3;
const byte Y_POS_LIM_SW = 18, Y_NEG_LIM_SW = 19;

// Function prototypes
void setup_uart0_rx_interrupts();
// void x_pos_lim_interrupt();

bool home_x_y_axes();
bool move_X_axis(float distance);

bool initialize_robot(){
  bool status = false;



  return status;
}

bool home_x_y_axes(){

  return true;
}

bool move_X_axis(float distance){

}

void setup_uart0_rx_interrupts(){
  noInterrupts();

  UCSR0B |= bit(RXCIE0); 
  UCSR0B |= bit(RXEN0); 


  // Enable interrupts
  interrupts();
}

// UART0 receive interrupt service routine.
ISR(USART0_RX_vect){


  
}

void setup(){
  // Status indicator
  pinMode(LED_BUILTIN, OUTPUT);

  //Setup limit switches
  pinMode(X_POS_LIM_SW, INPUT_PULLUP);
  pinMode(X_NEG_LIM_SW, INPUT);
  pinMode(Y_POS_LIM_SW, INPUT);
  pinMode(Y_POS_LIM_SW, INPUT);


  attachInterrupt(digitalPinToInterrupt(X_POS_LIM_SW), x_pos_lim_interrupt, CHANGE);

  setup_uart0_rx_interrupts();

  initialize_robot();
}

void x_pos_lim_interrupt(){
  //digitalWrite(LED_BUILTIN, HIGH);
  bool state = digitalRead(X_POS_LIM_SW);
  x_pos_lim_state = state;
}

void loop(){
  if (x_pos_lim_state){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }
  else{
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}
