/*
Author:       Dhanushka Liyanage
Description:  Card robot (cartesian robot) controller.
              Read ROS serial input commands and run stepper motors.
Date:         2023-11-01
*/

// Variables
bool _x_top_limit_sw, _x_bot_limit_sw;


// Constants
const int SERIAL_BAUD = 19200;

// Function prototypes
void setup_uart0_rx_interrupts();

bool initialize_robot(){
  bool status = false;



  return status;
}

void setup_uart0_rx_interrupts(){
  noInterrupts();

  UCSR0B |= bit(RXCIE0); 
  UCSR0B |= bit(RXEN0); 


  // Enable interrupts
  interrupts();
}

void setup(){
  // Status indicator
  pinMode(LED_BUILTIN, OUTPUT);

  setup_uart0_rx_interrupts();

  initialize_robot();
}

// UART0 receive interrupt service routine.
ISR(USART0_RX_vect){

  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  
}

void loop(){

}
