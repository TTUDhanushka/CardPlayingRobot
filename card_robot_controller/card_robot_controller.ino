/*
Author:       Dhanushka Liyanage
Description:  Card robot (cartesian robot) controller.
              Read ROS serial input commands and run stepper motors.
Date:         2023-11-01
*/

// Globale variables
volatile bool x_pos_lim_state = true, x_neg_lim_state = true;
volatile bool y_pos_lim_state = true, y_neg_lim_state = true;

// Constants
const float SERIAL_BAUD = 19200;
const byte X_POS_LIM_SW = 2, X_NEG_LIM_SW = 3;
const byte Y_POS_LIM_SW = 18, Y_NEG_LIM_SW = 19;
const byte ENABLE_MOTOR_A = 14, ENABLE_MOTOR_B = 15;
const float F_OSC = 16000000;     // 16MHz


// Function prototypes
void setup_uart0_rx_interrupts();
// void x_pos_lim_interrupt();

bool WriteToSerialPort(unsigned char data);
bool home_x_y_axes();
bool move_X_axis(float distance);

bool initialize_robot(){

  bool status = false;

  // Set digital pins as outputs to enable motors
  pinMode(ENABLE_MOTOR_A, OUTPUT);
  pinMode(ENABLE_MOTOR_B, OUTPUT);

  // Enable motors
  digitalWrite(ENABLE_MOTOR_A, HIGH);
  digitalWrite(ENABLE_MOTOR_B, HIGH);

  return status;
}

bool home_x_y_axes(){

  return true;
}

bool move_X_axis(float distance){

}

void setup_uart0_rx_interrupts(){

  float ubrrValue = (F_OSC / (16 * SERIAL_BAUD)) - 1;

  noInterrupts();

  UCSR0B |= bit(RXCIE0); 

  // Enable receiver
  UCSR0B |= bit(RXEN0); 

  // Enable transmitter
  UCSR0B |= bit(TXEN0); 

  // Set baudrate registers
  UBRR0 = round(ubrrValue); 

  // Enable interrupts
  interrupts();
}

bool WriteToSerialPort(unsigned char data){
  while (!(UCSR0A &= bit(UDRE0))) 
  {
    ;
  }
  
  UDR0 = data;
  
  return true;
}

// UART0 receive interrupt service routine.
ISR(USART0_RX_vect){

// Do something
  
}

// X-axis positive  direction limit switch interrupt.
void x_pos_lim_interrupt(){
  x_pos_lim_state = digitalRead(X_POS_LIM_SW);
}

void x_neg_lim_interrupt(){
  x_neg_lim_state = digitalRead(X_NEG_LIM_SW);
}

void setup(){
  // Status indicator
  pinMode(LED_BUILTIN, OUTPUT);

  //Setup limit switches
  pinMode(X_POS_LIM_SW, INPUT_PULLUP);
  pinMode(X_NEG_LIM_SW, INPUT_PULLUP);
  pinMode(Y_POS_LIM_SW, INPUT_PULLUP);
  pinMode(Y_POS_LIM_SW, INPUT_PULLUP);

  // Input interrupts for limit switches.
  attachInterrupt(digitalPinToInterrupt(X_POS_LIM_SW), x_pos_lim_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(X_NEG_LIM_SW), x_neg_lim_interrupt, CHANGE);

  // UART - Serial communication interrupt setup.
  setup_uart0_rx_interrupts();

  initialize_robot();
}

void loop(){
  if (!x_neg_lim_state){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }
  else{
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }

  // unsigned char transmitBuff[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G'};

  // for (int n = 0; n < 7; n++){
  //   WriteToSerialPort(transmitBuff[n]);
  //   delay(1000);
  // }
}
