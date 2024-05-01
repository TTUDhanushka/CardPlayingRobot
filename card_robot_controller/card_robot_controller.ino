/*
Author:       Dhanushka Liyanage
Description:  Card robot (cartesian robot) controller.
              Read ROS serial input commands and run stepper motors.
Date:         2023-11-01
Updated:      2024-04-30
*/

// Globale variables
volatile bool x_pos_lim_state = true, x_neg_lim_state = true;
volatile bool y_pos_lim_state = true, y_neg_lim_state = true;

volatile double _tickCount = 0;
volatile uint16_t dividend = 10000;
double pulseCount = 0;

// Constants
// const float SERIAL_BAUD = 19200;
const byte X_POS_LIM_SW = 2, X_NEG_LIM_SW = 3;
const byte Y_POS_LIM_SW = 18, Y_NEG_LIM_SW = 19;
const byte ENABLE_MOTOR_A = 52, ENABLE_MOTOR_B = 48;
const byte DIR_MOTOR_A = 50, DIR_MOTOR_B = 46;
const byte PULS_MOTOR_A = 10, PULS_MOTOR_B = 11;
const byte ATTACHER = 6, ARM = 7;

const uint16_t MAX_PPS = 5000, MIN_PPS = 400;
const uint16_t MAX_RPM = 750, MIN_RPM = 0;

enum RobotState {Idle, Initializing, Running, ShuttingDown, Error};
RobotState robotCurrentState = Idle;

const float F_OSC = 16000000;     // 16MHz


// Function prototypes
//void setup_uart0_rx_interrupts();
void setup_timer_interrupts();
void setup_io_ports();

void initialize_robot();

// void x_pos_lim_interrupt();

bool WriteToSerialPort(unsigned char data);
bool home_x_y_axes();
bool move_X_axis(float distance);
void set_rpm_motor_X(uint16_t speed);
void set_rpm_motor_Y(uint16_t speed);

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

  // Setup pulse out pins
  pinMode(PULS_MOTOR_A, OUTPUT);
  pinMode(PULS_MOTOR_B, OUTPUT);

  // Setup eletro magnet for grabbing
  pinMode(ATTACHER, OUTPUT);

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

void setup_timer_interrupts(){
  noInterrupts();

  // Clear Timer Control Register
  TCCR1A = 0;
  TCCR1B = 0;

  TCCR2A = 0;
  TCCR2B = 0;

  // Timer Interrupt Mask Register
  TIMSK1 |= (1 << OCIE1A);  // Timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);  // Timer compare interrupt

  // Timer Interrupt Flag Register
  TIFR1 |= (0 << TOV1);
  TIFR2 |= (0 << TOV2);

  // Clock source
  TCCR1B |= (1 << CS11);  // Clk / 8 from prescaler
  TCCR1B |= (1 << WGM12); //| (1 << WGM13)

  TCCR2B |= (1 << CS21);  // Clk / 8 from prescaler
  TCCR2B |= (1 << WGM22); //| (1 << WGM13)

  // Set pin toggle for Channel A of timer 1 and 2
   TCCR1A |= (1 << COM1A0);     // normal operation - external pins disconnected
   TCCR2A |= (1 << COM2A0);

  OCR1A = 49;             // This will generate 10kHz clock
  OCR2A = 49;

  // Enable interrupts
  interrupts();
}

void setup()
{

  setup_io_ports();

  //setup_uart0_rx_interrupts();          // UART - Serial communication interrupt setup.

  setup_timer_interrupts();             // Setup timer interrupts for stepper motor control.

  // Input interrupts for limit switches.
  attachInterrupt(digitalPinToInterrupt(X_POS_LIM_SW), x_pos_lim_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(X_NEG_LIM_SW), x_neg_lim_interrupt, CHANGE);

  Serial.begin(9600);
}

void loop(){

  // State machine
  switch(robotCurrentState){

    case Idle:
      // Do nothing and switch off motors.
      //digitalWrite(PULS_MOTOR_A, LOW);
      //digitalWrite(PULS_MOTOR_B, LOW);

      //digitalWrite(DIR_MOTOR_A, HIGH);
      //digitalWrite(DIR_MOTOR_B, HIGH);

      //Serial.println("Idle");

      set_rpm_motor_X(60);
      // set_rpm_motor_Y(100);

      break;

    case Initializing:

      initialize_robot();
      Serial.println("Initializing");
      break;

  }

  // RUN magnet
  //analogWrite(ATTACHER, 127);
    // digitalWrite(ATTACHER, HIGH);
  x_neg_lim_state = 0;

  if (!x_neg_lim_state){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }
  else{
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }

}

void initialize_robot(){

  // Enable motors
  digitalWrite(ENABLE_MOTOR_A, HIGH);
  digitalWrite(ENABLE_MOTOR_B, HIGH);

  // Rotation directions -> Must be set in
  digitalWrite(DIR_MOTOR_A, LOW);
  digitalWrite(DIR_MOTOR_B, LOW);

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

bool home_x_y_axes(){

  return true;
}

bool move_X_axis(float distance){

}


ISR(TIMER1_OVF_vect) 
{
  // Clear the interrupt flag.
  TIFR1 |= (0 << TOV1);
}

ISR(TIMER2_OVF_vect) 
{
  // Clear the interrupt flag.
  TIFR2 |= (0 << TOV2);
}

volatile bool state = false;

ISR(TIMER1_COMPA_vect) 
{
  noInterrupts();

  // if (_tickCount >= pulseCount){

  //   // Serial.println("Pulse count");
  //   // Serial.println(pulseCount);
  //   // Serial.println("Tick Count");
  //   // Serial.println(_tickCount);

  //   _tickCount = 0;

  //   // if (state){
  //   //   // digitalWrite(10, !digitalRead(10));
  //   //         digitalWrite(10, state);
  //   //         state = false;
  //   // }
  //   // else{
  //   //          digitalWrite(10, state);
  //   //                     state = true;
  //   // }

  // }

  // Count the number of ticks.
  _tickCount++;

  TCNT1 = 0;

  interrupts();
}

ISR(TIMER2_COMPA_vect) 
{
  noInterrupts();
  // Count the number of ticks.
  //_tickCount++;

  TCNT2 = 0;
  interrupts();
}

// X-axis positive  direction limit switch interrupt.
void x_pos_lim_interrupt(){
  x_pos_lim_state = digitalRead(X_POS_LIM_SW);
}

void x_neg_lim_interrupt(){
  x_neg_lim_state = digitalRead(X_NEG_LIM_SW);
}

void set_rpm_motor_X(uint16_t speed)
{
  // pulseCount = 15000 / speed;
pulseCount = 200;
  //int pps = map(speed, MIN_RPM, MAX_RPM, MIN_PPS, MAX_PPS);

  // Serial.println(pulseCount, DEC);

 // uint16_t OCR1A_setpoint = (uint16_t) (1000000 / (pps)) - 1;

 //OCR1A_setpoint;
  // Serial.println("Motor 1");
  // Serial.println(OCR1A_setpoint, DEC);
}

void set_rpm_motor_Y(uint16_t speed)
{
  // int pps = map(speed, MIN_RPM, MAX_RPM, MIN_PPS, MAX_PPS);

  // uint16_t OCR2A_setpoint = (uint16_t) (1000000 / (pps)) - 1;

  // OCR2A = OCR2A_setpoint;
  // Serial.println("Motor 2");
  // Serial.println(OCR2A_setpoint);
}

// #include <Servo.h>

// Servo myservo;  // create servo object to control a servo

// int val;  

// void setup() {
//   // put your setup code here, to run once:
//   myservo.attach(7);
// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   val = 720;            // reads the value of the potentiometer (value between 0 and 1023)
//   val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
//   myservo.write(val);                  // sets the servo position according to the scaled value
//   delay(15);
// }
