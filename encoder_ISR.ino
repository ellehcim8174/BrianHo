/*
*Quadrature Decoder 
*/
#include "Arduino.h"
#include <digitalWriteFast.h>  

// Quadrature encoders
#define EncInterruptA 0
#define EncInterruptB 1
#define encA 2
#define encB 3
#define LeftEncoderIsReversed
//Motor Driver Pins
#define ENABLE  9
#define dir1    10
#define dir2    11

volatile bool encA_set = 0;
volatile bool encB_set = 0;
bool encA_prev = 0;
bool encB_prev = 0;
long enc_counts = 0;

// Mike's stuff
double desired_location = 100;            // step response input
int current_location;                     // feedback
double error;                             // error after subracting negative feedback
double PID_value;                         // PID value for input
long raw_PWM;                             // signed PWM
int PWM_mag;                              // just the magnitude
float error_decimal;                      // for adjusting gain of input to system

float Kp = 0.9;                         // Proportional gain, 0.211 (up to 0.9)
float Ki = 0.170;                         // Integrator gain, 0.170
float Kd = 0.058;                         // Derivative gain, 0.058

/*float Kp = 1;                             // only P
float Ki = 0;               
float Kd = 0;  */

double integral = 0;                      // Integrator term
double derivative = 0;                    // Derivative term 
double last_error = 0;                    // Used for saving last error to calculate derivative
long last_micros_integral = 0;            // microseconds since last loop of motor.ino integrator calculation
long last_micros_derivative = 0;          // microseconds sine last loop of motor.ino derivative calculation
int derivative_counter = 0;               // sets a delay so we can get somewhat of a real derivative

void setup()
{
  Serial.begin(2000000);

  // Quadrature encoders
  pinMode(encA, INPUT);                   // sets pin A as input
  digitalWrite(encA, LOW);                // turn on pulldown resistors
  pinMode(encB, INPUT);                   // sets pin B as input
  digitalWrite(encB, LOW);                // turn on pulldown resistors
  attachInterrupt(digitalPinToInterrupt(encA), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB), ISR_B, CHANGE);
  //Motor Pins Setup
  pinMode(ENABLE, OUTPUT);
  //setPWMFrequency(3, 2);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
}

void loop()
{ 
  runMotor();
}


// Interrupt service routines for the left motor's quadrature encoder
void ISR_A(){
  encB_set = digitalReadFast(encB);
  encA_set = digitalReadFast(encA);

  enc_counts+=ParseEncoder();
  
  encA_prev = encA_set;
  encB_prev = encB_set;
}

// Interrupt service routines for the right motor's quadrature encoder
void ISR_B(){
  // Test transition;
  encB_set = digitalReadFast(encB);
  encA_set = digitalReadFast(encA);

  enc_counts+=ParseEncoder();
  
  encA_prev = encA_set;
  encB_prev = encB_set;
}

int ParseEncoder(){
  if(encA_prev && encB_prev){
    if(!encA_set && encB_set) return 1;
    if(encA_set && !encB_set) return -1;
  }else if(!encA_prev && encB_prev){
    if(!encA_set && !encB_set) return 1;
    if(encA_set && encB_set) return -1;
  }else if(!encA_prev && !encB_prev){
    if(encA_set && !encB_set) return 1;
    if(!encA_set && encB_set) return -1;
  }else if(encA_prev && !encB_prev){
    if(encA_set && encB_set) return 1;
    if(!encA_set && !encB_set) return -1;
  }
 
}
