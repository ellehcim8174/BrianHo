/*
*Quadrature Decoder 
*uses Arduino MEGA 2560
*/
#include "Arduino.h"
#include <digitalWriteFast.h>  

// Quadrature encoders
//Big Motor
#define EncInterruptA 0
#define EncInterruptB 1
#define encA 2    //interrupt pin
#define encB 3    //interrupt pin
#define LeftEncoderIsReversed
//Driver Pins
#define ENABLE  8
#define dir1    9
#define dir2    10

//Small Motor
#define EncInterruptA_2 4
#define EncInterruptB_2 5
#define encA_2 20    //interrupt pin
#define encB_2 21    //interrupt pin
#define LeftEncoderIsReversed_2
//Driver Pins
#define ENABLE_2  11
#define dir1_2    12
#define dir2_2    13

volatile bool encA_set = 0;
volatile bool encB_set = 0;
bool encA_prev = 0;
bool encB_prev = 0;
long enc_counts = 0;

volatile bool encA_set_2 = 0;
volatile bool encB_set_2 = 0;
bool encA_prev_2 = 0;
bool encB_prev_2 = 0;
long enc_counts_2 = 0;

// Mike's stuff

// BIG MOTOR
double desired_location = 80;            // step response input
int current_location;                     // feedback
double error;                             // error after subracting negative feedback
double PID_value;                         // PID value for input
long raw_PWM;                             // signed PWM
int PWM_mag;                              // just the magnitude
float error_decimal;                      // for adjusting gain of input to system

/*float Kp = 0.211;                         // Proportional gain, 0.211 (up to 0.9)
float Ki = 0.170;                         // Integrator gain, 0.170
float Kd = 0.058;                         // Derivative gain, 0.058*/

float Kp = 1;                             // only P
float Ki = 0;               
float Kd = 0;

double integral = 0;                      // Integrator term
double derivative = 0;                    // Derivative term 
double last_error = 0;                    // Used for saving last error to calculate derivative
long last_micros_integral = 0;            // microseconds since last loop of motor.ino integrator calculation
long last_micros_derivative = 0;          // microseconds sine last loop of motor.ino derivative calculation
int derivative_counter = 0;               // sets a delay so we can get somewhat of a real derivative

// SMALL MOTOR

double desired_location_2 = 80;            // step response input
int current_location_2;                     // feedback
double error_2;                             // error after subracting negative feedback
double PID_value_2;                         // PID value for input
long raw_PWM_2;                             // signed PWM
int PWM_mag_2;                              // just the magnitude
float error_decimal_2;                      // for adjusting gain of input to system

float Kp_2 = 0.9;                         // Proportional gain, 0.211 (up to 0.9)
float Ki_2 = 0.170;                         // Integrator gain, 0.170
float Kd_2 = 0.058;                         // Derivative gain, 0.058

/*float Kp_2 = 1;                             // only P
float Ki_2 = 0;               
float Kd_2 = 0;  */

double integral_2 = 0;                      // Integrator term
double derivative_2 = 0;                    // Derivative term 
double last_error_2 = 0;                    // Used for saving last error to calculate derivative
long last_micros_integral_2 = 0;            // microseconds since last loop of motor.ino integrator calculation
long last_micros_derivative_2 = 0;          // microseconds sine last loop of motor.ino derivative calculation
int derivative_counter_2 = 0;               // sets a delay so we can get somewhat of a real derivative


void setup()
{
  Serial.begin(9600);

  // Big encoders
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

  pinMode(encA_2, INPUT);                   // sets pin A as input
  digitalWrite(encA_2, LOW);                // turn on pulldown resistors
  pinMode(encB_2, INPUT);                   // sets pin B as input
  digitalWrite(encB_2, LOW);                // turn on pulldown resistors
  attachInterrupt(digitalPinToInterrupt(encA_2), ISR_A_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB_2), ISR_B_2, CHANGE);
  //Motor Pins Setup
  pinMode(ENABLE_2, OUTPUT);
  //setPWMFrequency(3, 2);
  pinMode(dir1_2, OUTPUT);
  pinMode(dir2_2, OUTPUT);
}

void loop()
{ 
  runMotor();
}


// Interrupt service routines for the big motor's quadrature encoder
void ISR_A(){
  encB_set = digitalReadFast(encB);
  encA_set = digitalReadFast(encA);

  enc_counts+=ParseEncoder(encA_prev, encA_set, encB_prev, encB_set);
  
  encA_prev = encA_set;
  encB_prev = encB_set;
}

void ISR_B(){
  // Test transition;
  encB_set = digitalReadFast(encB);
  encA_set = digitalReadFast(encA);

  enc_counts+=ParseEncoder(encA_prev, encA_set, encB_prev, encB_set);
  
  encA_prev = encA_set;
  encB_prev = encB_set;
}

// Interrupt service routines for the small motor's quadrature encoder
void ISR_A_2(){
  encB_set_2 = digitalReadFast(encB_2);
  encA_set_2 = digitalReadFast(encA_2);

  enc_counts_2+=ParseEncoder(encA_prev_2, encA_set_2, encB_prev_2, encB_set_2);
  
  encA_prev_2 = encA_set_2;
  encB_prev_2 = encB_set_2;
}

void ISR_B_2(){
  // Test transition;
  encB_set_2 = digitalReadFast(encB_2);
  encA_set_2 = digitalReadFast(encA_2);

  enc_counts_2+=ParseEncoder(encA_prev_2, encA_set_2, encB_prev_2, encB_set_2);
  
  encA_prev_2 = encA_set_2;
  encB_prev_2 = encB_set_2;
}

//changed ParseEncoder to function passing parameters of prev and set values
//avoids two ParseEncoder function for each motor
int ParseEncoder(int Aprev, int Aset, int Bprev, int Bset){
  if(Aprev && Bprev){
    if(!Aset && Bset) return 1;
    if(Aset && !Bset) return -1;
  }else if(!Aprev && Bprev){
    if(!Aset && !Bset) return 1;
    if(Aset && Bset) return -1;
  }else if(!Aprev && !Bprev){
    if(Aset && !Bset) return 1;
    if(!Aset && Bset) return -1;
  }else if(Aprev && !Bprev){
    if(Aset && Bset) return 1;
    if(!Aset && !Bset) return -1;
  }
}
