/*
*Quadrature Decoder 
*/
#include "Arduino.h"
#include <digitalWriteFast.h>  

// Quadrature encoders
#define encA 2
#define encB 3
#define LeftEncoderIsReversed
//Motor Driver Pins
#define ENABLE  9
#define dir1    10
#define dir2    11

bool encA_set;
bool encB_set;
bool encA_prev;
bool encB_prev;
long enc_counts = 0;

// Mike's stuff
int desired_location;       // step response input
int current_location;           // feedback
float error;                      // error after subracting negative feedback
int raw_PWM;                    // signed PWM
int PWM_mag;                    // just the magnitude
int step_delay = 50;                 // step doesn't happen at t = 0;
float error_decimal;

void setup()
{
  Serial.begin(9600);

  // Quadrature encoders
  pinMode(encA, INPUT);      // sets pin A as input
  digitalWrite(encA, LOW);  // turn on pulldown resistors
  pinMode(encB, INPUT);      // sets pin B as input
  digitalWrite(encB, LOW);  // turn on pulldown resistors
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
