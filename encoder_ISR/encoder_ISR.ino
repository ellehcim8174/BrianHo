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

volatile bool encA_set;
volatile bool encB_set;
volatile bool encA_prev;
volatile bool encB_prev;
volatile long enc_counts = 0;

void setup()
{
  Serial.begin(9600);

  // Quadrature encoders
  pinMode(encA, INPUT);      // sets pin A as input
  digitalWrite(encA, LOW);  // turn on pulldown resistors
  pinMode(encB, INPUT);      // sets pin B as input
  digitalWrite(encB, LOW);  // turn on pulldown resistors
  attachInterrupt(EncInterruptA, ISR_A, CHANGE);
  attachInterrupt(EncInterruptB, ISR_B, CHANGE);
  //Motor Pins Setup
  pinMode(ENABLE, OUTPUT);
  //setPWMFrequency(3, 2);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
}

void loop()
{ 
  runMotor();/*
  Serial.print("Encoder Counts: ");
  Serial.print(enc_counts);
  Serial.print("  Revolutions: ");
  Serial.print(enc_counts/400.0);//400 Counts Per Revolution
  Serial.print("\n");*/
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
