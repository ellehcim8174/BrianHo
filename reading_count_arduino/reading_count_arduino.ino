#include "Arduino.h"

//Brian motor driver pins 
#define ENABLE_BR    5   //PWM pin
#define dir1_BR      7
#define dir2_BR      6
//Ho motor driver pins
#define ENABLE_HO    2
#define dir1_HO      3
#define dir2_HO      4


//Brian encoder select/OE pins
double  count_BR = 0;
int     SEL1_BR = 11;
//int   SEL2_BR = 12;    //SEL2 always low to read 3rd byte and LSB, keep pin GND
int     OE_BR = 13;
int     RST_BR = 12;
//Ho encoder select/OE pins
double  count_HO = 0;
int     SEL1_HO = 8;
//int   SEL2_HO = 9;
int     OE_HO = 10;
int     RST_HO = 9;

//bit reading variables
byte    result_high_BR;
byte    result_low_BR;
byte    result_high_HO;
byte    result_low_HO;
byte    high_new_BR;
byte    high_old_BR;
byte    high_new_HO;
byte    high_old_HO;
byte    low_new_BR;
byte    low_old_BR;
byte    low_new_HO;
byte    low_old_HO;
byte    byte_data_BR;
byte    byte_data_HO;

//runMotor, PID variables
int  desired_location_HO = 0;           // step response input
int  desired_location_BR = 0;           // step response input
double  error_HO;                          // error after subracting negative feedback
double  error_BR;
double  PID_value_HO;                      // PID value for input
double  PID_value_BR;                      // PID value for input
long    raw_PWM_HO;
long    raw_PWM_BR;                        // signed PWM
float   error_decimal_HO;                  // for adjusting gain of input to system
float   error_decimal_BR;                  // for adjusting gain of input to system

float   Kp_HO = 2.058;                     // Proportional gain, 0.211 (up to 0.9)
float   Ki_HO = 9.354;                     // Integrator gain, 0.170
float   Kd_HO = 0.111;                     // Derivative gain, 0.058

float   Kp_BR = 0.864;                       // Proportional gain, 0.864
float   Ki_BR = 0.512;                     // Integrator gain, 0.512
float   Kd_BR = 0.316;                     // Derivative gain, 0.316

/*float   Kp_HO = 1;                     // ONLY FEEDBACK
float   Ki_HO = 0;                     
float   Kd_HO = 0;                     

float   Kp_BR = 1;                       
float   Ki_BR = 0;                     
float   Kd_BR = 0;                          */


double  integral_HO = 0;                      // Integrator term
double  integral_BR = 0;                      // Integrator term
double  derivative_HO = 0;                    // Derivative term 
double  derivative_BR = 0;                    // Derivative term 
double  last_error_HO = 0;                    // Used for saving last error to calculate derivative
double  last_error_BR = 0;                    // Used for saving last error to calculate derivative
long    last_micros_integral_HO = 0;          // microseconds since last loop of motor.ino integrator calculation
long    last_micros_integral_BR = 0;          // microseconds since last loop of motor.ino integrator calculation
long    last_micros_derivative_HO = 0;        // microseconds sine last loop of motor.ino derivative calculation
long    last_micros_derivative_BR = 0;        // microseconds sine last loop of motor.ino derivative calculation
int     derivative_counter_HO = 0;            // sets a delay so we can get somewhat of a real derivative
int     derivative_counter_BR = 0;            // sets a delay so we can get somewhat of a real derivative


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //pin declaration for Brian Motor
  pinMode(30, INPUT);
  pinMode(32, INPUT); 
  pinMode(34, INPUT);
  pinMode(36, INPUT); 
  pinMode(38, INPUT);
  pinMode(40, INPUT); 
  pinMode(42, INPUT);
  pinMode(44, INPUT);
  pinMode(SEL1_BR, OUTPUT);
  //pinMode(SEL2, OUTPUT);
  pinMode(OE_BR, OUTPUT);
  pinMode(ENABLE_BR, OUTPUT);
  pinMode(dir1_BR, OUTPUT);
  pinMode(dir2_BR, OUTPUT);
  digitalWrite(RST_BR, LOW);

  //pin declaration for Ho Motor
  pinMode(31, INPUT);
  pinMode(33, INPUT); 
  pinMode(35, INPUT);
  pinMode(37, INPUT); 
  pinMode(39, INPUT);
  pinMode(41, INPUT); 
  pinMode(43, INPUT);
  pinMode(45, INPUT);
  pinMode(SEL1_HO, OUTPUT);
  //pinMode(SEL2_HO, OUTPUT);
  pinMode(OE_HO, OUTPUT);
  pinMode(ENABLE_HO, OUTPUT);
  pinMode(dir1_HO, OUTPUT);
  pinMode(dir2_HO, OUTPUT);
  digitalWrite(RST_HO, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  // software reset
  digitalWrite(RST_BR, HIGH);
  digitalWrite(RST_HO, HIGH);
  //Input to decoder chip to select upper byte 
  digitalWrite(OE_BR, LOW);
  digitalWrite(SEL1_BR, LOW);
  //digitalWrite(SEL2_BR, LOW);
  get_high_BR();
  digitalWrite(OE_HO, LOW);
  digitalWrite(SEL1_HO, LOW);
  //digitalWrite(SEL2_HO, LOW);
  get_high_HO();

  //Input to chip to select lower byte
  digitalWrite(SEL1_BR, HIGH);
  //digitalWrite(SEL2_BR, LOW);
  get_low_BR();
  digitalWrite(SEL1_HO, HIGH);
  //digitalWrite(SEL2_HO, LOW);
  get_low_HO();
 
  digitalWrite(OE_BR, HIGH);
  digitalWrite(OE_HO, HIGH);

  //run motor and get count readings
  runMotor();
  //data_manipulations
  count_BR = (result_high_BR<<8) | result_low_BR;   //concatenating upper byte and lower byte to 16 bits - BRIAN
  count_BR = double(count_BR);
  count_HO = (result_high_HO<<8) | result_low_HO;   //concatenating upper byte and lower byte to 16 bits - HO
  count_HO = double(count_HO);
  Serial.print(count_BR);
  Serial.print("\t");
  Serial.println(count_HO);
}

//function for getting 2nd LSB
void get_high_BR (){
  high_new_BR = get_byte_BR();
  high_old_BR = get_byte_BR();

  while (!(high_new_BR == high_old_BR)){
    high_new_BR = get_byte_BR();
    high_old_BR = get_byte_BR();
    }
  result_high_BR = high_new_BR ;
  return;
}
void get_high_HO (){
  high_new_HO = get_byte_HO();
  high_old_HO = get_byte_HO();

  while (!(high_new_HO == high_old_HO)){
    high_new_HO = get_byte_HO();
    high_old_HO = get_byte_HO();
    }
  result_high_HO = high_new_HO ;
  return;
}

//function for getting LSB
void get_low_BR () {
  low_new_BR = get_byte_BR();
  low_old_BR = get_byte_BR();
  
  while (!(low_new_BR == low_old_BR)){
    low_new_BR = get_byte_BR();
    low_old_BR = get_byte_BR();
    }
  result_low_BR = low_new_BR ;
  return;
}
void get_low_HO () {
  low_new_HO = get_byte_HO();
  low_old_HO = get_byte_HO();
  
  while (!(low_new_HO == low_old_HO)){
    low_new_HO = get_byte_HO();
    low_old_HO = get_byte_HO();
    }
  result_low_HO = low_new_HO ;
  return;
}
//function for converting 8-bits to a byte
byte get_byte_BR (){
  bitWrite(byte_data_BR, 0, digitalRead(30));
  bitWrite(byte_data_BR, 1, digitalRead(32));
  bitWrite(byte_data_BR, 2, digitalRead(34));
  bitWrite(byte_data_BR, 3, digitalRead(36));
  bitWrite(byte_data_BR, 4, digitalRead(38));
  bitWrite(byte_data_BR, 5, digitalRead(40));
  bitWrite(byte_data_BR, 6, digitalRead(42));
  bitWrite(byte_data_BR, 7, digitalRead(44));   
  return byte_data_BR;     
}
byte get_byte_HO (){
  bitWrite(byte_data_HO, 0, digitalRead(31));
  bitWrite(byte_data_HO, 1, digitalRead(33));
  bitWrite(byte_data_HO, 2, digitalRead(35));
  bitWrite(byte_data_HO, 3, digitalRead(37));
  bitWrite(byte_data_HO, 4, digitalRead(39));
  bitWrite(byte_data_HO, 5, digitalRead(41));
  bitWrite(byte_data_HO, 6, digitalRead(43));
  bitWrite(byte_data_HO, 7, digitalRead(45));   
  return byte_data_HO;     
}

