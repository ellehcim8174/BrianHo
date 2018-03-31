#include "Arduino.h"

//motor driver pins 
#define ENABLE    5   //PWM pin
#define dir1      6
#define dir2      7
//encoder reading variables
double  count;
int     SEL1 = 11;
//int   SEL2 = 12;    //SEL2 always low to read 3rd byte and LSB, keep pin GND
int     OE = 13;
byte    result_high;
byte    result_low;
byte    high_new;
byte    high_old;
byte    low_new;
byte    low_old;
byte    byte_data;

//runMotor, PID variables
double  desired_location = 80;             // step response input
int     current_location;                  // feedback
double  error;                             // error after subracting negative feedback
double  PID_value;                         // PID value for input
long    raw_PWM;                           // signed PWM
int     PWM_mag;                           // just the magnitude
float   error_decimal;                     // for adjusting gain of input to system

float   Kp = 0.9;                          // Proportional gain, 0.211 (up to 0.9)
float   Ki = 0.170;                        // Integrator gain, 0.170
float   Kd = 0.058;                        // Derivative gain, 0.058

/*float Kp = 1;                            // only P
float   Ki = 0;               
float   Kd = 0;  */

double  integral = 0;                      // Integrator term
double  derivative = 0;                    // Derivative term 
double  last_error = 0;                    // Used for saving last error to calculate derivative
long    last_micros_integral = 0;          // microseconds since last loop of motor.ino integrator calculation
long    last_micros_derivative = 0;        // microseconds sine last loop of motor.ino derivative calculation
int     derivative_counter = 0;            // sets a delay so we can get somewhat of a real derivative


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //pins for reading decoder
  pinMode(30, INPUT);
  pinMode(31, INPUT); 
  pinMode(32, INPUT);
  pinMode(33, INPUT); 
  pinMode(34, INPUT);
  pinMode(35, INPUT); 
  pinMode(36, INPUT);
  pinMode(37, INPUT);
  pinMode(SEL1, OUTPUT);
  //pinMode(SEL2, OUTPUT);
  pinMode(OE, OUTPUT);

  //pins for bottom motor
  pinMode(ENABLE, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Input to decoder chip to select upper byte 
  digitalWrite(OE, LOW);
  digitalWrite(SEL1, LOW);
  //digitalWrite(SEL2, LOW);
  get_high();

  //Input to chip to select lower byte
  digitalWrite(SEL1, HIGH);
  //digitalWrite(SEL2, LOW);
  get_low();
 
  digitalWrite(OE, HIGH);

  //run motor and get count readings
  runMotor();
  //data_manipulations
  count = (result_high<<8) | result_low;   //concatenating upper byte and lower byte to 16 bits
  count = double(count);
  Serial.println(count);
}

//function for getting 2nd LSB
void get_high (){
  high_new = get_byte();
  high_old = get_byte();

  while (!(high_new == high_old)){
    high_new = get_byte();
    high_old = get_byte();
    }
  result_high = high_new ;
  return;
}

//function for getting LSB
void get_low () {
  low_new = get_byte();
  low_old = get_byte();
  
  while (!(low_new == low_old)){
    low_new = get_byte();
    low_old = get_byte();
    }
  result_low = low_new ;
  return;
}

//function for converting 8-bits to a byte
byte get_byte (){
  bitWrite(byte_data, 0, digitalRead(30));
  bitWrite(byte_data, 1, digitalRead(31));
  bitWrite(byte_data, 2, digitalRead(32));
  bitWrite(byte_data, 3, digitalRead(33));
  bitWrite(byte_data, 4, digitalRead(34));
  bitWrite(byte_data, 5, digitalRead(35));
  bitWrite(byte_data, 6, digitalRead(36));
  bitWrite(byte_data, 7, digitalRead(37));   
  return byte_data;     
}


