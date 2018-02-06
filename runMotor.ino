
void runMotor(){
  //turn CW increasingly to quarter speed
  /*for (int i = 0; i < 64; i++) {
    analogWrite(ENABLE, 128);
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
    delay(100);
  }
   /*
   //when reached quarter speed, hard stop
   digitalWrite(ENABLE, HIGH);
   digitalWrite(dir1, HIGH);
   digitalWrite(dir2, HIGH);
   delay(1000);
   */
  
   //turn CCW increasingly to quarter speed
  // for(int i = 0; i < 64; i++) {
 
    Serial.print("Encoder Counts: ");
    Serial.print(enc_counts);
    Serial.print("  Revolutions: ");
    Serial.print(enc_counts/400.0);//400 Counts Per Revolution
    Serial.print("\n");
    //write code for number between -100 & 100. FIX LOGIC BELOW
    //below deal with overshoot


    
    /*if(enc_counts >= 100) {
      analogWrite(ENABLE, 32);
      digitalWrite(dir1, LOW);
      digitalWrite(dir2, HIGH);
    }
    else if(enc_counts <= -100){
      analogWrite(ENABLE, 32);
      digitalWrite(dir1, HIGH);
      digitalWrite(dir2, LOW);
    }


    
   //}
    
    /*
    //when reached quarter speed, hard stop
    digitalWrite(ENABLE, HIGH);
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, HIGH);
    delay(1000);*/

    step_delay = step_delay - 1;  // delays the input step by a second or two
    if (step_delay == 0)
      desired_location = 60;
 

    current_location = enc_counts;
    error = desired_location - current_location;
    
    
    raw_PWM = 100 * (error/60); // PWM is scaled by the error, maxing out at 60 slots
    
    if (raw_PWM <= 31 && raw_PWM >= 0)  // this stuff sets the absolute value of raw_PWM to at least 26
      raw_PWM = 32;
    else if (raw_PWM >= -31 && raw_PWM < 0)
      raw_PWM = -32;

    if (error < 5 && error > -5)
      raw_PWM = 0;
      
    if (error >= 0) {
      digitalWrite(dir1, HIGH);
      digitalWrite(dir2, LOW);
    }
    else if (error < 0) {
      digitalWrite(dir1, LOW);
      digitalWrite(dir2, HIGH);
    }
    PWM_mag = abs(raw_PWM);
    analogWrite(ENABLE, PWM_mag);
}

