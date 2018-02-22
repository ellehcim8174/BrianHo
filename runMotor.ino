
void runMotor(){
   
 
    /*Serial.print("Encoder Counts: ");
    Serial.print(enc_counts);
    Serial.print("  Revolutions: ");
    Serial.print(enc_counts/400.0);//400 Counts Per Revolution
    Serial.print("\n");*/
  
    //step_delay = step_delay - 1;  // delays the input step by a second or two
    //if (step_delay == 0)
      desired_location = 50;
 

    current_location = enc_counts;
    error = desired_location - current_location;
    
    
    error_decimal = abs(error)/100;
    raw_PWM =  error_decimal * 226 + 29; // PWM is scaled by the error, maxing out at 60 slots
    if (raw_PWM > 255)
      raw_PWM = 255;

    Serial.println(current_location);

 

    analogWrite(ENABLE, raw_PWM);
      
    if (error >= 0) {
      digitalWrite(dir1, HIGH);
      digitalWrite(dir2, LOW);
    }
    else {
      digitalWrite(dir1, LOW);
      digitalWrite(dir2, HIGH);
    }
}

