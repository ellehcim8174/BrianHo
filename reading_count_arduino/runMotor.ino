void runMotor(){

    //**********************HO HO HO*******************
  
    desired_location_HO = 20*sin(20*millis()/1000.0+10);                                        // sine wave input -- just set whole thing to 80 for a step!
    
    error_HO = desired_location_HO - count_HO;

    // *****   PID   *****
        integral_HO += error_HO*((micros()-last_micros_integral_HO)*0.000001);                      // accumulate ongoing error
          last_micros_integral_HO = micros();
        if (derivative_counter_HO == 10){             
          derivative_HO = (error_HO - last_error_HO)/(((micros()-last_micros_derivative_HO)*0.000001));    // difference in current error minus the last error
          last_micros_derivative_HO = micros();
          last_error_HO = error_HO;                                                                // update last error to current error
          derivative_counter_HO = 0;
        }
        derivative_counter_HO++;
        PID_value_HO = (Kp_HO * error_HO) + (Ki_HO * integral_HO) + (Kd_HO * derivative_HO);                    // updating variable "error" using PID values
    // ***** END PID *****
    
    error_decimal_HO = abs(PID_value_HO)/100;
    raw_PWM_HO =  error_decimal_HO*205+50;                                                          // PWM is scaled by the error, maxing out at 100 slots
    if (raw_PWM_HO  > 255)
      raw_PWM_HO = 255;
      
      //Serial.print(millis());
      //Serial.print("\t");
      //Serial.println(micros()-last_micros_integral_HO);
      
      

    analogWrite(ENABLE_HO, raw_PWM_HO);
      
    if (PID_value_HO >= 0) {
      digitalWrite(dir1_HO, HIGH);
      digitalWrite(dir2_HO, LOW);
    }
    else {
      digitalWrite(dir1_HO, LOW);
      digitalWrite(dir2_HO, HIGH);
    }










    



    //**********************BRIAN*******************

    desired_location_BR = 20*sin(20*millis()/1000.0);                                        // sine wave input -- just set whole thing to 80 for a step!
    
    error_BR = desired_location_BR - count_BR;

    // *****   PID   *****
        integral_BR += error_BR*((micros()-last_micros_integral_BR)*0.000001);                      // accumulate ongoing error
          last_micros_integral_BR = micros();
        if (derivative_counter_BR == 20){             
          derivative_BR = (error_BR - last_error_BR)/(((micros()-last_micros_derivative_BR)*0.000001));    // difference in current error minus the last error
          last_micros_derivative_BR = micros();
          last_error_BR = error_BR;                                                                // update last error to current error
          derivative_counter_BR = 0;
        }
        derivative_counter_BR++;
        PID_value_BR = (Kp_BR * error_BR) + (Ki_BR * integral_BR) + (Kd_BR * derivative_BR);                    // updating variable "error" using PID values
    // ***** END PID *****
    
    error_decimal_BR = abs(PID_value_BR)/100;
    raw_PWM_BR =  error_decimal_BR*175+80;                                                          // PWM is scaled by the error, maxing out at 100 slots
    if (raw_PWM_BR  > 255)
      raw_PWM_BR = 255;
      
      //Serial.print(millis());
      //Serial.print("\t");
      //Serial.print(count_BR);

    analogWrite(ENABLE_BR, raw_PWM_BR);
      
    if (PID_value_BR >= 0) {
      digitalWrite(dir1_BR, HIGH);
      digitalWrite(dir2_BR, LOW);
    }
    else {
      digitalWrite(dir1_BR, LOW);
      digitalWrite(dir2_BR, HIGH);
    }


    /*if(millis() > 10000)
      Serial.end();           // ends display for easy copying after 10 seconds*/
}
