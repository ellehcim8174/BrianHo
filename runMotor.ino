
void runMotor(){
   
      desired_location = 50;
 

    error = desired_location - enc_counts;

    // *****   PID   *****
        /*integral += error*((micros()-last_micros_integral)*0.000001);                      // accumulate ongoing error
          last_micros_integral = micros();
        derivative = (error - last_error)/((micros()-last_micros_derivative)*0.000001);    // difference in current error minus the last error
          last_micros_derivative = micros();
        last_error = error;                                                                // update last error to current error
        PID = (Kp * error) + (Ki * integral) + (Kd * derivative);                       // updating variable "error" using PID values*/

        integral += error;
        if (derivative_counter == 40){             
          derivative = (error - last_error)/40;
          last_error = error;                                                                // update last error to current error
          derivative_counter = 0;
        }
        derivative_counter++;
        PID = (Kp * error) + (Ki * integral) + (Kd * derivative);                       // updating variable "error" using PID values
    // ***** END PID *****
    
    error_decimal = abs(error)/100;
    raw_PWM =  error_decimal*255; //* 226 + 29; // PWM is scaled by the error, maxing out at 100 slots
    if (raw_PWM > 255)
      raw_PWM = 255;

    //Serial.println(enc_counts);

    analogWrite(ENABLE, raw_PWM);
      
    if (error >= 0) {
      digitalWrite(dir1, HIGH);
      digitalWrite(dir2, LOW);
    }
    else {
      digitalWrite(dir1, LOW);
      digitalWrite(dir2, HIGH);
    }

    //if(millis() > 7000)
     // Serial.end();           // ends display for easy copying after 7 seconds
}

