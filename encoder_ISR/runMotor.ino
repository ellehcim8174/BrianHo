
void runMotor(){
  //turn CW increasingly to quarter speed
  /*for (int i = 0; i < 64; i++) {
    analogWrite(ENABLE, 32);
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
  for(int i = 0; i < 64; i++) {
 
    Serial.print("Encoder Counts: ");
    Serial.print(enc_counts);
    Serial.print("  Revolutions: ");
    Serial.print(enc_counts/400.0);//400 Counts Per Revolution
    Serial.print("\n");
    //write code for number between -100 & 100. FIX LOGIC BELOW
    //below deal with overshoot
    if(enc_counts >= 100) {
      analogWrite(ENABLE, 32);
      digitalWrite(dir1, LOW);
      digitalWrite(dir2, HIGH);
    }
    else if(enc_counts <= -100){
      analogWrite(ENABLE, 32);
      digitalWrite(dir1, HIGH);
      digitalWrite(dir2, LOW);
    }
   }
    
    
    /*//when reached quarter speed, hard stop
    digitalWrite(ENABLE, HIGH);
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, HIGH);
    delay(1000);*/
}

