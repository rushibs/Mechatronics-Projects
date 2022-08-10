Serial.println(servopos);
// sets the servo position according to the scaled value 

Serial.println (myservo.read()); 
delay(100); 
analogReading = analogRead(FORCE SENSOR PIN); 
buttonstat = digitalRead(buttonPin);
delay(5000);

int analogReading = analogRead (FORCE_SENSOR_PIN);
myservo.write (180); //goes back to default position 

int buttonstat = digitalRead(buttonPin); 
while((analogReading > forceLimit) && (buttonstat == LOW)) {
// force sensor reading 
    Serial.println("Force"); 
    Serial.println(analogReading); 
    // flex sensor reading 
    int flexpos = analogRead(flexPin); 
    Serial.println("Flex"); 
    Serial.println(flexpos); 
    int servopos = map (flexpos, 0, 850, 0, 180); 
    servopos = constrain (servopos, 0, 180); 
    int x = 1; 

    for (int i = myservo.read(); i != servopos; i = i+x) { 
        buttonstat = digitalRead(buttonPin); 
        if (buttonstat == LOW){
            analogReading = analogRead(FORCE SENSOR PIN); 
            if (analogReading > forceLimit) { 
                myservo.write(i);
            } else {
                myservo.write(180); 
                break;
            }
            if (myservo.read() > servopos) {
                x = -1; // change direction of increment
                }
                delay(10);  
            } else {
                myservo.write(180); 
                break;
            }
}
Serial.println(servopos);

