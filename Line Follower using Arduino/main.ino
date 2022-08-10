#include<Servo.h>
#include <QTRSensors.h>

Servo servo1, servo2, servo3;  //defining three servo motor

const uint8_t SensorCount = 6; //selecting a number of sensors 
uint16_t sensorValues[SensorCount];
int right = 0;                          //Counter used for detecting an object on right
int left = 0;                            //Counter used for detecting an object on left
int straight = 0;                     //Counter used for detecting an object on the straight path
const int pingPin = 9;           // Assigning Arduino pin 9 to the Ping Sensor
int object_count = 0;            // Defining variable to calculate the number of objects
const int  redLED = 11;       // Assigning Arduino pin 11 to the Red LED
const int  greenLED = 13;   // Assigning Arduino pin 13 to the Green LED
const int yellowLED = 12;  // Assigning Arduino pin 12 to the Red LED


void setup() 
{  
  Serial.begin(9600);
  
   //configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
 calibrate();
  
  //attach servos to pins
  servo1.attach(3);
  servo2.attach(5);
  servo3.attach(6);
  
  // Initialize with speed 0 and angle 0
  servo1.write(94);
  servo2.write(93);
  servo3.write(90);

  //pin configuration
  pinMode(redLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(greenLED, OUTPUT);

  //first intersection
  intersect();
  servo1.write(94);
  servo2.write(93);
  digitalWrite(redLED, HIGH);

  delay(3000);
  digitalWrite(redLED, LOW);
  forward();

}
void loop() 
{
  intersect();
  servo1.write(94);
  servo2.write(93);
  digitalWrite(redLED, HIGH);
  delay(3000);
  digitalWrite(redLED, LOW);
  
  int object_count = ultrasonicSweep(); 
  delay(1000);

}

//ULTRASONIC SWEEP
int ultrasonicSweep()
{
  long det_loc = 52;

  // Detect object at RIGHT
  servo3.write(0);
  delay(2000);
  long loc_comp = ping();
  if (loc_comp < det_loc)
  {
    right = 1;
    object_count = object_count + 1;
    digitalWrite(greenLED, HIGH);
    delay(2000);
    digitalWrite(greenLED, LOW);
    
  }

  // detect object at FRONT
  servo3.write(90);
  delay(2000);
  loc_comp = ping();
  if (loc_comp < det_loc)
  {
    straight = 1;
    object_count = object_count + 1;
    digitalWrite(greenLED, HIGH);
    delay(2000);
    digitalWrite(greenLED, LOW);
  }

   // detect object at left
  servo3.write(180);
  delay(2000);
  loc_comp = ping();
  if (loc_comp < det_loc)
  {
    left = 1;
    object_count = object_count + 1;
    digitalWrite(greenLED, HIGH);
    delay(2000);
    digitalWrite(greenLED, LOW);
  }

  //Servo3 Back to home position
  servo3.write(90);
  delay(2000);

  if (left ==0 && right == 0)       //when no object is detected either on left or right side
  {
    forward();
  }

  // Moving towards the object on right side
  if (right==1)
  {
     qtr.read(sensorValues);
     
     //turn right by 90, facing the object       
     while(sensorValues[3]>200)
     {
      qtr.read(sensorValues);
      Serial.print(greeting);
      servo1.write(100);
      servo2.write(93);
     }

     while(sensorValues[3]<150)
     {
      qtr.read(sensorValues);
      Serial.print(greeting);
      servo1.write(100);
      servo2.write(93);
     }

     right = 0;
     
     //move towards the object upto 8 cm
     loc_comp = ping();
     while (loc_comp > 8)
     {
        loc_comp = ping();
        qtr.read(sensorValues);
        line_follow(sensorValues); 
     }

     //stop the robot and indicate
     stop_bot();
     digitalWrite(greenLED, HIGH);
     delay(2000);
     digitalWrite(greenLED, LOW);

     //turn the ROBOT 180
     while (sensorValues[0]<150)
     {
        qtr.read(sensorValues);
        servo1.write(114);
        servo2.write(114);
     }

     while (sensorValues[0]>200)
     {
        qtr.read(sensorValues);
        bot_rotate(); 
     }
    
     //move back to intersection
     intersect();
     stop_bot();

     //Go towards object on the left
     if (left == 1)
     {
        left=0;
       //move towards the object upto 8 cm
        loc_comp = ping();
        while (loc_comp > 8)
        {
          loc_comp = ping();
          qtr.read(sensorValues);
          line_follow(sensorValues); 
        }
        

        //stop the robot and indicate for object
        stop_bot();
        digitalWrite(greenLED, HIGH);
        delay(2000);
        digitalWrite(greenLED, LOW);
     
        //turn the ROBOT 180
        while (sensorValues[0]<150)
        {
          qtr.read(sensorValues);
          servo1.write(114);
          servo2.write(114);
        }

        while (sensorValues[0]>200)
        {
          qtr.read(sensorValues);
          bot_rotate(); 
        }
    
        //move back to intersection
        intersect();
        stop_bot();

        while(sensorValues[1]>200)
        {
            qtr.read(sensorValues);
            Serial.print(greeting);
            servo1.write(94);
            servo2.write(85);
        }

        while(sensorValues[1]<150)
        {
            qtr.read(sensorValues);
            Serial.print(greeting);
            servo1.write(94);
            servo2.write(85);
            Serial.print(greeting);
        }
     }

     
     else
     {
      while(sensorValues[3]>200)
      {
          qtr.read(sensorValues);
          Serial.print(greeting);
          servo1.write(100);
          servo2.write(93);
      }

      while(sensorValues[3]<150)
      {
        qtr.read(sensorValues);
        Serial.print(greeting);
        servo1.write(100);
        servo2.write(93);
      }

      intersect();
      stop_bot();
      
     }
      
  }

  else if(left == 1 && right == 0)
  {
     //turn left by 90, facing the object       
     Serial.print(greeting);
     while(sensorValues[1]>200)
     {
      qtr.read(sensorValues);
      Serial.print(greeting);
      servo1.write(94);
      servo2.write(85);
     }

     while(sensorValues[1]<10)
     {
      qtr.read(sensorValues);
      Serial.print(greeting);
      servo1.write(94);
      servo2.write(85);
      Serial.print(greeting);
     }
     
     left = 0;

     //move towards the object upto 8 cm
     loc_comp = ping();
     while (loc_comp > 8)
     {
        loc_comp = ping();
        qtr.read(sensorValues);
        line_follow(sensorValues); 
     }

     //stop the robot and indicate
     stop_bot();
     digitalWrite(greenLED, HIGH);
     delay(2000);
     digitalWrite(greenLED, LOW);

                                  //turn the ROBOT 180
     while (sensorValues[0]<150)
     {
        qtr.read(sensorValues);
        bot_rotate();
     }

     while (sensorValues[0]>200)
     {
        qtr.read(sensorValues);
        bot_rotate(); 
     }

     //move back to intersection
     intersect();
     stop_bot();

     //turn left by 90, facing the object       
     while(sensorValues[1]>200)
     {
      qtr.read(sensorValues);
      Serial.print(greeting);
      servo1.write(94);
      servo2.write(85);
     }

     while(sensorValues[1]<130)
     {
      qtr.read(sensorValues);
      Serial.print(greeting);
      servo1.write(94);
      servo2.write(85);
     }
     
     intersect();
     stop_bot();
     //forward();    
  }

  if (straight == 1)
  {
   
//move towards the object upto 8 cm
     loc_comp = ping();
     while (loc_comp > 8)
     {
        loc_comp = ping();
        qtr.read(sensorValues);
        line_follow(sensorValues); 
     }
     straight = 0;
     stop_bot();
     digitalWrite(greenLED, HIGH);
     delay(2000);
     digitalWrite(greenLED, LOW);

     //turn the ROBOT 180
     while (sensorValues[0]<150)
     {
        qtr.read(sensorValues);
        bot_rotate();
     }

     while (sensorValues[0]>200)
     {
        qtr.read(sensorValues);
        bot_rotate(); 
     }
     
     stop_bot();
     delay(10000);
    
  }

}

// LINE FOLLOW
void intersect()
{
  while (true)
  {
    qtr.read(sensorValues);

    line_follow(sensorValues);
    if (sensorValues[1] > 220 && sensorValues[4] > 220 && sensorValues[0]>220 && sensorValues[5]>220)
    {
      break;
    }
 
  }
}

void  line_follow(uint16_t sensorValues[6]){
  qtr.read(sensorValues);
  if (sensorValues[1] > 220 && sensorValues[3] > 220)
  {
    servo1.write(105);
    servo2.write(83);
    qtr.read(sensorValues);
  }
  if (sensorValues[1]>220 && sensorValues[3]<130)
  {   
      servo1.write(100);  //slight_right
      servo2.write(93);
  }
  else if(sensorValues[3]>220 && sensorValues[1]<130)
  {
     servo1.write(94); //slight_left
     servo2.write(73);  
  } 
}

//FORWARD MOTION OF ROBOT
void forward()
{
  servo1.write(105);
  servo2.write(82);
  delay(500);
}

//STOP MOTION OF ROBOT
void stop_bot()
{
  servo1.write(94);
  servo2.write(93);
  delay(2000);
}

void bot_rotate()
{
  servo1.write(114);
  servo2.write(114);
}

//ULTRASONIC SENSOR READINGS
long ping(){
  long duration, cm; 
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  
  cm = microsecondsToCentimeters(duration);

  //Serial.print(cm);
  Serial.println();

  delay(20);
  return cm;
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29/ 2;
}
