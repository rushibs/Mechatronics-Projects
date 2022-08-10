// LINE FOLLOWING USING PARALLAX PROPELLER

#include "simpletools.h"                      // Include simple tools
#include "abdrive.h"                          // abdrive library
#include "servo.h"
#include "ping.h"                             // Include ping header
#include "adcDCpropab.h"
 
float * read_line_sensor();
void  line_follow();
void detect_enemy();
void object_detection(void *par);
void intersection_led_blink(void *par);
void bot_forward(int speed_value);
void bot_back(int speed_value);
void navigate(void *par1);
void bot_stop();
void i2_routine();
void i3_routine();
void i5_routine();
void bot_180();
void b1_b4();
void b4_a4();
void b4_a4_for_i2();
void a1_b1();
void b1_b5();
int distance();
int object_detect();
volatile int count_intersect = 0;
volatile int count_object = 0;
int count_intersect_b1b4 = 0;
unsigned int stack1[40+25];
unsigned int stack2[40+25];
int object;
int status;
int cmDist;
float v[4];
//float v_l[4];
volatile int detection = 0;
volatile int intersect = 0;
volatile int dist;
int flag = 0;
 
int main()                                    // Main function
{
 
   cogstart((void*)navigate, NULL, stack2, sizeof(stack2));
   cogstart((void*)intersection_led_blink,NULL, stack1,sizeof(stack1));
 
}
 
// read line sensor values in terms of voltage---------------------------------------------------
float * read_line_sensor()
{
      adc_init(21,20,19,18);
      //float v0,v1,v2,v3;
      v[0] = adc_volts(0);
      v[1] = adc_volts(1);
      v[2] = adc_volts(2);
      v[3] = adc_volts(3);
      //print("A/D2 = %f V%c\n", v0, CLREOL);     // Display volts
      //print("A/D3 = %f V%c\n", v1, CLREOL);
      //print("A/D3 = %f V%c\n", v2, CLREOL);
      //print("A/D3 = %f V%c\n", v3, CLREOL);
      return v;
}
 
//line follower--------------------------------------------------------------------------------------
void  line_follow()
{
    //2  6 10 14
    float * v;
    v = read_line_sensor();
    //print("A/D3 = %f V%c\n", v[2], CLREOL);
     
    if (v[1] > 1 && v[2] > 1)
    {  
        bot_forward(50);            
    }
    else if (v[1]>2 && v[2]<1)
    {  
       servo_set(12,1400); //slight_left
       servo_set(13,1370);    
    }
    else if(v[2]>2 && v[1]<1)
    {
       servo_set(12,1430);  //slight_right
       servo_set(13,1400);
 
     }
}
 
int distance()
{
  low(6);
  pulse_out(6,10);
  volatile long techo = pulse_in(5,1);
  volatile int dist = techo / 58;
  return dist;  
}
 
 
 
// robot forward motion-------------------------------------------------------------------------
void bot_forward(int speed_value)
{
  int left_motor_speed, right_motor_speed;
  left_motor_speed = 1400 + speed_value;
  right_motor_speed = 1400 - speed_value;
  servo_set(12, left_motor_speed);
  servo_set(13,right_motor_speed);
}
 
// robot backwards motion
void bot_back(int speed_value)
{
  int left_motor_speed, right_motor_speed;
  left_motor_speed = 1400 - speed_value;
  right_motor_speed = 1400 + speed_value;
  servo_set(12, left_motor_speed);
  servo_set(13,right_motor_speed);
}
 
// turn the robot left ------------------------------------------------------------------------------
void bot_left()
{
  float * v;
  v = read_line_sensor();
  while(v[2]>2)
  {
    float * v;
    v = read_line_sensor();
    //printf("%d \n", v[1]);
    servo_set(12,1400);
    servo_set(13,1350);
  }
 
  while(v[2]<2)
  {
    float * v;
    v = read_line_sensor();
    //printf("low= %d \n", v[1]);
    servo_set(12,1400);
    servo_set(13,1350);
  }  
}
 
//turn the robot right------------------------------------------------------------------------------------------
void bot_right()
{
  float * v;
  v = read_line_sensor();
  while(v[1]>2)
  {
    float * v;
    v = read_line_sensor();
    servo_set(12,1450);
    servo_set(13,1400);
  }
  while(v[1]<2)
  {
    float * v;
    v = read_line_sensor();
    servo_set(12,1450);
    servo_set(13,1400);
  }
}
 
// Turn the BOT by 180 degrees
void bot_180()
{
  float * v;
  v = read_line_sensor();
  while(v[3]<1)
  {
    float * v;
    v = read_line_sensor();
    servo_set(12,1430);
    servo_set(13,1430);
  }
  print("%f", cmDist);
  while(v[3]>1)
  {
    float * v;
    v = read_line_sensor();
    servo_set(12,1430);
    servo_set(13,1430);
  }
  print("%f", cmDist);
 
}
 
//stop the robot-----------------------------------------------------------------------------------------------------------
void bot_stop()
{
  servo_set(12,1400);
  servo_set(13,1400);
}
 
// Intersection LED Blink Function--------------------------------------------------------------------------
void intersection_led_blink(void *par)
{
   ////print("%d \n", intersect);
 
    while(1)
   {  
        while(intersect)
        {
            high(2);
            pause(200);
            low(2);
            pause(200);        
        }    
        //intersect = 0;    
    }        
 
}
 
 
//obstacle LED blink ---------------------------------------------------------------------------------
void obstacle_led_blink()
{
    high(4);
    //pause(50);
}
 
void detect_enemy()
{  
   set_direction(11,0);
   int distance = object_detect();
   if (distance < 13 && flag==0)
   {
     bot_stop();
     pause(500);
      if (input(11) == 1)
      {
       bot_back(30);
       pause(100);
       
       servo_angle(15,1800);
       flag = 1;
       
       //pause(2500);
       //bot_forward(50);
       //pause(200);
       
      }      
    }
}        
 
// Detecting object using Ultrasonic Sensor
int object_detect()
{
  low(6);
  pulse_out(6,10);
  volatile long tEcho = pulse_in(5,1);
  volatile int distance = tEcho/58;
  return distance;
  }
 
 
// B1 to B4 Routine
void b1_b4()
{
  bot_forward(30);
  pause(500);
  count_intersect = 0;
  while(1)
  {
  do{
    line_follow();
    detect_enemy();
   }while(v[0]<1 && v[3]<1);
  intersect = 1;
  flag = 0;
  servo_angle(15,900);
  count_intersect = count_intersect + 1;
  if (count_intersect == 3)
    {
      break;
    }
  bot_forward(30);
  pause(500);
  intersect = 0;
  }  
}
 
//B4 to A4 via F4 and F3 for obstacle at i3
void b4_a4()
{
  do{
    line_follow();
    if (v[0]>2 && v[1]>2 && v[3]>2 && v[2]>2)
    {
      intersect = 1;
      break;
    }    
    }while(1);
  bot_right();               ///Right from i4 towards i3
  intersect = 0;
  do{
    line_follow();
    detect_enemy();
    cmDist = ping_cm(10);
    }while(cmDist>4);
  flag = 0;
  servo_angle(15, 900);
  bot_180();
  do{
    line_follow();
    detect_enemy();
    }while(v[0]<1 && v[3]<1);   // Came back to i4
  intersect =1;
  flag = 0;
  servo_angle(15, 900);
  bot_forward(50);
  pause(500);
  intersect = 0;
  do{
    line_follow();
    detect_enemy();
    }while(v[0]<1 && v[3]<1);   // reached i5
  intersect = 1;
  flag=0;
  servo_angle(15, 900);
  bot_back(50);
  pause(600);
  bot_180();
  intersect = 0;
  do{
    line_follow();
    detect_enemy();
    }while(v[0]<1 && v[3]<1);   //reached i4 again
  intersect = 1;
  flag=0;
  servo_angle(15, 900);
  bot_left();
  do{
    line_follow();
    }while(v[0]<1 && v[3]<1);  // reached a4
  intersect = 1;
}
 
 
 
//B4 to A4 for obstacle at i2
 
void b4_a4_for_i2()
{
  do{
    line_follow();
    if (v[0]>2 && v[1]>2 && v[3]>2 && v[2]>2)
    {
      intersect = 1;
      break;
    }    
    }while(1);
  bot_right();               ///Right from i4 towards i3
  intersect = 0;
  flag = 0;
  servo_angle(15,900);
 
  do{
    line_follow();
    detect_enemy();
    }while(v[0]<1 && v[3]<1);   // beyond i4 towards i3
  intersect = 1;
  flag = 0;
  servo_angle(15,900);
  bot_forward(50);
  pause(500);
  intersect = 0;
 
  do{
    line_follow();
    detect_enemy();
    cmDist = ping_cm(10);   //i3 to i2
    }while(cmDist>5);
  flag = 0;
  servo_angle(15,900);
 
  bot_180();
  do{
    line_follow();
    detect_enemy();
    }while(v[0]<1 && v[3]<1);   // Came back to i3
  intersect = 1;
  flag = 0;
  servo_angle(15,900);
 
  bot_forward(50);
  pause(500);
  intersect = 0;
  do{
    line_follow();
    detect_enemy();
    }while(v[0]<1 && v[3]<1);   // reached i4
  intersect = 1;
  flag=0;
  servo_angle(15, 900);
  bot_forward(50);
  pause(500);
  intersect = 0;
 
  do{
    line_follow();
    detect_enemy();
    }while(v[0]<1 && v[3]<1);   // reached i5
  intersect = 1;
  flag = 0;
  servo_angle(15, 900);
   
  bot_back(50);
  pause(600);
  bot_180();
  intersect = 0;
  do{
    line_follow();
    detect_enemy();
    }while(v[0]<1 && v[3]<1);   //reached i4 again
  intersect = 1;
  flag=0;
  servo_angle(15, 900);
  bot_left();
  intersect = 0;
  do{
    line_follow();
    }while(v[0]<1 && v[3]<1);  // reached a4
  intersect = 1;
}
 
 
void a1_b1()
{
  count_intersect = 0;
  while(1)
  {
  do{
    line_follow();
   }while(v[0]<1 && v[3]<1);
  intersect = 1;
  count_intersect = count_intersect + 1;
  if (count_intersect == 2)
    {
      break;
    }
  bot_forward(50);
  pause(500);
  intersect = 0;
  }
}
 
void b1_b5()
{
  count_intersect = 0;
  flag=0;
  servo_angle(15, 900);
  while(1)
  {
  do{
    detect_enemy();
    line_follow();
   }while(v[0]<1 && v[3]<1);
  intersect = 1;
  flag = 0;
  servo_angle(15, 900);
  count_intersect = count_intersect + 1;
  if (count_intersect == 4)
    {
      break;
    }
  bot_forward(50);
  pause(750);
  intersect = 0;
  }
}  
 
void navigate(void *par1)
{
  servo_angle(14,900);
  servo_angle(15,900);
  int cmDist = ping_cm(10);
  int dist = distance();
  float * v;
  v = read_line_sensor();
  while(1)
   {
      do{
        line_follow();
        float * v;
        v = read_line_sensor();
        detect_enemy();
        if (v[0]>1  &&  v[1]>1 &&  v[2]>1  && v[3]>1)
        {
            servo_angle(15,900);
            intersect = 1;
            flag = 0;
            break;
        }
        }while(1);
      //intersect = 0;
      int cmDist = ping_cm(10);
      count_intersect = count_intersect + 1;
      float * v;
      v = read_line_sensor();
 
      if (cmDist < 35 && v[0]>2 && v[3]>2)
      {
        obstacle_led_blink();
        intersect  = 1;
        break;
      }
      bot_forward(50);
      pause(500);
      intersect=0;
    }
  intersect = 0;
  if (count_intersect == 5)
  {
    count_intersect = 0;
    i5_routine();
  }    
  if (count_intersect == 3)
  {
    count_intersect = 0;
    i3_routine();  
  }
  if (count_intersect == 2)
  {
    count_intersect = 0;
    i2_routine();
  }  
 
}
 
void i3_routine()
{
  bot_forward(50);
  pause(500);
  flag = 0;
  servo_angle(15, 900);
  do{
    line_follow();
    detect_enemy();
    float * v;
    v = read_line_sensor();       ////////REPEAT
    cmDist = ping_cm(10);
   
    if (cmDist < 5)  //// check for obstacle
    {
      break;
    }
    }while(1);
   
  bot_180();                /////rotate 180 when obstacle at i3 intersection
  flag = 0;
  servo_angle(15, 900);
  do{
    line_follow();
    detect_enemy();
    }while(v[0]<1 && v[3]<1);
  intersect = 1;
  flag=0;
  servo_angle(15, 900);
  bot_forward(50);
  pause(500);
  //count_intersect = count_intersect + 1;
  do{
    line_follow();
    detect_enemy();
    }while(v[0]<1 && v[3]<1);
  intersect = 1;
  bot_right();                 //////Reached i1 intersection    
  intersect = 0;
  flag = 0;
  servo_angle(15, 900);
  do{
    line_follow();
    }while(v[0]<1 && v[3]<1);
  intersect = 1;
  bot_right();                  /////Reached B1 and took right
  intersect = 0;
  detect_enemy();
  flag=0;    
  servo_angle(15, 900);    
  b1_b4();                      ////Reached B4
  bot_right();
  intersect = 0;
  b4_a4();
  bot_right();
  intersect = 0;
  b1_b4();
  bot_right();
  intersect = 0;
  a1_b1();
  bot_right();
  b1_b5();
  bot_stop();
}
 
void i5_routine()
{
  bot_forward(50);
  pause(500);
 
  do{
    line_follow();  
    detect_enemy();
    float * v;
    v = read_line_sensor();       ////////REPEAT
    cmDist = ping_cm(10);
   
    if (cmDist < 4)  //// check for obstacle
    {
      flag = 0;
      servo_angle(15,900);
      break;
    }
    }while(1);
   
  bot_180();
  count_intersect = 0;
  while(1)
  {
  do{
    line_follow();
    detect_enemy();
   }while(v[0]<1 && v[3]<1);
  intersect = 1;
  flag = 0;
  servo_angle(15,900);
  count_intersect = count_intersect + 1;
  if (count_intersect == 4)  /// go till i1
    {
      break;
    }
  bot_forward(50);
  pause(500);
  intersect = 0;
  }
 
  bot_right();
  do{
    line_follow();
   }while(v[0]<1 && v[3]<1);
  intersect = 1;
  bot_right();
  intersect = 0;
  detect_enemy();
  flag=0;
  servo_angle(15, 900);
  b1_b4();
  bot_right();
  intersect =0;
  a1_b1();
  bot_right();
  b1_b4();
  bot_right();
  intersect = 0;
  a1_b1();
  bot_right();
  b1_b5();
  bot_stop();
  pause(5000);
}  
   
void i2_routine()
{
  bot_forward(50);
  pause(500);
  flag = 0;
  servo_angle(15, 900);
  do{
    line_follow();
    detect_enemy();
    float * v;
    v = read_line_sensor();       ////////REPEAT
    cmDist = ping_cm(10);
   
    if (cmDist < 5)  //// check for obstacle
    {
      break;
    }
    }while(1);
   
  bot_180();                /////rotate 180 when obstacle at i2 intersection
  flag = 0;
  servo_angle(15, 900);
  do{
    line_follow();
    detect_enemy();
    }while(v[0]<1 && v[3]<1);
  flag=0;
  servo_angle(15, 900);
 
  //count_intersect = count_intersect + 1;
  bot_right();
  do{
    line_follow();
    }while(v[0]<1 && v[3]<1);
  bot_right();                  /////Reached B1 and took right
  detect_enemy();
  flag=0;    
  servo_angle(15, 900);    
  b1_b4();                      ////Reached B4
  bot_right();
  intersect = 0;
  b4_a4_for_i2();
  bot_right();
  intersect = 0;
  detect_enemy();
  flag = 0;
  servo_angle(15, 900);
  b1_b4();
  bot_right();
  intersect = 0;
  a1_b1();
  bot_right();
  b1_b5();
  bot_stop();
}
