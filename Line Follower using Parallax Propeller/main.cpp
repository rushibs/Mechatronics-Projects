#include "simpletools.h"
#include "abdrive.h"                          // abdrive library
#include "servo.h"
#include "ping.h"                             // Include ping header
#include "adcDCpropab.h"



float * read_line_sensor();
void  line_follow();
void intersection_led_blink(void *par);
void object_detection(void *par);
void bot_forward(int speed_value);
void navigate(void *par1);
void bot_stop();
void i2_routine();
void i3_routine();
void i5_routine();
volatile int count_intersect = 0;
volatile int count_object = 0;
int count_intersect_b1b4 = 0;
unsigned int stack1[40+25]; 
unsigned int stack2[40+25]; 
unsigned int stack3[40+25]; 
int cmDist;
float v[4];
//float v_l[4];
volatile int detection = 0;
volatile int intersect = 0;
int main()                                    // Main function
{
   //adc_init(21,20,19,18);
   cogstart((void*)navigate, NULL, stack2, sizeof(stack2));
   //static int cog_id = cog_start(intersection_led_blink,NULL, 128);
   cogstart((void*)intersection_led_blink,NULL, stack1,sizeof(stack1));
   cogstart((void*)object_detection,NULL, stack3,sizeof(stack3));
}

// read line sensor values interms of voltage---------------------------------------------------
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
    float * v;
    v = read_line_sensor();
    //print("A/D3 = %f V%c\n", v[2], CLREOL);
     
    if (v[1] > 1 && v[2] > 1)
    {   
        bot_forward(50);            
    }
    else if (v[1]>2 && v[2]<1)
    {   
        servo_set(12,1440);  //slight_right
        servo_set(13,1400);       
    }
    else if(v[2]>2 && v[1]<1)
    {
      servo_set(12,1400); //slight_left
      servo_set(13,1360); 
     }
} 

// Intersection LED Blink Function--------------------------------------------------------------------------
void intersection_led_blink(void *par)
{
    while(1)
   {  
        while(intersect)
        {
            high(3);
            pause(200);
            low(3);
            pause(200);         
        }    
    }         
  
}
  
// Object detection LED Blink Function--------------------------------------------------------------------------
void object_detection(void *par)
{
  
  while (1)
  {
    while(detection)
    {
        high(5);
        pause(200);
        low(5);
        pause(200);
    }
         
   }  
}



//obstacle LED blink ---------------------------------------------------------------------------------
void obstacle_led_blink()
{
    high(4);
    //pause(50);
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

void bot_left()
{
  float * v;
  v = read_line_sensor();
  while(v[1]>2)
  {
    float * v;
    v = read_line_sensor();
    //printf("%d \n", v[1]);
    servo_set(12,1400);
    servo_set(13,1350);
  }
  
  while(v[1]<2)
  {
    float * v;
    v = read_line_sensor();
    //printf("low= %d \n", v[1]);
    servo_set(12,1400);
    servo_set(13,1350);
  }  
 
}

void bot_right()
{
  float * v;
  v = read_line_sensor();
  while(v[2]>2)
  {
    float * v;
    v = read_line_sensor();
    servo_set(12,1450);
    servo_set(13,1400);
  }
  while(v[2]<2)
  {
    float * v;
    v = read_line_sensor();
    servo_set(12,1450);
    servo_set(13,1400);
  }
}

void bot_stop()
{
  servo_set(12,1400);
  servo_set(13,1400);
}

int b1_counter_check()
{
   while(1)
   {
      do{
        line_follow();   
        }while(v[0]<1 && v[3]<1);
 
      count_intersect = count_intersect + 1;
      float * v;
      v = read_line_sensor();
      if (v[0]>2 && v[3]>2)
      {
        //int count_object = object_count();
        intersect  =  1;         
        int cmDist = ping_cm(15);
        if (cmDist < 15)
        {         
           detection = 1;
           count_object = count_object +1;
           if(count_object ==2){
              bot_stop();
              pause(10000);          
          }            
         }
        break;
      }
      //bot_forward(50);
      //pause(500);      
    }
    
    return count_intersect;
}  

int b1_b4_counter_check()
{
   count_intersect = 0;
   while(1)
   {
      do{
        line_follow();
        }while(v[0]<1 && v[3]<1);
      intersect =  1;
      int cmDist = ping_cm(15);
      if (cmDist < 15)
      {
           detection = 1;
           count_object = count_object +1;
           if(count_object ==2){
              bot_stop();
              pause(10000);          
          }            
       }   
      count_intersect = count_intersect + 1;
      
      float * v;
      v = read_line_sensor();
      //printf("%d \n",count_intersect);
      if (v[0]>2 && v[3]>2 && count_intersect==3)
      {
        int cmDist = ping_cm(15);
        if (cmDist < 15)
        {         
           detection = 1;
           count_object = count_object +1;
           if(count_object ==2){
              bot_stop();
              pause(10000);          
          }            
         }
        break;
      }
      bot_forward(50);
      pause(500);
      intersect = 0;
      detection =0;
    }
    return count_intersect;
}  

int b4_a4_counter_check()
{
   count_intersect = 0;
   while(1)
   {
      do{
        line_follow();
        }while(v[0]<1 && v[3]<1);
      intersect =1;  
      int cmDist = ping_cm(15);
      count_intersect = count_intersect + 1;
      float * v;
      v = read_line_sensor();
      if (v[0]>2 && v[3]>2 && count_intersect==2)
      {
        //count_object = object_count();
        int cmDist = ping_cm(15);
        if (cmDist < 15)
        {         
           detection = 1;
           count_object = count_object +1;
           if(count_object ==2){
              bot_stop();
              pause(10000);          
          }            
         }
        break;
      }
      bot_forward(50);
      pause(500);
      intersect = 0;
    }
    
    return count_intersect;
}  

int b1_b5_counter_check()
{
   count_intersect = 0;
   while(1)
   {
      do{
        line_follow();
        }while(v[0]<1 && v[3]<1);
      intersect  = 1;  
      count_intersect = count_intersect + 1;
      
      float * v;
      v = read_line_sensor();
      if (v[0]>2 && v[3]>2 && count_intersect==4)
      {
        int cmDist = ping_cm(15);
        if (cmDist < 15)
        {         
           detection = 1;
           count_object = count_object +1;
           if(count_object ==2){
              bot_stop();
              pause(10000);          
          }            
         }
        break;
      }
      bot_forward(50);
      pause(500);
      intersect = 0;
    }  
    return count_intersect;
}  

void navigate(void *par1)
{
  servo_angle(14,900);
  int cmDist = ping_cm(15);
  float * v;
  v = read_line_sensor(); 
  while(1)
   {
      do{
        line_follow();
        float * v;
        v = read_line_sensor(); 
        if (v[0]>1  &&  v[1]>1 &&  v[2]>1  && v[3]>1)
        {
            intersect = 1;
            break;
        }        
        }while(1);
      //intersect = 0;
      int cmDist = ping_cm(15);
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
  if (count_intersect == 2)
   {
      count_intersect = 0;
      i2_routine(); 
   }
   
  else if (count_intersect == 3)
  {
    count_intersect = 0;
    i3_routine();        
   }
   
  else if (count_intersect ==  5)
  {
      count_intersect = 0;
      i5_routine();
  }   
}

void i2_routine()
{
  bot_left();
  low(4);
  int count_intersect = b1_counter_check();  // to  B1
  intersect =  0;
  detection = 0;
  bot_right();
  //count_intersect = 0;
  servo_angle(14, 1800);
  count_intersect = b1_b4_counter_check(); // B1  to  B4
  bot_right();
  intersect = 0;
  detection = 0;
  servo_angle(14, 900);
  count_intersect = b4_a4_counter_check();  // B4 to A4
  bot_right();
  intersect = 0;
  detection = 0;
  servo_angle(14, 1800);
  count_intersect = b1_b4_counter_check();    //A4 to A1
  bot_right();
  intersect = 0;
  detection = 0;
  servo_angle(14, 900);
  count_intersect = b4_a4_counter_check();   //A1 to B1 
  bot_right();
  intersect = 0;
  detection = 0;
  servo_angle(14,1800);
  count_intersect  =  b1_b5_counter_check(); // B1to B5
  intersect =0;
  detection = 0;
  bot_stop(); 
  pause(2000);   
}

void i3_routine()
{
  bot_right();
  low(4);
  count_intersect = b1_counter_check(); //i2 to  a2
  bot_right();
  intersect = 0;
  detection =  0;
  servo_angle(14, 1800);
  count_intersect = b1_counter_check(); // a2 to a1
  bot_right();
  intersect = 0;
  detection =  0;
  count_object = 0;  
  servo_angle(14, 900);
  count_intersect = b4_a4_counter_check(); //a1  to b1
  bot_right();
  intersect = 0;
  detection =  0;
  servo_angle(14, 1800);
  count_intersect = b1_b4_counter_check();
  bot_right();
  intersect = 0;
  detection =  0;
  servo_angle(14, 900);
  count_intersect = b4_a4_counter_check();
  bot_right();
  intersect = 0;
  detection =  0;
  servo_angle(14,  1800);
  count_intersect = b1_b4_counter_check();
  intersect = 0;
  detection =  0;
  count_intersect  =  b1_b5_counter_check(); // B1to B5
  intersect = 0;
  detection =  0;
  bot_stop(); 
  
}
  
void i5_routine()
{
    bot_right();
    low(4);
    count_intersect  = b1_counter_check();
    bot_right();
    intersect = 0;
    detection =  0;   
    servo_angle(14,1800);
    count_intersect = b1_b4_counter_check();    //A4 to A1
    bot_right();
    intersect = 0;
    detection =  0;
    servo_angle(14, 900);
    count_intersect = b4_a4_counter_check();   //A1 to B1 
    bot_right();
    intersect = 0;
    detection =  0;
    servo_angle(14,  1800);
    count_intersect = b1_b4_counter_check();
    bot_stop();
    intersect = 0;
    detection =  0;
}   
