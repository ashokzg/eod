#include <ros.h>
#include <ros/time.h>
#include "DualVNH5019MotorShield.h"
#include <std_msgs/UInt32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <ultrasonic/Ultrasonic.h>


#define FS 400
#define ULTRA_FRONT A0
//#define ULTRA_RIGHT A1
//#define ULTRA_LEFT A2
#define ENC_LEFT 3
#define ENC_RIGHT 2
#define SERVO_PIN 11


DualVNH5019MotorShield md;
ros::NodeHandle  nh;
ultrasonic::Ultrasonic us_msg;
std_msgs::Int16 lencoder, rencoder;

int LeftEncoderPos, RightEncoderPos;
int Lcount,Rcount;
int LdVal = 0, RdVal = 0;
int leftVel, rightVel;
int leftDir = 0, rightDir = 0;
unsigned long rangeTime = 0, encTime = 0;


ros::Publisher pub_range( "/ultrasound", &us_msg);
ros::Publisher pub_lenc( "/lwheel", &lencoder);
ros::Publisher pub_renc( "/rwheel", &rencoder);


void updateRightEncoder()
{
  Rcount++;
  if(Rcount%11==0)
  {   
    RightEncoderPos++;Rcount=0;
    if(rightDir == 0)
      RdVal= RdVal + RightEncoderPos; 
    else
      RdVal= RdVal - RightEncoderPos;   
    RightEncoderPos=0;
  }
}
  
void updateLeftEncoder()
{
  Lcount++;
  if(Lcount%11==0)
  {   
    LeftEncoderPos++;
    Lcount=0;
    if(leftDir == 0)      
      LdVal= LdVal + LeftEncoderPos;  
    else
      LdVal= LdVal - LeftEncoderPos;              
    LeftEncoderPos=0;
  }
}

void leftCb( const std_msgs::Float32& data)
{
  leftVel = (int)data.data;
  md.setM2Speed(-leftVel); 
  //nh.loginfo("In left Call Back %d", leftVel);
  
  if(leftVel < 0)
    leftDir = 1; 
  else
    leftDir = 0;  
}

void rightCb( const std_msgs::Float32& data)
{
  rightVel = (int)data.data;
  md.setM1Speed(rightVel); 
  if(rightVel < 0)
    rightDir = 1; 
  else
    rightDir = 0;  
}

void navCb( const std_msgs::UInt32& data)
{
  if(data.data == 2)
  {
    LdVal = 0;
    RdVal = 0;
  }  
}

ros::Subscriber<std_msgs::Float32> lsub("lmotor_cmd", &leftCb);  
ros::Subscriber<std_msgs::Float32> rsub("rmotor_cmd", &rightCb);  
ros::Subscriber<std_msgs::UInt32> navsub("Nav_State", &navCb);  

void setup()
{ 
  //pinMode(PROBE_PIN, OUTPUT);
  md.init();
  nh.initNode();
  nh.subscribe(lsub);
  nh.subscribe(rsub);  
  nh.subscribe(navsub);  
  nh.advertise(pub_range);
  nh.advertise(pub_lenc);
  nh.advertise(pub_renc);  
  
  pinMode(ENC_RIGHT, INPUT);
  digitalWrite(ENC_RIGHT, HIGH);
  pinMode(ENC_LEFT, INPUT);
  digitalWrite(ENC_LEFT, HIGH);
  attachInterrupt(0, updateLeftEncoder, RISING); 
  attachInterrupt(1, updateRightEncoder, RISING);
  rangeTime = millis() + 250; 
  encTime = millis() + 20; 
}

void loop()
{
  if(millis() > rangeTime)
  {
    us_msg.ultra_centre = analogRead(ULTRA_FRONT)*1.27;
    us_msg.ultra_right = 600;// getRange_Ultrasound(ULTRA_RIGHT)*1.27;
    us_msg.ultra_left = 600; //getRange_Ultrasound(ULTRA_LEFT)*1.27;        
    us_msg.header.stamp = nh.now();
    pub_range.publish(&us_msg);
    rangeTime = millis() + 50; //Publish every 50 ms    
  }
  
  if(millis() > encTime)
  {
     lencoder.data = LdVal;
     rencoder.data = RdVal;     
     pub_lenc.publish(&lencoder);
     pub_renc.publish(&rencoder);     
     encTime = millis() + 20; //Publish every 20 ms
  }
  nh.spinOnce();  
}

