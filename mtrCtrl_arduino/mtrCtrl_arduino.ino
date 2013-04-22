/* 
 * MRSD Project Blitzkrieg Controller
 * April 20th, 2013
 */

/*------------------------------------------------------*/
//Motor Ctrl independantly in Arduino - Done
//Ros Simulation with Publisher       - Done
//Integration with Custom Msg         - Done
//Ultrasonic
//-----TBD------
//Encoder
//
/*------------------------------------------------------*/

#include <ros.h>
#include <ros/time.h>
#include <vel_msgs/Velocity.h>
#include <ultrasonic/Ultrasonic.h>
#include <geometry_msgs/Vector3.h>
#include "DualVNH5019MotorShield.h"
//Full Speed = 400 : 0-255 for Analog Write mapped to 400

#define FS 400
#define ULTRA_FRONT A1
#define ULTRA_RIGHT A0
#define ULTRA_LEFT A2
#define ENC_LEFT 3
#define ENC_RIGHT 2
#define PROBE_PIN 8

#define UREAD 0
#define UPROBE_START 1
#define UPROBE_END 2
#define UWAIT 3

// Volatile variables that can be changed inside of interrupt functions
unsigned int RightEncoderPos, LeftEncoderPos;
unsigned int Lcount,Rcount;
int LdVal = 0;
int RdVal = 0;

int readOrProbe = UWAIT;

DualVNH5019MotorShield md;

ros::NodeHandle  nh;

ultrasonic::Ultrasonic us_msg;

geometry_msgs::Vector3 encoder_info;

ros::Publisher pub_range( "/ultrasound", &us_msg);
ros::Publisher pub_enc( "/encTicks", &encoder_info);
const int adc_pin = 0;

float linp;
float angp;
int linvel;
int angvel;
int rightVel;
int leftVel;

float getRange_Ultrasound(int pin_num){
  int val = 0;
  return analogRead(pin_num);  
  for(int i=0; i<4; i++) val += analogRead(pin_num);
  float range =  val;
  return range /322.519685;   // (0.0124023437 /4) ; //cvt to meters
}

void updateRightEncoder(){
 Rcount++;
 if(Rcount%11==0)
    {   
		RightEncoderPos++;Rcount=0;
		RdVal= RdVal+RightEncoderPos; 
		RightEncoderPos=0;
	}
  }
  
void updateLeftEncoder(){
Lcount++;
if(Lcount%11==0)
    {   
      LeftEncoderPos++;
      Lcount=0;
      LdVal= LdVal+LeftEncoderPos;  
      LeftEncoderPos=0;
    }

}

void messageCb( const vel_msgs::Velocity& vel){
  
  nh.loginfo("In Call Back");
  linp = vel.linVelPcent;
  angp = vel.angVelPcent;
  
  // Velocity = k1*FS + k2*FS
  linvel = (FS*linp); 
  angvel = (FS*angp);
  rightVel = linvel+angvel;
  leftVel = linvel-angvel;
  
  md.setM1Speed(rightVel);
  md.setM2Speed(leftVel);
}

ros::Subscriber<vel_msgs::Velocity> sub("/cmd_vel", &messageCb );  

void setup()
{ 
  pinMode(PROBE_PIN, OUTPUT);
  md.init();
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_range);
  nh.advertise(pub_enc);

  pinMode(ENC_RIGHT, INPUT);
  digitalWrite(ENC_RIGHT, HIGH);
  pinMode(ENC_LEFT, INPUT);
  digitalWrite(ENC_LEFT, HIGH);
  
  attachInterrupt(0, updateRightEncoder, RISING); 
  attachInterrupt(1, updateLeftEncoder, RISING);

}

long range_time, probe_time, enc_time;

void loop()
{  

  if(readOrProbe == UPROBE_START)
  {
     digitalWrite(PROBE_PIN, HIGH);   // sets the trigger to HIGH
     readOrProbe = UPROBE_END;
     probe_time = millis() + 5;
  }
  else if(readOrProbe == UPROBE_END)
  {
    if(millis() > probe_time)
    {
      digitalWrite(PROBE_PIN, LOW);   // sets the trigger to LOW
      readOrProbe = UWAIT;
      range_time = millis() + 150; 
    }    
  }
  else if(readOrProbe == UWAIT)
  {
    if(millis() > range_time)
      readOrProbe = UREAD;
  }  
  //publish the adc value every 50 milliseconds
  //since it takes that long for the sensor to stablize
  else if(readOrProbe == UREAD)
  {    
      us_msg.ultra_centre = getRange_Ultrasound(ULTRA_FRONT)*1.27;
      us_msg.ultra_right = getRange_Ultrasound(ULTRA_RIGHT)*1.27;
      us_msg.ultra_left = getRange_Ultrasound(ULTRA_LEFT)*1.27;    
      us_msg.header.stamp = nh.now();
      pub_range.publish(&us_msg);
      readOrProbe = UPROBE_START;    
  }


  //publish the value every 25 milliseconds
  if(millis() >= enc_time)
  {
     encoder_info.x = LdVal;
     encoder_info.y = RdVal;
     pub_enc.publish(&encoder_info); 
     enc_time = millis() + 25;
  }
  
  nh.spinOnce();
}
