/* 
 * MRSD Project Blitzkrieg Controller
 * April 1st, 2013
 */

/*------------------------------------------------------*/
//Motor Ctrl independantly in Arduino - Done
//Ros Simulation with Publisher       - Done
//Integration with Custom Msg         - Done
//-----TBD------
//Ultrasonic
//Encoder
//
/*------------------------------------------------------*/

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <vel_msgs/Velocity.h>
#include "DualVNH5019MotorShield.h"
//Full Speed = 400 : 0-255 for Analog Write mapped to 400
#define FS 400
#define ULTRA_PIN A0

DualVNH5019MotorShield md;

ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);

const int adc_pin = 0;

char frameid[] = "/ultrasound";

float linp;
float angp;
int linvel;
int angvel;
int rightVel;
int leftVel;

float getRange_Ultrasound(int pin_num){
  int val = 0;
  for(int i=0; i<4; i++) val += analogRead(pin_num);
  float range =  val;
  return range /322.519685;   // (0.0124023437 /4) ; //cvt to meters
}

//--------------------- CHECK IF NEEDED------------------------------
void ramp_up(int vel){
for (int i = 0; i <= vel; i++)
  {
    md.setM1Speed(i);
  }
}

void ramp_down(int vel){
for (int i = vel; i >=0; i--)
  {
    md.setM1Speed(i);
  }
}

void stop_mtrs(){
    md.setM1Speed(0);
    md.setM2Speed(0);
}
//---------------------------------------------------------------------

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

ros::Subscriber<vel_msgs::Velocity> sub("cmd_vel", &messageCb );

void setup()
{ 
  md.init();
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_range);
  
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  //range_msg.header.frame_id =  frameid;
  //range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 0.0;
  range_msg.max_range = 6.47;
  
  pinMode(8,OUTPUT);
  digitalWrite(8, LOW);
}

long range_time;

void loop()
{  

  //publish the adc value every 50 milliseconds
  //since it takes that long for the sensor to stablize
  if ( millis() >= range_time ){
    int r =0;

    range_msg.range = getRange_Ultrasound(ULTRA_PIN);
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    range_time =  millis() + 50;
  }

  nh.spinOnce();
}
