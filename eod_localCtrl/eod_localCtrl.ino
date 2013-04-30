#include <DualVNH5019MotorShieldMega.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <ultrasonic/Ultrasonic.h>
#include <Servo.h> 
#include <geometry_msgs/Twist.h>


#define FS 400
#define ULTRA_FRONT A3
//#define ULTRA_RIGHT A1
//#define ULTRA_LEFT A2
#define ENC_RIGHT 19
#define ENC_LEFT 18
#define ENC_INT_RIGHT 4
#define ENC_INT_LEFT 5
#define SERVO_PIN 25
#define BATTERY_MOTOR_PIN A10
#define BATTERY_PC_PIN A9

DualVNH5019MotorShield md;
ros::NodeHandle  nh;
ultrasonic::Ultrasonic us_msg;
std_msgs::Int16 lencoder, rencoder;

int LeftEncoderPos, RightEncoderPos;
int Lcount,Rcount;
int LdVal = 0, RdVal = 0;
int leftVel, rightVel;
float linp, angp;
int linvel, angvel;
int leftDir = 0, rightDir = 0;
unsigned long rangeTime = 0, encTime = 0, batteryTime = 0;
Servo servo1;
std_msgs::Float32 batteryMotor, batteryPc;

ros::Publisher pub_range( "/ultrasound", &us_msg);
ros::Publisher pub_lenc( "/lwheel", &lencoder);
ros::Publisher pub_renc( "/rwheel", &rencoder);
ros::Publisher pub_battery_motor("/battery_motor_voltage", &batteryMotor);
ros::Publisher pub_battery_pc("/battery_pc_voltage", &batteryPc);

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
  md.setM1Speed(leftVel);   
  if(leftVel < 0)
    leftDir = 1; 
  else
    leftDir = 0;  
}

void rightCb( const std_msgs::Float32& data)
{
  rightVel = (int)data.data;
  md.setM2Speed(-rightVel); 
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

void servoCb( const std_msgs::UInt32& data)
{
  float temp;
    if(data.data >= 0 && data.data <= 180)
    {
      temp = data.data*162.0/180 + 18;
      servo1.write((int)temp);
    }    
}


void twistCb( const geometry_msgs::Twist& vel){
  
  nh.loginfo("In Call Back");
  linp = vel.linear.x;
  angp = vel.angular.z;
  
  // Velocity = k1*FS + k2*FS
  linvel = (FS*linp); 
  angvel = (FS*angp);
  rightVel = linvel+angvel;
  leftVel = linvel-angvel;
  
  md.setM1Speed(-rightVel);
  md.setM2Speed(leftVel);
  if(rightVel < 0)
  {
    rightDir = 1;
  }
  else
  {
    rightDir = 0;
  }
  if(leftVel < 0)
  {
    leftDir = 1; 
  }
  else
  {
    leftDir = 0;
  }  
}

ros::Subscriber<std_msgs::Float32> lsub("lmotor_cmd", &leftCb);  
ros::Subscriber<std_msgs::Float32> rsub("rmotor_cmd", &rightCb);  
ros::Subscriber<geometry_msgs::Twist> twistsub("twist", &twistCb);  
ros::Subscriber<std_msgs::UInt32> navsub("Nav_State", &navCb);  
ros::Subscriber<std_msgs::UInt32> servosub("servo_angle", &servoCb );  

void setup()
{ 

  md.init();
  nh.initNode();
  nh.subscribe(lsub);
  nh.subscribe(rsub);  
  //nh.subscribe(twistsub);  
  nh.subscribe(navsub);  
  nh.subscribe(servosub);   
  nh.advertise(pub_range);
  nh.advertise(pub_lenc);
  nh.advertise(pub_renc);  
  nh.advertise(pub_battery_motor);
  //nh.advertise(pub_battery_pc);
  
  pinMode(ENC_RIGHT, INPUT);
  digitalWrite(ENC_RIGHT, HIGH);
  pinMode(ENC_LEFT, INPUT);
  digitalWrite(ENC_LEFT, HIGH);
  attachInterrupt(ENC_INT_LEFT, updateLeftEncoder, RISING); 
  attachInterrupt(ENC_INT_RIGHT, updateRightEncoder, RISING);
  rangeTime = millis() + 250; 
  encTime = millis() + 20; 
  servo1.attach(SERVO_PIN);
  servo1.write(98); 
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
  
  if(millis() < batteryTime)
  {
    batteryMotor.data = analogRead(BATTERY_MOTOR_PIN)/35.42;
    batteryPc.data = analogRead(BATTERY_PC_PIN)/35.42;
    pub_battery_motor.publish(&batteryMotor);
    pub_battery_pc.publish(&batteryPc);
    batteryTime = millis() + 5000; //Publish every 5s
  }
  nh.spinOnce();  
}

