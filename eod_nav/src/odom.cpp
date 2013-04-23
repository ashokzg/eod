#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

#define LENGTH_PER_REV 35.0
#define COUNTS_PER_REV 80.0
#define WHEEL_TRACK 26.0//To be filled
long _PreviousLeftEncoderCounts = 0;
long _PreviousRightEncoderCounts = 0;
ros::Time current_time_encoder, last_time_encoder;
double DistancePerCount = LENGTH_PER_REV/COUNTS_PER_REV;

double x,y,th;
double vxy, vth;
double dxy_ave,dth;
double deltaLeft, deltaRight;
double dt;
double delta_x, delta_y;

void WheelCallback(const geometry_msgs::Vector3::ConstPtr& ticks)
{

  current_time_encoder = ros::Time::now();

  deltaLeft = (ticks->x - _PreviousLeftEncoderCounts)* DistancePerCount;
  deltaRight = (ticks->y - _PreviousRightEncoderCounts)* DistancePerCount;
  
  ROS_INFO("EncTicks: %f, %f", ticks->x, ticks->y);
 
  dt = (current_time_encoder - last_time_encoder).toSec();

  dxy_ave = (deltaLeft + deltaRight)/ 2.0; 
  //vy = deltaRight * DistancePerCount; // (current_time_encoder - last_time_encoder).toSec();
  dth = (deltaRight - deltaLeft)/ WHEEL_TRACK;
 
  vxy = dxy_ave / dt;
  vth = dth /dt; 
  ROS_INFO("dxy and dth: %f, %f", dxy_ave, dth);
 
  _PreviousLeftEncoderCounts = ticks->x;
  _PreviousRightEncoderCounts = ticks->y;
 
  last_time_encoder = current_time_encoder;
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("encTicks", 100, WheelCallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);   
  tf::TransformBroadcaster odom_broadcaster;

  ROS_INFO("Here in Main");
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0);
  while(n.ok()){

    current_time = ros::Time::now();
    if(dxy_ave != 0){
	    dt = (current_time - last_time).toSec();
	    delta_x =  cos(dth)*dxy_ave;
	    delta_y =  -sin(dth)*dxy_ave;
	    x = cos(th)*delta_x - sin(th)*delta_y;
	    y = sin(th)*delta_x + cos(th)*delta_y; 
    }

    if(dth!=0){
	    th += dth;
    }

    std::cout<<"x: "<<x<<" y: "<<y<<" th :"<<th<<std::endl;
    //ROS_INFO("X: %f, Y: %f, TH: %f",x,y,th);
    //ROS_INFO("");

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vxy;
    odom.twist.twist.angular.z = vth;

    odom_pub.publish(odom);

    last_time = current_time;
    ros::spin();
    r.sleep();
  }
}
