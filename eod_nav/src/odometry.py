#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('eod_nav')

import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from math import sin, cos, pi
from geometry_msgs.msg import Quaternion, Twist, Pose
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import UInt32

class Odo:
  def __init__(self):
    rospy.init_node("odo")
    rospy.logwarn("Odometry measurement initialization")
    self.updateParameters()
    self.highLevelInits()
    self.lowLevelInits()  
    self.initSubAndPub()  
  
  def highLevelInits(self):
    self.odomBroadcaster = tf.TransformBroadcaster()
    
    
  def lowLevelInits(self):
    now = rospy.Time.now()
    self.then = now # time for determining dx/dy
    self.t_delta = rospy.Duration(1.0 / self.rate)
    self.t_next = now + self.t_delta
  
    # internal data
    self.enc_left = None # encoder readings
    self.enc_right = None
    self.x = 0 # position in xy plane
    self.y = 0
    self.th = 0 # rotation in radians
    self.v_left = 0
    self.v_right = 0
    self.v_des_left = 0 # cmd_vel setpoint
    self.v_des_right = 0
    self.last_cmd_vel = now  

  
  def initSubAndPub(self):    
    #Init the subscribers  
    #  UI
    rospy.Subscriber("encTicks", Vector3, self.poll)
    rospy.Subscriber("Nav_State", UInt32, self.navState)
    self.odomPub = rospy.Publisher('odom', Odometry)
  
  
  def updateParameters(self):
    self.rate = float(rospy.get_param("~base_controller_rate", 10))
    self.timeout = rospy.get_param('~base_controller_timeout', 1.0)
    #Reset the parameters so that it would be easily visible to debug
    self.wheel_diameter = rospy.get_param("~wheel_diameter", 10)
    self.wheel_track = rospy.get_param("~wheel_track", 0.26)
    self.encoder_resolution = rospy.get_param("~encoder_resolution", 48)
    self.gear_reduction = rospy.get_param("~gear_reduction", 1.0)
    self.accel_limit = rospy.get_param('~accel_limit', 0.1)
    self.motors_reversed = rospy.get_param("~motors_reversed", False)
    # How many encoder ticks are there per meter?
    #self.ticks_per_meter = self.encoder_resolution * self.gear_reduction / (self.wheel_diameter * pi)
    self.ticks_per_meter = 80*100/35.0 # 80 ticks/rev, 35cm/rev

  def navState(self, data):
    if data.data == 2:
      self.lowLevelInits()

  def poll(self, data):
    now = rospy.Time.now()
    if now > self.t_next:
      # Read the encoders
      left_enc = data.x
      right_enc = data.y 
              
      dt = now - self.then
      self.then = now
      dt = dt.to_sec()
      
      # calculate odometry
      if self.enc_left == None:
        dright = 0
        dleft = 0
      else:
        dright = (right_enc - self.enc_right) / self.ticks_per_meter
        dleft = (left_enc - self.enc_left) / self.ticks_per_meter
      self.enc_right = right_enc
      self.enc_left = left_enc
      
      dxy_ave = (dright + dleft) / 2.0
      dth = (dright - dleft) / self.wheel_track
      vxy = dxy_ave / dt
      vth = dth / dt
        
      if (dxy_ave != 0):
        dx = cos(dth) * dxy_ave
        dy = -sin(dth) * dxy_ave
        self.x += (cos(self.th) * dx - sin(self.th) * dy)
        self.y += (sin(self.th) * dx + cos(self.th) * dy)
  
      if (dth != 0):
        self.th += dth
  
      quaternion = Quaternion()
      quaternion.x = 0.0
      quaternion.y = 0.0
      quaternion.z = sin(self.th / 2.0)
      quaternion.w = cos(self.th / 2.0)
  
      # Create the odometry transform frame broadcaster.
      self.odomBroadcaster.sendTransform(
        (self.x, self.y, 0),
        (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
        rospy.Time.now(),
        "base_link",
        "world"        
        )
      self.odomBroadcaster.sendTransform((0,0,0.23), (-0.500, 0.485, -0.500, 0.515), rospy.Time.now(), "camera", "base_link")
  
      odom = Odometry()
      odom.header.frame_id = "base_link"
      odom.child_frame_id = "world"
      odom.header.stamp = now
      odom.pose.pose.position.x = self.x
      odom.pose.pose.position.y = self.y
      odom.pose.pose.position.z = 0
      odom.pose.pose.orientation = quaternion
      odom.twist.twist.linear.x = vxy
      odom.twist.twist.linear.y = 0
      odom.twist.twist.angular.z = vth
  
      self.odomPub.publish(odom)
        
      self.t_next = now + self.t_delta  
  


if __name__ == "__main__":  
  nav = Odo()  
  rospy.spin()
  rospy.logwarn("Exiting EOD Navigation! Bye Bye")

 
