#!/usr/bin/env python
import rospy
import roslib
import copy

from std_msgs.msg import Float32
from numpy import array

class directDrive:  
  lmotor = Float32()
  rmotor = Float32()
  lvel = Float32()
  rvel = Float32()
  set = False
  count = 0
  def __init__(self):
    rospy.init_node("direct_drive")
    rospy.Subscriber("lwheel_vtarget", Float32, self.lcb_1)
    rospy.Subscriber("rwheel_vtarget", Float32, self.rcb_1)  
    self.pub_lmotor = rospy.Publisher('lmotor_cmd', Float32)
    self.pub_rmotor = rospy.Publisher('rmotor_cmd', Float32)
  
  def handler(self):
    if self.set == True or self.count<5:
      self.lcb(self.lvel)
      self.rcb(self.rvel)    
      self.set = False
    else:
      self.lvel.data = 0
      self.rvel.data = 0
      self.lcb(self.lvel)
      self.rcb(self.rvel)          
    self.count+=1
    
  def lcb_1(self, data):
    self.set = True
    self.lvel = copy.deepcopy(data)
    self.count = 0

  def rcb_1(self, data):
    self.set = True
    self.rvel = copy.deepcopy(data)
    self.count = 0      
    
  def lcb(self, data):
    temp = data.data*300;
    if abs(temp) > 400:
      if temp > 0:
        temp = 400
      else:
        temp = -400
    self.lmotor.data = temp
    self.pub_lmotor.publish(self.lmotor)
    
  def rcb(self, data):
    temp = data.data*300;
    if abs(temp) > 400:
      if temp > 0:
        temp = 400
      else:
        temp = -400
    self.rmotor.data = temp
    self.pub_rmotor.publish(self.rmotor)        
    
if __name__ == "__main__":
  d = directDrive()
  r = rospy.Rate(20) #Run every 50 ms
  while not rospy.is_shutdown():
    d.handler()
    r.sleep()
  rospy.logwarn("Exiting Direct Drive! Bye Bye")
