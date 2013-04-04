#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('eod_nav')

from teamb_ui.msg import Dest
import std_msgs.msg
from sensor_msgs.msg import CameraInfo, Range
from eod_nav.msg import Velocity

import numpy

class eodNav:
  #States of the robot
  IDLE          = 1
  TRACKING_ONLY = 2
  AUTO_MODE     = 3
  DEST_LOST     = 4
  ROBOT_LOST    = 5
  MANUAL_MODE   = 6   
  
  #Move commands
  STRAIGHT = 0
  LEFT = 1
  RIGHT = 2
  STOP = 3
  
  #Used for smoothing the ultrasonic
  val = numpy.zeros(15)  
  
  def __init__(self):
    rospy.init_node("eod_navigation")
    self.navState = self.IDLE
    self.vel = Velocity()
    self.vel.linVelPcent = 0.0
    self.vel.angVelPcent = 0.0
    print "Came to init"
    #Init the subscribers    
    #    UI
    rospy.Subscriber("UserDestination", Dest, self.userDest)
    rospy.Subscriber("UiStatus", std_msgs.msg.Int32, self.uiState)
    #    Tracker
    rospy.Subscriber("destination", Dest, self.trackedDest)
    rospy.Subscriber("ultrasound", Range, self.ultraSound)
    #Init the publishers
    self.navStatePub = rospy.Publisher('Nav_State', std_msgs.msg.UInt32)
    self.robotCmdPub = rospy.Publisher('cmd_vel', Velocity)
    
    self.navStatePub.publish(self.navState)
    self.robotCmdPub.publish(self.vel)
    self.Stopped = False
    self.prevState = self.IDLE
    self.updateParameters()
    print "Velocity parameters", self.stLinVel, self.rotLinVel, self.rotAngVel
    #Initialize values
    try:
      img = rospy.wait_for_message("camera/camera_info", CameraInfo, 2.0)
      self.imgWidth = img.width
      self.imgHeight = img.height
    except rospy.ROSException:
      self.imgWidth = 640
      self.imgHeight = 480
    
  def updateParameters(self):
    self.stLinVel = rospy.get_param("~st_lin_vel", 0.5)
    self.rotLinVel = rospy.get_param("~rot_lin_vel", 0.4)
    self.rotAngVel = rospy.get_param("~rot_ang_vel", 0.2)    
    
  def spin(self):
    rospy.spin()
  
  def ultraSound(self, data):
    #Smoothes the ultrasonic data with a gaussian filter
    distance = self.processUltrasound(data)
    print distance, self.navState, self.prevState, self.Stopped
    self.navStatePub.publish(self.navState)
    if distance < 0.7 and self.navState == self.AUTO_MODE:
      print "STOPPING"
      self.robotMove(self.STOP)
      self.prevState = self.navState      
      self.navState = self.ROBOT_LOST   
      self.Stopped = True
    elif distance > 0.8 and self.Stopped == True and self.navState == self.ROBOT_LOST:
      self.navState = self.prevState
      self.prevState = self.MANUAL_MODE
      self.Stopped = False
    
  def userDest(self, data):
    if data.destPresent == True:
      self.navState = self.TRACKING_ONLY
    print "Destination received; Tracking"
    
  def uiState(self, data):
    if data.data == 0:
      self.navState = self.IDLE
      self.robotMove(self.STOP)      
    elif data.data == 1:
      self.navState = self.MANUAL_MODE
      self.robotMove(self.STOP)
    elif data.data == 2:
      self.navState = self.TRACKING_ONLY
      self.robotMove(self.STOP)      
    elif data.data == 3:
      self.updateParameters()
      self.navState = self.AUTO_MODE
    else:
      #Should not come to this state
      print "This is not good! Unknown state from UI"
      self.navState = self.IDLE
      self.robotMove(self.STOP)      
  
  def trackedDest(self, data):
    #print "tracking in state", self.navState 
    if self.navState in [self.AUTO_MODE, self.DEST_LOST]:
      if data.destPresent == True:
        self.navState = self.AUTO_MODE
        cx = data.destX + data.destWidth/2
        cy = data.destY + data.destHeight/2
        #If the destination is slipping towards the left, turn left
        if cx < (self.imgWidth/2 - 50):
          self.robotMove(self.LEFT)
        #If the destination is slipping towards the right, turn right
        elif cx > (self.imgWidth/2 + 50):
          self.robotMove(self.RIGHT)
        #We are good to zip towards the destination
        else:
          self.robotMove(self.STRAIGHT) 
      else:
        self.navState = self.DEST_LOST
        self.robotMove(self.STOP)
  
  
  def robotMove(self, dir):
    if dir == self.STRAIGHT:
      self.vel.linVelPcent = self.stLinVel
      self.vel.angVelPcent = 0.0
      rospy.logdebug("Straight")
      #print "Straight"
    elif dir == self.LEFT:
      self.vel.linVelPcent = self.rotLinVel
      self.vel.angVelPcent = self.rotAngVel
      rospy.logdebug("Left")
      #print "Left"
    elif dir == self.RIGHT:
      self.vel.linVelPcent = self.rotLinVel
      self.vel.angVelPcent = -self.rotAngVel
      rospy.logdebug("Right")
      #print "Right"
    else:
      self.vel.linVelPcent = 0.0
      self.vel.angVelPcent = 0.0
      rospy.logdebug("Stop")
      #print "STOP"
    self.robotCmdPub.publish(self.vel)        

  def processUltrasound(self, data): 
    self.val = numpy.append(self.val, data.range)
    self.val = numpy.delete(self.val, 1)  
    v = self.smooth(self.val)
    return v.mean()
    
  def smooth(self, x, window_len=11,window='hanning'):
    if x.ndim != 1:
      raise ValueError, "smooth only accepts 1 dimension arrays."
    if x.size < window_len:
      raise ValueError, "Input vector needs to be bigger than window size."
    if window_len<3:
      return x
    if not window in ['flat', 'hanning', 'hamming', 'bartlett', 'blackman']:
      raise ValueError, "Window is on of 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'"
    s=numpy.r_[x[window_len-1:0:-1],x,x[-1:-window_len:-1]]
    #print(len(s))
    if window == 'flat': #moving average
      w=numpy.ones(window_len,'d')
    else:
      w=eval('numpy.'+window+'(window_len)')
    y=numpy.convolve(w/w.sum(),s,mode='valid')
    return y
  
if __name__ == "__main__":
  print "came here"
  nav = eodNav()
  nav.spin()
