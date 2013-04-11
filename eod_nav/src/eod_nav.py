#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('eod_nav')

from teamb_ui.msg import Dest
import std_msgs.msg
from sensor_msgs.msg import CameraInfo, Range
from eod_nav.msg import Velocity, Ultrasonic

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
  
  #Ultrasonic sensors
  UL = 0
  UC = 1
  UR = 2
  
  #Obstacle
  OBS_L = 1
  OBS_C = 2
  OBS_R = 4
  OBS_IDX = [OBS_L, OBS_C, OBS_R]
  OBS  = 0
  
  #Used for smoothing the ultrasonic
  val_c = numpy.zeros(15)  
  val_r = numpy.zeros(15)  
  val_l = numpy.zeros(15)      
  
  def __init__(self):
    rospy.init_node("eod_navigation")
    print "Came to init"
    self.navState = self.IDLE
    self.vel = Velocity()
    self.vel.linVelPcent = 0.0
    self.vel.angVelPcent = 0.0    
    self.Stopped = False
    self.prevState = self.IDLE
    self.updateParameters()
    #Initialize values
    try:
      img = rospy.wait_for_message("camera/camera_info", CameraInfo, 2.0)
      self.imgWidth = img.width
      self.imgHeight = img.height
    except rospy.ROSException:
      self.imgWidth = 640
      self.imgHeight = 480
      
    #Init the subscribers    
    #    UI
    rospy.Subscriber("UserDestination", Dest, self.userDest)
    rospy.Subscriber("UiStatus", std_msgs.msg.Int32, self.uiState)
    #    Tracker
    rospy.Subscriber("destination", Dest, self.trackedDest)
    rospy.Subscriber("ultrasound", Ultrasonic, self.ultraSound)
    #Init the publishers
    self.navStatePub = rospy.Publisher('Nav_State', std_msgs.msg.UInt32)
    self.robotCmdPub = rospy.Publisher('cmd_vel', Velocity)
    
    self.navStatePub.publish(self.navState)
    self.robotCmdPub.publish(self.vel)
    print "Velocity parameters", self.stLinVel, self.rotLinVel, self.rotAngVel

    
  def updateParameters(self):
    self.stLinVel = rospy.get_param("~st_lin_vel", 0.5)
    self.rotLinVel = rospy.get_param("~rot_lin_vel", 0.4)
    self.rotAngVel = rospy.get_param("~rot_ang_vel", 0.2)    
    
  def spin(self):
    rospy.spin()
  
  def ultraSound(self, data):
    #Smoothes the ultrasonic data with a gaussian filter
    #Distance is a list of ultrasonic [left, center, right]
    distance = self.processUltrasound(data)
    try:
      l, r = self.calcZone()
      print l, r, 
    except:
      print "ALL BLOCKED"

    self.navStatePub.publish(self.navState)    
    self.OBS = 0
    for i in range(3):
      if distance[i] < 1.5:
        self.OBS |= self.OBS_IDX[i]
    #print data.ultra_left, data.ultra_centre, data.ultra_right,
    print [self.OBS&self.OBS_L, self.OBS&self.OBS_C, self.OBS&self.OBS_R],
    print ["%0.3f" %i for i in distance],
    print self.navState, self.prevState, self.Stopped
    if distance[self.UC] < 0.7 and self.navState == self.AUTO_MODE:
      print "STOPPING"
      self.robotMove(self.STOP)
      self.prevState = self.navState      
      self.navState = self.ROBOT_LOST   
      self.Stopped = True
    elif distance[self.UC] > 0.8 and self.Stopped == True and self.navState == self.ROBOT_LOST:
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
    try:
      leftLimit, rightLimit = self.calcZone();
      if self.navState in [self.AUTO_MODE, self.DEST_LOST]:
        if data.destPresent == True:
          self.navState = self.AUTO_MODE
          cx = data.destX + data.destWidth/2
          cy = data.destY + data.destHeight/2
          #If the destination is slipping towards the left, turn left
          if cx < leftLimit:
            self.robotMove(self.LEFT)
          #If the destination is slipping towards the right, turn right
          elif cx > rightLimit:
            self.robotMove(self.RIGHT)
          #We are good to zip towards the destination
          else:
            self.robotMove(self.STRAIGHT) 
        else:
          self.navState = self.DEST_LOST
          self.robotMove(self.STOP)
    except:
      print "ALL WAYS BLOCKED"
      self.robotMove(self.STOP)   
  
  def calcZone(self):
    leftLimit = 0
    rightLimit = 0    
    if self.OBS == 0:
      leftLimit = self.imgWidth/2 - 50
      rightLimit = self.imgWidth/2 + 50
    #Both Left and Center are blocked, try to maintain destination towards left end of camera
    elif self.OBS == (self.OBS_L | self.OBS_C):
      leftLimit = (self.imgWidth/2 - 50) - 200
      rightLimit = (self.imgWidth/2 + 50) - 200
    #Both Center and Right are blocked, try to maintain destination towards right end of camera      
    elif self.OBS == (self.OBS_C | self.OBS_R):
      leftLimit = (self.imgWidth/2 - 50) + 200
      rightLimit = (self.imgWidth/2 + 50) + 200            
    #Weirdly right and left are blocked and there seems to be a way in the middle
    #Try to go through it      
    elif self.OBS == (self.OBS_L | self.OBS_R):
      leftLimit = (self.imgWidth/2 - 50)
      rightLimit = (self.imgWidth/2 + 50)
    #Only left is blocked, try to keep destination slighly towards the left      
    elif self.OBS == self.OBS_L:
      leftLimit = (self.imgWidth/2 - 50) - 100
      rightLimit = (self.imgWidth/2 + 50) - 100      
    #Only right is blocked, try to keep destination slighly towards the left      
    elif self.OBS == self.OBS_R:
      leftLimit = (self.imgWidth/2 - 50) + 100
      rightLimit = (self.imgWidth/2 + 50) + 100      
    #Center is blocked, we choose to go towards left, 
    #so that the longer right side of the robot does not hit the obstacle. 
    #Robot is longer on right because we use the left camera for navigation
    elif self.OBS == self.OBS_C:
      leftLimit = (self.imgWidth/2 - 50) - 150
      rightLimit = (self.imgWidth/2 + 50) - 150            
    #All paths are blocked, poor robot. Stop it
    elif self.OBS == (self.OBS_L | self.OBS_C | self.OBS_R):
      print "All paths blocked"
      raise Exception
    return leftLimit, rightLimit
  
  def robotMove(self, dir):
    if dir == self.STRAIGHT:
      velRamp = rampUp(0,self.stLinVel, 0.05)
      for vel_steps in velRamp:
          self.vel.linVelPcent = vel_steps
          self.vel.angVelPcent = 0.0
          self.robotCmdPub.publish(self.vel)
      rospy.logdebug("Straight")
    elif dir == self.LEFT:
      self.vel.linVelPcent = self.rotLinVel
      self.vel.angVelPcent = self.rotAngVel
      rospy.logdebug("Left")
    elif dir == self.RIGHT:
      self.vel.linVelPcent = self.rotLinVel
      self.vel.angVelPcent = -self.rotAngVel
      rospy.logdebug("Right")
    else:
      self.vel.linVelPcent = 0.0
      self.vel.angVelPcent = 0.0
      rospy.logdebug("Stop")
    self.robotCmdPub.publish(self.vel)

  def processUltrasound(self, data): 
    #Values based on calibration. Calculated in inches and then converted to m
    c = ((data.ultra_centre + 1.7)/1.325)*0.0254
    l = ((data.ultra_left + 1.7)/1.325)*0.0254
    r = ((data.ultra_right + 5)/2)*0.0254
    self.val_c = numpy.append(self.val_c, c)
    self.val_r = numpy.append(self.val_r, r)
    self.val_l = numpy.append(self.val_l, l)   
    self.val_c = numpy.delete(self.val_c, 1)  
    self.val_r = numpy.delete(self.val_r, 1)  
    self.val_l = numpy.delete(self.val_l, 1)          
    v_c = self.smooth(self.val_c)
    v_r = self.smooth(self.val_r)
    v_l = self.smooth(self.val_l)            
    return [v_l.mean(), v_c.mean(), v_r.mean()]
    
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
    
  def rampUp(x, y, jump):
    while x < y:
        yield x
        x += jump

  def rampDown(x, y, jump):
    while x < y:
        yield x
        x -= jump


  
if __name__ == "__main__":
  print "came here"
  nav = eodNav()
  nav.spin()
