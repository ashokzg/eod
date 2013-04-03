#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('eod_nav')

from teamb_ui.msg import Dest
import std_msgs.msg
from sensor_msgs.msg import CameraInfo

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
  
  def __init__(self):
    rospy.init_node("eod_navigation")
    self.navState = self.IDLE
    self.linVel = 0.0
    self.angVel = 0.0
    
    #Init the subscribers    
    #    UI
    rospy.Subscriber("UserDestination", Dest, self.userDest)
    rospy.Subscriber("uiState", std_msgs.msg.String, self.uiState)
    #    Tracker
    rospy.Subscriber("destination", Dest, self.trackedDest)
    #Init the publishers
    self.navStatePub = rospy.Publisher('Nav_State', std_msgs.msg.UInt32)
    self.robotLinVelPub = rospy.Publisher('Robot_Lin_Vel', std_msgs.msg.Float32)
    self.robotAngVelPub = rospy.Publisher('Robot_Ang_Vel', std_msgs.msg.Float32)
    
    self.navStatePub.publish(self.navState)
    self.robotLinVelPub.publish(self.linVel)
    self.robotAngVelPub.publish(self.angVel)
    #Initialize values
    try:
      img = rospy.wait_for_message("camera/camera_info", CameraInfo, 2.0)
      self.imgWidth = img.width
      self.imgHeight = img.height
    except rospy.ROSException:
      self.imgWidth = 640
      self.imgHeight = 480
    
  def spin(self):
    rospy.spin()
    
  def userDest(self, data):
    if data.destPresent == True:
      self.navState = self.TRACKING_ONLY
      #TODO (set to automode only after receiving command from UI)
      self.navState = self.AUTO_MODE
    print "Destination received; Tracking"
    
  def uiState(self):
    pass
  
  def trackedDest(self, data):
    if self.navState in [self.AUTO_MODE, self.DEST_LOST]:
      if data.destPresent == True:
        self.navState = self.AUTO_MODE
        cx = data.destX + data.destWidth/2
        cy = data.destY + data.destHeight/2
        #If the destination is slipping towards the left, turn left
        if cx < (self.imgWidth/2 - 20):
          self.robotMove(self.LEFT)
        #If the destination is slipping towards the right, turn right
        elif cx > (self.imgWidth/2 + 20):
          self.robotMove(self.RIGHT)
        #We are good to zip towards the destination
        else:
          self.robotMove(self.STRAIGHT) 
      else:
        self.navState = self.DEST_LOST
        self.robotMove(self.STOP)
  
  
  def robotMove(self, dir):
    if dir == self.STRAIGHT:
      self.linVel = 0.8
      self.angVel = 0.0
      rospy.logdebug("Straight")
      print "Straight"
    elif dir == self.LEFT:
      self.linVel = 0.3
      self.angVel = 0.3
      rospy.logdebug("Left")
      print "Left"
    elif dir == self.RIGHT:
      self.linVel = 0.3
      self.angVel = -0.3
      rospy.logdebug("Right")
      print "Right"
    else:
      self.linVel = 0.0
      self.angVel = 0.0
      rospy.logdebug("Stop")
      print "STOP"
    self.robotLinVelPub.publish(self.linVel)
    self.robotAngVelPub.publish(self.angVel)
  
if __name__ == "__main__":
  print "came here"
  nav = eodNav()
  nav.spin()