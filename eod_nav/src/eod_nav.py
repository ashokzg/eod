#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('eod_nav')

from teamb_ui.msg import Dest
import std_msgs.msg
from sensor_msgs.msg import CameraInfo, Range
from eod_nav.msg import Velocity, Ultrasonic

import numpy
import copy

def enum(**enums):
    return type('Enum', (), enums)

AUTO = enum(
  #States of the automonous driving statemachine
  IDLE              = 0,   #Auto mode has not started
  DEST_IN_SIGHT     = 1,   #Normal navigation mode
  OBS_AVOIDANCE     = 2,   #We have sighted an obstacle at 3m distance and want to avoid it
  OBS_BACKTRACK     = 3,   #We have lost the destination when we tried to avoid, we are gonna backtrack
  OBS_RE_AVOIDANCE  = 5,   #we have backtracked and we will try to avoid it again now
  OBS_CUTTING_CORNER= 6,   #The robot just close to avoiding an obstacle, and tries to cut corner now
  DEST_SEARCH       = 7,   #We have lost the destination, go into destination search mode
  DEST_REACHED      = 8,   #The destination has been reached
  ERROR             = 9)   #Error state, with error code in comments  

NAV = enum(
  #States of the robot
  IDLE          = 1,
  TRACKING_ONLY = 2,
  AUTO_MODE     = 3,
  ROBOT_LOST    = 4,
  MANUAL_MODE   = 5)

ERR = enum(
  #Error codes
  ERR_ILLEGAL           = 0,
  ERR_STUPID            = 1,
  ERR_UNABLE_TO_AVOID   = 2,
  ERR_UNABLE_TO_SEARCH  = 3,
  ERR_TIMEOUT           = 4,
  NONE                  = 5,
  PREVIOUS              = 6)

USR = enum(
  #States of the UI
  UI_READY      = 0,
  IDLE_OR_MANUAL= 1,
  TRACKING_ONLY = 2,
  AUTO_MODE     = 3)          

  

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
  
  #Obstacle presence
  OBS_L = 1
  OBS_C = 2
  OBS_R = 4
  OBS_IDX = [OBS_L, OBS_C, OBS_R]
  OBS_AVOID  = 0

  #Used for smoothing the ultrasonic
  val_c = numpy.zeros(15)  
  val_r = numpy.zeros(15)  
  val_l = numpy.zeros(15)      
  #===============================================================
  #
  #    INITIALIZATIONS AND PRE-NAVIGATION FUNCTIONS
  #
  #===============================================================
  def __init__(self):
    rospy.init_node("nav")
    print "EOD Navigation Initializing"
    self.highLevelInits()
    self.lowLevelInits()
    self.initSubAndPub()    
    self.updateParameters()
    self.initCamParams()
    
  def highLevelInits(self):
    #Keep track of state
    self.autoState  = AUTO.IDLE
    self.navState   = NAV.IDLE
    self.uiState    = USR.UI_READY 
    self.prevState = NAV.IDLE
        
  def lowLevelInits(self):
    self.vel = Velocity()          
    self.vel.linVelPcent = 0.0
    self.vel.angVelPcent = 0.0
    self.dest = Dist()
    self.dest.destPresent = False     
    self.Stopped = False
    self.defineAutoStateTransitionMatrix()
    
  def initCamParams(self):    
    #Initialize values
    try:
      img = rospy.wait_for_message("camera/camera_info", CameraInfo, 2.0)
      self.imgWidth = img.width
      self.imgHeight = img.height
    except rospy.ROSException:
      self.imgWidth = 640
      self.imgHeight = 480

  def initSubAndPub(self):      
    #Init the subscribers    
    #    UI
    rospy.Subscriber("UserDestination", Dest, self.userDest)
    rospy.Subscriber("UiStatus", std_msgs.msg.Int32, self.uiStateHdl)
    #    Tracker
    rospy.Subscriber("destination", Dest, self.trackedDest)
    rospy.Subscriber("ultrasound", Ultrasonic, self.ultraSound)
    #Init the publishers
    self.navStatePub = rospy.Publisher('Nav_State', std_msgs.msg.UInt32)
    self.robotCmdPub = rospy.Publisher('cmd_vel', Velocity)
    #Publish the messages now
    self.navStatePub.publish(self.navState)
    self.robotCmdPub.publish(self.vel)
    
  def updateParameters(self):
    self.stLinVel = rospy.get_param("~st_lin", 0.4)
    self.rotLinVel = rospy.get_param("~rot_lin", 0.4)
    self.rotAngVel = rospy.get_param("~rot_ang", 0.2) 
    self.OBS_AVOID_DIST = rospy.get_param("~OBS_AVOID_DIST", 2.8)   
    self.OBS_STOP_DIST = rospy.get_param("~stop_dist", 0.8)
    self.obsStLinVel = rospy.get_param("~obs_st_lin", 0.2)
    self.obsRotLinVel = rospy.get_param("~obs_rot_lin", 0.1)
    self.obsRotAngVel = rospy.get_param("~obs_rot_ang", 0.1) 
    #Reset the parameters so that it would be easily visible to debug
    rospy.set_param("~st_lin_vel", self.stLinVel)
    rospy.set_param("~rot_lin_vel", self.rotLinVel)
    rospy.set_param("~rot_ang_vel", self.rotAngVel)
    rospy.set_param("~OBS_AVOID_DIST", self.OBS_AVOID_DIST)        
    rospy.set_param("~stop_dist", self.OBS_STOP_DIST)        
    rospy.set_param("~obs_st_lin", self.obsStLinVel)
    rospy.set_param("~obs_rot_lin", self.obsRotLinVel)
    rospy.set_param("~obs_rot_ang", self.obsRotAngVel)
    print "Velocity parameters", self.stLinVel, self.rotLinVel, self.rotAngVel
     
    
  def spin(self):
    rospy.spin()

  #===============================================================
  #
  #    ULTRASONIC SENSOR and CAMERA PROCESSING
  #
  #===============================================================    
  def ultraSound(self, data):
    #Smoothes the ultrasonic data with a gaussian filter
    #Distance is a list of ultrasonic [left, center, right]
    self.dist = self.processUltrasound(data)
    try:
      l, r = self.calcZone()
      print l, r,
      allPathsBlocked = 0 
    except:
      allPathsBlocked = 1

    self.navStatePub.publish(self.navState)    
    self.OBS_AVOID = 0
    for i in range(3):
      if self.dist[i] < self.OBS_AVOID_DIST:
        self.OBS_AVOID |= self.OBS_IDX[i]
      elif self.dist[i] < self.OBS_STOP_DIST:
        self.OBS_STOP |= self.OBS_IDX[i]


  def trackedDest(self, data):
    self.dest = copy.deepcopy(data)    

  #===============================================================
  #
  #    MAIN NAVIGATION HANDLING
  #
  #===============================================================
  def navHandler(self):
    pass

  def autoModeHandler(self):
    if self.navState == NAV.AUTO_MODE:
      #All paths are blocked
      if self.dist[self.UC] < self.OBS_STOP_DIST:
        print "STOPPING"
        self.robotMove(self.STOP)
        self.autoState = self.AUTO.OBS_STOP        
      elif  self.dist[self.UC] > self.OBS_STOP_DIST and self.Stopped == True and self.navState == NAV.ROBOT_LOST:
        self.navState = self.prevState
        self.prevState = NAV.MANUAL_MODE
        self.Stopped = False    
    
  def userDest(self, data):
    if data.destPresent == True:
      self.uiState = USR.TRACKING_ONLY
  
  def uiStateHdl(self, data):
    self.uiState = data.data
    if data.data > USR.AUTO_MODE:
      raise ValueError('Unknown state from the EOD UI')
      
  
  '''  
  def uiStateHdl(self, data):
    if data.data == 0:
      self.navState = NAV.IDLE
      self.robotMove(self.STOP)      
    elif data.data == 1:
      self.navState = NAV.MANUAL_MODE
      self.robotMove(self.STOP)
    elif data.data == 2:
      self.navState = NAV.TRACKING_ONLY
      self.robotMove(self.STOP)      
    elif data.data == 3:
      self.updateParameters()
      self.navState = NAV.AUTO_MODE
    else:
      #Should not come to this state
      print "This is not good! Unknown state from UI"
      self.navState = NAV.IDLE
      self.robotMove(self.STOP)      
  '''
  def control(self, data):
    #print "tracking in state", self.navState 
    try:
      leftLimit, rightLimit = self.calcZone();
    except:
      self.robotMove(self.STOP)   
      return
    if self.navState in [NAV.AUTO_MODE, NAV.DEST_LOST]:
      if data.destPresent == True:
        if self.navState == NAV.DEST_LOST:
          self.navState = NAV.AUTO_MODE
        self.navState = NAV.AUTO_MODE
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
        if self.navState != NAV.DEST_LOST:
          self.frameCount = 0
          self.navState = NAV.DEST_LOST
        else:
          self.search_destination()
        #self.robotMove(self.STOP)
  
  def search_destination(self):    
    print "Searching for Destination"
    if self.dist[1] > 0.8:
      #Arbitrarily choose to go forward
      if self.frameCount <= 40:
        self.vel.linVelPcent = 0.2
        self.vel.angVelPcent = 0
      #after some time choose to turn LEFT
      elif self.frameCount <= 80:
        self.vel.linVelPcent = 0.3
        self.vel.angVelPcent = 0.1
      elif self.frameCount <= 120:
        self.vel.linVelPcent = 0.0
        self.vel.angVelPcent = 0.0
      else:
        self.frameCount = 0
      self.robotCmdPub.publish(self.vel)      
      self.frameCount += 1
                      
  
  def calcZone(self):
    leftLimit = 0
    rightLimit = 0    
    if self.OBS_AVOID == 0:
      leftLimit = self.imgWidth/2 - 50
      rightLimit = self.imgWidth/2 + 50
    #Both Left and Center are blocked, try to maintain destination towards left end of camera
    elif self.OBS_AVOID == (self.OBS_L | self.OBS_C):
      leftLimit = (self.imgWidth/2 - 50) - 250
      rightLimit = (self.imgWidth/2 + 50) - 250
    #Both Center and Right are blocked, try to maintain destination towards right end of camera      
    elif self.OBS_AVOID == (self.OBS_C | self.OBS_R):
      leftLimit = (self.imgWidth/2 - 50) + 250
      rightLimit = (self.imgWidth/2 + 50) + 250            
    #Weirdly right and left are blocked and there seems to be a way in the middle
    #Try to go through it      
    elif self.OBS_AVOID == (self.OBS_L | self.OBS_R):
      leftLimit = (self.imgWidth/2 - 50)
      rightLimit = (self.imgWidth/2 + 50)
    #Only left is blocked, try to keep destination slighly towards the left      
    elif self.OBS_AVOID == self.OBS_L:
      leftLimit = (self.imgWidth/2 - 50) - 250
      rightLimit = (self.imgWidth/2 + 50) - 250      
    #Only right is blocked, try to keep destination slighly towards the left      
    elif self.OBS_AVOID == self.OBS_R:
      leftLimit = (self.imgWidth/2 - 50) + 250
      rightLimit = (self.imgWidth/2 + 50) + 250      
    #Center is blocked, we choose to go towards left, 
    #so that the longer right side of the robot does not hit the obstacle. 
    #Robot is longer on right because we use the left camera for navigation
    elif self.OBS_AVOID == self.OBS_C:
      leftLimit = (self.imgWidth/2 - 50) - 250
      rightLimit = (self.imgWidth/2 + 50) - 250            
    #All paths are blocked, poor robot. Stop it
    elif self.OBS_AVOID == (self.OBS_L | self.OBS_C | self.OBS_R):
      leftLimit = 20
      rightLimit = 80
    return leftLimit, rightLimit
  
  def robotMove(self, dir):
    if dir == self.STRAIGHT:
      #velRamp = rampUp(0,self.stLinVel, 0.05)
      #for vel_steps in velRamp:
      self.vel.linVelPcent = self.stLinVel
      self.vel.angVelPcent = 0.0
      #self.robotCmdPub.publish(self.vel)
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

  #===============================================================
  #
  #    MISCELLANEOUS HELPER FUNCTIONS
  #
  #===============================================================

  def printNavStatus(self):
    #print data.ultra_left, data.ultra_centre, data.ultra_right,
    print [self.OBS_AVOID & self.OBS_L, self.OBS_AVOID & self.OBS_C, self.OBS_AVOID & self.OBS_R],
    print ["%0.3f" %i for i in distance],
    print self.navState, self.prevState, self.Stopped,
    if allPathsBlocked == 1:
      print "ALL PATHS BLOCKED"
    else:
      print ""

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

  def defineAutoStateTransitionMatrix(self):
    self.autoStateTrans = {
      #(Current State, Obs Stopping, Obs avoid, Dest Present) : [Next State, Comments]
      (AUTO.IDLE,                0,0,0) : (AUTO.IDLE                , AUTO.NONE                ),
      (AUTO.IDLE,                0,0,1) : (AUTO.DEST_IN_SIGHT       , AUTO.NONE                ),
      (AUTO.IDLE,                0,1,0) : (AUTO.ERROR               , AUTO.ERR_ILLEGAL         ),
      (AUTO.IDLE,                0,1,1) : (AUTO.OBS_AVOIDANCE       , AUTO.NONE                ),
      (AUTO.IDLE,                1,0,0) : (AUTO.ERROR               , AUTO.ERR_ILLEGAL         ),
      (AUTO.IDLE,                1,0,1) : (AUTO.ERROR               , AUTO.ERR_ILLEGAL         ),
      (AUTO.IDLE,                1,1,0) : (AUTO.ERROR               , AUTO.ERR_ILLEGAL         ),
      (AUTO.IDLE,                1,1,1) : (AUTO.ERROR               , AUTO.ERR_STUPID          ),
      (AUTO.DEST_IN_SIGHT,       0,0,0) : (AUTO.DEST_SEARCH         , AUTO.NONE                ),
      (AUTO.DEST_IN_SIGHT,       0,0,1) : (AUTO.DEST_IN_SIGHT       , AUTO.NONE                ),
      (AUTO.DEST_IN_SIGHT,       0,1,0) : (AUTO.OBS_BACKTRACK       , AUTO.NONE                ),
      (AUTO.DEST_IN_SIGHT,       0,1,1) : (AUTO.OBS_AVOIDANCE       , AUTO.NONE                ),
      (AUTO.DEST_IN_SIGHT,       1,0,0) : (AUTO.ERROR               , AUTO.ERR_ILLEGAL         ),
      (AUTO.DEST_IN_SIGHT,       1,0,1) : (AUTO.ERROR               , AUTO.ERR_ILLEGAL         ),
      (AUTO.DEST_IN_SIGHT,       1,1,0) : (AUTO.ERR_DYNAMIC_OBSTACLE, AUTO.NONE                ),
      (AUTO.DEST_IN_SIGHT,       1,1,1) : (AUTO.ERR_DYNAMIC_OBSTACLE, AUTO.NONE                ),
      (AUTO.OBS_AVOIDANCE,       0,0,0) : (AUTO.DEST_SEARCH         , AUTO.NONE                ),
      (AUTO.OBS_AVOIDANCE,       0,0,1) : (AUTO.DEST_IN_SIGHT       , AUTO.NONE                ),
      (AUTO.OBS_AVOIDANCE,       0,1,0) : (AUTO.OBS_BACKTRACK       , AUTO.NONE                ),
      (AUTO.OBS_AVOIDANCE,       0,1,1) : (AUTO.OBS_AVOIDANCE       , AUTO.NONE                ),
      (AUTO.OBS_AVOIDANCE,       1,0,0) : (AUTO.ERROR               , AUTO.ERR_ILLEGAL         ),
      (AUTO.OBS_AVOIDANCE,       1,0,1) : (AUTO.ERROR               , AUTO.ERR_ILLEGAL         ),
      (AUTO.OBS_AVOIDANCE,       1,1,0) : (AUTO.OBS_BACKTRACK       , AUTO.NONE                ),
      (AUTO.OBS_AVOIDANCE,       1,1,1) : (AUTO.OBS_CUTTING_CORNER  , AUTO.NONE                ),
      (AUTO.OBS_BACKTRACK,       0,0,0) : (AUTO.DEST_SEARCH         , AUTO.NONE                ),
      (AUTO.OBS_BACKTRACK,       0,0,1) : (AUTO.OBS_RE_AVOIDANCE    , AUTO.NONE                ),
      (AUTO.OBS_BACKTRACK,       0,1,0) : (AUTO.OBS_BACKTRACK       , AUTO.NONE                ),
      (AUTO.OBS_BACKTRACK,       0,1,1) : (AUTO.OBS_BACKTRACK       , AUTO.NONE                ),
      (AUTO.OBS_BACKTRACK,       1,0,0) : (AUTO.ERROR               , AUTO.ERR_ILLEGAL         ),
      (AUTO.OBS_BACKTRACK,       1,0,1) : (AUTO.ERROR               , AUTO.ERR_ILLEGAL         ),
      (AUTO.OBS_BACKTRACK,       1,1,0) : (AUTO.OBS_BACKTRACK       , AUTO.NONE                ),
      (AUTO.OBS_BACKTRACK,       1,1,1) : (AUTO.OBS_BACKTRACK       , AUTO.NONE                ),
      (AUTO.OBS_RE_AVOIDANCE,    0,0,0) : (AUTO.DEST_SEARCH         , AUTO.NONE                ),
      (AUTO.OBS_RE_AVOIDANCE,    0,0,1) : (AUTO.OBS_RE_AVOIDANCE    , AUTO.NONE                ),
      (AUTO.OBS_RE_AVOIDANCE,    0,1,0) : (AUTO.OBS_BACKTRACK       , AUTO.NONE                ),
      (AUTO.OBS_RE_AVOIDANCE,    0,1,1) : (AUTO.OBS_RE_AVOIDANCE    , AUTO.NONE                ),
      (AUTO.OBS_RE_AVOIDANCE,    1,0,0) : (AUTO.ERROR               , AUTO.ERR_ILLEGAL         ),
      (AUTO.OBS_RE_AVOIDANCE,    1,0,1) : (AUTO.ERROR               , AUTO.ERR_ILLEGAL         ),
      (AUTO.OBS_RE_AVOIDANCE,    1,1,0) : (AUTO.ERROR               , AUTO.ERR_UNABLE_TO_AVOID ),
      (AUTO.OBS_RE_AVOIDANCE,    1,1,1) : (AUTO.OBS_CUTTING_CORNER  , AUTO.NONE                ),
      (AUTO.OBS_CUTTING_CORNER,  0,0,0) : (AUTO.OBS_CUTTING_CORNER  , AUTO.NONE                ),
      (AUTO.OBS_CUTTING_CORNER,  0,0,1) : (AUTO.DEST_IN_SIGHT       , AUTO.NONE                ),
      (AUTO.OBS_CUTTING_CORNER,  0,1,0) : (AUTO.OBS_CUTTING_CORNER  , AUTO.NONE                ),
      (AUTO.OBS_CUTTING_CORNER,  0,1,1) : (AUTO.OBS_AVOIDANCE       , AUTO.NONE                ),
      (AUTO.OBS_CUTTING_CORNER,  1,0,0) : (AUTO.ERROR               , AUTO.ERR_ILLEGAL         ),
      (AUTO.OBS_CUTTING_CORNER,  1,0,1) : (AUTO.ERROR               , AUTO.ERR_ILLEGAL         ),
      (AUTO.OBS_CUTTING_CORNER,  1,1,0) : (AUTO.OBS_CUTTING_CORNER  , AUTO.NONE                ),
      (AUTO.OBS_CUTTING_CORNER,  1,1,1) : (AUTO.OBS_CUTTING_CORNER  , AUTO.NONE                ),
      (AUTO.DEST_SEARCH,         0,0,0) : (AUTO.DEST_SEARCH         , AUTO.NONE                ),
      (AUTO.DEST_SEARCH,         0,0,1) : (AUTO.DEST_IN_SIGHT       , AUTO.NONE                ),
      (AUTO.DEST_SEARCH,         0,1,0) : (AUTO.DEST_SEARCH         , AUTO.NONE                ),
      (AUTO.DEST_SEARCH,         0,1,1) : (AUTO.OBS_AVOIDANCE       , AUTO.NONE                ),
      (AUTO.DEST_SEARCH,         1,0,0) : (AUTO.ERROR               , AUTO.ERR_ILLEGAL         ),
      (AUTO.DEST_SEARCH,         1,0,1) : (AUTO.ERROR               , AUTO.ERR_ILLEGAL         ),
      (AUTO.DEST_SEARCH,         1,1,0) : (AUTO.ERROR               , AUTO.ERR_UNABLE_TO_SEARCH),
      (AUTO.DEST_SEARCH,         1,1,1) : (AUTO.ERROR               , AUTO.ERR_UNABLE_TO_SEARCH),
      (AUTO.ERROR,               0,0,0) : (AUTO.ERROR               , AUTO.PREVIOUS            ),
      (AUTO.ERROR,               0,0,1) : (AUTO.ERROR               , AUTO.PREVIOUS            ),
      (AUTO.ERROR,               0,1,0) : (AUTO.ERROR               , AUTO.PREVIOUS            ),
      (AUTO.ERROR,               0,1,1) : (AUTO.ERROR               , AUTO.PREVIOUS            ),
      (AUTO.ERROR,               1,0,0) : (AUTO.ERROR               , AUTO.PREVIOUS            ),
      (AUTO.ERROR,               1,0,1) : (AUTO.ERROR               , AUTO.PREVIOUS            ),
      (AUTO.ERROR,               1,1,0) : (AUTO.ERROR               , AUTO.PREVIOUS            ),
      (AUTO.ERROR,               1,1,1) : (AUTO.ERROR               , AUTO.PREVIOUS            )
    }  
    
  
if __name__ == "__main__":  
  nav = eodNav()
  r = rospy.Rate(20)
  while not rospy.is_shutdown():
    nav.navHandler()
    r.sleep()
  print "Exiting EOD Navigation! Bye Bye"
