#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('eod_nav')

from teamb_ui.msg import Dest
from std_msgs.msg import UInt32, Int32
from sensor_msgs.msg import CameraInfo, Range
from eod_nav.msg import NavDebug
from ultrasonic.msg import Ultrasonic
from vel_msgs.msg import Velocity
from pcl_eod.msg import Clusters
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import numpy
import copy
import math
from odometry import Odo

DEBUG = True

def enum(**enums):
    return type('Enum', (), enums)

AUTO = enum(
  #States of the automonous driving statemachine
  IDLE              = 0,   #Auto mode has not started
  DEST_IN_SIGHT     = 1,   #Normal navigation mode
  OBS_AVOIDANCE     = 2,   #We have sighted an obstacle at 3m distance and want to avoid it
  OBS_BACKTRACK     = 3,   #We have lost the destination when we tried to avoid, we are gonna backtrack
  OBS_RE_AVOIDANCE  = 4,   #we have backtracked and we will try to avoid it again now
  SLOPE             = 5,   #Used for slope. Wrongly named because of laziness
  DEST_SEARCH       = 6,   #We have lost the destination, go into destination search mode
  DEST_REACHED      = 7,   #The destination has been reached
  ERROR             = 8)   #Error state, with error code in comments  

NAV = enum(
  #States of the robot
  IDLE          = 0,
  TRACKING_ONLY = 1,
  AUTO_MODE     = 2,  
  MANUAL_MODE   = 3,
  ERROR         = 4)

ERR = enum(
  #Error codes
  #--- ERROR CODES OF AUTO STATE ---#
  ERR_ILLEGAL             = 0,    #Illegal Autonomous mode. Comes when ultrasonic is < 0.5 but also > 3.
  ERR_STUPID              = 1,    #Someone does crazy things
  ERR_DYNAMIC_OBSTACLE    = 2,    #Dynamic obstacles
  ERR_UNABLE_TO_AVOID     = 3,    #Unable to avoid the obstacle after 1 retry
  ERR_UNABLE_TO_SEARCH    = 4,    #Unable to locate the destination after search
  ERR_TIMEOUT             = 5,    #Timeout error
  #--- ERROR CODES OF NAV STATE ---#
  ERR_UNKNOWN_DESTINATION = 6,    #Asked robot to move autonomously without giving destination
  ERR_ROBOT_LOST          = 7,    #The robot is lost.
  #--- ERROR CODES OF GENERAL ---#
  NONE                    = 8,    #No errors. Default value of any error state
  PREVIOUS                = 9)    #Same error as previous

USR = enum(
  #States of the UI
  UI_READY      = 0,
  IDLE_OR_MANUAL= 1,
  TRACKING_ONLY = 2,
  AUTO_MODE     = 3)          

AUTO_OUTPUT =enum(
  ROBOT_LOST = 0,
  REACHED_DESTINATION = 1,
  STILL_RUNNING = 2)


class AutoNavError(Exception):
  def __init__(self, errId, msg):
    self.errId = errId
    self.msg = msg
  def __str__(self):
    return repr(self.msg)

class NavError(Exception):
  def __init__(self, errId, msg):
    self.errId = errId
    self.msg = msg
  def __str__(self):
    return repr(self.msg)

class eodNav:
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
  
  GROUNDLEVEL = 0
  UPSLOPE = 2
  DOWNSLOPE = 1
  
  #===============================================================
  #
  #    INITIALIZATIONS AND PRE-NAVIGATION FUNCTIONS
  #
  #===============================================================
  def __init__(self):
    rospy.init_node("nav")
    rospy.logwarn("EOD Navigation Initializing")
    self.highLevelInits()
    self.lowLevelInits()
    self.updateParameters()
    self.initSubAndPub()    
    self.initCamParams()
    
    
  def highLevelInits(self):
    #Keep track of state
    self.autoState  = AUTO.IDLE
    self.navState   = NAV.IDLE
    self.uiState    = USR.UI_READY 
    self.prevState  = NAV.IDLE
        
        
  def lowLevelInits(self):
    self.vel = Velocity() 
    self.manCmdVel = Velocity()  
    self.twist = Twist()
    self.navDebug = NavDebug()     
    self.servoAngle = UInt32()
    self.odom = Odometry()  
    self.vel.linVelPcent = 0.0
    self.vel.angVelPcent = 0.0
    self.manCmdVel.linVelPcent = 0.0
    self.manCmdVel.angVelPcent = 0.0
    self.pclObsCountLeft = 0 
    self.pclObsCountRight = 0
    self.dest = Dest()
    self.clusters = Clusters()
    self.dest.destPresent = False     
    self.Stopped = False
    self.robotResp = AUTO_OUTPUT.REACHED_DESTINATION #TODO Automode return status
    self.defineNavStateTransitionMatrix()
    self.defineAutoStateTransitionMatrix()
    self.definePrintNames()
    self.dist = [0,0,0]
    self.OBS_STOP = 0
    self.OBS_AVOID = 0
    self.TIME_STEP = 50 #ms
    self.AREA_THRESHOLD = 0.12 #If destination is greater than 50% of the image stop.
    self.ultraCount = 0
    self.avoidState = 0
    self.sweepSet = False
    self.sweepState = 0
    self.contSweepSet = False
    self.contSweepState = 0 
    self.totalSweepStates = 25
    self.contTotalSweepStates = 3
    self.contSweepAngle = 20
    self.sweepDist = [600]*self.totalSweepStates
    self.contSweepDist = [600]*self.contTotalSweepStates           
    self.obsAvoidStart = False
    self.desPose = [0,0,0]
    self.robotPose = [0,0,0]
    self.UltraFilterSize = 8
    self.avoidDirection = self.LEFT
    self.slope = self.GROUNDLEVEL
    #Used for smoothing the ultrasonic
    self.val_c = numpy.zeros(self.UltraFilterSize)  
    self.val_r = numpy.zeros(self.UltraFilterSize)  
    self.val_l = numpy.zeros(self.UltraFilterSize)
    
  def initCamParams(self):    
    #Initialize values
    try:
      img = rospy.wait_for_message(self.camInfoName, CameraInfo, 2.0)
      self.imgWidth = img.width
      self.imgHeight = img.height
    except rospy.ROSException:
      self.imgWidth = 640
      self.imgHeight = 480


  def initSubAndPub(self):      
    #Init the subscribers    
    #    UI
    rospy.Subscriber("UserDestination", Dest, self.userDest)
    rospy.Subscriber("UiStatus", Int32, self.uiStateHdl)
    rospy.Subscriber("man_cmd_vel", Velocity, self.manCmdVelHdl)
    #    Tracker
    rospy.Subscriber("destination", Dest, self.trackedDest)
    rospy.Subscriber("ultrasound", Ultrasonic, self.ultraSound)
    rospy.Subscriber("clusters", Clusters, self.clusterHdl)
    rospy.Subscriber("odom", Odometry, self.odomHdl)
    #Init the publishers
    self.navStatePub = rospy.Publisher('Nav_State', UInt32)
    self.robotCmdPub = rospy.Publisher('cmd_vel', Velocity)
    self.robotTwistPub = rospy.Publisher('twist', Twist)    
    self.errStatePub = rospy.Publisher('Nav_Error_Id', UInt32)
    self.navDebugPub = rospy.Publisher('Nav_Debug', NavDebug)
    self.servoPub = rospy.Publisher('servo_angle', UInt32)
    #Publish the messages now
    self.navStatePub.publish(self.navState)
    self.setCmdVel()
    
    
  def updateParameters(self):
#     self.stLinVel = rospy.get_param("~st_lin", 0.4)
#     self.rotLinVel = rospy.get_param("~rot_lin", 0.4)
#     self.rotAngVel = rospy.get_param("~rot_ang", 0.2) 
#     self.OBS_AVOID_DIST = rospy.get_param("~obs_avoid_dist", 50)   
#     self.OBS_STOP_DIST = rospy.get_param("~obs_stop_dist", 20)
#     self.obsStLinVel = rospy.get_param("~obs_st_lin", 0.3)
#     self.obsRotLinVel = rospy.get_param("~obs_rot_lin", 0.3)
#     self.obsRotAngVel = rospy.get_param("~obs_rot_ang", 0.1)
    self.stLinVel = 0.3
    self.rotLinVel = 0.3
    self.rotAngVel = 2.0    
    self.OBS_AVOID_DIST = 250 #rospy.get_param("~obs_avoid_dist", 100)   
    self.OBS_STOP_DIST = rospy.get_param("~obs_stop_dist", 20)
    self.obsStLinVel = 0.3
    self.obsRotLinVel = 0.3
    self.obsRotAngVel = 2.0   
    self.cameraName = rospy.get_param("/eod_cam", "camera/image_raw") 
    #Reset the parameters so that it would be easily visible to debug
    rospy.set_param("~st_lin_vel", self.stLinVel)
    rospy.set_param("~rot_lin_vel", self.rotLinVel)
    rospy.set_param("~rot_ang_vel", self.rotAngVel)
    rospy.set_param("~obs_avoid_dist", self.OBS_AVOID_DIST)        
    rospy.set_param("~obs_stop_dist", self.OBS_STOP_DIST)        
    rospy.set_param("~obs_st_lin", self.obsStLinVel)
    rospy.set_param("~obs_rot_lin", self.obsRotLinVel)
    rospy.set_param("~obs_rot_ang", self.obsRotAngVel)
    rospy.set_param("/eod_cam", self.cameraName)    
    self.camInfoName = '/' + self.cameraName.split('/')[0] + '/camera_info'
    rospy.logwarn("Camera Name " + self.camInfoName)
    #print "Velocity parameters", self.stLinVel, self.rotLinVel, self.rotAngVel
     
    
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
    self.OBS_AVOID = 0
    self.OBS_STOP = 0
    #Time required to stablize
    if self.ultraCount > 4:
      for i in range(3):
        if self.dist[i] < self.OBS_AVOID_DIST:
          pass
          self.OBS_AVOID |= self.OBS_IDX[i]
        if self.dist[i] < self.OBS_STOP_DIST:
          pass
          self.OBS_STOP |= self.OBS_IDX[i]
    self.ultraCount += 1

  def contSweep(self):
    if self.contSweepState <= self.contTotalSweepStates:
      if self.contSweepSet == False:
        self.servoAngle.data = (self.contSweepState-((self.contTotalSweepStates-1)/2))*self.contSweepAngle + 90              
        self.servoPub.publish(self.servoAngle)
        self.contSweepSet = True
        self.contSweepCount = 0
      elif self.contSweepCount*self.TIME_STEP > 300:        
        self.contSweepSet = False
        if self.contSweepState < self.contTotalSweepStates:
          self.contSweepDist[self.contSweepState] = self.dist[1]
        self.contSweepState += 1
        if self.contSweepState == self.contTotalSweepStates:
          self.contSweepState = 0
          self.contSweepCount = 0
          self.contSweepSet = False         
    self.contSweepCount +=  1    

  def trackedDest(self, data):
    self.dest = copy.deepcopy(data)    
    self.destcx = self.dest.destX + self.dest.destWidth/2
    self.destcy = self.dest.destY + self.dest.destHeight/2
    if self.dest.destPresent == True:
      if self.destcy > 180 and self.destcy < 300:
        self.slope = self.GROUNDLEVEL
      elif self.destcy < 180:
        self.slope = self.DOWNSLOPE
      elif self.destcy > 280:
        self.slope = self.UPSLOPE   
    

  def clusterHdl(self, data):
    print "Coming to cluster"
    self.clusters = copy.deepcopy(data)
    self.pclObs = []
    origin = Point(0,0,0)
    try:
      area = self.destArea
    except:
      area = 0.0
    for i in range(len(data.minpoint)):
      mp = data.minpoint[i]
      xp = data.maxpoint[i]
      x = (mp.x + xp.x)/2
      #If overall distance is less than 4 m
      if area < 0.1:
        if self.eucdist(mp, origin) < 3.0:
          #if midpoint of the object is located at less than 1.5 m from our straight line path
          if abs(x) < 0.5:
            self.pclObs.append(mp)
            self.prevOBS_AVOID = self.OBS_AVOID
            if mp.x > 0:
              self.OBS_AVOID |= self.OBS_L
              self.pclObsCountLeft = 0
            else:
              self.OBS_AVOID |= self.OBS_R
              self.pclObsCountRight = 0
      
  def eucdist(self, p1, p2):    
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)

  def odomHdl(self, data):
    X = data.pose.pose.position.x
    Y = data.pose.pose.position.y
    o = data.pose.pose.orientation      
    T = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
    self.robotPose = [X, Y, T]
  #===============================================================
  #
  #    MAIN NAVIGATION HANDLING
  #
  #===============================================================
  def navHandler(self):    
    self.assessPclData()
    self.navStateChange = self.navStateChanger()
    if self.navState == NAV.IDLE:
      self.idleStateHandler()
    elif self.navState == NAV.TRACKING_ONLY:
      self.trackingOnlyStateHandler()
    elif self.navState == NAV.AUTO_MODE:
      try:
        self.autoModeStateHandler()
      except AutoNavError as e:
        if e.errId <= ERR.ERR_DYNAMIC_OBSTACLE:
          rospy.logerr("Something really stupid has happened")
        else:
          rospy.logerr("Poor robot lost it's destination")
        rospy.logerr("Auto Mode error:"  + self.errPrintNames[e.errId] + " " + e.msg)
        self.errStatePub.publish(e.errId)
        #Setting this will make the state go to Manual mode in the next iteration
        self.robotResp = AUTO_OUTPUT.ROBOT_LOST
    elif self.navState == NAV.MANUAL_MODE:
      self.manualModeStateHandler()
    elif self.navState == NAV.ERROR:
      self.errorStateHandler()
    self.printNavStatus()
    if self.navStateChange == True:
      self.navStatePub.publish(self.navState)
      


  def idleStateHandler(self):
    if self.navStateChange == True:
      self.robotMove(self.STOP)
  
  
  def trackingOnlyStateHandler(self):
    if self.navStateChange == True:
      self.robotMove(self.STOP)


  def manualModeStateHandler(self):
   # if self.navStateChange == 0:
   #   self.sweepState = 0
   #   self.sweepCount = 0
   # if self.sweepState < 20:
   #   self.sweep()
   # else:
   #   print self.sweepDist
    if self.navStateChange == True:
      self.vel.linVelPcent = 0
      self.vel.angVelPcent = 0
    else:
      self.vel = self.manCmdVel
    self.twist.linear.x = self.vel.linVelPcent
    self.twist.angular.z = self.vel.angVelPcent*5
    self.robotTwistPub.publish(self.twist)


  def errorStateHandler(self):
    #Wait for UI to take action
    if self.navStateChange == True:
      rospy.logerr("Navigation Error occurred!" + self.errPrintNames[self.navErrId])
      rospy.logerr("Waiting for UI to take action")      
      self.errStatePub.publish(self.navErrId)    


  def navStateChanger(self):
    stateChange = False
    try:
      self.prevNavState = self.navState
      if self.navState == NAV.AUTO_MODE and self.autoState == AUTO.DEST_REACHED:
        self.robotResp = AUTO_OUTPUT.REACHED_DESTINATION
      t = self.navStateTrans[(self.navState, self.uiState, self.robotResp)]
      self.navState = t[0]
      self.navErrId = t[1]
      if self.navState == self.prevNavState:
        stateChange = False
      else:
        stateChange = True          
    except KeyError:
      pass
    return stateChange    

  def userDest(self, data):
    if data.destPresent == True:
      self.uiState = USR.TRACKING_ONLY
  
  def uiStateHdl(self, data):
    self.uiState = data.data
    if data.data > USR.AUTO_MODE:
      raise ValueError('Unknown state from the EOD UI')
  
  def manCmdVelHdl(self, data):
    self.manCmdVel = copy.deepcopy(data)
        
  def setCmdVel(self):
    if self.dist[1] > 20:
      pass
      #self.robotCmdPub.publish(self.vel)
    else:
      print "STOPPING"
      self.vel.linVelPcent = 0.0
      self.vel.angVelPcent = 0.0
    self.twist.linear.x = self.vel.linVelPcent
    self.twist.angular.z = self.vel.angVelPcent
    self.robotTwistPub.publish(self.twist)      
      

  #===============================================================
  #
  #    AUTONOMOUS NAVIGATION HANDLING
  #
  #===============================================================    
  
  def autoModeStateHandler(self):
    if self.navStateChange == True:
      self.autoModeInit()
      self.obsAvoidStart = False
      self.obsAvoidCount = 0
    if self.autoCount*self.TIME_STEP > 5*60*1000:
      raise AutoNavError(ERR.ERR_TIMEOUT, "Automode exceeded 5 minutes")    
    self.autoCount += 1
    self.assessPclData()
    self.autoStateChange = self.autoStateChanger()    
    if self.autoState == AUTO.IDLE:
      #No possibility of coming here
      pass
    elif self.autoState == AUTO.DEST_IN_SIGHT:
      self.autoDestInSight()
      pass
    elif self.autoState == AUTO.OBS_AVOIDANCE:
      self.autoObsAvoidance()
      pass
    elif self.autoState == AUTO.OBS_BACKTRACK:
      self.autoObsBacktrack()
      pass
    elif self.autoState == AUTO.OBS_RE_AVOIDANCE:
      self.autoObsReAvoidance()
      pass
    elif self.autoState == AUTO.SLOPE:
      self.autoSlope()
      pass
    elif self.autoState == AUTO.DEST_SEARCH:
      self.autoDestSearch()
      pass
    elif self.autoState == AUTO.DEST_REACHED:
      self.autoDestReached()
      pass
    elif self.autoState == AUTO.ERROR:
      self.autoError()
      pass
  
  
  def autoDestInSight(self):
    if self.autoStateChange == True:
      self.servoAngle.data = 90
      self.servoPub.publish(self.servoAngle)
      self.obsAvoidStart = False
    self.contSweep()
    #print "tracking in state", self.navState 
    leftLimit, rightLimit = self.calcZone();      
    cx = self.dest.destX + self.dest.destWidth/2
    cy = self.dest.destY + self.dest.destHeight/2
    #If the destination is slipping towards the left, turn left
    if cx < leftLimit:
      self.vel.linVelPcent = self.rotLinVel
      self.vel.angVelPcent = self.rotAngVel
    #If the destination is slipping towards the right, turn right
    elif cx > rightLimit:
      self.vel.linVelPcent = self.rotLinVel
      self.vel.angVelPcent = -self.rotAngVel
    #We are good to zip towards the destination
    else:
      self.vel.linVelPcent = self.stLinVel
      self.vel.angVelPcent = 0.0
    self.setCmdVel()

  def autoObsAvoidance(self):
    if self.autoStateChange == True:
      self.servoAngle.data = 90
      self.servoPub.publish(self.servoAngle)
      self.swept = False
      self.sweepCount = 0
      self.sweepState = 0
      self.obsAvoidCount = 0
    if self.obsAvoidCount*self.TIME_STEP >= 40000:
      raise AutoNavError(ERR.ERR_TIMEOUT, "Obstacle Avoidance timeout")
    if self.swept == False:
      self.sweep()
      self.rightObsCount = 0
      self.leftObsCount = 0
      midP = (self.totalSweepStates-1)/2
      if self.sweepState == self.totalSweepStates :
        self.swept = True
        for i in range(0,midP):
          if self.sweepDist[i] < self.OBS_AVOID_DIST:
            self.rightObsCount += 1
          if self.sweepDist[i+midP+1] < self.OBS_AVOID_DIST:
            self.leftObsCount += 1
        if self.leftObsCount > self.rightObsCount:
          self.avoidDirection = self.RIGHT
          leftLimit = 40
          rightLimit = 120
        elif self.leftObsCount == self.rightObsCount:
          if self.dest.destPresent == True:
            cx = self.dest.destX + self.dest.destWidth/2
            if cx < self.imgWidth/2:
              self.avoidDirection = self.LEFT
            else:
              self.avoidDirection = self.RIGHT
          else:
            self.avoidDirection = self.RIGHT
        else:
          self.avoidDirection = self.LEFT
        if self.avoidDirection == self.LEFT:
          self.leftLimit = self.imgWidth - 160
          self.rightLimit = self.imgWidth - 80
        elif self.avoidDirection == self.RIGHT:
          self.leftLimit = 40
          self.rightLimit = 120                    
    else:
      self.obsAvoidCount += 1
      leftLimit = self.leftLimit
      rightLimit = self.rightLimit
      #print "tracking in state", self.navState 
      cx = self.dest.destX + self.dest.destWidth/2
      cy = self.dest.destY + self.dest.destHeight/2
      #If the destination is slipping towards the left, turn left
      if cx < leftLimit:
        self.vel.linVelPcent = self.obsRotLinVel
        self.vel.angVelPcent = self.obsRotAngVel
      #If the destination is slipping towards the right, turn right
      elif cx > rightLimit:
        self.vel.linVelPcent = self.obsRotLinVel
        self.vel.angVelPcent = -self.obsRotAngVel
      #We are good to zip towards the destination
      else:
        self.vel.linVelPcent = self.obsStLinVel
        self.vel.angVelPcent = 0.0
      self.setCmdVel()    
  
  def autoObsAvoidance2(self):
    if self.autoStateChange == True:
      self.obsAvoidCount = 0
      self.avoidState = 0
      self.sweepState = 0
    self.obsAvoidCount += 1
    #Simply moving forward
    if self.obsAvoidCount < 4:
      print "Avoidance initializing"
      self.vel.linVelPcent = 0.25
      self.vel.angVelPcent = 0
      if self.obsAvoidCount == 3:
        self.startPose = copy.deepcopy(self.robotPose)
        self.desPose = copy.deepcopy(self.robotPose)
        self.desPose[2] -= numpy.pi/2
        self.avoidState = 1
    #Rotate by 90 degrees
    elif self.avoidState == 1 and self.robotPose[2]-self.desPose[2] > 0.1:
      self.moveBase()
    #Turn the servo
    elif self.avoidState == 1:
      self.avoidState = 2
      self.servoAngle.data = 180
      self.vel.linVelPcent = 0
      self.vel.angVelPcent = 0
      self.servoPub.publish(self.servoAngle)  
      self.wait = self.obsAvoidCount + (8000/self.TIME_STEP)
    #wait to finish turn
    elif self.avoidState == 2:
      if self.wait < self.obsAvoidCount:
        self.avoidState = 3
    #Move until no obstacle seen
    elif self.avoidState == 3:
      if self.OBS_AVOID > 0:
        self.vel.linVelPcent = 0.25
        self.vel.angVelPcent = 0.0
      else:
        self.vel.linVelPcent = 0
        self.vel.angVelPcent = 0
        self.avoidState = 4
        self.desPose = copy.deepcopy(self.robotPose)
        self.desPose[2] += numpy.pi/2
    #turn to face the destination        
    elif self.avoidState == 4 and self.robotPose[2]-self.desPose[2] > 1:      
      self.moveBase()     
    #Dummy state
    elif self.avoidState == 4:
      self.avoidState = 5
    #Move forward towards destination
    elif self.avoidState == 5:
      self.vel.linVelPcent = 0.25
      self.vel.angVelPcent = 0
      if self.OBS_AVOID > 0:
        self.avoidState = 6
    #Keep moving forward
    elif self.avoidState == 6:
      self.vel.linVelPcent = 0.25
      self.vel.angVelPcent = 0
      if self.OBS_AVOID == 0:
        self.avoidState = 7
    #Turn the servo back to straight position      
    elif self.avoidState == 7:
      self.vel.linVelPcent = 0.0
      self.vel.angVelPcent = 0.0
      self.servoAngle.data = 90
      self.servoPub.publish(self.servoAngle)
      self.wait = self.obsAvoidCount + (8000/self.TIME_STEP)
      self.avoidState = 8
    #wait for servo to finish
    elif self.avoidState == 8:
      if self.wait < self.obsAvoidCount:
        self.avoidState = 9
        self.wait = self.obsAvoidCount + (20000/self.TIME_STEP)
        self.vel.linVelPcent = 0
        self.vel.angVelPcent = 0.40
        self.setCmdVel()                 
    elif self.avoidState == 9:
      if self.wait < self.obsAvoidCount:
        self.vel.linVelPcent = 0
        self.vel.angVelPcent = 0
        self.avoidState = 10
    else:
      rospy.loginfo("Obstacle avoidance algo complete")
    self.setCmdVel()      
 
  def sweep(self):
    if self.sweepState <= self.totalSweepStates:
      if self.sweepSet == False:
        self.servoAngle.data = (self.sweepState-((self.totalSweepStates-1)/2))*5 + 90              
        self.servoPub.publish(self.servoAngle)
        self.sweepSet = True
        self.sweepCount = 0
      elif self.sweepCount*self.TIME_STEP > 500:        
        self.sweepSet = False
        if self.sweepState < self.totalSweepStates:
          self.sweepDist[self.sweepState] = self.dist[1]
        self.sweepState += 1
        if self.sweepState == self.totalSweepStates:
          #Terminal case
          self.servoAngle.data = 90
          self.servoPub.publish(self.servoAngle)         
    self.sweepCount +=  1

      
  def autoObsBacktrack(self):
    print "Backtracking"
    if self.autoStateChange == True:
      self.backTrackCount = 0
    #Stop trying after 25 s
    if self.backTrackCount*self.TIME_STEP >= 25000:
      raise AutoNavError(ERR.ERR_TIMEOUT, "Backtracking timeout")
    self.vel.linVelPcent = 0.0 #-self.obsStLinVel TODO Changed temp for stopping
    if self.OBS_AVOID & self.OBS_R == self.OBS_R:
      mult = -1
    else:  
      mult = 1
    if self.backTrackCount % 120 < 40:
      self.vel.angVelPcent = 0.35*mult
    elif self.backTrackCount %120 < 80:
      self.vel.angVelPcent = -0.35*mult
    else:
      self.vel.angVelPcent = 0.0
    self.backTrackCount += 1 #Time increment every TIME_STEP    
    self.setCmdVel() 
  
  
  def autoObsReAvoidance(self):
    if self.autoStateChange == True:
      self.reavoidanceTime = 0
    #Stop trying after 25 s
    if self.reavoidanceTime*self.TIME_STEP >= 25000:
      raise AutoNavError(ERR.ERR_TIMEOUT, "Re-avoidance timeout")
    self.reavoidanceTime += 1 #Time increment every TIME_STEP        
    #calczone defines where we maintain our destination
    leftLimit, rightLimit = self.calcZone();      
    cx = self.dest.destX + self.dest.destWidth/2
    cy = self.dest.destY + self.dest.destHeight/2
    #If the destination is slipping towards the left, turn left
    if cx < leftLimit:
      self.vel.linVelPcent = self.obsRotLinVel
      self.vel.angVelPcent = self.obsRotAngVel
    #If the destination is slipping towards the right, turn right
    elif cx > rightLimit:
      self.vel.linVelPcent = self.obsRotLinVel
      self.vel.angVelPcent = -self.obsRotAngVel
    #We are good to zip towards the destination
    else:
      self.vel.linVelPcent = self.obsStLinVel
      self.vel.angVelPcent = 0.0
    self.setCmdVel()
  
  
  def autoSlope(self):
    if self.autoStateChange == True:
      self.slopeTime = 0
      self.blindTime = 0
    if self.slopeTime*self.TIME_STEP >= 15000:
      #raise AutoNavError(ERR.ERR_TIMEOUT, "slope timeout")
      self.autoState = AUTO.DEST_IN_SIGHT
    self.slopeTime += 1
    if self.dist[1] > 0.2:      
      if self.dest.destPresent == True:	
        #calczone defines where we maintain our destination
        leftLimit = self.imgWidth/2 - 50
	rightLimit = self.imgWidth/2 + 50      
        cx = self.dest.destX + self.dest.destWidth/2
        cy = self.dest.destY + self.dest.destHeight/2
	print leftLimit, cx, rightLimit
        #If the destination is slipping towards the left, turn left
        if cx < leftLimit:
          self.vel.linVelPcent = self.obsRotLinVel
          self.vel.angVelPcent = self.obsRotAngVel
        #If the destination is slipping towards the right, turn right
        elif cx > rightLimit:
          self.vel.linVelPcent = self.obsRotLinVel
          self.vel.angVelPcent = -self.obsRotAngVel
        #We are good to zip towards the destination
        else:
          self.vel.linVelPcent = self.obsStLinVel
          self.vel.angVelPcent = 0.0
      else:        
        print "No destination, going blind"
	if self.blindTime*self.TIME_STEP < 10000:
          self.vel.linVelPcent = self.obsStLinVel
          self.vel.angVelPcent = 0.0          
        elif self.blindTime*self.TIME_STEP < 12000:
          if self.blindTime*self.TIME_STEP == 10000:
            self.desPose[2] = self.odom.pose.pose.orientation.z
          self.vel.linVelPcent = 0
          self.vel.linVelPcent = self.obsRotAngVel
        elif self.blindTime*self.TIME_STEP < 14000:
          self.vel.linVelPcent = 0
          self.vel.linVelPcent = -self.obsRotAngVel
        elif self.blindTime < 18000:
          self.moveBase()
        else:
          self.blindTime = 0
        self.blindTime += 1
    else:
      #Obstacle in slope case
      self.vel.linVelPcent = 0.0
      self.vel.angVelPcent = 0.0
    self.setCmdVel()  
    if self.slope == self.GROUNDLEVEL:
      self.autoState = AUTO.DEST_IN_SIGHT    
    pass
  
  
  def autoDestSearch(self):
    if self.autoStateChange == True:
      self.destSearchTime = 0
      self.autoDestSearchAction = 0
    #Stop trying after 25 s
    if self.destSearchTime*self.TIME_STEP >= 25000:
      raise AutoNavError(ERR.ERR_TIMEOUT, "Destination search timeout")
    self.destSearchTime += 1 #Time increment every TIME_STEP
    #Change actions every 1 second
    if self.destSearchTime*self.TIME_STEP % 3000 == 0:
      self.autoDestSearchAction += 1    
    #Arbitrarily choose to go forward
    if self.autoDestSearchAction % 3 == 0:
      self.vel.linVelPcent = 0.0
      self.vel.angVelPcent = 0
    #after some time choose to turn LEFT
    elif self.autoDestSearchAction % 3 == 1:
      self.vel.linVelPcent = 0.0
      if self.avoidDirection == self.RIGHT:
        self.vel.angVelPcent = 0.45
      else:
        self.vel.angVelPcent = -0.45
    elif self.autoDestSearchAction % 3 == 2:
      self.vel.linVelPcent = 0.0
      self.vel.angVelPcent = 0.0
    self.setCmdVel()      
  
  
  def autoDestReached(self):
    rospy.logwarn("Yay! Destination reached")
    self.robotMove(self.STOP)
    rospy.logwarn("Please implement destination reached fucntion boss")
    
    
  def autoError(self):    
    if self.autoErrId <= ERR.ERR_DYNAMIC_OBSTACLE:
      if self.errorCount < 5:
        #Severity is less. Some allakai error. Try to get back to prev state
        self.applyAutoState = self.prevAutoState
      else:
        raise AutoNavError(self.autoErrId, self.errPrintNames[self.autoErrId])
    else:
        #Fatal error. Exit
        raise AutoNavError(self.autoErrId, self.errPrintNames[self.autoErrId])      
        
                
  def autoModeInit(self):
    self.autoState = AUTO.IDLE
    self.autoCount = 0
    self.robotResp = AUTO_OUTPUT.STILL_RUNNING
    self.errorCount = 0
    self.applyAutoState = None
    self.prevOBS_AVOID = self.OBS_AVOID
    self.obsAvoidStart == False
    
    
  def autoCalcDestArea(self):
    if self.dest.destPresent == True:
      return (self.dest.destHeight*self.dest.destWidth)*1.0/(self.imgHeight*self.imgWidth)
    
    
  def autoStateChanger(self):
    stateChange = False
    oa = False
    os = False
    dl = False
    if self.OBS_AVOID > 0:
      if self.destArea < 0.03:
        oa = True
    if self.OBS_STOP > 0:
      os = True
    if self.dest.destPresent == True:
      dl = True    
      self.destArea = self.autoCalcDestArea()
    try:
      self.prevAutoState = self.autoState
      if self.dest.destPresent == True and self.destArea > self.AREA_THRESHOLD:
        self.autoState = AUTO.DEST_REACHED
      else:
        #Someone requested to apply this state
        if self.applyAutoState != None:
          self.autoState = self.applyAutoState
        else:
          if self.obsAvoidStart == False:
            if self.slope == self.GROUNDLEVEL:
              if self.autoState == AUTO.SLOPE:
                self.autoState = AUTO.DEST_IN_SIGHT
              t = self.autoStateTrans[(self.autoState, os, oa, dl)]
              self.autoState = t[0]
              self.autoErrId = t[1]
            else:
              self.autoState = AUTO.SLOPE        #Slope state          
          if self.autoState == AUTO.OBS_AVOIDANCE and self.obsAvoidStart == False:
            self.vel.linVelPcent = 0
            self.vel.angVelPcent = 0
            self.setCmdVel()
            self.desPose = copy.deepcopy(self.robotPose)
            self.desPose[0] = self.desPose[0] + self.dist[1]/100 + 0.1
            self.obsAvoidStart = True
          if self.obsAvoidStart == True and self.robotPose[0] > self.desPose[0]:
            self.obsAvoidStart = False            
      if self.prevAutoState != self.autoState:
        stateChange = True
    except KeyError:
      rospy.logfatal("Auto mode key error. Should not have happened")      
    return stateChange

  def assessPclData(self):
    #Some obstacle has been detected for the first time
    #print self.pclObsCountLeft, self.pclObsCountRight
    self.pclObsCountLeft += 1
    self.pclObsCountRight += 1    
    if self.pclObsCountLeft*self.TIME_STEP > 5000:          
      self.OBS_AVOID = self.OBS_AVOID & (~self.OBS_L)
    if self.pclObsCountRight*self.TIME_STEP > 5000:
      self.OBS_AVOID = self.OBS_AVOID & (~self.OBS_R)
  
  def moveBase(self):
    print "moving base"
    if abs(self.desPose[2] - self.robotPose[2]) > 0.1:      
      self.vel.linVelPcent = 0.0
      if self.desPose[2] - self.robotPose[2] > 0:
        self.vel.angVelPcent = self.obsRotAngVel
      else:
        self.vel.angVelPcent = -self.obsRotAngVel              
    else:
      self.vel.linVelPcent = 0.0
      self.vel.angVelPcent = 0.0
        
  
  def search_destination(self):    
    print "Searching for Destination"
    if self.dist[1] > 0.8:
      #Arbitrarily choose to go forward
      if self.frameCount <= 40:
        self.vel.linVelPcent = 0.0
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
      self.setCmdVel()      
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
    #TODO Not sure if I changed this correctly. A distance based strategy might be better
    elif self.OBS_AVOID == (self.OBS_L | self.OBS_R):
      leftLimit = (self.imgWidth/2 - 50) - 250
      rightLimit = (self.imgWidth/2 + 50) - 250
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
      #self.setCmdVel()
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
    self.setCmdVel()

  #===============================================================
  #
  #    MISCELLANEOUS HELPER FUNCTIONS
  #
  #===============================================================

  def printNavStatus(self):
    s = ""
    #print data.ultra_left, data.ultra_centre, data.ultra_right,
    s += "Nav: " + self.navStatePrintNames[self.navState] + " Auto: " + self.autoStatePrintNames[self.autoState]
    s += " Dest: " + str(self.dest.destPresent)
    #s += " A: " + str(self.avoidState) 
    s += " Obs : " + str([self.OBS_AVOID & self.OBS_L, self.OBS_AVOID & self.OBS_C, self.OBS_AVOID & self.OBS_R])
    #s += " Obs Stop: " + str([self.OBS_STOP & self.OBS_L, self.OBS_STOP & self.OBS_C, self.OBS_STOP & self.OBS_R])
    s += " DesX %0.3f " %self.desPose[0] + " CurX %0.3f " %self.robotPose[0]
    try:
      s += " cx " + str(self.destcx) + " cy " + str(self.destcy)
    except:
      s += " cx " + "NaN" + " cy " + "NaN"
    try:
      s += " Des area %0.4f" %self.destArea
    except:
      s += " Des area -NaN"
    if DEBUG == True:
      s += " Ultra "
      #s += "%0.2f " %self.dist[0]
      s += "%0.2f " %self.dist[1]
      #s += "%0.2f " %self.dist[2]
      s += " Vel: " + str(self.vel.linVelPcent) + " " + str(self.vel.angVelPcent)
      s += " Slope: " + str(self.slope)
      #print s
      rospy.loginfo(s)
      self.navDebug.autoState = self.autoStatePrintNames[self.autoState]
      self.navDebug.navState = self.navStatePrintNames[self.navState]
      self.navDebug.obsAvoid = [self.OBS_AVOID & self.OBS_L, (self.OBS_AVOID & self.OBS_C)/self.OBS_C, (self.OBS_AVOID & self.OBS_R)/self.OBS_R]
      self.navDebug.obsStop = [self.OBS_STOP & self.OBS_L, (self.OBS_STOP & self.OBS_C)/self.OBS_C, (self.OBS_STOP & self.OBS_R)/self.OBS_R]
      self.navDebug.dist = self.dist
      self.navDebug.robotReturn = self.autoOutputPrintNames[self.robotResp]
      self.navDebug.uiState = self.uiPrintNames[self.uiState]
      self.navDebug.destRect = self.dest
      self.navDebug.header.stamp = rospy.get_rostime()
      try:
        self.navDebug.destArea = self.destArea
      except AttributeError:      
        self.navDebug.destArea = -1
      self.navDebugPub.publish(self.navDebug) 
      
     


  def processUltrasound(self, data): 
    #Values based on calibration. Calculated in inches and then converted to m
    c = data.ultra_centre
    l = data.ultra_left
    r = data.ultra_right
    self.val_c = numpy.append(self.val_c, c)
    self.val_r = numpy.append(self.val_r, r)
    self.val_l = numpy.append(self.val_l, l)   
    self.val_c = numpy.delete(self.val_c, 1)  
    self.val_r = numpy.delete(self.val_r, 1)  
    self.val_l = numpy.delete(self.val_l, 1)          
    v_c = self.smooth(self.val_c, self.UltraFilterSize)
    v_r = self.smooth(self.val_r, self.UltraFilterSize)
    v_l = self.smooth(self.val_l, self.UltraFilterSize)            
    return [numpy.median(v_l), numpy.median(v_c), numpy.median(v_r)]
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
      (AUTO.IDLE,                0,0,0) : (AUTO.IDLE                , ERR.NONE                ),
      (AUTO.IDLE,                0,0,1) : (AUTO.DEST_IN_SIGHT       , ERR.NONE                ),
      (AUTO.IDLE,                0,1,0) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.IDLE,                0,1,1) : (AUTO.OBS_AVOIDANCE       , ERR.NONE                ),
      (AUTO.IDLE,                1,0,0) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.IDLE,                1,0,1) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.IDLE,                1,1,0) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.IDLE,                1,1,1) : (AUTO.ERROR               , ERR.ERR_STUPID          ),
      (AUTO.DEST_IN_SIGHT,       0,0,0) : (AUTO.DEST_SEARCH         , ERR.NONE                ),
      (AUTO.DEST_IN_SIGHT,       0,0,1) : (AUTO.DEST_IN_SIGHT       , ERR.NONE                ),
      (AUTO.DEST_IN_SIGHT,       0,1,0) : (AUTO.OBS_AVOIDANCE       , ERR.NONE                ),
      (AUTO.DEST_IN_SIGHT,       0,1,1) : (AUTO.OBS_AVOIDANCE       , ERR.NONE                ),
      (AUTO.DEST_IN_SIGHT,       1,0,0) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.DEST_IN_SIGHT,       1,0,1) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.DEST_IN_SIGHT,       1,1,0) : (AUTO.OBS_AVOIDANCE       , ERR.ERR_DYNAMIC_OBSTACLE),
      (AUTO.DEST_IN_SIGHT,       1,1,1) : (AUTO.OBS_AVOIDANCE       , ERR.ERR_DYNAMIC_OBSTACLE),
      (AUTO.OBS_AVOIDANCE,       0,0,0) : (AUTO.DEST_SEARCH         , ERR.NONE                ),
      (AUTO.OBS_AVOIDANCE,       0,0,1) : (AUTO.DEST_IN_SIGHT       , ERR.NONE                ),
      (AUTO.OBS_AVOIDANCE,       0,1,0) : (AUTO.OBS_AVOIDANCE       , ERR.NONE                ),
      (AUTO.OBS_AVOIDANCE,       0,1,1) : (AUTO.OBS_AVOIDANCE       , ERR.NONE                ),
      (AUTO.OBS_AVOIDANCE,       1,0,0) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.OBS_AVOIDANCE,       1,0,1) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.OBS_AVOIDANCE,       1,1,0) : (AUTO.OBS_AVOIDANCE       , ERR.NONE                ),
      (AUTO.OBS_AVOIDANCE,       1,1,1) : (AUTO.OBS_AVOIDANCE       , ERR.NONE                ),
      (AUTO.DEST_SEARCH,         0,0,0) : (AUTO.DEST_SEARCH         , ERR.NONE                ),
      (AUTO.DEST_SEARCH,         0,0,1) : (AUTO.DEST_IN_SIGHT       , ERR.NONE                ),
      (AUTO.DEST_SEARCH,         0,1,0) : (AUTO.DEST_SEARCH         , ERR.NONE                ),
      (AUTO.DEST_SEARCH,         0,1,1) : (AUTO.OBS_AVOIDANCE       , ERR.NONE                ),
      (AUTO.DEST_SEARCH,         1,0,0) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.DEST_SEARCH,         1,0,1) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.DEST_SEARCH,         1,1,0) : (AUTO.ERROR               , ERR.ERR_UNABLE_TO_SEARCH),
      (AUTO.DEST_SEARCH,         1,1,1) : (AUTO.ERROR               , ERR.ERR_UNABLE_TO_SEARCH),
      (AUTO.ERROR,               0,0,0) : (AUTO.ERROR               , ERR.PREVIOUS            ),
      (AUTO.ERROR,               0,0,1) : (AUTO.ERROR               , ERR.PREVIOUS            ),
      (AUTO.ERROR,               0,1,0) : (AUTO.ERROR               , ERR.PREVIOUS            ),
      (AUTO.ERROR,               0,1,1) : (AUTO.ERROR               , ERR.PREVIOUS            ),
      (AUTO.ERROR,               1,0,0) : (AUTO.ERROR               , ERR.PREVIOUS            ),
      (AUTO.ERROR,               1,0,1) : (AUTO.ERROR               , ERR.PREVIOUS            ),
      (AUTO.ERROR,               1,1,0) : (AUTO.ERROR               , ERR.PREVIOUS            ),
      (AUTO.ERROR,               1,1,1) : (AUTO.ERROR               , ERR.PREVIOUS            )}    


  def defineAutoStateTransitionMatrix1(self):
    self.autoStateTrans = {
      #(Current State, Obs Stopping, Obs avoid, Dest Present) : [Next State, Comments]
      (AUTO.IDLE,                0,0,0) : (AUTO.IDLE                , ERR.NONE                ),
      (AUTO.IDLE,                0,0,1) : (AUTO.DEST_IN_SIGHT       , ERR.NONE                ),
      (AUTO.IDLE,                0,1,0) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.IDLE,                0,1,1) : (AUTO.OBS_AVOIDANCE       , ERR.NONE                ),
      (AUTO.IDLE,                1,0,0) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.IDLE,                1,0,1) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.IDLE,                1,1,0) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.IDLE,                1,1,1) : (AUTO.ERROR               , ERR.ERR_STUPID          ),
      (AUTO.DEST_IN_SIGHT,       0,0,0) : (AUTO.DEST_SEARCH         , ERR.NONE                ),
      (AUTO.DEST_IN_SIGHT,       0,0,1) : (AUTO.DEST_IN_SIGHT       , ERR.NONE                ),
      (AUTO.DEST_IN_SIGHT,       0,1,0) : (AUTO.OBS_AVOIDANCE       , ERR.NONE                ),
      (AUTO.DEST_IN_SIGHT,       0,1,1) : (AUTO.OBS_AVOIDANCE       , ERR.NONE                ),
      (AUTO.DEST_IN_SIGHT,       1,0,0) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.DEST_IN_SIGHT,       1,0,1) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.DEST_IN_SIGHT,       1,1,0) : (AUTO.ERROR               , ERR.ERR_DYNAMIC_OBSTACLE),
      (AUTO.DEST_IN_SIGHT,       1,1,1) : (AUTO.ERROR               , ERR.ERR_DYNAMIC_OBSTACLE),
      (AUTO.OBS_AVOIDANCE,       0,0,0) : (AUTO.OBS_AVOIDANCE       , ERR.NONE                ),
      (AUTO.OBS_AVOIDANCE,       0,0,1) : (AUTO.DEST_IN_SIGHT       , ERR.NONE                ),
      (AUTO.OBS_AVOIDANCE,       0,1,0) : (AUTO.OBS_AVOIDANCE       , ERR.NONE                ),
      (AUTO.OBS_AVOIDANCE,       0,1,1) : (AUTO.OBS_AVOIDANCE       , ERR.NONE                ),
      (AUTO.OBS_AVOIDANCE,       1,0,0) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.OBS_AVOIDANCE,       1,0,1) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.OBS_AVOIDANCE,       1,1,0) : (AUTO.OBS_AVOIDANCE       , ERR.NONE                ),
      (AUTO.OBS_AVOIDANCE,       1,1,1) : (AUTO.OBS_AVOIDANCE       , ERR.NONE                ),
      (AUTO.OBS_BACKTRACK,       0,0,0) : (AUTO.DEST_SEARCH         , ERR.NONE                ),
      (AUTO.OBS_BACKTRACK,       0,0,1) : (AUTO.DEST_IN_SIGHT       , ERR.NONE                ),
      (AUTO.OBS_BACKTRACK,       0,1,0) : (AUTO.OBS_BACKTRACK       , ERR.NONE                ),
      (AUTO.OBS_BACKTRACK,       0,1,1) : (AUTO.OBS_AVOIDANCE       , ERR.NONE                ),
      (AUTO.OBS_BACKTRACK,       1,0,0) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.OBS_BACKTRACK,       1,0,1) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.OBS_BACKTRACK,       1,1,0) : (AUTO.OBS_BACKTRACK       , ERR.NONE                ),
      (AUTO.OBS_BACKTRACK,       1,1,1) : (AUTO.OBS_BACKTRACK       , ERR.NONE                ),
      (AUTO.OBS_RE_AVOIDANCE,    0,0,0) : (AUTO.DEST_SEARCH         , ERR.NONE                ),
      (AUTO.OBS_RE_AVOIDANCE,    0,0,1) : (AUTO.OBS_RE_AVOIDANCE    , ERR.NONE                ),
      (AUTO.OBS_RE_AVOIDANCE,    0,1,0) : (AUTO.OBS_BACKTRACK       , ERR.NONE                ),
      (AUTO.OBS_RE_AVOIDANCE,    0,1,1) : (AUTO.OBS_RE_AVOIDANCE    , ERR.NONE                ),
      (AUTO.OBS_RE_AVOIDANCE,    1,0,0) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.OBS_RE_AVOIDANCE,    1,0,1) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.OBS_RE_AVOIDANCE,    1,1,0) : (AUTO.ERROR               , ERR.ERR_UNABLE_TO_AVOID ),
      (AUTO.OBS_RE_AVOIDANCE,    1,1,1) : (AUTO.SLOPE  , ERR.NONE                ),
      (AUTO.SLOPE,  0,0,0) : (AUTO.SLOPE  , ERR.NONE                ),
      (AUTO.SLOPE,  0,0,1) : (AUTO.DEST_IN_SIGHT       , ERR.NONE                ),
      (AUTO.SLOPE,  0,1,0) : (AUTO.SLOPE  , ERR.NONE                ),
      (AUTO.SLOPE,  0,1,1) : (AUTO.OBS_AVOIDANCE       , ERR.NONE                ),
      (AUTO.SLOPE,  1,0,0) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.SLOPE,  1,0,1) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.SLOPE,  1,1,0) : (AUTO.SLOPE  , ERR.NONE                ),
      (AUTO.SLOPE,  1,1,1) : (AUTO.SLOPE  , ERR.NONE                ),
      (AUTO.DEST_SEARCH,         0,0,0) : (AUTO.DEST_SEARCH         , ERR.NONE                ),
      (AUTO.DEST_SEARCH,         0,0,1) : (AUTO.DEST_IN_SIGHT       , ERR.NONE                ),
      (AUTO.DEST_SEARCH,         0,1,0) : (AUTO.DEST_SEARCH         , ERR.NONE                ),
      (AUTO.DEST_SEARCH,         0,1,1) : (AUTO.OBS_AVOIDANCE       , ERR.NONE                ),
      (AUTO.DEST_SEARCH,         1,0,0) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.DEST_SEARCH,         1,0,1) : (AUTO.ERROR               , ERR.ERR_ILLEGAL         ),
      (AUTO.DEST_SEARCH,         1,1,0) : (AUTO.ERROR               , ERR.ERR_UNABLE_TO_SEARCH),
      (AUTO.DEST_SEARCH,         1,1,1) : (AUTO.ERROR               , ERR.ERR_UNABLE_TO_SEARCH),
      (AUTO.ERROR,               0,0,0) : (AUTO.ERROR               , ERR.PREVIOUS            ),
      (AUTO.ERROR,               0,0,1) : (AUTO.ERROR               , ERR.PREVIOUS            ),
      (AUTO.ERROR,               0,1,0) : (AUTO.ERROR               , ERR.PREVIOUS            ),
      (AUTO.ERROR,               0,1,1) : (AUTO.ERROR               , ERR.PREVIOUS            ),
      (AUTO.ERROR,               1,0,0) : (AUTO.ERROR               , ERR.PREVIOUS            ),
      (AUTO.ERROR,               1,0,1) : (AUTO.ERROR               , ERR.PREVIOUS            ),
      (AUTO.ERROR,               1,1,0) : (AUTO.ERROR               , ERR.PREVIOUS            ),
      (AUTO.ERROR,               1,1,1) : (AUTO.ERROR               , ERR.PREVIOUS            )
    }  

  def defineNavStateTransitionMatrix(self):
    self.navStateTrans = {
      #Refer to file eod_nav/NavState.ods
      (NAV.IDLE,           1,0) : (NAV.MANUAL_MODE    , ERR.NONE                     ),
      (NAV.IDLE,           2,0) : (NAV.TRACKING_ONLY  , ERR.NONE                     ),
      (NAV.IDLE,           3,0) : (NAV.ERROR          , ERR.ERR_UNKNOWN_DESTINATION  ),
      (NAV.IDLE,           1,1) : (NAV.MANUAL_MODE    , ERR.NONE                     ),
      (NAV.IDLE,           2,1) : (NAV.TRACKING_ONLY  , ERR.NONE                     ),
      (NAV.IDLE,           3,1) : (NAV.ERROR          , ERR.ERR_UNKNOWN_DESTINATION  ),
      (NAV.IDLE,           1,2) : (NAV.MANUAL_MODE    , ERR.NONE                     ),
      (NAV.IDLE,           2,2) : (NAV.TRACKING_ONLY  , ERR.NONE                     ),
      (NAV.IDLE,           3,2) : (NAV.ERROR          , ERR.ERR_UNKNOWN_DESTINATION  ),
      (NAV.TRACKING_ONLY,  1,0) : (NAV.MANUAL_MODE    , ERR.NONE                     ),
      (NAV.TRACKING_ONLY,  2,0) : (NAV.TRACKING_ONLY  , ERR.NONE                     ),
      (NAV.TRACKING_ONLY,  3,0) : (NAV.AUTO_MODE      , ERR.NONE                     ),
      (NAV.TRACKING_ONLY,  1,1) : (NAV.MANUAL_MODE    , ERR.NONE                     ),
      (NAV.TRACKING_ONLY,  2,1) : (NAV.TRACKING_ONLY  , ERR.NONE                     ),
      (NAV.TRACKING_ONLY,  3,1) : (NAV.AUTO_MODE      , ERR.NONE                     ),
      (NAV.TRACKING_ONLY,  1,2) : (NAV.MANUAL_MODE    , ERR.NONE                     ),
      (NAV.TRACKING_ONLY,  2,2) : (NAV.TRACKING_ONLY  , ERR.NONE                     ),
      (NAV.TRACKING_ONLY,  3,2) : (NAV.AUTO_MODE      , ERR.NONE                     ),
      (NAV.AUTO_MODE,      1,0) : (NAV.ERROR          , ERR.ERR_ROBOT_LOST           ),
      (NAV.AUTO_MODE,      2,0) : (NAV.ERROR          , ERR.ERR_ROBOT_LOST           ),
      (NAV.AUTO_MODE,      3,0) : (NAV.ERROR          , ERR.ERR_ROBOT_LOST           ),
      (NAV.AUTO_MODE,      1,1) : (NAV.IDLE           , ERR.NONE                     ),
      (NAV.AUTO_MODE,      2,1) : (NAV.IDLE           , ERR.NONE                     ),
      (NAV.AUTO_MODE,      3,1) : (NAV.IDLE           , ERR.NONE                     ),
      (NAV.AUTO_MODE,      1,2) : (NAV.MANUAL_MODE    , ERR.NONE                     ),
      (NAV.AUTO_MODE,      2,2) : (NAV.AUTO_MODE      , ERR.NONE                     ),
      (NAV.AUTO_MODE,      3,2) : (NAV.AUTO_MODE      , ERR.NONE                     ),
      (NAV.MANUAL_MODE,    1,0) : (NAV.MANUAL_MODE    , ERR.NONE                     ),
      (NAV.MANUAL_MODE,    2,0) : (NAV.TRACKING_ONLY  , ERR.NONE                     ),
      (NAV.MANUAL_MODE,    3,0) : (NAV.ERROR          , ERR.ERR_UNKNOWN_DESTINATION  ),
      (NAV.MANUAL_MODE,    1,1) : (NAV.MANUAL_MODE    , ERR.NONE                     ),
      (NAV.MANUAL_MODE,    2,1) : (NAV.TRACKING_ONLY  , ERR.NONE                     ),
      (NAV.MANUAL_MODE,    3,1) : (NAV.ERROR          , ERR.ERR_UNKNOWN_DESTINATION  ),
      (NAV.MANUAL_MODE,    1,2) : (NAV.MANUAL_MODE    , ERR.NONE                     ),
      (NAV.MANUAL_MODE,    2,2) : (NAV.TRACKING_ONLY  , ERR.NONE                     ),
      (NAV.MANUAL_MODE,    3,2) : (NAV.ERROR          , ERR.ERR_UNKNOWN_DESTINATION  ),
      (NAV.ERROR,          1,0) : (NAV.MANUAL_MODE    , ERR.NONE                     ),
      (NAV.ERROR,          2,0) : (NAV.TRACKING_ONLY  , ERR.NONE                     ),
      (NAV.ERROR,          3,0) : (NAV.ERROR          , ERR.ERR_UNKNOWN_DESTINATION  ),
      (NAV.ERROR,          1,1) : (NAV.MANUAL_MODE    , ERR.NONE                     ),
      (NAV.ERROR,          2,1) : (NAV.TRACKING_ONLY  , ERR.NONE                     ),
      (NAV.ERROR,          3,1) : (NAV.ERROR          , ERR.ERR_UNKNOWN_DESTINATION  ),
      (NAV.ERROR,          1,2) : (NAV.MANUAL_MODE    , ERR.NONE                     ),
      (NAV.ERROR,          2,2) : (NAV.TRACKING_ONLY  , ERR.NONE                     ),
      (NAV.ERROR,          3,2) : (NAV.ERROR          , ERR.ERR_UNKNOWN_DESTINATION  )
    }

  def definePrintNames(self):
    self.autoStatePrintNames = [
      "IDLE"              ,   #Auto mode has not started
      "DEST_IN_SIGHT"     ,   #Normal navigation mode
      "OBS_AVOIDANCE"     ,   #We have sighted an obstacle at 3m distance and want to avoid it
      "OBS_BACKTRACK"     ,   #We have lost the destination when we tried to avoid, we are gonna backtrack
      "OBS_RE_AVOIDANCE"  ,   #we have backtracked and we will try to avoid it again now
      "SLOPE",   #The robot just close to avoiding an obstacle, and tries to cut corner now
      "DEST_SEARCH"       ,   #We have lost the destination, go into destination search mode
      "DEST_REACHED"      ,   #The destination has been reached
      "ERROR"]
    
    self.navStatePrintNames = [
      "IDLE"          ,
      "TRACKING_ONLY" ,
      "AUTO_MODE"     ,  
      "MANUAL_MODE"   ,
      "ERROR"   ]                                 
    self.errPrintNames = [
      "ERR_ILLEGAL"             ,    #Illegal Autonomous mode. Comes when ultrasonic is < 0.5 but also > 3.
      "ERR_STUPID"              ,    #Someone does crazy things
      "ERR_DYNAMIC_OBSTACLE"    ,    #Dynamic obstacles      
      "ERR_UNABLE_TO_AVOID"     ,    #Unable to avoid the obstacle after 1 retry
      "ERR_UNABLE_TO_SEARCH"    ,    #Unable to locate the destination after search
      "ERR_TIMEOUT"             ,    #Timeout error
      "ERR_UNKNOWN_DESTINATION" ,    #Asked robot to move autonomously without giving destination
      "ERR_ROBOT_LOST"          ,    #The robot is lost.
      "NONE"                    ,    #No errors. Default value of any error state
      "PREVIOUS" ]                   #Same error as previous 
    self.uiPrintNames = [
      "UI_READY"      ,
      "IDLE_OR_MANUAL",
      "TRACKING_ONLY",
      "AUTO_MODE"]
    self.autoOutputPrintNames = [
      "ROBOT_LOST",
      "REACHED_DESTINATION",
      "STILL_RUNNING"]                                

  
if __name__ == "__main__":  
  nav = eodNav()
  r = rospy.Rate(20) #Run every 50 ms
  while not rospy.is_shutdown():
    nav.navHandler()
    r.sleep()
  rospy.logwarn("Exiting EOD Navigation! Bye Bye")
