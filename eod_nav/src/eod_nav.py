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
  
  val = numpy.zeros(15)  
  count = 0
  
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
  
  def ultraSound(self, data):
    distance = self.processUltrasound(data)
    #print distance
    print self.navState
    if distance < 0.15:
      self.robotMove(self.STOP)
      self.navState = self.MANUAL_MODE
      self.prevState = self.AUTO_MODE
    elif distance > 0.5 and self.navState == self.MANUAL_MODE and self.prevState == self.AUTO_MODE:
      self.navState = self.AUTO_MODE
      self.prevState = self.MANUAL_MODE
    
  def userDest(self, data):
    if data.destPresent == True:
      self.navState = self.TRACKING_ONLY
      #TODO (set to automode only after receiving command from UI)
      self.navState = self.AUTO_MODE
    print "Destination received; Tracking"
    
  def uiState(self, data):
    if data.data == 0:
      self.navState = self.IDLE
    elif data.data == 1:
      self.navState = self.MANUAL_MODE
    elif data.data == 2:
      self.navState = self.TRACKING_ONLY
    elif data.data == 3:
      self.navState = self.AUTO_MODE
    else:
      self.navState = self.IDLE
  
  def trackedDest(self, data):
    #print "tracking in state", self.navState 
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
      self.vel.linVelPcent = 0.8
      self.vel.angVelPcent = 0.0
      rospy.logdebug("Straight")
      #print "Straight"
    elif dir == self.LEFT:
      self.vel.linVelPcent = 0.5
      self.vel.angVelPcent = 0.1
      rospy.logdebug("Left")
      #print "Left"
    elif dir == self.RIGHT:
      self.vel.linVelPcent = 0.5
      self.vel.angVelPcent = -0.1
      rospy.logdebug("Right")
      #print "Right"
    else:
      self.vel.linVelPcent = 0.0
      self.vel.angVelPcent = 0.0
      rospy.logdebug("Stop")
      #print "STOP"
    self.robotCmdPub.publish(self.vel)        

  def processUltrasound(self, data):
    self.count += 1    
    self.val = numpy.append(self.val, data.range)
    self.val = numpy.delete(self.val, 1)  
    v = self.smooth(self.val)
    return v.mean()
    


  def smooth(self, x, window_len=11,window='hanning'):
    """smooth the data using a window with requested size.

    This method is based on the convolution of a scaled window with the signal.
    The signal is prepared by introducing reflected copies of the signal 
    (with the window size) in both ends so that transient parts are minimized
    in the begining and end part of the output signal.

    input:
        x: the input signal 
        window_len: the dimension of the smoothing window; should be an odd integer
        window: the type of window from 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'
            flat window will produce a moving average smoothing.

    output:
        the smoothed signal
        
    example:

    t=linspace(-2,2,0.1)
    x=sin(t)+randn(len(t))*0.1
    y=smooth(x)

    see also: 

    numpy.hanning, numpy.hamming, numpy.bartlett, numpy.blackman, numpy.convolve
    scipy.signal.lfilter

    TODO: the window parameter could be the window itself if an array instead of a string
    NOTE: length(output) != length(input), to correct this: return y[(window_len/2-1):-(window_len/2)] instead of just y.
    """

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
