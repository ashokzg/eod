#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('eod_nav')
import numpy
from sensor_msgs.msg import CameraInfo, Range
from eod_nav.msg import Ultrasonic
from std_msgs.msg import UInt32

count = 0

class temp:
  val = numpy.zeros(15)  
  count = 0
  TIME_STEP = 50
  totalSweepStates = 25
  servoAngle = UInt32()
  servoPub = rospy.Publisher('servo_angle', UInt32)
  sweepDist = [601]*totalSweepStates
  sweepcall = False
  dist = [600]*3
  OBS_AVOID_DIST = 300
  def ultrasound(self, data):
    self.count += 1    
    self.val = numpy.append(self.val, data.ultra_centre)
    self.val = numpy.delete(self.val, 1)  
    #print val
    if self.count > 11:
      v = self.smooth(self.val)
      #print v
      #print v.mean()
      self.dist[1] = v.mean()

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
  
  def sweepcb(self, data):
    self.sweepState = 0
    self.sweepCount = 0
    self.sweepSet = False
    self.sweepcall = True
    
  def sweep(self):
    if self.sweepState <= self.totalSweepStates:
      if self.sweepSet == False:
        self.servoAngle.data = (self.sweepState-((self.totalSweepStates-1)/2))*5 + 90              
        self.servoPub.publish(self.servoAngle)
        self.sweepSet = True
        self.sweepCount = 0
      elif self.sweepCount*self.TIME_STEP > 700:        
        self.sweepSet = False
        if self.sweepState < self.totalSweepStates:
          print self.servoAngle.data, self.dist[1]
          self.sweepDist[self.sweepState] = self.dist[1]
        self.sweepState += 1
        if self.sweepState == self.totalSweepStates:
          #Terminal case
          self.servoAngle.data = 90
          self.servoPub.publish(self.servoAngle)         
    self.sweepCount +=  1
  
  def handler(self):
    if self.sweepcall == True:
      self.sweep()
      self.rightObsCount = 0
      self.leftObsCount = 0
      midP = (self.totalSweepStates-1)/2
      if self.sweepState == self.totalSweepStates :
        self.sweepcall = False
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
              pass
        else:
          self.avoidDirection = self.LEFT
          leftLimit = self.imgWidth - 120
          rightLimit = self.imgWidth - 40      
  
if __name__ == "__main__":  
  rospy.init_node("testultra")  
  a = temp()
  rospy.Subscriber("ultrasound", Ultrasonic, a.ultrasound)
  rospy.Subscriber("sweep", UInt32, a.sweepcb)
  r = rospy.Rate(20) #Run every 50 ms
  while not rospy.is_shutdown():
    a.handler()
    r.sleep()
