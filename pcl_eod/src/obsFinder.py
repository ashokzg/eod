#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('pcl_eod')

from pcl_eod.msg import Clusters

import numpy
import tf

class ClusterSub(Exception):
  br = tf.TransformBroadcaster()
  def __init__(self):
    rospy.init_node("cluster")
    rospy.logwarn("EOD Cluster Organizing")
    self.initSubAndPub()    
     
  
  def initSubAndPub(self):      
    #Init the subscribers    
    rospy.loginfo("In Sub and pub Func")
    rospy.Subscriber("clusters", Clusters, self.ClustersCb)
    #Init the publishers
    self.clusterPub = rospy.Publisher('obstacles', Clusters)
  
  def segment(selfs):
    pass
        
  def ClustersCb(self, data):
    mp = data.minpoint
    xp = data.maxpoint
    for i in range(len(mp)):
      self.br.sendTransform([mp.x, mp.y, mp.z], [0,0,0,1], rospy.Time.now(), "obs"+str(i), "/camera")
      j = 0
      for p in numpy.linspace(data.minpoint[i].x, data.maxpoint[i].x, 10):
        self.br.sendTransform([p, mp.y, mp.z], [0,0,0,1], rospy.Time.now(), "obs"+str(i)+str(j), "/camera")
        j += 1

if __name__ == "__main__":  
  cls = ClusterSub()
  rospy.spin()
  rospy.logwarn("Exiting EOD Navigation! Bye Bye")






