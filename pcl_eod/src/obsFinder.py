#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('pcl_eod')

from pcl_eod.msg import Clusters

import numpy

class ClusterSub(Exception):
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
    min = 100
    for p in data.point:
      if min > p.z:
        min = p.z
        minP = p
    print min
    print minP
    print "\n"

if __name__ == "__main__":  
  cls = ClusterSub()
  rospy.spin()
  rospy.logwarn("Exiting EOD Navigation! Bye Bye")






