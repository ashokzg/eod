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
        rospy.Subscriber("clusters", Clusters, self.Clusters)
        #Init the publishers
        self.clusterPub = rospy.Publisher('obstacles', pcl_eod.msg.Clusters)

    def segment(selfs):
        pass
        

if __name__ == "__main__":  
  cls = ClusterSub()
  r = rospy.Rate(20) #Run every 50 ms
  while not rospy.is_shutdown():
    r.sleep()
  rospy.logwarn("Exiting EOD Navigation! Bye Bye")




