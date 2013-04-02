import roslib
roslib.load_manifest('OpenTLD')
import rospy
from _Dest import *
import time
import sys

def sendDest():
    pub = rospy.Publisher('UserDestination', Dest)
    rospy.init_node('TestUser')
    d = Dest()
    d.destX = 200
    d.destY = 200
    d.destWidth = 100
    d.destHeight = 100
    d.destPresent = True
    pub.publish(d)
    rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        sendDest()
    except rospy.ROSInterruptException:
        pass
