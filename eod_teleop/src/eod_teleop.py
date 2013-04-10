#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('eod_teleop')


from eod_nav.msg import Velocity
from sensor_msgs.msg import Joy


def joy_callback(data):
    print data.buttons



if __name__ == "__main__":
    rospy.init_node("eod_teleop")
    rospy.Subscriber("joy", Joy, joy_callback)
    rospy.Subscriber("uiState", std_msgs.msg.String, self.uiState)
    rospy.spin()
