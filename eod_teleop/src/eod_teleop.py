#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('eod_teleop')


from eod_nav.msg import Velocity
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32

global manual
manual = [False]
cmd_vel = Velocity()
cmd_vel.linVelPcent = 0
cmd_vel.angVelPcent = 0
count = [0]
joytime = [0]


def joy_callback(data):
    print "joy cb"
    cmd_vel.linVelPcent = 0
    cmd_vel.angVelPcent = 0
    #Only if dead man's button is pressed take any action
    if data.buttons[4] == 1:
        print data.buttons[5] + 1, data.axes[4]
        cmd_vel.linVelPcent = -(data.buttons[5] + 1)*data.axes[4]/2
        cmd_vel.angVelPcent = -(data.buttons[5] + 1)*data.axes[3]/2
    robotCmdPub.publish(cmd_vel)

def uiState(data):
    if data.data == 1:
        manual[0] = True
    else:
        manual[0] = False

rospy.init_node("eod_teleop")
rospy.Subscriber("joy", Joy, joy_callback)
rospy.Subscriber("UiStatus", Int32, uiState)
robotCmdPub = rospy.Publisher('man_cmd_vel', Velocity)
print "EOD Teleop started. Start joystick"    
rospy.spin()
