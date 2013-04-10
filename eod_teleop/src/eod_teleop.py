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
    cmd_vel.linVelPcent = 0
    cmd_vel.angVelPcent = 0
    if manual[0] == True:
        joytime[0] += 1
        count[0] = 0
        #Only if dead man's button is pressed take any action
        if data.buttons[4] == 1:
            print data.buttons[5] + 1, data.axes[4]
            cmd_vel.linVelPcent = (data.buttons[5] + 1)*data.axes[4]/2
            cmd_vel.angVelPcent = (data.buttons[5] + 1)*data.axes[3]/2
        if joytime[0] % 50 == 0:
            robotCmdPub.publish(cmd_vel)
    #If RED button and the deadman's switch is pressed, stop the robot
    if data.buttons[1] == 1 and data.buttons[4] == 1:
        count[0] += 1        
        if count[0] == 10:
            robotCmdPub.publish(cmd_vel)        
            manual[0] = True

def uiState(data):
    if data.data == 1:
        manual[0] = True
    else:
        manual[0] = False

rospy.init_node("eod_teleop")
rospy.Subscriber("joy", Joy, joy_callback)
rospy.Subscriber("UiStatus", Int32, uiState)
robotCmdPub = rospy.Publisher('cmd_vel', Velocity)
print "EOD Teleop started. Start joystick"    
rospy.spin()
