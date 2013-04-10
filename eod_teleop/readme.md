Troubleshooting:  
-----------------
1. Is joystick working. Test this by running the following command. Replace X with the proper number by looking up in /dev/input/  

ls /dev/input  
sudo jstest /dev/input/jsX  

2. Is the joystick parameter set properly in ROS. The parameter "/joy_node/dev" is by default set to "/dev/input/js0". This should be the same value as above. If it is different set the parameter using the following command. If this was different change the default value in the eod_teleop.launch file.  

rosparam set /joy_node/dev "/dev/input/jsX"  
