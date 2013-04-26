roslaunch eod_nav primStereo.launch  
rosrun eod_nav eod_nav.py  
roslaunch eod_ctrl eod_ctrl.launch  
rosrun rviz rviz  (load eod/misc/odom.vcg)  
roslaunch teamb_ui teamb_ui.launch  


Other info:  
Publish velocity using following command  
rostopic pub twist geometry_msgs/Twist "linear:  
  x: 0.0  
  y: 0.0  
  z: 0.0  
angular:  
  x: 0.0  
  y: 0.0  
  z: 0.0" -r 20  
  
  
Only x and z are significant  yena y
