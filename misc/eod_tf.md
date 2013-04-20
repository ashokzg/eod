For robot placed on lab bench:  
[x y z y p r] = [0 0 1.17] [ -pi/2 0 -pi/2]  
In this case the robot moves forward in its x. y is towards its left and z towards to sky.  
rosrun tf static_transform_publisher 0 0 1.17 -1.54 0 -1.54 world camera 100  


