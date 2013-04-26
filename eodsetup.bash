#!/usr/bin/env bash
alias gs="git status"
alias ga="git add"
alias gc="git commit"
alias gpl="git pull"
alias gp="git push"
alias gpll="git pull anusha@anusha:~/eod/.git"
alias gitsave="git config credential.helper 'cache --timeout=300'"
alias shut="sudo shutdown -h now"
alias reshut="sudo shutdown -rh now"
alias tfp="rosrun tf static_transform_publisher 0 0 0.23 -1.54 0 -1.54 base_link camera 100 &"
alias eod_nav="roslaunch eod_nav primStereo.launch"
alias plotvel="rxplot /lwheel_vel/data /rwheel_vel/data -p 100 --ymin -2 --ymax 2"
alias plotmotor="rxplot /lmotor_cmd/data /rmotor_cmd/data -p 100 --ymin -400 --ymax 400"
alias plotdvel="rxplot /lwheel_vtarget/data /rwheel_vtarget/data -p 100 --ymin -2 --ymax 2"
#alias plotm ="plotvel & plotmotor & plotdvel &"
alias mvrobot="rostopic pub twist geometry_msgs/Twist \"linear: 
  x: 0.3  
  y: 0.0  
  z: 0.0 
angular: 
  x: 0.0 
  y: 0.0 
  z: 0.0\" -r 20"

