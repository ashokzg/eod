#!/usr/bin/env bash
alias gs="git status"
alias ga="git add"
alias gc="git commit"
alias gpl="git pull"
alias gp="git push"
alias gpll="git pull anusha@anusha:~/eod/.git"
alias gpllr="git pull teamb@teamb:~/eod/.git"
alias gitsave="git config credential.helper 'cache --timeout=300'"
alias gd="git difftool"
alias gdf="git diff"
alias gm="git mergetool"
alias shut="sudo shutdown -h now"
alias reshut="sudo shutdown -rh now"
alias tfp="rosrun tf static_transform_publisher 0 0 0.23 -1.54 0 -1.54 base_link camera 100 &"
alias eod_nav="roslaunch eod_nav primStereo.launch"
alias plotvel="rxplot /lwheel_vel/data /rwheel_vel/data -p 100 --ymin -2 --ymax 2"
alias plotmotor="rxplot /lmotor_cmd/data /rmotor_cmd/data -p 100 --ymin -400 --ymax 400"
alias plotdvel="rxplot /lwheel_vtarget/data /rwheel_vtarget/data -p 100 --ymin -2 --ymax 2"
alias plotenc="rxplot /lwheel/data /rwheel/data -p 100"
alias plotultra="rxplot ultrasound/ultra_centre --ymax 700 --ymin 0"

alias eodctrl="roslaunch eod_ctrl eod_ctrl_open.launch"
alias eodctrlpid="roslaunch eod_ctrl eod_ctrl.launch"
alias eodnavl="roslaunch eod_nav prim.launch"
alias eodnav="rosrun eod_nav eod_nav.py"
alias eodui="roslaunch teamb_ui teamb_ui.launch"
alias eoduio="rosrun teamb_ui teamb_ui"

#alias plotm ="plotvel & plotmotor & plotdvel &"
alias mvrobot="rostopic pub twist geometry_msgs/Twist \"linear: 
  x: 0.3  
  y: 0.0  
  z: 0.0 
angular: 
  x: 0.0 
  y: 0.0 
  z: 0.0\" -r 20"
  
alias stoprobot="rostopic pub twist geometry_msgs/Twist \"linear: 
  x: 0.0  
  y: 0.0  
  z: 0.0 
angular: 
  x: 0.0 
  y: 0.0 
  z: 0.0\" -r 20"
