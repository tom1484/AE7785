#!/bin/zsh
# launch/navigation.launch.py. Real robot by default (start the robot bringup first).
# --sim: also starts Gazebo on the sim ROS domain (gui:=false hides its window).
#   If Gazebo hangs on exit: pkill -9 gzserver gzclient
# Other arguments pass through, e.g. planner:=graph rviz:=false
source ${0:A:h}/../common.zsh "$@"

if (( SIM )); then
  ros2 launch team85_final navigation.launch.py sim:=true "${ARGS[@]}"
else
  ros2 launch team85_final navigation.launch.py "${ARGS[@]}"
fi
