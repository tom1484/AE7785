#!/bin/zsh
# Real robot: Cartographer + RViz (launch/mapping.launch.py).
#   Start the robot bringup first.
# --sim: Gazebo maze + the same (launch/mapping_sim.launch.py); gui:=false
#   hides the Gazebo window. If Gazebo hangs on exit: pkill -9 gzserver gzclient
# Drive with teleop.zsh, save with save.zsh (both with --sim in sim).
source ${0:A:h}/../common.zsh "$@"

if (( SIM )); then
  ros2 launch team85_lab5 mapping_sim.launch.py "${ARGS[@]}"
else
  ros2 launch team85_lab5 mapping.launch.py "${ARGS[@]}"
fi
