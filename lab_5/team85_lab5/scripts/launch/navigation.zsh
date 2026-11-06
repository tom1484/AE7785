#!/bin/zsh
# Real robot: recovery node, Nav2, RViz (launch/navigation.launch.py).
#   Start the robot bringup first.
#   Add set_initial_pose:=true initial_x:=... initial_y:=... initial_yaw:=...
#   (print_initial_pose.zsh prints them), or use 2D Pose Estimate in RViz.
# --sim: Gazebo maze + the same, with map_sim, the sim candidates and the
#   initial pose at the spawn point (launch/navigation_sim.launch.py);
#   gui:=false hides the Gazebo window (lighter on the CPU).
#   If Gazebo hangs on exit: pkill -9 gzserver gzclient
# Add waypoints:=true to also run the waypoints (wayPoints.txt, or
# wayPoints_sim.txt in sim).
source ${0:A:h}/../common.zsh "$@"

if (( SIM )); then
  ros2 launch team85_lab5 navigation_sim.launch.py "${ARGS[@]}"
else
  ros2 launch team85_lab5 navigation.launch.py "${ARGS[@]}"
fi
