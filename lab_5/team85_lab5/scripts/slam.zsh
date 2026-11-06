#!/bin/zsh
# Cartographer + RViz for mapping (turtlebot3_cartographer).
# --sim: sim time; start after gazebo_sim.zsh. (mapping.zsh does both.)
source ${0:A:h}/common.zsh "$@"

if (( SIM )); then
  ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True "${ARGS[@]}"
else
  ros2 launch turtlebot3_cartographer cartographer.launch.py "${ARGS[@]}"
fi
