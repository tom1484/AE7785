#!/bin/zsh
# Save the sim map as maps/map_sim.pgm and maps/map_sim.yaml, separate from
# the real robot's maps/map.*. Run while slam_sim.zsh is still running.
source ${0:A:h}/sim_env.zsh

ros2 run nav2_map_server map_saver_cli \
  -f $HOME/Workspace/turtlebot3_ws/src/team85_lab5/maps/map_sim \
  --ros-args -p use_sim_time:=true
