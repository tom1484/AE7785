#!/bin/zsh
# Nav2 with the lab's recovery behavior tree, on the sim map (map_sim.yaml).
# Start after gazebo_sim.zsh and recovery_sim.zsh, then run initial_pose_sim.zsh.
source ${0:A:h}/sim_env.zsh

ros2 launch nav2_bringup bringup_launch.py \
  map:=$HOME/Workspace/turtlebot3_ws/src/team85_lab5/maps/map_sim.yaml \
  params_file:=$HOME/Workspace/turtlebot3_ws/src/turtlebot3_recovery_lab/config/nav2_params.yaml \
  use_sim_time:=True autostart:=True
