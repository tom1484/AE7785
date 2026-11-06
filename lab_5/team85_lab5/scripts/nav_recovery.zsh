#!/bin/zsh
# macOS strips DYLD_LIBRARY_PATH when running /bin/zsh (SIP), so Nav2 can't
# find the workspace's BT plugin unless the setup is sourced again in here.
# Run this from a terminal where `conda activate humble` is already done.
source $HOME/Workspace/turtlebot3_ws/install/setup.zsh

ros2 launch nav2_bringup bringup_launch.py \
  map:=$HOME/Workspace/turtlebot3_ws/src/team85_lab5/maps/map.yaml \
  params_file:=$HOME/Workspace/turtlebot3_ws/src/turtlebot3_recovery_lab/config/nav2_params.yaml \
  use_sim_time:=False autostart:=True