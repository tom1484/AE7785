#!/bin/zsh
# macOS strips DYLD_LIBRARY_PATH when running /bin/zsh (SIP), so Nav2 can't
# find the workspace's BT plugin unless the setup is sourced again in here.
# Run this from a terminal where `conda activate humble` is already done.
source $HOME/Workspace/turtlebot3_ws/install/setup.zsh

ros2 run nav2_map_server map_saver_cli -f ./src/team85_lab5/maps/map