#!/bin/zsh
# macOS strips DYLD_LIBRARY_PATH when running /bin/zsh (SIP), so Nav2 can't
# find the workspace's BT plugin unless the setup is sourced again in here.
# Run this from a terminal where `conda activate humble` is already done.
source $HOME/Workspace/turtlebot3_ws/install/setup.zsh

ros2 launch turtlebot3_cartographer cartographer.launch.py