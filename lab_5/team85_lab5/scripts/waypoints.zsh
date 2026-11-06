#!/bin/zsh
# Waypoint navigator. Start after Nav2 is up and the initial pose is set.
# (waypoint_navigator is still a stub: it only logs "not implemented".)
# macOS strips DYLD_LIBRARY_PATH when running /bin/zsh (SIP), so the setup is
# sourced again in here.
# Run this from a terminal where `conda activate humble` is already done.
# Extra arguments are passed to the launch file, e.g. params_file:=/path.yaml
source $HOME/Workspace/turtlebot3_ws/install/setup.zsh

ros2 launch team85_lab5 robot.launch.py "$@"
