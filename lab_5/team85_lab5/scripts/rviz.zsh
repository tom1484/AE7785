#!/bin/zsh
# RViz for navigation: 2D Pose Estimate, Nav2 Goal, Publish Point.
# nav_recovery.zsh doesn't open RViz, so run this alongside it.
# macOS strips DYLD_LIBRARY_PATH when running /bin/zsh (SIP), so the setup is
# sourced again in here.
# Run this from a terminal where `conda activate humble` is already done.
source $HOME/Workspace/turtlebot3_ws/install/setup.zsh

rviz2 -d $(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/rviz/tb3_navigation2.rviz
