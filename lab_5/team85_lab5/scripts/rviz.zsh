#!/bin/zsh
# RViz for navigation: 2D Pose Estimate, Nav2 Goal, Publish Point.
# nav_recovery.zsh doesn't open RViz, so run this alongside it.
# --sim: simulator, with sim time.
source ${0:A:h}/common.zsh "$@"

rviz2 -d $(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/rviz/tb3_navigation2.rviz \
  --ros-args -p use_sim_time:=$( (( SIM )) && echo true || echo false )
