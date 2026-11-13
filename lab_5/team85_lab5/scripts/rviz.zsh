#!/bin/zsh
# RViz for navigation: 2D Pose Estimate, Nav2 Goal, Publish Point.
# nav_recovery.zsh doesn't open RViz, so run this alongside it.
# --sim: simulator, with sim time.
source ${0:A:h}/common.zsh "$@"

rviz2 -d $(ros2 pkg prefix team85_lab5)/share/team85_lab5/rviz/navigation.rviz \
  --ros-args -p use_sim_time:=$( (( SIM )) && echo true || echo false )
