#!/bin/zsh
# RViz for navigation in sim: 2D Pose Estimate, Nav2 Goal, Publish Point.
source ${0:A:h}/sim_env.zsh

rviz2 -d $(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/rviz/tb3_navigation2.rviz \
  --ros-args -p use_sim_time:=true
