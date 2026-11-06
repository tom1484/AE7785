#!/bin/zsh
# Tell AMCL the robot starts at the maze spawn point. map_sim was built by
# Cartographer starting from the spawn, so the spawn is (0, 0) facing +x.
# Only valid right after gazebo_sim.zsh; if the robot has moved, use
# 2D Pose Estimate in RViz instead.
source ${0:A:h}/sim_env.zsh

ros2 topic pub -t 3 -r 1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}"
