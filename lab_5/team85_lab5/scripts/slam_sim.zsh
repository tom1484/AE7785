#!/bin/zsh
# Cartographer + RViz for mapping in sim. Start after gazebo_sim.zsh.
source ${0:A:h}/sim_env.zsh

ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
