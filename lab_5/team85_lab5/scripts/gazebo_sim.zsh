#!/bin/zsh
# Gazebo with the 7785 maze (from Lab5_Gazebo). Replaces robot bringup in sim.
# If Gazebo hangs on exit: pkill -9 gzserver; pkill -9 gzclient
source ${0:A:h}/sim_env.zsh

ros2 launch turtlebot3_gazebo 7785_maze.launch.py
