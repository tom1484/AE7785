#!/bin/zsh
# Keyboard teleop for the simulated robot.
source ${0:A:h}/sim_env.zsh

ros2 run team85_teleop wasd_teleop
