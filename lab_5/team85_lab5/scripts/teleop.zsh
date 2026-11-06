#!/bin/zsh
# Keyboard teleop. --sim: drive the simulated robot.
source ${0:A:h}/common.zsh "$@"

ros2 run team85_teleop wasd_teleop
