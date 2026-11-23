#!/bin/zsh
# Keyboard teleop. --sim: drive the simulated robot. Other arguments pass through, e.g.
# for mapping (slow turns keep Cartographer's walls sharp):
#   scripts/teleop.zsh --ros-args -p angular_speed:=0.4 -p linear_speed:=0.12
source ${0:A:h}/common.zsh "$@"

ros2 run team85_teleop wasd_teleop "${ARGS[@]}"
