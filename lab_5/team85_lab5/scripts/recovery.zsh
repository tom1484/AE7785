#!/bin/zsh
# Recovery server (/divert_robot). Start this BEFORE nav_recovery.zsh, or
# bt_navigator crashes when it can't find the service.
# macOS strips DYLD_LIBRARY_PATH when running /bin/zsh (SIP), so the setup is
# sourced again in here; the node is started with `python` for the same reason.
# Run this from a terminal where `conda activate humble` is already done.
# Extra arguments are passed to the node, e.g. --ros-args -p use_sim_time:=true
source $HOME/Workspace/turtlebot3_ws/install/setup.zsh

python $(ros2 pkg prefix turtlebot3_recovery_lab)/lib/turtlebot3_recovery_lab/recovery_student.py "$@"
