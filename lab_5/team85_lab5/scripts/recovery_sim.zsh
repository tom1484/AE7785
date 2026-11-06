#!/bin/zsh
# Recovery server in sim, with the maze candidates. Start this BEFORE
# nav_recovery_sim.zsh, or bt_navigator crashes when it can't find the service.
# "Cannot beep: /sound service unavailable" is normal: Gazebo has no buzzer.
source ${0:A:h}/sim_env.zsh

python $(ros2 pkg prefix turtlebot3_recovery_lab)/lib/turtlebot3_recovery_lab/recovery_student.py \
  --ros-args -p use_sim_time:=true \
  -p candidates_file:=$HOME/Workspace/turtlebot3_ws/src/turtlebot3_recovery_lab/config/recovery_candidates_sim.yaml
