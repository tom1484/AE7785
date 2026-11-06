#!/bin/zsh
# Recovery server (/divert_robot). Start this BEFORE nav_recovery.zsh, or
# bt_navigator crashes when it can't find the service.
# --sim: sim time and the maze candidates (recovery_candidates_sim.yaml).
#   "Cannot beep: /sound service unavailable" is normal there: no buzzer.
# Extra arguments are passed to the node, e.g. --ros-args -p candidates_file:=...
# Started with `python`: a #!/usr/bin/env shebang would drop DYLD_LIBRARY_PATH.
source ${0:A:h}/common.zsh "$@"

node=$(ros2 pkg prefix turtlebot3_recovery_lab)/lib/turtlebot3_recovery_lab/recovery_student.py
if (( SIM )); then
  python $node --ros-args -p use_sim_time:=true \
    -p candidates_file:=$WS/src/turtlebot3_recovery_lab/config/recovery_candidates_sim.yaml \
    "${ARGS[@]}"
else
  python $node "${ARGS[@]}"
fi
