#!/bin/zsh
# Nav2 only, with the lab's recovery behavior tree. Start recovery.zsh first.
# (navigation.zsh starts all of it at once.)
# --sim: the sim map (map_sim.yaml) and sim time. Start after gazebo_sim.zsh
#   and recovery.zsh --sim, then set the initial pose
#   with 2D Pose Estimate in RViz (the spawn point is 0, 0 facing +x).
source ${0:A:h}/common.zsh "$@"

if (( SIM )); then map=map_sim.yaml; sim_time=True; else map=map.yaml; sim_time=False; fi
ros2 launch nav2_bringup bringup_launch.py \
  map:=$WS/src/team85_lab5/maps/$map \
  params_file:=$WS/src/turtlebot3_recovery_lab/config/nav2_params.yaml \
  use_sim_time:=$sim_time autostart:=True "${ARGS[@]}"
