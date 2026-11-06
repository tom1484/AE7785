#!/bin/zsh
# Waypoint navigator. Start after Nav2 is up and the initial pose is set.
# Reads wayPoints.txt; --sim: wayPoints_sim.txt, with sim time.
# Extra arguments are passed to the launch file, e.g. waypoints_file:=/path.txt
source ${0:A:h}/../common.zsh "$@"

if (( SIM )); then
  ros2 launch team85_lab5 robot.launch.py use_sim_time:=true \
    waypoints_file:=$WS/src/team85_lab5/wayPoints_sim.txt "${ARGS[@]}"
else
  ros2 launch team85_lab5 robot.launch.py "${ARGS[@]}"
fi
