#!/bin/zsh
# Save the map being built (Cartographer running) as maps/map.pgm + map.yaml.
# --sim: simulator, saved as maps/map_sim.* so the real map isn't overwritten.
# Rebuild afterwards (colcon build --packages-select team85_final) so the
# launch files find the new map.
source ${0:A:h}/common.zsh "$@"

if (( SIM )); then
  ros2 run nav2_map_server map_saver_cli -f $WS/src/team85_final/maps/map_sim \
    --ros-args -p use_sim_time:=true
else
  ros2 run nav2_map_server map_saver_cli -f $WS/src/team85_final/maps/map
fi
