#!/usr/bin/env bash
# Run the final on the robot itself (headless), e.g. on the burger:
#   ~/final_ws/src/team85_final/scripts/onboard.sh planner:=graph start:=0,5,N
#   ~/final_ws/src/team85_final/scripts/onboard.sh --nav start:=2,0,W    # Nav2 only (no mission)
# Loads the same DDS profile as the bringup (~/.config/ros/tailscale-env.bash: FastDDS over
# Tailscale, 1200-byte packets). Without it the nodes use FastDDS defaults and large messages
# (map, costmaps) never reach a laptop over Tailscale. View from the laptop with
# scripts/launch/view.zsh. Other arguments pass through to final.launch.py.
set -e
source /opt/ros/humble/setup.bash
[[ -f ~/dontTouch_ws/install/setup.bash ]] && source ~/dontTouch_ws/install/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)/install/setup.bash"
[[ -f ~/.config/ros/tailscale-env.bash ]] && source ~/.config/ros/tailscale-env.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-85} TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-burger}
launch=final.launch.py
[[ "${1:-}" == --nav ]] && { launch=navigation.launch.py; shift; }
exec ros2 launch team85_final $launch rviz:=false onboard:=true "$@"
