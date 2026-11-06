# Sourced by the scripts here, first thing:  source ${0:A:h}/common.zsh "$@"
# (from scripts/launch: ${0:A:h}/../common.zsh)
#
# --sim (anywhere in the arguments) selects the simulator: sim_env.zsh sets
# ROS_DOMAIN_ID=77, localhost only and TURTLEBOT3_MODEL=burger. Without it,
# the real robot's environment from the terminal is used as is.
# Sets SIM (1 or 0), ARGS (the other arguments, to pass on) and WS.
#
# macOS strips DYLD_LIBRARY_PATH when running /bin/zsh (SIP), so the setup is
# sourced again in here (otherwise e.g. Nav2 can't load the lab's BT plugin).
# Run the scripts from a terminal where `conda activate humble` is done.
WS=$HOME/Workspace/turtlebot3_ws
SIM=0
ARGS=()
for a in "$@"; do
  if [[ "$a" == "--sim" ]]; then SIM=1; else ARGS+=("$a"); fi
done
if (( SIM )); then
  source ${0:A:h}/sim_env.zsh
else
  source $WS/install/setup.zsh
fi
