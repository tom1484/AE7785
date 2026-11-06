# Shared setup for the *_sim.zsh scripts. Sourced, not run.
# macOS strips DYLD_LIBRARY_PATH when running /bin/zsh (SIP), so the workspace
# setup is sourced again here.
# Run the scripts from a terminal where `conda activate humble` is already done.
source $HOME/Workspace/turtlebot3_ws/install/setup.zsh

# Keep simulation traffic off the network so nothing reaches a real robot.
# Every sim terminal must use the same values to see each other.
export ROS_DOMAIN_ID=77
export ROS_LOCALHOST_ONLY=1
export TURTLEBOT3_MODEL=burger
