# Obstacle avoidance state, published by obstacle_state.

# TODO: Define the states once the avoidance logic is decided.
uint8 GO_TO_GOAL = 0    # No obstacle in the way; head to the goal

std_msgs/Header header
uint8 state
