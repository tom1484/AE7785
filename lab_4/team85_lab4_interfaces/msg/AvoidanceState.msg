# Obstacle avoidance state, published by obstacle_state.
# Vectors are in the robot frame (x: forward, y: left).

std_msgs/Header header

# Whether an obstacle is close and on the robot's path; when false, go to the goal
bool avoiding

# Unit vector along the wall: the obstacle normal rotated by 90 degrees.
# Zero when not avoiding.
geometry_msgs/Vector3 wall_vector

# Whether wall_vector is the normal rotated counterclockwise (else clockwise),
# chosen when avoidance starts. Only meaningful while avoiding.
# Note: ccw means the wall is on the robot's left, so the steering correction
# away from it is clockwise (sigma = -1 in go_to_goal); cw gives sigma = +1.
bool ccw

# Inputs and criteria, for monitoring
bool obstacle_detected                  # A fresh obstacle reading was available
geometry_msgs/Point obstacle_position   # Closest obstacle point (when detected)
geometry_msgs/Vector3 obstacle_normal   # Wall normal used (see wall_normal), towards the robot
float64 avoid_distance                  # Criterion: distance < avoid_distance
float64 path_radius                     # Criterion: in the path zone of this half width
geometry_msgs/Vector3 path_direction    # Unit direction of the path zone (towards the goal)
float64 path_length                     # Length of the path zone: the goal distance (inf without a goal)
