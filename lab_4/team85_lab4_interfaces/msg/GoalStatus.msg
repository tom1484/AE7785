# Stage of the goal node, published on every odometry reading.
std_msgs/Header header

# World frame expressed in the odom frame, set by the first odometry reading
bool origin_set
float64 origin_x
float64 origin_y
float64 origin_yaw

# Waypoints in the world frame
geometry_msgs/Point[] goals

# Index of the current goal (equal to the number of goals once finished)
uint32 goal_index
bool finished

# Progress on the current goal
float64 distance      # Distance to the current goal (m), 0 once finished
float64 goal_tolerance  # A goal is reached within this distance (m)
float64 hold_elapsed  # Time spent within tolerance so far (s)
float64 hold_time     # Time required within tolerance before switching (s)
