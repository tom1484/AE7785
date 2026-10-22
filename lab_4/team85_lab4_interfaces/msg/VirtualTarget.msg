# Virtual target computed by go_to_goal (see the virtual target planner).
# Angles in radians, distances in meters.

std_msgs/Header header

# Robot-relative virtual heading (positive: left) and distance
float64 theta
float64 distance

# OK, or why the target is zeroed: GOAL_REACHED, FINISHED, STALE_INPUT,
# NONFINITE_INPUT, MISSING_OBSTACLE, INVALID_OBSTACLE_DIRECTION,
# MISSING_TANGENT, INVALID_TANGENT, INVALID_WALL_NORMAL, DEGENERATE_WALL_GEOMETRY
string status

# Behavior flags used (both follow AvoidanceState.avoiding)
bool enable_fw
bool enable_ao

# Diagnostics
float64 u_gtg_theta         # GTG heading contribution (after FW suppression)
float64 u_fw_theta          # FW heading contribution
float64 u_gtg_distance      # GTG attraction magnitude, before projection
float64 u_ao_distance       # AO repulsion magnitude, before projection
float64 u_fw_distance       # FW drive along the wall, before projection
float64 gtg_cosine          # Goal direction projected on the heading
float64 ao_cosine           # Obstacle-to-robot direction projected on the heading
float64 fw_cosine           # Wall tangent projected on the heading
float64 fw_sigma            # FW correction direction (+1: CCW, -1: CW, 0: FW off)
float64 fw_error_z          # sigma * theta_FW; negative when heading into the wall
float64 fw_recovery_gamma   # Fraction of opposing GTG steering restored
float64 fw_proximity_weight # FW steering scale from the obstacle distance, in (0, 1]
float64 raw_distance        # Distance before clamping
