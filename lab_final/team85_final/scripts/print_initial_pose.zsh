#!/bin/zsh
# Prints the pose set with "2D Pose Estimate" in RViz (/initialpose): x, y and
# the direction (yaw), plus the launch arguments to reuse it, e.g.
#   scripts/launch/navigation.zsh set_initial_pose:=true initial_x:=... ...
# Keep it running and click as many times as you like; Ctrl-C to stop.
#
# Real robot by default; add --sim to listen on the simulator's ROS domain.
source ${0:A:h}/common.zsh "$@"

echo "Listening on /initialpose (ROS_DOMAIN_ID=$ROS_DOMAIN_ID). Use 2D Pose Estimate in RViz."

# `python -` rather than a #!/usr/bin/env script: macOS drops
# DYLD_LIBRARY_PATH when going through /usr/bin/env.
python -u - <<'EOF'
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped


def on_pose(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    print(
        f"\nframe {msg.header.frame_id}: x = {p.x:.3f}, y = {p.y:.3f}, "
        f"yaw = {yaw:.3f} rad ({math.degrees(yaw):.1f} deg)\n"
        f"  set_initial_pose:=true initial_x:={p.x:.3f} "
        f"initial_y:={p.y:.3f} initial_yaw:={yaw:.3f}",
        flush=True,
    )


rclpy.init()
node = rclpy.create_node("print_initial_pose")
node.create_subscription(PoseWithCovarianceStamped, "/initialpose", on_pose, 10)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
EOF
