#!/bin/zsh
# Record waypoints with RViz "Publish Point" and write them to wayPoints.txt.
# Click the points in order; after the last one the file is rewritten (the
# comment lines are kept, the old points go to wayPoints.txt.bak).
#
#   scripts/record_waypoints.zsh          # 3 points -> wayPoints.txt
#   scripts/record_waypoints.zsh 5        # 5 points
#   scripts/record_waypoints.zsh --sim    # simulator -> wayPoints_sim.txt
#
# The installed wayPoints.txt links to the source file, so no rebuild needed.
source ${0:A:h}/common.zsh "$@"

count=${ARGS[1]:-3}
if (( SIM )); then
  file=$WS/src/team85_lab5/wayPoints_sim.txt
else
  file=$WS/src/team85_lab5/wayPoints.txt
fi

echo "Click $count points with Publish Point in RViz (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)."
echo "They'll be written to $file"

# `python -` rather than a #!/usr/bin/env script: macOS drops
# DYLD_LIBRARY_PATH when going through /usr/bin/env.
python -u - "$count" "$file" <<'EOF'
import shutil
import sys

import rclpy
from geometry_msgs.msg import PointStamped

count, path = int(sys.argv[1]), sys.argv[2]
points = []


def on_point(msg):
    if len(points) >= count:
        return
    if msg.header.frame_id != 'map':
        print(f'  ignored: point in frame {msg.header.frame_id!r}, not map')
        return
    points.append((msg.point.x, msg.point.y))
    print(f'  {len(points)}/{count}: {msg.point.x:.3f} {msg.point.y:.3f}')


rclpy.init()
node = rclpy.create_node('record_waypoints')
node.create_subscription(PointStamped, '/clicked_point', on_point, 10)
try:
    while rclpy.ok() and len(points) < count:
        rclpy.spin_once(node, timeout_sec=0.1)
except KeyboardInterrupt:
    print(f'\nStopped after {len(points)} points; file not changed.')
    sys.exit(1)
finally:
    node.destroy_node()
    rclpy.try_shutdown()

with open(path) as f:
    comments = [line for line in f if line.lstrip().startswith('#')]
shutil.copyfile(path, path + '.bak')
with open(path, 'w') as f:
    f.writelines(comments)
    f.writelines(f'{x:.3f} {y:.3f}\n' for x, y in points)
print(f'Wrote {count} waypoints to {path} (old file: {path}.bak)')
EOF
