"""
drive_cells: drive through every cell of a maze grid (for mapping), sim or robot.

Plans a depth-first walk over the cell graph of --maze (geometry + walls, e.g. a rough
fit from an older map), then drives it with /cmd_vel: rotate in place toward the next
cell center, drive with heading correction. Pose = TF map -> base_footprint (Cartographer
running). Stops if the LIDAR sees something closer than --stop-dist ahead.

    python <prefix>/lib/team85_final/drive_cells --maze /path/maze.yaml [--speed 0.15]
Run with Cartographer up (launch/mapping.launch.py). Ctrl-C stops the robot.
--maze must be in the CURRENT map frame (Cartographer's map starts at the robot's start
pose). Slow turns (--max-turn) matter: fast spins made Cartographer slip in the sim.
"""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from ..maze.geometry import Cell, Heading
from ..maze.grid import MazeGrid


def dfs_walk(grid: MazeGrid, start: Cell) -> list[Cell]:
    """Cells in visiting order (with backtracking) so that every reachable cell is visited."""
    walk, seen = [start], {start}

    def visit(c: Cell):
        for h in (Heading.EAST, Heading.NORTH, Heading.WEST, Heading.SOUTH):
            n = c.step(h)
            if not grid.blocked(c, h) and n not in seen:
                seen.add(n)
                walk.append(n)
                visit(n)
                walk.append(c)                      # come back
    visit(start)
    while len(walk) > 1 and walk[-1] in walk[:-1]:
        walk.pop()                                  # no need to walk back at the end
    return walk


class Driver(Node):
    def __init__(self, grid: MazeGrid, speed: float, stop_dist: float, frame: str, base: str,
                 max_turn: float, settle: float, blocked_timeout: float):
        super().__init__('drive_cells')
        self.grid, self.speed, self.stop_dist = grid, speed, stop_dist
        self.max_turn, self.settle, self.blocked_timeout = max_turn, settle, blocked_timeout
        self.t_start = None
        self.blocked_since = None
        self.frame, self.base = frame, base
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.front = math.inf
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.route: list[tuple[float, float]] = []
        self.i = 0
        self.done = False
        self.create_timer(0.05, self.step)

    def on_scan(self, s: LaserScan):
        r = [v for k, v in enumerate(s.ranges)
             if s.range_min < v < s.range_max
             and abs(math.atan2(math.sin(s.angle_min + k * s.angle_increment),
                                math.cos(s.angle_min + k * s.angle_increment))) < 0.35]
        self.front = min(r) if r else math.inf

    def pose(self):
        try:
            t = self.tf.lookup_transform(self.frame, self.base, Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def step(self):
        p = self.pose()
        if p is None:
            return
        if not self.route:
            here = self.grid.locate(*p)
            if here is None:
                self.get_logger().error(f'robot at {p[:2]} is outside the maze grid')
                self.finish()
                return
            cells = dfs_walk(self.grid, here.cell)
            self.route = [self.grid.cell_center(c) for c in cells]
            self.get_logger().info(f'{len(cells)} waypoints: ' +
                                   ' '.join(f'({c.row},{c.col})' for c in cells))
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t_start is None:
            self.t_start = now
        if now - self.t_start < self.settle:          # let SLAM lock on before moving
            return
        if self.i >= len(self.route):
            self.finish()
            return
        x, y, yaw = p
        gx, gy = self.route[self.i]
        dist = math.hypot(gx - x, gy - y)
        if dist < 0.08:
            self.i += 1
            self.get_logger().info(f'waypoint {self.i}/{len(self.route)}')
            return
        err = math.atan2(math.sin(math.atan2(gy - y, gx - x) - yaw),
                         math.cos(math.atan2(gy - y, gx - x) - yaw))
        cmd = Twist()
        # Slow turns: fast rotation between 5 Hz scans makes scan matching slip.
        cmd.angular.z = max(-self.max_turn, min(self.max_turn, 1.5 * err))
        if abs(err) < 0.25:
            cmd.linear.x = min(self.speed, 0.6 * dist + 0.03)
            if self.front < self.stop_dist:
                cmd.linear.x = 0.0
                self.blocked_since = self.blocked_since or now
                self.get_logger().warn('obstacle ahead, holding', throttle_duration_sec=2.0)
                if now - self.blocked_since > self.blocked_timeout:
                    self.get_logger().error(f'waypoint {self.i + 1} blocked; skipping it')
                    self.i += 1
                    self.blocked_since = None
            else:
                self.blocked_since = None
        self.cmd.publish(cmd)

    def finish(self):
        self.cmd.publish(Twist())
        if not self.done:
            self.get_logger().info('route done')
        self.done = True


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--maze', required=True)
    ap.add_argument('--speed', type=float, default=0.12)
    ap.add_argument('--max-turn', type=float, default=0.4, help='rad/s')
    ap.add_argument('--settle', type=float, default=5.0, help='s still before moving')
    ap.add_argument('--blocked-timeout', type=float, default=8.0)
    ap.add_argument('--stop-dist', type=float, default=0.2)
    ap.add_argument('--frame', default='map')
    ap.add_argument('--base', default='base_footprint')
    a, ros_args = ap.parse_known_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=ros_args)
    node = Driver(MazeGrid.from_yaml(a.maze), a.speed, a.stop_dist, a.frame, a.base,
                  a.max_turn, a.settle, a.blocked_timeout)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
