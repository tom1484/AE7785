#!/usr/bin/env python3

"""
CS/ME/ECE/AE/BME 7785 - Final.

Authors:
Chu-Rong Chen
Xingyu Zhu

Node:
    set_initial_pose

Initial localization when the robot is placed by hand ("cell (0,5) facing N"), as the
Nav2 guide suggests: give AMCL a good pose, then rotate in place so it converges.

  1. refine (optional): ICP of a few LIDAR scans against the saved map, starting from
     the given pose (maze/scan_match.py). Kept only if the fit is good and the
     correction small (max_correction / max_yaw_correction); else the given pose is used.
  2. publish /initialpose every second until AMCL reports a pose near it (AMCL ignores
     /initialpose until it is active, so a single message at startup can be lost).
  3. spin (optional): turn in place by `spin` rad on /cmd_vel (odometry-measured) so AMCL
     sees the walls from every side, then report AMCL's pose and spread.
  4. publish latched std_msgs/Bool True on /localization_ready (the mission waits for it
     with wait_localized) and stay up idle so the latched message stays available.
"""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('set_initial_pose')
        p = self.declare_parameters('', [
            ('x', 0.0), ('y', 0.0), ('yaw', 0.0),
            ('frame_id', 'map'),
            ('base_frame', 'base_footprint'),
            ('tolerance', 0.3),           # m: AMCL's pose this close = accepted
            ('refine', False),            # ICP against map_yaml
            ('map_yaml', ''),
            ('scans', 5),                 # scans stacked for ICP (robot standing still)
            ('max_correction', 0.35),     # m: larger ICP moves are rejected
            ('max_yaw_correction', 0.6),  # rad (~35 deg)
            ('min_inliers', 0.8),         # fraction of scan points on a map wall (right cell ~1.0, wrong <= 0.6)
            ('spin', 0.0),                # rad to turn in place after AMCL accepts (0 = off)
            ('spin_speed', 0.6),          # rad/s
        ])
        self.cfg = {x.name: x.value for x in p}
        self.pose = (float(self.cfg['x']), float(self.cfg['y']), float(self.cfg['yaw']))
        self.refined = False

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.ready_pub = self.create_publisher(Bool, '/localization_ready', LATCHED)
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.inputs = [
            self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, LATCHED),
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data),
            self.create_subscription(Odometry, '/odom', self.on_odom, 10),
        ]

        self.scans: list[np.ndarray] = []
        self.amcl = None
        self.odom_yaw = None
        self.spun = 0.0
        self.spin_last = None
        self.state = 'refine' if self.cfg['refine'] and self.cfg['map_yaml'] else 'publish'
        self.state_since = self.now()
        self.last_publish = 0.0
        self.done = False
        self.timer = self.create_timer(0.1, self.step)
        self.get_logger().info(
            f'placed at ({self.pose[0]:.2f}, {self.pose[1]:.2f}), yaw '
            f'{math.degrees(self.pose[2]):.0f} deg; '
            + ('refining with the LIDAR' if self.state == 'refine' else 'waiting for AMCL'))

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def enter(self, state):
        self.state, self.state_since = state, self.now()

    # ---- inputs ---------------------------------------------------------
    def on_amcl(self, msg):
        self.amcl = msg

    def on_odom(self, msg):
        yaw = yaw_of(msg.pose.pose.orientation)
        if self.state == 'spin' and self.odom_yaw is not None:
            self.spun += abs(math.remainder(yaw - self.odom_yaw, math.tau))
        self.odom_yaw = yaw

    def on_scan(self, msg: LaserScan):
        if self.state != 'refine' or len(self.scans) >= self.cfg['scans']:
            return
        from team85_final.maze.scan_match import scan_points
        lp = self.laser_pose(msg.header.frame_id)
        if lp is None:
            return
        self.scans.append(scan_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                      max(msg.range_min, 0.12), min(msg.range_max, 3.5), lp))

    def laser_pose(self, frame):
        try:
            t = self.tf.lookup_transform(self.cfg['base_frame'], frame, Time())
        except TransformException:
            if self.now() - self.state_since > 3.0:      # no TF: the burger's LDS mount
                self.get_logger().warn(f'no TF {self.cfg["base_frame"]} <- {frame}; '
                                       'assuming the burger mount (-0.032, 0)', once=True)
                return (-0.032, 0.0, 0.0)
            return None
        return (t.transform.translation.x, t.transform.translation.y,
                yaw_of(t.transform.rotation))

    # ---- steps ----------------------------------------------------------
    def step(self):
        if not self.done:
            getattr(self, 'step_' + self.state)()

    def step_refine(self):
        if len(self.scans) < self.cfg['scans']:
            if self.now() - self.state_since > 15.0:
                self.get_logger().warn('no /scan: using the given pose')
                self.enter('publish')
            return
        from team85_final.maze.scan_match import MapMatcher, localize
        log = self.get_logger()
        try:
            fit = localize(MapMatcher.from_yaml(self.cfg['map_yaml']),
                           np.concatenate(self.scans), self.pose)
        except Exception as e:                            # noqa: BLE001 - never block startup
            log.error(f'ICP failed ({e}); using the given pose')
            self.enter('publish')
            return
        dxy = math.hypot(fit.x - self.pose[0], fit.y - self.pose[1])
        dyaw = math.remainder(fit.yaw - self.pose[2], math.tau)
        msg = (f'ICP: moved {dxy * 100:.0f} cm, {math.degrees(dyaw):+.1f} deg; '
               f'{fit.inliers:.0%} of points on walls, rms {fit.rms * 100:.1f} cm')
        if (fit.inliers >= self.cfg['min_inliers'] and dxy <= self.cfg['max_correction']
                and abs(dyaw) <= self.cfg['max_yaw_correction']):
            self.pose, self.refined = fit.pose, True
            log.info(f'{msg} -> using ({fit.x:.3f}, {fit.y:.3f}, '
                     f'{math.degrees(fit.yaw):.1f} deg)')
        else:
            log.warn(f'{msg} -> REJECTED (check the start cell/heading); using the given pose')
        self.enter('publish')

    def step_publish(self):
        a = self.amcl
        if a is not None and Time.from_msg(a.header.stamp).nanoseconds > 0:
            pos = a.pose.pose.position
            if (math.hypot(pos.x - self.pose[0], pos.y - self.pose[1]) <= self.cfg['tolerance']
                    and self.now() - self.state_since > 0.5):
                self.get_logger().info(f'AMCL accepted the initial pose: ({pos.x:.2f}, {pos.y:.2f})')
                if self.cfg['spin'] > 0.0:
                    self.spun, self.odom_yaw = 0.0, None
                    self.get_logger().info(f'spinning {math.degrees(self.cfg["spin"]):.0f} deg '
                                           'in place so AMCL converges')
                    self.enter('spin')
                else:
                    self.finish()
                return
        if self.now() - self.last_publish >= 1.0:
            self.last_publish = self.now()
            self.publish()

    def step_spin(self):
        left = self.cfg['spin'] - self.spun
        cmd = Twist()
        timeout = 3.0 * self.cfg['spin'] / self.cfg['spin_speed'] + 5.0
        if left > 0.02 and self.now() - self.state_since < timeout:
            cmd.angular.z = max(0.15, min(self.cfg['spin_speed'], 1.5 * left))
            self.cmd_vel.publish(cmd)
            return
        for _ in range(3):
            self.cmd_vel.publish(cmd)                     # stop
        if left > 0.02:
            self.get_logger().warn(f'spin timed out after {math.degrees(self.spun):.0f} deg '
                                   '(no /odom?)')
        self.enter('settle')

    def step_settle(self):
        if self.now() - self.state_since >= 1.0:          # let AMCL take the last scans
            self.finish()

    def finish(self):
        a = self.amcl
        if a is not None:
            c = a.pose.covariance
            pos = a.pose.pose.position
            self.get_logger().info(
                f'AMCL: ({pos.x:.3f}, {pos.y:.3f}, {math.degrees(yaw_of(a.pose.pose.orientation)):.1f}'
                f' deg), std {math.sqrt(max(c[0], 0)) * 100:.1f} / {math.sqrt(max(c[7], 0)) * 100:.1f}'
                f' cm, {math.degrees(math.sqrt(max(c[35], 0))):.1f} deg')
        self.ready_pub.publish(Bool(data=True))
        self.get_logger().info('localization ready')
        self.done = True
        # Idle from here on (only the latched publisher matters): drop the inputs, or every
        # /scan and /odom message keeps costing CPU (~25% of a core on the robot's Pi).
        for sub in self.inputs:
            self.destroy_subscription(sub)
        self.destroy_timer(self.timer)
        self.inputs = []
        self.tf_listener.unregister()             # its /tf decoding in Python was ~14% more

    def publish(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.cfg['frame_id']
        msg.header.stamp = self.get_clock().now().to_msg()
        x, y, yaw = self.pose
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # A refined pose is good to a few cm; keep some spread so the spin can still fix it.
        sxy, syaw = (0.08, math.radians(5)) if self.refined else (0.5, 0.26)
        msg.pose.covariance[0] = msg.pose.covariance[7] = sxy ** 2
        msg.pose.covariance[35] = syaw ** 2
        self.publisher.publish(msg)


def main(args=None):
    # No rclpy SIGINT handler: it would shut the context down before the KeyboardInterrupt
    # reaches us, and the stop command for an interrupted spin could not be published.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = InitialPoseSetter()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
        # Stay up (idle): a latched topic disappears with its publisher, and the mission
        # may subscribe to /localization_ready later.
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        if node.state == 'spin':
            for _ in range(3):
                node.cmd_vel.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
