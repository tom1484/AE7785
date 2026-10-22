#!/usr/bin/env python3

"""
CS/ME/ECE/AE/BME 7785 - Lab 4

Authors:
Chu-Rong Chen
Xingyu Zhu

Node:
    low_level_controller

Subscribes:
    /virtual_target
      team85_lab4_interfaces/msg/VirtualTarget
      Robot-relative virtual heading and distance from go_to_goal.
    /pause
      std_msgs/msg/Bool (transient local)
      true stops the robot until false; sent by avoidance_monitor (space).

Publishes:
    /cmd_vel
      geometry_msgs/msg/Twist

A P controller (after Lab 3's chase_object) that turns each virtual target
into velocities as it arrives. The robot is stopped when the target is not OK
(e.g. FINISHED, STALE_INPUT), when no target arrived for target_timeout,
while paused, and on shutdown. Pausing here (not in go_to_goal) keeps the
virtual target published, so the monitors still show it.
"""

import math
import signal
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from team85_lab4_interfaces.msg import VirtualTarget as VirtualTargetMsg

from team85_lab4.scripts.go_to_goal import Target
from team85_lab4.verbose_logger import VerboseLogger


WATCHDOG_PERIOD = 0.05  # Seconds between checks for a missing target (20 Hz)

# /pause keeps its last value for late subscribers, so a restarted
# low_level_controller stays paused
PAUSE_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


@dataclass
class ControllerConfig:
    """Gains and limits of the low-level controller."""
    kp_linear: float          # Linear speed per meter of virtual distance (1/s)
    kp_angular: float         # Angular speed per radian of virtual heading (1/s)
    distance_deadband: float  # |distance_vt| below this gives no linear speed (m)
    angle_deadband: float     # |theta_vt| below this gives no angular speed (rad)
    max_linear_speed: float   # (m/s)
    max_angular_speed: float  # (rad/s)

    def __post_init__(self):
        for name in ("kp_linear", "kp_angular", "max_linear_speed", "max_angular_speed"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"{name} must be finite and > 0, got {value}")
        for name in ("distance_deadband", "angle_deadband"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f"{name} must be finite and >= 0, got {value}")


class LowLevelController:
    """
    P controller from the virtual target to velocities.

    distance_vt is a signed longitudinal reference along the robot's heading,
    so it maps to the linear speed directly (negative: backing up); it is
    never turned into a point, which would flip the bearing when negative.
    theta_vt is positive to the left, like ROS angular.z. The heading is
    still controlled when distance_vt is zero, so the robot turns in place.
    """

    def __init__(self, config: ControllerConfig):
        self.cfg = config

    def compute(self, target: Target):
        """Returns (linear, angular) speeds; zero unless the target is OK."""
        cfg = self.cfg
        if target.status != "OK":
            return 0.0, 0.0

        linear = 0.0
        if abs(target.distance_vt) > cfg.distance_deadband:
            linear = cfg.kp_linear * target.distance_vt

        angular = 0.0
        if abs(target.theta_vt) > cfg.angle_deadband:
            angular = cfg.kp_angular * target.theta_vt

        linear = min(max(linear, -cfg.max_linear_speed), cfg.max_linear_speed)
        angular = min(max(angular, -cfg.max_angular_speed), cfg.max_angular_speed)
        return linear, angular


# =============================================================
# Node
# =============================================================

class LowLevelControllerNode(Node):

    def __init__(self):
        super().__init__("low_level_controller")

        def param(name, default):
            return self.declare_parameter(name, default).value

        self.controller = LowLevelController(ControllerConfig(
            kp_linear=float(param("kp_linear", 0.4)),
            kp_angular=float(param("kp_angular", 1.2)),
            distance_deadband=float(param("distance_deadband", 0.005)),
            angle_deadband=float(param("angle_deadband", 0.05)),
            max_linear_speed=float(param("max_linear_speed", 0.15)),
            max_angular_speed=float(param("max_angular_speed", 0.8)),
        ))
        # Stop when no virtual target arrived for this long (s)
        self.target_timeout = float(param("target_timeout", 0.5))
        self.log = VerboseLogger(self)

        self.received: Optional[Time] = None
        # Whether the robot was stopped for a missing target, so the stop is
        # sent once (and teleop is not overridden while go_to_goal is down)
        self.timed_out = False
        # While paused, the stop is sent once and nothing after it (so teleop
        # can move the robot)
        self.paused = False

        self.target_subscription = self.create_subscription(
            VirtualTargetMsg, "/virtual_target", self.target_callback, 10
        )
        self.pause_subscription = self.create_subscription(
            Bool, "/pause", self.pause_callback, PAUSE_QOS
        )
        self.cmd_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.watchdog = self.create_timer(WATCHDOG_PERIOD, self.check_timeout)

        self.log.basic("low_level_controller started")

    def pause_callback(self, msg: Bool):
        if msg.data == self.paused:
            return
        self.paused = msg.data
        self.log.basic("Paused" if self.paused else "Resumed")
        if self.paused:
            self.stop_robot()

    def target_callback(self, msg: VirtualTargetMsg):
        self.received = self.get_clock().now()
        if self.timed_out:
            self.log.basic("Virtual target back")
            self.timed_out = False

        if self.paused:
            return

        target = Target(msg.theta, msg.distance, msg.status)
        linear, angular = self.controller.compute(target)
        self.publish(linear, angular)
        self.log.continuous(
            f"{msg.status} theta_vt={math.degrees(msg.theta):6.1f} deg "
            f"distance_vt={msg.distance:.3f} m | v={linear:.3f} m/s w={angular:.3f} rad/s",
            throttle_duration_sec=0.5,
        )

    def check_timeout(self):
        if self.received is None or self.timed_out:
            return
        age = (self.get_clock().now() - self.received).nanoseconds * 1e-9
        if age > self.target_timeout:
            self.log.basic(f"No virtual target for {age:.2f} s, stopping")
            self.timed_out = True
            self.stop_robot()

    def publish(self, linear: float, angular: float):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_publisher.publish(cmd)

    def stop_robot(self):
        self.cmd_publisher.publish(Twist())


def main(args=None):
    # Handle Ctrl+C ourselves, so the robot can be stopped before shutdown.
    # Raise KeyboardInterrupt even when SIGINT was inherited as ignored (e.g.
    # started in the background by a non-interactive shell).
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = None

    try:
        node = LowLevelControllerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if node.context.ok():
                node.stop_robot()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
