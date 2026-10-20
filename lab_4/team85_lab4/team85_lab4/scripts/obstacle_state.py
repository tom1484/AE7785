#!/usr/bin/env python3

"""
CS/ME/ECE/AE/BME 7785 - Lab 4

Authors:
Chu-Rong Chen
Xingyu Zhu

Node:
    obstacle_state

Subscribes:
    /obstacle_position
      geometry_msgs/msg/PointStamped
    /obstacle_normal
      geometry_msgs/msg/Vector3Stamped

Publishes:
    /avoidance_state
      team85_lab4_interfaces/msg/AvoidanceState
    TODO: Avoidance output (vector or twist) for go_to_goal, format to be decided.

Runs ObstacleState at a fixed rate on the latest obstacle reading.
"""

from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time

from geometry_msgs.msg import PointStamped, Vector3Stamped
from team85_lab4_interfaces.msg import AvoidanceState

import numpy as np

from team85_lab4.verbose_logger import VerboseLogger


UPDATE_PERIOD = 0.05  # Seconds between state updates (20 Hz)


class ObstacleState:
    """
    Obstacle-level state management: whether an obstacle is encountered, the
    avoidance state, and the avoidance output used by go_to_goal.

    All vectors are in the robot frame (x: forward, y: left).
    """

    def __init__(self):
        # Current avoidance state (an AvoidanceState constant)
        self.state = AvoidanceState.GO_TO_GOAL

        # TODO: Configuration (e.g. distance thresholds) and internal state.

    def update(
        self,
        obstacle_position: Optional[np.ndarray],
        obstacle_normal: Optional[np.ndarray],
        # TODO: More inputs to be added.
    ) -> Any:
        """
        Update the avoidance state (self.state).

        Args:
            obstacle_position: Closest obstacle point, or None if none is detected.
            obstacle_normal: Unit surface normal at that point (towards the
                             robot), or None if none is detected.

        Returns:
            TODO: Avoidance output (vector or twist), format to be decided.
        """
        # TODO: Update self.state and compute the avoidance output.
        return None


class ObstacleStateNode(Node):

    def __init__(self):
        super().__init__("obstacle_state")

        # Obstacle readings older than this are ignored, since scan_processor
        # publishes nothing when no obstacle is in view
        self.obstacle_timeout = float(self.declare_parameter("obstacle_timeout", 0.5).value)
        self.log = VerboseLogger(self)

        self.obstacle_state = ObstacleState()

        # Latest obstacle reading (robot frame) and when it was received
        self.obstacle_position: Optional[np.ndarray] = None
        self.obstacle_normal: Optional[np.ndarray] = None
        self.position_received: Optional[Time] = None
        self.normal_received: Optional[Time] = None

        self.position_subscription = self.create_subscription(
            PointStamped, "/obstacle_position", self.position_callback, 10
        )
        self.normal_subscription = self.create_subscription(
            Vector3Stamped, "/obstacle_normal", self.normal_callback, 10
        )
        self.state_publisher = self.create_publisher(AvoidanceState, "/avoidance_state", 10)
        # TODO: Publisher for the avoidance output (vector or twist).

        self.timer = self.create_timer(UPDATE_PERIOD, self.update)

        self.log.basic("Listening to obstacle position and normal...")

    def position_callback(self, msg: PointStamped):
        self.obstacle_position = np.array([msg.point.x, msg.point.y])
        self.position_received = self.get_clock().now()

    def normal_callback(self, msg: Vector3Stamped):
        self.obstacle_normal = np.array([msg.vector.x, msg.vector.y])
        self.normal_received = self.get_clock().now()

    def is_fresh(self, received: Optional[Time]) -> bool:
        if received is None:
            return False
        age = (self.get_clock().now() - received).nanoseconds * 1e-9
        return age <= self.obstacle_timeout

    def update(self):
        position = self.obstacle_position if self.is_fresh(self.position_received) else None
        normal = self.obstacle_normal if self.is_fresh(self.normal_received) else None

        output = self.obstacle_state.update(position, normal)

        state = AvoidanceState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.state = self.obstacle_state.state
        self.state_publisher.publish(state)

        # TODO: Publish the avoidance output once its format is decided.

        self.log.continuous(
            f"state={state.state} "
            f"obstacle={None if position is None else np.round(position, 3).tolist()} "
            f"normal={None if normal is None else np.round(normal, 3).tolist()} "
            f"output={output}",
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ObstacleStateNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
