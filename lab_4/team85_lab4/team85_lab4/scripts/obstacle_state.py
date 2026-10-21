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
    /goal_vector
      geometry_msgs/msg/Vector3Stamped
      Direction of the path zone; the robot's heading without a recent one.

Publishes:
    /avoidance_state
      team85_lab4_interfaces/msg/AvoidanceState
      Whether to avoid, and the wall-following vector (robot frame).

Runs ObstacleState at a fixed rate on the latest obstacle reading.
"""

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time

from geometry_msgs.msg import PointStamped, Vector3Stamped
from team85_lab4_interfaces.msg import AvoidanceState

import numpy as np

from team85_lab4.path_zone import HEADING, GoalDirectionListener, in_path_zone
from team85_lab4.verbose_logger import VerboseLogger


UPDATE_PERIOD = 0.05  # Seconds between state updates (20 Hz)


def rotate_ccw(vector: np.ndarray) -> np.ndarray:
    """Rotate a 2D vector by 90 degrees counterclockwise."""
    return np.array([-vector[1], vector[0]])


class ObstacleState:
    """
    Obstacle-level state management: whether to avoid an obstacle, and the
    wall-following vector to use while avoiding.

    The robot avoids while the obstacle point is both
        - closer than avoid_distance, and
        - in the path zone: ahead along the path direction (towards the goal,
          or the robot's heading without one), short of the goal, and within
          path_radius of the path line, i.e. the robot would hit it on its way
          to the goal.
    It goes back to goal following as soon as either condition fails.

    The wall vector is the wall normal rotated by 90 degrees. The rotation
    (CCW or CW) is chosen when avoidance starts, as the one pointing towards
    the goal (non-negative dot product with the path direction; the robot's
    heading without a goal), and kept until it ends.

    The wall normal is either
        - "fitted": the surface normal fitted by scan_processor, or
        - "radial": the direction from the obstacle point to the robot.
    go_to_goal derives its wall-following side (sigma) from the same normal,
    which is published in AvoidanceState.obstacle_normal.

    All vectors are in the robot frame (x: forward, y: left).
    """

    WALL_NORMALS = ("fitted", "radial")

    def __init__(self, avoid_distance: float, path_radius: float, wall_normal: str = "fitted"):
        if wall_normal not in self.WALL_NORMALS:
            raise ValueError(f"wall_normal must be one of {self.WALL_NORMALS}, got {wall_normal!r}")
        self.avoid_distance = avoid_distance
        self.path_radius = path_radius
        self.wall_normal = wall_normal

        self.avoiding = False
        # Rotation of the normal giving the wall vector; chosen when avoidance
        # starts and kept until it ends, so the wall side cannot flip midway
        self.ccw = False
        # Wall normal used in the latest update, or None without an obstacle
        self.normal: Optional[np.ndarray] = None

    def update(
        self,
        obstacle_position: Optional[np.ndarray],
        obstacle_normal: Optional[np.ndarray],
        path_direction: np.ndarray = HEADING,
        path_length: float = np.inf,
    ) -> Optional[np.ndarray]:
        """
        Update the avoidance state.

        Args:
            obstacle_position: Closest obstacle point, or None if none is detected.
            obstacle_normal: Unit surface normal at that point (towards the
                             robot), or None if none is detected.
            path_direction: Unit direction of the path zone.
            path_length: Length of the path zone (the goal distance).

        Returns:
            Unit wall vector while avoiding, otherwise None.
        """
        detected = obstacle_position is not None and obstacle_normal is not None
        self.normal = self.select_normal(obstacle_position, obstacle_normal) if detected else None
        avoiding = (
            detected
            and self.normal is not None
            and self.blocks_path(obstacle_position, path_direction, path_length)
        )

        if avoiding and not self.avoiding:
            # Follow the wall towards the goal: use the CCW rotation unless
            # its dot product with the path direction is negative
            self.ccw = bool(rotate_ccw(self.normal) @ path_direction >= 0.0)
        self.avoiding = avoiding

        if not avoiding:
            return None
        wall_vector = rotate_ccw(self.normal)
        return wall_vector if self.ccw else -wall_vector

    def select_normal(self, position: np.ndarray, fitted: np.ndarray) -> Optional[np.ndarray]:
        if self.wall_normal == "fitted":
            return fitted
        distance = np.linalg.norm(position)
        if distance <= 1e-9:
            return None
        return -position / distance

    def blocks_path(
        self, position: np.ndarray, path_direction: np.ndarray, path_length: float = np.inf
    ) -> bool:
        return bool(
            np.linalg.norm(position) < self.avoid_distance
            and in_path_zone(position, path_direction, self.path_radius, path_length)[0]
        )


class ObstacleStateNode(Node):

    def __init__(self):
        super().__init__("obstacle_state")

        # Obstacle readings older than this are ignored, since scan_processor
        # publishes nothing when no obstacle is in view
        self.obstacle_timeout = float(self.declare_parameter("obstacle_timeout", 0.5).value)
        self.goal_direction = GoalDirectionListener(
            self, timeout=float(self.declare_parameter("goal_vector_timeout", 0.5).value)
        )
        self.log = VerboseLogger(self)

        self.obstacle_state = ObstacleState(
            avoid_distance=float(self.declare_parameter("avoid_distance", 0.3).value),
            path_radius=float(self.declare_parameter("path_radius", 0.2).value),
            wall_normal=str(self.declare_parameter("wall_normal", "fitted").value),
        )

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

        obstacle_state = self.obstacle_state
        was_avoiding = obstacle_state.avoiding
        path_direction, path_length = self.goal_direction.path()
        wall_vector = obstacle_state.update(position, normal, path_direction, path_length)

        state = AvoidanceState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.avoiding = obstacle_state.avoiding
        if wall_vector is not None:
            state.wall_vector.x, state.wall_vector.y = (float(v) for v in wall_vector)
        state.ccw = obstacle_state.ccw
        state.obstacle_detected = obstacle_state.normal is not None
        if state.obstacle_detected:
            state.obstacle_position.x, state.obstacle_position.y = (float(v) for v in position)
            state.obstacle_normal.x, state.obstacle_normal.y = (float(v) for v in obstacle_state.normal)
        state.avoid_distance = obstacle_state.avoid_distance
        state.path_radius = obstacle_state.path_radius
        state.path_direction.x, state.path_direction.y = (float(v) for v in path_direction)
        state.path_length = float(path_length)
        self.state_publisher.publish(state)

        if obstacle_state.avoiding != was_avoiding:
            self.log.basic(
                f"Avoiding obstacle at {np.round(position, 3).tolist()}, "
                f"following the wall {'CCW' if obstacle_state.ccw else 'CW'}"
                if obstacle_state.avoiding else "Back to goal following"
            )
        self.log.continuous(
            f"avoiding={state.avoiding} "
            f"obstacle={None if position is None else np.round(position, 3).tolist()} "
            f"normal={None if obstacle_state.normal is None else np.round(obstacle_state.normal, 3).tolist()} "
            f"wall_vector={None if wall_vector is None else np.round(wall_vector, 3).tolist()} "
            f"{'ccw' if state.ccw else 'cw'}",
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
