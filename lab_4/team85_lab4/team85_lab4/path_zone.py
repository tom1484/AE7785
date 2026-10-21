from typing import Optional, Tuple

import numpy as np

from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Vector3Stamped


# Robot heading in the robot frame (x: forward, y: left)
HEADING = np.array([1.0, 0.0])


def path_direction(goal_vector: Optional[np.ndarray], epsilon: float = 1e-6) -> np.ndarray:
    """
    Unit direction of the robot's path: towards the goal, or the robot's
    heading when there is no goal (or the robot is at it).
    """
    if goal_vector is None:
        return HEADING
    norm = float(np.linalg.norm(goal_vector))
    if norm <= epsilon:
        return HEADING
    return np.asarray(goal_vector, dtype=float) / norm


def path_coordinates(points: np.ndarray, direction: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Coordinates of points relative to the path line through the robot:
    along: distance along the path direction (positive: ahead on the path),
    lateral: signed distance from the path line (positive: left of it).
    """
    points = np.atleast_2d(points)
    along = points @ direction
    lateral = direction[0] * points[:, 1] - direction[1] * points[:, 0]
    return along, lateral


def in_path_zone(
    points: np.ndarray,
    direction: np.ndarray,
    path_radius: float,
    path_length: float = np.inf,
) -> np.ndarray:
    """
    Points in the strip ahead along the path, within path_radius of the path
    line, and short of path_length (the goal distance) along it.
    """
    along, lateral = path_coordinates(points, direction)
    return (along > 0.0) & (along < path_length) & (np.abs(lateral) < path_radius)


class GoalDirectionListener:
    """
    Latest /goal_vector (robot frame, published by the goal node) as the path:
    its direction, and its length (the goal distance) capping the zone. Falls
    back to the robot's heading with an unbounded zone without a goal vector
    newer than timeout (before the first goal, after the last, or when stale).
    """

    def __init__(self, node: Node, timeout: float):
        self.node = node
        self.timeout = timeout
        self.goal_vector: Optional[np.ndarray] = None
        self.received: Optional[Time] = None
        self.subscription = node.create_subscription(
            Vector3Stamped, "/goal_vector", self.callback, 10
        )

    def callback(self, msg: Vector3Stamped):
        self.goal_vector = np.array([msg.vector.x, msg.vector.y])
        self.received = self.node.get_clock().now()

    def latest_goal_vector(self) -> Optional[np.ndarray]:
        if self.received is None:
            return None
        age = (self.node.get_clock().now() - self.received).nanoseconds * 1e-9
        return self.goal_vector if age <= self.timeout else None

    def path(self) -> Tuple[np.ndarray, float]:
        """Unit path direction and path length (np.inf without a goal)."""
        goal_vector = self.latest_goal_vector()
        direction = path_direction(goal_vector)
        if goal_vector is None or direction is HEADING:
            return direction, np.inf
        return direction, float(np.linalg.norm(goal_vector))
