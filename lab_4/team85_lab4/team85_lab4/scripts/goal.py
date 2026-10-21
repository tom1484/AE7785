#!/usr/bin/env python3

"""
CS/ME/ECE/AE/BME 7785 - Lab 4

Authors:
Chu-Rong Chen
Xingyu Zhu

Node:
    goal

Subscribes:
    /odom
      nav_msgs/msg/Odometry

Publishes:
    /goal
      geometry_msgs/msg/PointStamped
      Current goal position in the odom frame.
    /goal_vector
      geometry_msgs/msg/Vector3Stamped
      Vector from the robot to the current goal, in the robot frame
      (x: forward, y: left). Not published once all goals are reached.
    /goal_status
      team85_lab4_interfaces/msg/GoalStatus
      World origin, waypoints, and progress on the current goal.

Reads the waypoints and runs GoalState on every odometry reading: the first
reading defines the world frame, and the stage advances as goals are reached.
"""

import os
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PointStamped, Vector3Stamped
from team85_lab4_interfaces.msg import GoalStatus

import numpy as np

from team85_lab4.verbose_logger import VerboseLogger


def yaw_from_quaternion(q) -> float:
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def load_waypoints(path: str) -> np.ndarray:
    """Read one "x y" world position per line, ignoring blank lines."""
    waypoints = []
    with open(path) as f:
        for line in f:
            if line.strip():
                x, y = line.split()[:2]
                waypoints.append([float(x), float(y)])
    return np.array(waypoints, dtype=float).reshape(-1, 2)


def rotation(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]])


class GoalState:
    """
    Top-level stage management: which goal is current, when to switch to the
    next one, and the vector to the current goal.

    Frames:
        world: Defined by the pose given to the first update()
               (x: forward, y: left at start). Waypoints are in this frame.
        odom:  Frame of the poses given to update(). Goals are converted into
               this frame once, so the goal vector is (goal - position).
        robot: x: forward, y: left, rotating with the robot.
    """

    def __init__(self, waypoints_file: str, goal_tolerance: float = 0.03, hold_time: float = 3.0):
        self.goals_world = load_waypoints(waypoints_file)
        # A goal is reached within this distance (m)
        self.goal_tolerance = goal_tolerance
        # The robot must stay at the goal this long before switching (s)
        self.hold_time = hold_time
        # Time the robot entered the current goal's tolerance, or None if outside
        self.reached_since: Optional[float] = None
        self.goals_odom: Optional[np.ndarray] = None
        # Pose of the world frame in the odom frame, set by the first update()
        self.origin_position: Optional[np.ndarray] = None
        self.origin_yaw: Optional[float] = None
        self.goal_index = 0

    @property
    def num_goals(self) -> int:
        return len(self.goals_world)

    @property
    def origin_set(self) -> bool:
        return self.goals_odom is not None

    @property
    def finished(self) -> bool:
        return self.goal_index >= self.num_goals

    @property
    def current_goal(self) -> Optional[np.ndarray]:
        """Current goal in the odom frame, or None before the origin is set / when finished."""
        if not self.origin_set or self.finished:
            return None
        return self.goals_odom[self.goal_index]

    @property
    def next_goal(self) -> Optional[np.ndarray]:
        """Goal after the current one in the odom frame, or None if there is none."""
        if not self.origin_set or self.goal_index + 1 >= self.num_goals:
            return None
        return self.goals_odom[self.goal_index + 1]

    def update(self, position: np.ndarray, yaw: float, time: float) -> Optional[np.ndarray]:
        """
        Advance the stage given the current pose, and return the goal vector.

        Args:
            position: Current [x, y] in the odom frame.
            yaw: Current heading (radians) in the odom frame.
            time: Current time (s), used to time the hold at the goal.

        Returns:
            Vector from the robot to the current goal, in the robot frame,
            or None when all goals are reached.
        """
        position = np.asarray(position, dtype=float)

        # The first pose defines the world frame
        if not self.origin_set:
            self.set_world_origin(position, yaw)

        if not self.finished and self.should_switch(position, time):
            self.goal_index += 1
            self.reached_since = None

        if self.finished:
            return None

        goal_vector = self.current_goal - position
        # Undo the robot's current heading
        return rotation(yaw).T @ goal_vector

    def set_world_origin(self, position: np.ndarray, yaw: float):
        self.origin_position, self.origin_yaw = position, yaw
        # world -> odom: rotate by the starting yaw, then shift by the starting position
        self.goals_odom = self.goals_world @ rotation(yaw).T + position

    def distance_to_goal(self, position: np.ndarray) -> Optional[float]:
        """Distance (m) to the current goal, or None if there is none."""
        if self.current_goal is None:
            return None
        return float(np.linalg.norm(self.current_goal - position))

    def should_switch(self, position: np.ndarray, time: float) -> bool:
        """Whether the robot has stayed within tolerance of the goal for hold_time."""
        if self.distance_to_goal(position) > self.goal_tolerance:
            # Left the goal: restart the hold
            self.reached_since = None
            return False

        if self.reached_since is None:
            self.reached_since = time
        return time - self.reached_since >= self.hold_time


class Goal(Node):

    def __init__(self):
        super().__init__("goal")

        default_waypoints = os.path.join(
            get_package_share_directory("team85_lab4"), "wayPoints.txt"
        )
        waypoints_file = str(self.declare_parameter("waypoints_file", default_waypoints).value)
        goal_tolerance = float(self.declare_parameter("goal_tolerance", 0.03).value)
        hold_time = float(self.declare_parameter("hold_time", 3.0).value)
        self.log = VerboseLogger(self)

        self.goal_state = GoalState(waypoints_file, goal_tolerance, hold_time)

        self.odom_subscription = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.goal_publisher = self.create_publisher(PointStamped, "/goal", 10)
        self.goal_vector_publisher = self.create_publisher(Vector3Stamped, "/goal_vector", 10)
        self.status_publisher = self.create_publisher(GoalStatus, "/goal_status", 10)

        self.log.basic(
            f"Loaded {self.goal_state.num_goals} waypoints from {waypoints_file}: "
            f"{self.goal_state.goals_world.tolist()}"
        )
        self.log.basic("Waiting for the first odometry to set the world origin...")

    def odom_callback(self, odom: Odometry):
        pose = odom.pose.pose
        position = np.array([pose.position.x, pose.position.y])
        yaw = yaw_from_quaternion(pose.orientation)

        state = self.goal_state
        origin_was_set = state.origin_set
        previous_index = state.goal_index

        now = self.get_clock().now().nanoseconds * 1e-9
        goal_vector = state.update(position, yaw, now)
        self.log_stage_changes(origin_was_set, previous_index, position, yaw)
        self.publish_status(odom, position, now)

        # TODO: Tell go_to_goal when all goals are reached. For now nothing is
        #       published once finished.
        if state.finished:
            return

        goal = PointStamped()
        goal.header = odom.header
        goal.point.x, goal.point.y = (float(v) for v in state.current_goal)
        self.goal_publisher.publish(goal)

        vector = Vector3Stamped()
        vector.header.stamp = odom.header.stamp
        vector.header.frame_id = odom.child_frame_id
        vector.vector.x, vector.vector.y = (float(v) for v in goal_vector)
        self.goal_vector_publisher.publish(vector)

        self.log.continuous(
            f"goal {state.goal_index + 1}/{state.num_goals} "
            f"at ({goal.point.x:.3f}, {goal.point.y:.3f}) (odom), "
            f"distance {state.distance_to_goal(position):.3f} m"
            f"{'' if state.reached_since is None else ' (holding)'}",
            throttle_duration_sec=0.5,
        )

    def publish_status(self, odom: Odometry, position: np.ndarray, now: float):
        state = self.goal_state

        status = GoalStatus()
        status.header = odom.header
        status.origin_set = state.origin_set
        if state.origin_set:
            status.origin_x, status.origin_y = (float(v) for v in state.origin_position)
            status.origin_yaw = float(state.origin_yaw)
        status.goals = [Point(x=float(x), y=float(y)) for x, y in state.goals_world]
        status.goal_index = state.goal_index
        status.finished = state.finished
        status.distance = state.distance_to_goal(position) or 0.0
        status.goal_tolerance = state.goal_tolerance
        status.hold_elapsed = 0.0 if state.reached_since is None else now - state.reached_since
        status.hold_time = state.hold_time
        self.status_publisher.publish(status)

    def log_stage_changes(self, origin_was_set, previous_index, position, yaw):
        state = self.goal_state
        if not origin_was_set:
            self.log.basic(
                f"World origin set at odom ({position[0]:.3f}, {position[1]:.3f}), "
                f"yaw {np.degrees(yaw):.1f} deg. Goals in odom: "
                f"{np.round(state.goals_odom, 3).tolist()}"
            )
        if state.goal_index != previous_index:
            if state.finished:
                self.log.basic("All goals reached")
            else:
                self.log.basic(
                    f"Switched to goal {state.goal_index + 1}/{state.num_goals}: "
                    f"{np.round(state.current_goal, 3).tolist()} (odom)"
                )


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = Goal()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
