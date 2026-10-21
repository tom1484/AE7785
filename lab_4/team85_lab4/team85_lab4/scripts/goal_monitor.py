#!/usr/bin/env python3

"""
CS/ME/ECE/AE/BME 7785 - Lab 4

Authors:
Chu-Rong Chen
Xingyu Zhu

Node:
    goal_monitor

Subscribes:
    /odom
      nav_msgs/msg/Odometry
    /goal_status
      team85_lab4_interfaces/msg/GoalStatus

Debug tool: draws the waypoints, the traversed trajectory, and the goal vector
in the world frame (set by the goal node), with a panel showing the goal state
and the robot pose. The view zooms out to keep everything in sight.
Press 'c' to clear the trajectory; 'q' / Esc, close the window, or Ctrl+C to quit.
"""

import os
import sys
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from nav_msgs.msg import Odometry
from team85_lab4_interfaces.msg import GoalStatus

import cv2
import numpy as np

from team85_lab4.verbose_logger import VerboseLogger


# World frame (set by the goal node from its first odometry reading):
#   x: forward at start
#   y: left at start

# Map layout:
#   up:   +x
#   left: +y

WINDOW_NAME = "goal_monitor"
PANEL_WIDTH = 180   # Width of the text panel on the right (pixels)
FONT_SCALE = 0.4
LINE_HEIGHT = 17    # Panel line spacing (pixels)
GRID_STEP = 0.5     # Distance between grid lines (m)
ROBOT_RADIUS = 0.1  # Drawn robot size (m)

GRID_COLOR = (45, 45, 45)
PATH_COLOR = (90, 90, 90)
TRAJECTORY_COLOR = (230, 180, 60)
ROBOT_COLOR = (255, 255, 255)
GOAL_VECTOR_COLOR = (230, 50, 240)
REACHED_COLOR = (75, 180, 60)
CURRENT_COLOR = (25, 225, 255)
UPCOMING_COLOR = (150, 150, 150)
TEXT_COLOR = (220, 220, 220)

QUIT_KEYS = (ord("q"), 27)  # 'q' and Esc
CLEAR_KEY = ord("c")


def yaw_from_quaternion(q) -> float:
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def rotation(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]])


def wrap_angle(angle: float) -> float:
    return float((angle + np.pi) % (2 * np.pi) - np.pi)


class GoalMonitor(Node):

    def __init__(self):
        super().__init__("goal_monitor")

        self.map_size = int(self.declare_parameter("image_size", 480).value)
        # Space kept around the waypoints and trajectory (m)
        self.margin = float(self.declare_parameter("margin", 0.5).value)
        # Minimum distance between recorded trajectory points (m)
        self.trajectory_step = float(self.declare_parameter("trajectory_step", 0.01).value)
        self.log = VerboseLogger(self)

        # Latest pose and trajectory, in the odom frame
        self.position: Optional[np.ndarray] = None
        self.yaw: Optional[float] = None
        self.trajectory: List[np.ndarray] = []

        self.status: Optional[GoalStatus] = None

        self.odom_subscription = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.status_subscription = self.create_subscription(
            GoalStatus, "/goal_status", self.status_callback, 10
        )

        self.running = True
        cv2.namedWindow(WINDOW_NAME)

        self.log.basic("Listening to odometry and goal status...")

    # =============================================================
    # Callbacks
    # =============================================================

    def odom_callback(self, odom: Odometry):
        pose = odom.pose.pose
        self.position = np.array([pose.position.x, pose.position.y])
        self.yaw = yaw_from_quaternion(pose.orientation)

        if (not self.trajectory
                or np.linalg.norm(self.position - self.trajectory[-1]) >= self.trajectory_step):
            self.trajectory.append(self.position)

    def status_callback(self, msg: GoalStatus):
        if self.status is None or self.status.goal_index != msg.goal_index:
            self.log.basic(
                "All goals reached" if msg.finished
                else f"Goal {msg.goal_index + 1}/{len(msg.goals)}"
            )
        self.status = msg

    # =============================================================
    # Frames
    # =============================================================

    def origin(self):
        status = self.status
        return np.array([status.origin_x, status.origin_y]), status.origin_yaw

    def to_world(self, points: np.ndarray) -> np.ndarray:
        # odom -> world: undo the origin shift, then the origin rotation
        origin_position, origin_yaw = self.origin()
        return (np.atleast_2d(points) - origin_position) @ rotation(origin_yaw)

    def fit_view(self, points: np.ndarray):
        """Center and scale (pixels per meter) that fit all points with margin."""
        low = points.min(axis=0) - self.margin
        high = points.max(axis=0) + self.margin
        self.view_center = (low + high) / 2
        self.scale = self.map_size / max(high - low)

    def to_pixel(self, points: np.ndarray) -> np.ndarray:
        # World -> image: +x goes up (-row), +y goes left (-col)
        points = np.atleast_2d(points)
        c = self.map_size / 2
        cols = c - (points[:, 1] - self.view_center[1]) * self.scale
        rows = c - (points[:, 0] - self.view_center[0]) * self.scale
        return np.round(np.stack([cols, rows], axis=1)).astype(np.int32)

    # =============================================================
    # Drawing
    # =============================================================

    def render(self) -> np.ndarray:
        image = np.zeros((self.map_size, self.map_size + PANEL_WIDTH, 3), dtype=np.uint8)
        map_image = image[:, :self.map_size]
        panel = image[:, self.map_size:]

        status = self.status
        if status is None or not status.origin_set:
            cv2.putText(map_image, "Waiting for /goal_status...", (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, FONT_SCALE, TEXT_COLOR, 1)
            return image

        goals = np.array([[g.x, g.y] for g in status.goals]).reshape(-1, 2)
        trajectory = self.to_world(np.array(self.trajectory)) if self.trajectory else np.empty((0, 2))
        self.fit_view(np.vstack([np.zeros((1, 2)), goals, trajectory]))

        self.draw_grid(map_image)
        self.draw_goals(map_image, goals, status)
        self.draw_axes(map_image)
        if len(trajectory) >= 2:
            cv2.polylines(map_image, [self.to_pixel(trajectory)], False, TRAJECTORY_COLOR, 2)

        if self.position is not None:
            position = self.to_world(self.position)[0]
            yaw = wrap_angle(self.yaw - status.origin_yaw)
            if not status.finished:
                self.draw_goal_vector(map_image, position, goals[status.goal_index])
            self.draw_robot(map_image, position, yaw)
        self.draw_panel(panel, goals, status)
        return image

    def draw_grid(self, image: np.ndarray):
        half = self.map_size / 2 / self.scale
        for axis in (0, 1):
            low = self.view_center[axis] - half
            for v in np.arange(np.ceil(low / GRID_STEP), (low + 2 * half) / GRID_STEP + 1) * GRID_STEP:
                ends = np.array([[v, self.view_center[1 - axis] - half],
                                 [v, self.view_center[1 - axis] + half]])
                if axis == 1:
                    ends = ends[:, ::-1]
                a, b = self.to_pixel(ends)
                cv2.line(image, tuple(map(int, a)), tuple(map(int, b)), GRID_COLOR, 1)

    def draw_axes(self, image: np.ndarray):
        # World axes at the origin: +x (red), +y (green)
        o, x, y = self.to_pixel(np.array([[0.0, 0.0], [GRID_STEP, 0.0], [0.0, GRID_STEP]]))
        cv2.arrowedLine(image, tuple(map(int, o)), tuple(map(int, x)), (0, 0, 255), 2, tipLength=0.15)
        cv2.arrowedLine(image, tuple(map(int, o)), tuple(map(int, y)), (0, 255, 0), 2, tipLength=0.15)

    def draw_goals(self, image: np.ndarray, goals: np.ndarray, status: GoalStatus):
        # Planned path: origin -> goal 1 -> goal 2 -> ...
        path = self.to_pixel(np.vstack([np.zeros((1, 2)), goals]))
        cv2.polylines(image, [path], False, PATH_COLOR, 1)

        tolerance_px = max(int(round(status.goal_tolerance * self.scale)), 3)
        for i, pixel in enumerate(path[1:]):
            if i < status.goal_index:
                color = REACHED_COLOR
            elif i == status.goal_index:
                color = CURRENT_COLOR
            else:
                color = UPCOMING_COLOR
            center = tuple(map(int, pixel))
            cv2.circle(image, center, tolerance_px, color, 2 if i == status.goal_index else 1)
            cv2.circle(image, center, 3, color, -1)
            cv2.putText(image, str(i + 1), (center[0] + 6, center[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, FONT_SCALE, color, 1)

    def draw_goal_vector(self, image: np.ndarray, position: np.ndarray, goal: np.ndarray):
        start, end = self.to_pixel(np.array([position, goal]))
        cv2.arrowedLine(image, tuple(map(int, start)), tuple(map(int, end)),
                        GOAL_VECTOR_COLOR, 2, tipLength=min(0.1, 10 / max(np.linalg.norm(end - start), 1)))

    def draw_robot(self, image: np.ndarray, position: np.ndarray, yaw: float):
        heading = position + ROBOT_RADIUS * 1.8 * np.array([np.cos(yaw), np.sin(yaw)])
        center, tip = self.to_pixel(np.array([position, heading]))
        radius = max(int(round(ROBOT_RADIUS * self.scale)), 4)
        cv2.circle(image, tuple(map(int, center)), radius, ROBOT_COLOR, 2)
        cv2.line(image, tuple(map(int, center)), tuple(map(int, tip)), ROBOT_COLOR, 2)

    def draw_panel(self, panel: np.ndarray, goals: np.ndarray, status: GoalStatus):
        panel[:] = (25, 25, 25)
        lines = []

        if status.finished:
            lines.append(("All goals reached", REACHED_COLOR))
        else:
            goal = goals[status.goal_index]
            holding = status.hold_elapsed > 0.0
            lines += [
                (f"Goal {status.goal_index + 1}/{len(goals)}", CURRENT_COLOR),
                (f"  at ({goal[0]:.2f}, {goal[1]:.2f})", TEXT_COLOR),
                (f"  distance {status.distance:.3f} m", TEXT_COLOR),
                (f"  tolerance {status.goal_tolerance:.3f} m", TEXT_COLOR),
                (f"  hold {status.hold_elapsed:.1f} / {status.hold_time:.1f} s"
                 if holding else "  not at goal", CURRENT_COLOR if holding else TEXT_COLOR),
            ]
        lines.append(("", TEXT_COLOR))

        if self.position is not None:
            position = self.to_world(self.position)[0]
            yaw = wrap_angle(self.yaw - status.origin_yaw)
            lines += [
                ("Robot (world)", ROBOT_COLOR),
                (f"  x {position[0]:7.3f} m", TEXT_COLOR),
                (f"  y {position[1]:7.3f} m", TEXT_COLOR),
                (f"  heading {np.degrees(yaw):6.1f} deg", TEXT_COLOR),
            ]
            if not status.finished:
                # Goal vector in the robot frame (x: forward, y: left)
                vector = rotation(yaw).T @ (goals[status.goal_index] - position)
                lines += [
                    ("", TEXT_COLOR),
                    ("Goal vector (robot)", GOAL_VECTOR_COLOR),
                    (f"  x {vector[0]:7.3f} m", TEXT_COLOR),
                    (f"  y {vector[1]:7.3f} m", TEXT_COLOR),
                    (f"  angle {np.degrees(np.arctan2(vector[1], vector[0])):6.1f} deg", TEXT_COLOR),
                ]
        else:
            lines.append(("Waiting for /odom...", TEXT_COLOR))

        lines += [("", TEXT_COLOR), ("c: clear trail   q: quit", UPCOMING_COLOR)]
        for i, (text, color) in enumerate(lines):
            cv2.putText(panel, text, (10, 22 + LINE_HEIGHT * i), cv2.FONT_HERSHEY_SIMPLEX,
                        FONT_SCALE, color, 1)

    def update_window(self):
        # Runs in the main loop, so the window keeps responding to keys and the
        # close button even when no messages arrive.
        cv2.imshow(WINDOW_NAME, self.render())
        key = cv2.waitKey(1) & 0xFF
        if key == CLEAR_KEY:
            self.trajectory = self.trajectory[-1:]
        closed = cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1
        if key in QUIT_KEYS or closed:
            self.running = False


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = GoalMonitor()
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.03)
            node.update_window()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        cv2.waitKey(1)  # Let the GUI process the close (needed on macOS)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

    # Skip the interpreter's normal teardown (see view_scan): avoids OpenCV's
    # Qt backend warning at exit. All cleanup is already done above.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
