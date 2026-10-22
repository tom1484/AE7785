#!/usr/bin/env python3

"""
CS/ME/ECE/AE/BME 7785 - Lab 4

Authors:
Chu-Rong Chen
Xingyu Zhu

Node:
    avoidance_monitor

Subscribes:
    /avoidance_state
      team85_lab4_interfaces/msg/AvoidanceState
    /virtual_target
      team85_lab4_interfaces/msg/VirtualTarget
    /scan_segments
      team85_lab4_interfaces/msg/ScanSegments
      Drawn faintly for context.
    /pause
      std_msgs/msg/Bool (transient local)
      Whether low_level_controller is paused, from any publisher.

Publishes:
    /pause
      std_msgs/msg/Bool (transient local)
      Toggled with the space key: true pauses low_level_controller, false
      resumes it.

Debug tool: draws what obstacle_state decides on, in the robot frame: the
path zone (along the goal direction) and avoid distance, the obstacle point with its normal (white), the
wall direction (magenta) while avoiding, and go_to_goal's virtual target
(cyan): a ray towards the virtual heading, and a point on the robot's x axis
at the signed virtual distance (behind the robot when backing up). A panel shows the state, each avoidance
criterion with its value and threshold (green: holds, red: fails), and the
virtual target with its steering terms.
Press space to pause / resume the robot (low_level_controller stops it,
while go_to_goal keeps planning, so the virtual target is still drawn).
Press 'q' / Esc, close the window, or Ctrl+C to quit.
"""

import math
import os
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time

from std_msgs.msg import Bool

from team85_lab4_interfaces.msg import AvoidanceState, ScanSegments, VirtualTarget

import cv2
import numpy as np

from team85_lab4.path_zone import path_coordinates
from team85_lab4.verbose_logger import VerboseLogger


# Robot frame (identical to the LiDAR frame):
#   x: forward
#   y: left

# Map layout:
#   center: robot
#   up:     +x
#   left:   +y

WINDOW_NAME = "avoidance_monitor"
PANEL_WIDTH = 210   # Width of the text panel on the right (pixels)
FONT_SCALE = 0.4
LINE_HEIGHT = 17    # Panel line spacing (pixels)
RING_STEP = 0.25    # Distance between range rings (m)
ROBOT_RADIUS = 0.105  # Drawn robot size (m)
ARROW_LENGTH = 0.2  # Length of the normal and wall arrows (m)
STATE_TIMEOUT = 0.5  # Show the state as missing after this long without one (s)
VT_RAY_LENGTH = 0.3  # Drawn length of the virtual heading ray (m)

RING_COLOR = (45, 45, 45)
SEGMENT_COLOR = (90, 90, 90)
ZONE_COLOR = (0, 120, 120)
AVOID_COLOR = (0, 90, 200)
ROBOT_COLOR = (200, 200, 200)
OBSTACLE_COLOR = (255, 255, 255)
WALL_COLOR = (230, 50, 240)
VT_COLOR = (230, 220, 40)
PASS_COLOR = (75, 180, 60)
FAIL_COLOR = (60, 60, 230)
TEXT_COLOR = (220, 220, 220)
DIM_COLOR = (150, 150, 150)

QUIT_KEYS = (ord("q"), 27)  # 'q' and Esc
PAUSE_KEY = ord(" ")

# Same as low_level_controller: the last value is kept for late subscribers
PAUSE_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class AvoidanceMonitor(Node):

    def __init__(self):
        super().__init__("avoidance_monitor")

        self.map_size = int(self.declare_parameter("image_size", 480).value)
        # Distance from the robot to the map border (m)
        self.max_range = float(self.declare_parameter("max_range", 1.0).value)
        self.log = VerboseLogger(self)

        # Pixels per meter, so that max_range reaches the map border
        self.scale = (self.map_size / 2) / self.max_range

        self.state: Optional[AvoidanceState] = None
        self.state_received: Optional[Time] = None
        self.segments: Optional[ScanSegments] = None
        self.target: Optional[VirtualTarget] = None
        self.target_received: Optional[Time] = None
        # Last value on /pause (ours or another publisher's); None until any
        self.paused: Optional[bool] = None

        self.target_subscription = self.create_subscription(
            VirtualTarget, "/virtual_target", self.target_callback, 10
        )
        self.state_subscription = self.create_subscription(
            AvoidanceState, "/avoidance_state", self.state_callback, 10
        )
        self.segments_subscription = self.create_subscription(
            ScanSegments, "/scan_segments", self.segments_callback, 10
        )
        self.pause_subscription = self.create_subscription(
            Bool, "/pause", self.pause_callback, PAUSE_QOS
        )
        self.pause_publisher = self.create_publisher(Bool, "/pause", PAUSE_QOS)

        self.running = True
        cv2.namedWindow(WINDOW_NAME)

        self.log.basic("Listening to avoidance state...")

    # =============================================================
    # Callbacks
    # =============================================================

    def state_callback(self, msg: AvoidanceState):
        if self.state is not None and msg.avoiding != self.state.avoiding:
            self.log.basic(
                f"Avoiding ({'CCW' if msg.ccw else 'CW'})" if msg.avoiding
                else "Back to goal following"
            )
        self.state = msg
        self.state_received = self.get_clock().now()

    def segments_callback(self, msg: ScanSegments):
        self.segments = msg

    def target_callback(self, msg: VirtualTarget):
        self.target = msg
        self.target_received = self.get_clock().now()

    def pause_callback(self, msg: Bool):
        self.paused = msg.data

    def toggle_pause(self):
        paused = not self.paused
        self.pause_publisher.publish(Bool(data=paused))
        self.paused = paused
        self.log.basic("Pause sent" if paused else "Resume sent")

    def is_fresh(self, received: Optional[Time]) -> bool:
        if received is None:
            return False
        age = (self.get_clock().now() - received).nanoseconds * 1e-9
        return age <= STATE_TIMEOUT

    # =============================================================
    # Drawing
    # =============================================================

    def to_pixel(self, points: np.ndarray) -> np.ndarray:
        # Robot frame -> image: +x goes up (-row), +y goes left (-col)
        points = np.atleast_2d(points)
        c = self.map_size / 2
        cols = c - points[:, 1] * self.scale
        rows = c - points[:, 0] * self.scale
        return np.round(np.stack([cols, rows], axis=1)).astype(np.int32)

    def pixel(self, point: np.ndarray):
        return tuple(int(v) for v in self.to_pixel(point)[0])

    def render(self) -> np.ndarray:
        image = np.zeros((self.map_size, self.map_size + PANEL_WIDTH, 3), dtype=np.uint8)
        map_image = image[:, :self.map_size]
        panel = image[:, self.map_size:]
        panel[:] = (25, 25, 25)

        state = self.state if self.is_fresh(self.state_received) else None
        target = self.target if self.is_fresh(self.target_received) else None

        self.draw_background(map_image)
        if state is not None:
            self.draw_criteria_zone(map_image, state)
        self.draw_segments(map_image)
        self.draw_robot(map_image)
        if state is not None:
            self.draw_obstacle(map_image, state)
        if target is not None:
            self.draw_target(map_image, target)

        self.draw_panel(panel, state, target)
        return image

    def draw_background(self, image: np.ndarray):
        center = self.pixel(np.zeros(2))
        for r in np.arange(RING_STEP, self.max_range * np.sqrt(2), RING_STEP):
            cv2.circle(image, center, int(round(r * self.scale)), RING_COLOR, 1)
            cv2.putText(image, f"{r:.2f}", (center[0] + 3, center[1] - int(round(r * self.scale)) - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, DIM_COLOR, 1)

    def draw_criteria_zone(self, image: np.ndarray, state: AvoidanceState):
        # Path zone: ahead along the path direction, within path_radius of it
        direction = np.array([state.path_direction.x, state.path_direction.y])
        side = np.array([-direction[1], direction[0]])
        length = min(state.path_length, self.max_range * 2)
        for offset in (-state.path_radius, state.path_radius):
            start = side * offset
            cv2.line(image, self.pixel(start), self.pixel(start + direction * length), ZONE_COLOR, 1)
        if math.isfinite(state.path_length):
            # Capped at the goal
            end = direction * state.path_length
            cv2.line(image, self.pixel(end - side * state.path_radius),
                     self.pixel(end + side * state.path_radius), ZONE_COLOR, 1)

        # Avoid distance, on the path side of the robot
        heading = np.arctan2(direction[1], direction[0])
        angles = heading + np.linspace(-np.pi / 2, np.pi / 2, 61)
        arc = state.avoid_distance * np.stack([np.cos(angles), np.sin(angles)], axis=1)
        cv2.polylines(image, [self.to_pixel(arc)], False, AVOID_COLOR, 1)

    def draw_segments(self, image: np.ndarray):
        if self.segments is None:
            return
        for segment in self.segments.segments:
            for p in segment.points:
                cv2.circle(image, self.pixel(np.array([p.x, p.y])), 1, SEGMENT_COLOR, -1)

    def draw_robot(self, image: np.ndarray):
        center = self.pixel(np.zeros(2))
        cv2.circle(image, center, max(int(round(ROBOT_RADIUS * self.scale)), 4), ROBOT_COLOR, 1)
        cv2.line(image, center, self.pixel(np.array([ROBOT_RADIUS * 1.5, 0.0])), ROBOT_COLOR, 2)

    def draw_obstacle(self, image: np.ndarray, state: AvoidanceState):
        if not state.obstacle_detected:
            return

        position = np.array([state.obstacle_position.x, state.obstacle_position.y])
        normal = np.array([state.obstacle_normal.x, state.obstacle_normal.y])
        start = self.pixel(position)

        cv2.circle(image, start, 6, OBSTACLE_COLOR, 1)
        cv2.arrowedLine(image, start, self.pixel(position + normal * ARROW_LENGTH),
                        OBSTACLE_COLOR, 2, tipLength=0.2)

        if state.avoiding:
            # Wall direction, along the wall
            wall = np.array([state.wall_vector.x, state.wall_vector.y])
            cv2.arrowedLine(image, start, self.pixel(position + wall * ARROW_LENGTH),
                            WALL_COLOR, 2, tipLength=0.2)

    def draw_target(self, image: np.ndarray, target: VirtualTarget):
        if target.status != "OK":
            return
        center = self.pixel(np.zeros(2))

        # Virtual heading: a ray towards theta_vt
        direction = np.array([np.cos(target.theta), np.sin(target.theta)])
        cv2.arrowedLine(image, center, self.pixel(direction * VT_RAY_LENGTH),
                        VT_COLOR, 2, tipLength=0.15)

        # Virtual distance: a signed forward/backward reference, so a point on
        # the robot's x axis (behind the robot when backing up)
        point = self.pixel(np.array([target.distance, 0.0]))
        cv2.line(image, center, point, VT_COLOR, 1)
        cv2.circle(image, point, 5, VT_COLOR, -1)

    def draw_panel(self, panel: np.ndarray, state: Optional[AvoidanceState],
                   target: Optional[VirtualTarget]):
        lines = []

        if self.paused:
            lines += [("PAUSED (space: resume)", FAIL_COLOR), ("", TEXT_COLOR)]

        if state is None:
            lines.append(("No /avoidance_state", FAIL_COLOR))
        else:
            if state.avoiding:
                lines.append((f"AVOIDING ({'CCW' if state.ccw else 'CW'})", WALL_COLOR))
            else:
                lines.append(("GOAL FOLLOWING", PASS_COLOR))
            lines.append(("", TEXT_COLOR))

            lines.append(("Criteria (all to avoid)", TEXT_COLOR))
            if state.obstacle_detected:
                position = np.array([state.obstacle_position.x, state.obstacle_position.y])
                direction = np.array([state.path_direction.x, state.path_direction.y])
                distance = float(np.linalg.norm(position))
                along, lateral = (float(v[0]) for v in path_coordinates(position, direction))
                criteria = [
                    (f"dist {distance:.3f} < {state.avoid_distance:.3f}",
                     distance < state.avoid_distance),
                    (f"along 0 < {along:.3f} < {state.path_length:.2f}",
                     0.0 < along < state.path_length),
                    (f"|lat| {abs(lateral):.3f} < {state.path_radius:.3f}",
                     abs(lateral) < state.path_radius),
                ]
                lines.append(("  obstacle detected", PASS_COLOR))
                lines += [(f"  {text}", PASS_COLOR if ok else FAIL_COLOR) for text, ok in criteria]
            else:
                lines.append(("  no obstacle", FAIL_COLOR))
            lines.append(("", TEXT_COLOR))

            if state.obstacle_detected:
                lines += [
                    ("Obstacle (robot)", OBSTACLE_COLOR),
                    (f"  pos ({state.obstacle_position.x:.3f}, {state.obstacle_position.y:.3f})", TEXT_COLOR),
                    (f"  normal ({state.obstacle_normal.x:.2f}, {state.obstacle_normal.y:.2f})", TEXT_COLOR),
                ]
            if state.avoiding:
                lines.append((f"Wall ({state.wall_vector.x:.2f}, {state.wall_vector.y:.2f}) "
                              f"{'CCW' if state.ccw else 'CW'}", WALL_COLOR))
        lines.append(("", TEXT_COLOR))

        if target is None:
            lines.append(("No /virtual_target", FAIL_COLOR))
        else:
            ok = target.status == "OK"
            lines += [
                (f"VT: {target.status}", VT_COLOR if ok else FAIL_COLOR),
                (f"  theta {np.degrees(target.theta):6.1f} deg", TEXT_COLOR),
                (f"  dist {target.distance:.3f} (raw {target.raw_distance:.3f})", TEXT_COLOR),
            ]
            if ok:
                lines += [
                    ("Steering", TEXT_COLOR),
                    (f"  u_gtg {target.u_gtg_theta:7.3f}", TEXT_COLOR),
                    (f"  u_fw  {target.u_fw_theta:7.3f}", TEXT_COLOR),
                    (f"  sigma {target.fw_sigma:+.0f} z {target.fw_error_z:.2f} "
                     f"g {target.fw_recovery_gamma:.2f}", TEXT_COLOR if target.enable_fw else DIM_COLOR),
                    (f"  fw weight {target.fw_proximity_weight:.2f}",
                     TEXT_COLOR if target.enable_fw else DIM_COLOR),
                    ("Distance", TEXT_COLOR),
                    (f"  gtg {target.u_gtg_distance:.2f} x cos {target.gtg_cosine:.2f}", TEXT_COLOR),
                    (f"  ao  {target.u_ao_distance:.2f} x cos {target.ao_cosine:.2f}",
                     TEXT_COLOR if target.enable_ao else DIM_COLOR),
                    (f"  fw  {target.u_fw_distance:.2f} x cos {target.fw_cosine:.2f}",
                     TEXT_COLOR if target.enable_fw else DIM_COLOR),
                ]

        lines += [("", TEXT_COLOR), ("space: pause/resume  q: quit", DIM_COLOR)]
        for i, (text, color) in enumerate(lines):
            cv2.putText(panel, text, (10, 22 + LINE_HEIGHT * i), cv2.FONT_HERSHEY_SIMPLEX,
                        FONT_SCALE, color, 1)

    def update_window(self):
        # Runs in the main loop, so the window keeps responding to keys and the
        # close button even when no messages arrive.
        cv2.imshow(WINDOW_NAME, self.render())
        key = cv2.waitKey(1) & 0xFF
        closed = cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1
        if key == PAUSE_KEY:
            self.toggle_pause()
        if key in QUIT_KEYS or closed:
            self.running = False


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = AvoidanceMonitor()
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
