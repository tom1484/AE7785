#!/usr/bin/env python3

"""
CS/ME/ECE/AE/BME 7785 - Lab 4

Authors:
Chu-Rong Chen
Xingyu Zhu

Node:
    go_to_goal

Subscribes:
    /odom
      nav_msgs/msg/Odometry
      Needed to express /goal (odom frame) relative to the robot.
    /goal
      geometry_msgs/msg/PointStamped
    /goal_status
      team85_lab4_interfaces/msg/GoalStatus
      Used to stop once all goals are reached.
    /avoidance_state
      team85_lab4_interfaces/msg/AvoidanceState
      Whether to avoid, the obstacle, and the wall-following vector (robot frame).

Publishes:
    /virtual_target
      team85_lab4_interfaces/msg/VirtualTarget
      Robot-relative virtual heading and distance, with diagnostics.

Runs the virtual target planner (README.md): heading from go-to-goal (GTG)
and follow-wall (FW), distance from GTG, avoid-obstacle (AO) and FW. FW and AO
are both enabled while obstacle_state is avoiding. The low_level_controller
node turns the virtual target into velocities.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from team85_lab4_interfaces.msg import AvoidanceState, GoalStatus
from team85_lab4_interfaces.msg import VirtualTarget as VirtualTargetMsg

import numpy as np

from team85_lab4.verbose_logger import VerboseLogger


CONTROL_PERIOD = 0.05  # Seconds between control updates (20 Hz)


def yaw_from_quaternion(q) -> float:
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def rotation(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]])


def wrap_angle(angle: float) -> float:
    """Wrap to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def cross2d(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


# =============================================================
# Virtual target planner
# =============================================================

@dataclass
class PlannerConfig:
    """Gains and limits of the virtual target planner (README.md, Parameters)."""
    k_gtg_theta: float             # GTG heading gain
    k_fw: float                    # FW steering gain
    s_fw: float                    # FW exponential sensitivity
    s_fw_distance: float           # FW proximity decay rate beyond l_d (1/m)
    delta: float                   # GTG recovery angular width (rad)
    k_gtg_distance: float          # GTG distance gain (1/m)
    u_gtg_min: float               # Minimum GTG attraction, so it does not fade near the goal
    u_gtg_max: float               # Maximum GTG attraction
    u_ao_max: float                # Maximum AO repulsion
    s_ao: float                    # AO exponential decay rate (1/m)
    l_d: float                     # Danger distance (m)
    u_fw_distance: float           # Constant FW drive along the wall
    distance_scale: float          # Arctangent distance scaling (m)
    max_reference_distance: float  # Maximum absolute virtual distance (m)
    goal_tolerance: float          # Goal reached within this distance (m)
    geometry_epsilon: float        # Minimum valid geometry norm
    min_tangent_cross: float       # Minimum valid |tangent x normal|
    max_fw_exponent: float         # FW exponential overflow guard
    forward_only: bool             # Disable negative distance references

    def __post_init__(self):
        positive = [
            "k_gtg_theta", "k_fw", "s_fw", "k_gtg_distance", "u_gtg_max", "u_ao_max",
            "s_ao", "l_d", "distance_scale", "max_reference_distance",
            "geometry_epsilon", "max_fw_exponent",
        ]
        for name in positive:
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"{name} must be finite and > 0, got {value}")
        if self.u_gtg_min > self.u_gtg_max:
            raise ValueError(f"u_gtg_min ({self.u_gtg_min}) must not exceed u_gtg_max ({self.u_gtg_max})")
        if not (0.0 < self.delta < math.pi):
            raise ValueError(f"delta must be in (0, pi), got {self.delta}")
        for name in ("goal_tolerance", "u_fw_distance", "s_fw_distance", "u_gtg_min"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f"{name} must be finite and >= 0, got {value}")
        if not (0.0 < self.min_tangent_cross < 1.0):
            raise ValueError(f"min_tangent_cross must be in (0, 1), got {self.min_tangent_cross}")


@dataclass
class Target:
    """Planner output: virtual heading (rad, positive left) and distance (m)."""
    theta_vt: float
    distance_vt: float
    status: str
    diagnostics: Dict[str, float] = field(default_factory=dict)


DIAGNOSTICS = (
    "u_gtg_theta", "u_fw_theta", "u_gtg_distance", "u_ao_distance", "u_fw_distance",
    "gtg_cosine", "ao_cosine", "fw_cosine", "fw_sigma", "fw_error_z", "fw_recovery_gamma",
    "fw_proximity_weight", "raw_distance_vt",
)


def safe_invalid_target(status: str) -> Target:
    """Zero heading and distance, with the reason in the status."""
    return Target(0.0, 0.0, status, {name: 0.0 for name in DIAGNOSTICS})


class VirtualTargetPlanner:
    """
    Virtual target planner (README.md): combines GTG and FW into the virtual
    heading, and GTG, AO and FW into the virtual distance. It does not decide when
    FW and AO are enabled, nor compute velocity commands.

    All inputs must be in one frame. The FW normal, used for the steering sign
    sigma, can be given explicitly (wall_normal) so that it matches the normal
    the wall tangent was built from; otherwise it is the obstacle-to-robot
    direction. AO always uses the obstacle-to-robot direction.
    """

    def __init__(self, config: PlannerConfig):
        self.cfg = config

    def compute(
        self,
        robot_pose: Sequence[float],
        goal_position: Sequence[float],
        obstacle_point: Optional[Sequence[float]],
        wall_tangent: Optional[Sequence[float]],
        enable_fw: bool,
        enable_ao: bool,
        wall_normal: Optional[Sequence[float]] = None,
    ) -> Target:
        cfg = self.cfg
        eps = cfg.geometry_epsilon

        inputs = [robot_pose, goal_position, obstacle_point, wall_tangent, wall_normal]
        if not all(np.all(np.isfinite(v)) for v in inputs if v is not None):
            return safe_invalid_target("NONFINITE_INPUT")

        x, y, psi = (float(v) for v in robot_pose)
        p = np.array([x, y])
        h = np.array([math.cos(psi), math.sin(psi)])

        # 1. Goal geometry
        goal_vec = np.asarray(goal_position, dtype=float) - p
        d_gtg = float(np.linalg.norm(goal_vec))
        if d_gtg <= max(cfg.goal_tolerance, eps):
            return Target(0.0, 0.0, "GOAL_REACHED", {name: 0.0 for name in DIAGNOSTICS})

        psi_gtg = math.atan2(goal_vec[1], goal_vec[0])
        theta_gtg = wrap_angle(psi - psi_gtg)

        # 2. Base GTG heading
        g = -cfg.k_gtg_theta * theta_gtg
        u_gtg_theta = g
        u_fw_theta = 0.0
        sigma = z = gamma = w_fw = 0.0

        # 3. Obstacle geometry
        if enable_fw or enable_ao:
            if obstacle_point is None:
                return safe_invalid_target("MISSING_OBSTACLE")
            obs_to_robot = p - np.asarray(obstacle_point, dtype=float)
            d_ao = float(np.linalg.norm(obs_to_robot))
            if d_ao <= eps:
                return safe_invalid_target("INVALID_OBSTACLE_DIRECTION")
            normal = obs_to_robot / d_ao

        # 4. FW heading control
        if enable_fw:
            if wall_tangent is None:
                return safe_invalid_target("MISSING_TANGENT")
            tangent = np.asarray(wall_tangent, dtype=float)
            if np.linalg.norm(tangent) <= eps:
                return safe_invalid_target("INVALID_TANGENT")
            tangent = tangent / np.linalg.norm(tangent)

            fw_normal = normal
            if wall_normal is not None:
                fw_normal = np.asarray(wall_normal, dtype=float)
                if np.linalg.norm(fw_normal) <= eps:
                    return safe_invalid_target("INVALID_WALL_NORMAL")
                fw_normal = fw_normal / np.linalg.norm(fw_normal)

            cross_tn = cross2d(tangent, fw_normal)
            if abs(cross_tn) < cfg.min_tangent_cross:
                return safe_invalid_target("DEGENERATE_WALL_GEOMETRY")
            sigma = math.copysign(1.0, cross_tn)

            psi_fw = math.atan2(tangent[1], tangent[0])
            theta_fw = wrap_angle(psi - psi_fw)
            z = sigma * theta_fw

            # FW proximity weight: full within l_d, decaying beyond it
            w_fw = math.exp(-cfg.s_fw_distance * max(0.0, d_ao - cfg.l_d))

            # FW steering, continuous at z = 0
            exponent = min(cfg.s_fw * max(0.0, -z), cfg.max_fw_exponent)
            u_fw_theta = sigma * cfg.k_fw * w_fw * math.expm1(exponent)

            # Suppress GTG only when it opposes FW
            gamma = min(max(z / cfg.delta, 0.0), 1.0)
            g_sigma = sigma * g
            u_gtg_theta = sigma * (max(g_sigma, 0.0) + gamma * min(g_sigma, 0.0))

        # 5. Virtual heading
        theta_vt = math.atan(u_gtg_theta + u_fw_theta)

        # 6. GTG longitudinal control
        u_gtg_d = min(max(cfg.k_gtg_distance * d_gtg, cfg.u_gtg_min), cfg.u_gtg_max)
        gtg_cosine = float(goal_vec @ h) / d_gtg
        gtg_longitudinal = u_gtg_d * gtg_cosine

        # 7. AO longitudinal control
        u_ao_d = ao_cosine = ao_longitudinal = 0.0
        if enable_ao:
            u_ao_d = cfg.u_ao_max * math.exp(-cfg.s_ao * max(0.0, d_ao - cfg.l_d))
            ao_cosine = float(normal @ h)
            ao_longitudinal = u_ao_d * ao_cosine

        # 8. FW longitudinal control: constant drive along the wall tangent
        u_fw_d = fw_cosine = fw_longitudinal = 0.0
        if enable_fw:
            u_fw_d = cfg.u_fw_distance
            fw_cosine = float(tangent @ h)
            fw_longitudinal = u_fw_d * fw_cosine

        # 9. Virtual distance
        raw_distance = cfg.distance_scale * math.atan(
            gtg_longitudinal + ao_longitudinal + fw_longitudinal
        )
        low = 0.0 if cfg.forward_only else -cfg.max_reference_distance
        distance_vt = min(max(raw_distance, low), cfg.max_reference_distance)

        return Target(theta_vt, distance_vt, "OK", {
            "u_gtg_theta": u_gtg_theta,
            "u_fw_theta": u_fw_theta,
            "u_gtg_distance": u_gtg_d,
            "u_ao_distance": u_ao_d,
            "u_fw_distance": u_fw_d,
            "gtg_cosine": gtg_cosine,
            "ao_cosine": ao_cosine,
            "fw_cosine": fw_cosine,
            "fw_sigma": sigma,
            "fw_error_z": z,
            "fw_recovery_gamma": gamma,
            "fw_proximity_weight": w_fw,
            "raw_distance_vt": raw_distance,
        })


# =============================================================
# Node
# =============================================================

class GoToGoal(Node):

    def __init__(self):
        super().__init__("go_to_goal")

        def param(name, default):
            return self.declare_parameter(name, default).value

        self.planner = VirtualTargetPlanner(PlannerConfig(
            k_gtg_theta=float(param("k_gtg_theta", 1.0)),
            k_fw=float(param("k_fw", 1.0)),
            s_fw=float(param("s_fw", 2.0)),
            s_fw_distance=float(param("s_fw_distance", 5.0)),
            delta=float(param("delta", 0.3)),
            k_gtg_distance=float(param("k_gtg_distance", 1.0)),
            u_gtg_min=float(param("u_gtg_min", 0.2)),
            u_gtg_max=float(param("u_gtg_max", 1.0)),
            u_ao_max=float(param("u_ao_max", 1.5)),
            s_ao=float(param("s_ao", 5.0)),
            l_d=float(param("l_d", 0.2)),
            u_fw_distance=float(param("u_fw_distance", 0.5)),
            distance_scale=float(param("distance_scale", 0.5)),
            max_reference_distance=float(param("max_reference_distance", 0.5)),
            # Shared with the goal node, so the robot stops where the hold runs
            goal_tolerance=float(param("goal_tolerance", 0.03)),
            geometry_epsilon=float(param("geometry_epsilon", 1e-6)),
            min_tangent_cross=float(param("min_tangent_cross", 0.2)),
            max_fw_exponent=float(param("max_fw_exponent", 20.0)),
            forward_only=bool(param("forward_only", True)),
        ))
        # Inputs older than this zero the target and stop the robot (s)
        self.input_timeout = float(param("input_timeout", 0.5))
        self.log = VerboseLogger(self)

        # Latest inputs and when they were received
        self.position: Optional[np.ndarray] = None  # odom frame
        self.yaw: Optional[float] = None
        self.goal: Optional[np.ndarray] = None      # odom frame
        self.finished = False
        self.avoidance_state: Optional[AvoidanceState] = None
        self.received: Dict[str, Optional[Time]] = {"odom": None, "goal": None, "avoidance": None}
        self.status: Optional[str] = None

        self.odom_subscription = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.goal_subscription = self.create_subscription(
            PointStamped, "/goal", self.goal_callback, 10
        )
        self.status_subscription = self.create_subscription(
            GoalStatus, "/goal_status", self.goal_status_callback, 10
        )
        self.state_subscription = self.create_subscription(
            AvoidanceState, "/avoidance_state", self.state_callback, 10
        )

        self.target_publisher = self.create_publisher(VirtualTargetMsg, "/virtual_target", 10)
        self.timer = self.create_timer(CONTROL_PERIOD, self.control_loop)

        self.log.basic("go_to_goal started")

    # =============================================================
    # Callbacks
    # =============================================================

    def odom_callback(self, odom: Odometry):
        pose = odom.pose.pose
        self.position = np.array([pose.position.x, pose.position.y])
        self.yaw = yaw_from_quaternion(pose.orientation)
        self.received["odom"] = self.get_clock().now()

    def goal_callback(self, msg: PointStamped):
        self.goal = np.array([msg.point.x, msg.point.y])
        self.received["goal"] = self.get_clock().now()

    def goal_status_callback(self, msg: GoalStatus):
        self.finished = msg.finished

    def state_callback(self, msg: AvoidanceState):
        self.avoidance_state = msg
        self.received["avoidance"] = self.get_clock().now()

    def is_fresh(self, name: str) -> bool:
        received = self.received[name]
        if received is None:
            return False
        age = (self.get_clock().now() - received).nanoseconds * 1e-9
        return age <= self.input_timeout

    # =============================================================
    # Control
    # =============================================================

    def control_loop(self):
        state = self.avoidance_state
        avoiding = False

        if self.finished:
            target = safe_invalid_target("FINISHED")
        elif not all(self.is_fresh(name) for name in self.received):
            target = safe_invalid_target("STALE_INPUT")
        else:
            # Everything in the robot frame (x: forward, y: left): the robot
            # pose is the origin, and the goal is converted from odom
            goal = rotation(self.yaw).T @ (self.goal - self.position)
            avoiding = state.avoiding
            detected = state.obstacle_detected
            target = self.planner.compute(
                robot_pose=(0.0, 0.0, 0.0),
                goal_position=goal,
                obstacle_point=(state.obstacle_position.x, state.obstacle_position.y) if detected else None,
                wall_tangent=(state.wall_vector.x, state.wall_vector.y) if avoiding else None,
                enable_fw=avoiding,
                enable_ao=avoiding,
                # Same normal obstacle_state built the wall vector from
                wall_normal=(state.obstacle_normal.x, state.obstacle_normal.y) if detected else None,
            )

        self.publish_target(target, avoiding)

        if target.status != self.status:
            self.log.basic(f"Virtual target status: {target.status}")
            self.status = target.status
        self.log.continuous(
            f"theta_vt={np.degrees(target.theta_vt):6.1f} deg "
            f"distance_vt={target.distance_vt:.3f} m avoiding={avoiding} {target.status}",
            throttle_duration_sec=0.5,
        )

    def publish_target(self, target: Target, avoiding: bool):
        msg = VirtualTargetMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.theta = float(target.theta_vt)
        msg.distance = float(target.distance_vt)
        msg.status = target.status
        msg.enable_fw = msg.enable_ao = avoiding
        d = target.diagnostics
        msg.u_gtg_theta = d["u_gtg_theta"]
        msg.u_fw_theta = d["u_fw_theta"]
        msg.u_gtg_distance = d["u_gtg_distance"]
        msg.u_ao_distance = d["u_ao_distance"]
        msg.u_fw_distance = d["u_fw_distance"]
        msg.gtg_cosine = d["gtg_cosine"]
        msg.ao_cosine = d["ao_cosine"]
        msg.fw_cosine = d["fw_cosine"]
        msg.fw_sigma = d["fw_sigma"]
        msg.fw_error_z = d["fw_error_z"]
        msg.fw_recovery_gamma = d["fw_recovery_gamma"]
        msg.fw_proximity_weight = d["fw_proximity_weight"]
        msg.raw_distance = d["raw_distance_vt"]
        self.target_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = GoToGoal()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
