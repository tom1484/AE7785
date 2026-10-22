#!/usr/bin/env python3

"""
CS/ME/ECE/AE/BME 7785 - Lab 4

Authors:
Chu-Rong Chen
Xingyu Zhu

Tool (not a ROS node):
    tune_planner

Interactive plots of the virtual target planner and the low-level controller
of go_to_goal, for tuning their parameters offline. The curves are computed
with the same VirtualTargetPlanner and LowLevelController classes the robot
runs, starting from the parameters in robot.yaml.

Scene (wall following, robot frame, robot at the origin heading +x):
    - The wall is a straight line at distance d_AO, on the robot's right
      (sigma = +1) or left (sigma = -1), with FW and AO enabled.
    - The robot's heading relative to the wall tangent is given by the FW
      error z = sigma * theta_FW (negative: pointing into the wall).
    - The goal is at a given distance, at an angle from the wall tangent
      towards the wall (0: along the wall, 90: straight behind it).

Plots:
    1. Heading terms vs z           4. Proximity terms vs d_AO (at the z slider)
    2. theta_VT and omega vs z      5. GTG only: distance terms vs goal distance
    3. Distance terms vs z          6. GTG only: theta_VT and omega vs goal bearing

"Print YAML" prints the current go_to_goal parameters, ready to paste into
robot.yaml.

Usage:
    ros2 run team85_lab4 tune_planner
    ros2 run team85_lab4 tune_planner --params /path/to/robot.yaml
"""

import argparse
import math
import os
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
from matplotlib.widgets import Button, CheckButtons, RadioButtons, Slider
import numpy as np
import yaml

from ament_index_python.packages import get_package_share_directory

from team85_lab4.scripts.go_to_goal import (
    ControllerConfig, LowLevelController, PlannerConfig, Target, VirtualTargetPlanner,
)


# (name, min, max): sliders for the go_to_goal parameters, grouped
PLANNER_SLIDERS = [
    ("k_gtg_theta", 0.1, 5.0),
    ("k_fw", 0.05, 5.0),
    ("s_fw", 0.1, 5.0),
    ("s_fw_distance", 0.0, 30.0),
    ("delta", 0.01, 3.0),
    ("k_gtg_distance", 0.1, 5.0),
    ("u_gtg_min", 0.0, 2.0),
    ("u_gtg_max", 0.1, 3.0),
    ("u_ao_max", 0.1, 5.0),
    ("s_ao", 0.1, 30.0),
    ("l_d", 0.05, 1.0),
    ("u_fw_distance", 0.0, 5.0),
    ("distance_scale", 0.05, 2.0),
    ("max_reference_distance", 0.05, 2.0),
    ("goal_tolerance", 0.0, 0.2),
]
CONTROLLER_SLIDERS = [
    ("kp_linear", 0.05, 3.0),
    ("kp_angular", 0.05, 5.0),
    ("max_linear_speed", 0.01, 0.22),
    ("max_angular_speed", 0.1, 2.84),
    ("distance_deadband", 0.0, 0.05),
    ("angle_deadband", 0.0, 0.3),
]
# Scene sliders: (name, label, min, max, initial)
SCENE_SLIDERS = [
    ("d_ao", "obstacle dist (m)", 0.05, 1.0, 0.35),
    ("goal_angle", "goal from wall (deg)", -180.0, 180.0, 60.0),
    ("goal_distance", "goal dist (m)", 0.05, 3.0, 1.5),
    ("z", "z for plot 4 (rad)", -2.0, 2.0, -0.5),
]

# Fixed parameters (numerical safeguards), not tuned here
FIXED = {"geometry_epsilon": 1e-6, "min_tangent_cross": 0.2, "max_fw_exponent": 20.0}

DEFAULTS = {
    "k_gtg_theta": 1.0, "k_fw": 1.0, "s_fw": 2.0, "s_fw_distance": 5.0, "delta": 0.3,
    "k_gtg_distance": 1.0, "u_gtg_min": 0.2, "u_gtg_max": 1.0, "u_ao_max": 1.5, "s_ao": 5.0,
    "l_d": 0.2, "u_fw_distance": 0.5, "distance_scale": 0.5, "max_reference_distance": 0.5,
    "goal_tolerance": 0.03, "forward_only": True,
    "kp_linear": 0.4, "kp_angular": 1.2, "max_linear_speed": 0.15, "max_angular_speed": 0.8,
    "distance_deadband": 0.005, "angle_deadband": 0.05,
}

REDRAW_INTERVAL_MS = 100  # Minimum time between redraws while dragging

Z_RANGE = np.linspace(-2.0, 2.0, 241)
D_AO_RANGE = np.linspace(0.02, 1.0, 197)
GOAL_DISTANCE_RANGE = np.linspace(0.0, 2.0, 201)
BEARING_RANGE = np.linspace(-180.0, 180.0, 361)


def load_parameters(path: str) -> Dict[str, float]:
    """go_to_goal parameters (with the shared goal_tolerance) from a robot.yaml."""
    params = dict(DEFAULTS)
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    for section in ("/**", "go_to_goal"):
        values = (data.get(section) or {}).get("ros__parameters") or {}
        params.update({k: v for k, v in values.items() if k in DEFAULTS})
    return params


def yaml_float(value: float) -> str:
    """Float with a decimal point (ROS rejects 2 for a float parameter), rounded."""
    return repr(round(float(value), 4))


def rotate_ccw(vector: np.ndarray) -> np.ndarray:
    return np.array([-vector[1], vector[0]])


def wall_scene(sigma: int, z: float, d_ao: float, goal_angle_deg: float, goal_distance: float):
    """
    Robot at the origin heading +x, straight wall at distance d_ao giving FW
    correction direction sigma and FW error z. Returns (goal, obstacle, tangent, normal).
    """
    # z = sigma * theta_FW, theta_FW = -psi_FW (robot heading 0)
    psi_fw = -sigma * z
    tangent = np.array([math.cos(psi_fw), math.sin(psi_fw)])
    # Normal towards the robot, with cross(tangent, normal) = sigma
    normal = sigma * rotate_ccw(tangent)
    obstacle = -d_ao * normal
    # Goal angle measured from the tangent towards the wall (the -normal side)
    bearing = psi_fw - sigma * math.radians(goal_angle_deg)
    goal = goal_distance * np.array([math.cos(bearing), math.sin(bearing)])
    return goal, obstacle, tangent, normal


class Tuner:

    def __init__(self, params_path: str):
        self.params_path = params_path
        self.initial = load_parameters(params_path)
        self.sigma = 1

        self.fig = plt.figure(figsize=(17, 9.5))
        self.fig.canvas.manager.set_window_title("tune_planner")
        grid = self.fig.add_gridspec(2, 3, left=0.04, right=0.645, top=0.92, bottom=0.07,
                                     wspace=0.45, hspace=0.32)
        self.axes = [self.fig.add_subplot(grid[i // 3, i % 3]) for i in range(6)]
        self.twins = {i: self.axes[i].twinx() for i in (1, 2, 3, 4, 5)}

        self.sliders: Dict[str, Slider] = {}
        self.build_controls()
        self.update()

        # Widget callbacks only mark the plots stale; a timer redraws at most
        # once per tick, so dragging a slider doesn't queue up full redraws
        self.stale = False
        self.timer = self.fig.canvas.new_timer(interval=REDRAW_INTERVAL_MS)
        self.timer.add_callback(self.redraw_if_stale)
        self.timer.start()

    def request_update(self):
        self.stale = True

    def redraw_if_stale(self):
        if self.stale:
            self.stale = False
            self.update()

    # =============================================================
    # Controls
    # =============================================================

    def build_controls(self):
        x, width, height, step = 0.80, 0.13, 0.016, 0.0235
        top = 0.965

        def label(y, text):
            self.fig.text(x - 0.07, y, text, fontsize=9, fontweight="bold")

        y = top
        label(y, "Planner")
        for name, low, high in PLANNER_SLIDERS:
            y -= step
            self.add_slider(name, name, [x, y, width, height], low, high, float(self.initial[name]))

        y -= step * 1.3
        label(y, "Controller")
        for name, low, high in CONTROLLER_SLIDERS:
            y -= step
            self.add_slider(name, name, [x, y, width, height], low, high, float(self.initial[name]))

        y -= step * 1.3
        label(y, "Scene")
        for name, text, low, high, initial in SCENE_SLIDERS:
            y -= step
            self.add_slider(name, text, [x, y, width, height], low, high, initial)

        y -= step * 3.6
        self.side_radio = RadioButtons(
            self.fig.add_axes([x - 0.07, y, 0.10, step * 2.8]),
            ("wall right (sigma +1)", "wall left (sigma -1)"),
        )
        self.side_radio.on_clicked(self.on_side)
        self.forward_check = CheckButtons(
            self.fig.add_axes([x + 0.04, y + step * 1.4, 0.09, step * 1.4]),
            ["forward_only"], [bool(self.initial["forward_only"])],
        )
        self.forward_check.on_clicked(lambda _: self.request_update())

        y -= step * 1.8
        self.reset_button = Button(self.fig.add_axes([x - 0.07, y, 0.09, step * 1.3]), "Reset")
        self.reset_button.on_clicked(self.on_reset)
        self.print_button = Button(self.fig.add_axes([x + 0.04, y, 0.09, step * 1.3]), "Print YAML")
        self.print_button.on_clicked(self.on_print)

        for text in ([*self.side_radio.labels, *self.forward_check.labels]):
            text.set_fontsize(8)

    def add_slider(self, name, text, rect, low, high, initial):
        initial = min(max(initial, low), high)
        slider = Slider(self.fig.add_axes(rect), text, low, high, valinit=initial)
        slider.label.set_fontsize(8)
        slider.valtext.set_fontsize(8)
        slider.on_changed(lambda _: self.request_update())
        self.sliders[name] = slider

    def on_side(self, label):
        self.sigma = 1 if "+1" in label else -1
        self.request_update()

    def on_reset(self, _):
        for slider in self.sliders.values():
            slider.reset()

    def on_print(self, _):
        values = self.parameter_values()
        print("\n# go_to_goal (paste under go_to_goal: ros__parameters: in robot.yaml)")
        for name, _, _ in PLANNER_SLIDERS + CONTROLLER_SLIDERS:
            if name == "goal_tolerance":
                continue
            print(f"    {name}: {yaml_float(values[name])}")
        print(f"    forward_only: {str(values['forward_only']).lower()}")
        print(f"# shared (/**): goal_tolerance: {yaml_float(values['goal_tolerance'])}")

    # =============================================================
    # Computation
    # =============================================================

    def parameter_values(self) -> Dict[str, float]:
        values = {name: slider.val for name, slider in self.sliders.items()}
        values["forward_only"] = self.forward_check.get_status()[0]
        return values

    def build(self) -> Tuple[VirtualTargetPlanner, LowLevelController]:
        values = self.parameter_values()
        planner = VirtualTargetPlanner(PlannerConfig(
            **{name: float(values[name]) for name, _, _ in PLANNER_SLIDERS},
            forward_only=bool(values["forward_only"]),
            **FIXED,
        ))
        controller = LowLevelController(ControllerConfig(
            **{name: float(values[name]) for name, _, _ in CONTROLLER_SLIDERS},
        ))
        return planner, controller

    def wall_target(self, planner, values, z, d_ao) -> Target:
        goal, obstacle, tangent, normal = wall_scene(
            self.sigma, z, d_ao, values["goal_angle"], values["goal_distance"]
        )
        return planner.compute((0.0, 0.0, 0.0), goal, obstacle, tangent, True, True,
                               wall_normal=normal)

    @staticmethod
    def collect(targets: List[Target], controller: LowLevelController) -> Dict[str, np.ndarray]:
        rows = {"theta_vt": [], "distance_vt": [], "v": [], "w": []}
        diagnostics = {name: [] for name in targets[0].diagnostics}
        for target in targets:
            v, w = controller.compute(target)
            rows["theta_vt"].append(target.theta_vt)
            rows["distance_vt"].append(target.distance_vt)
            rows["v"].append(v)
            rows["w"].append(w)
            for name, value in target.diagnostics.items():
                diagnostics[name].append(value)
        return {k: np.array(x) for k, x in {**rows, **diagnostics}.items()}

    # =============================================================
    # Plotting
    # =============================================================

    def update(self):
        try:
            planner, controller = self.build()
        except ValueError as error:
            self.fig.suptitle(f"Invalid parameters: {error}", color="red")
            self.fig.canvas.draw_idle()
            return

        values = self.parameter_values()
        side = "right" if self.sigma > 0 else "left"
        self.fig.suptitle(
            f"Wall on the {side} at {values['d_ao']:.2f} m, goal {values['goal_distance']:.2f} m "
            f"at {values['goal_angle']:.0f} deg from the wall tangent towards the wall\n"
            f"z < 0: robot points into the wall",
            x=0.34, fontsize=10,
        )
        for ax in self.axes + list(self.twins.values()):
            ax.clear()
        # clear() moves a twin axis back to the left
        for twin in self.twins.values():
            twin.yaxis.tick_right()
            twin.yaxis.set_label_position("right")

        d_ao = values["d_ao"]
        wall = self.collect([self.wall_target(planner, values, z, d_ao) for z in Z_RANGE], controller)
        self.plot_heading_terms(wall)
        self.plot_heading_output(wall, values)
        self.plot_distance_terms(wall, values)

        z_now = values["z"]
        prox = self.collect([self.wall_target(planner, values, z_now, d) for d in D_AO_RANGE], controller)
        self.plot_proximity(prox, values)

        gtg = self.collect(
            [planner.compute((0.0, 0.0, 0.0), (r, 0.0), None, None, False, False)
             for r in GOAL_DISTANCE_RANGE], controller)
        self.plot_gtg_distance(gtg, values)

        bearing = self.collect(
            [planner.compute((0.0, 0.0, 0.0),
                             values["goal_distance"] * np.array([math.cos(math.radians(b)),
                                                                 math.sin(math.radians(b))]),
                             None, None, False, False)
             for b in BEARING_RANGE], controller)
        self.plot_gtg_bearing(bearing, values)

        for ax in self.axes:
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=8)
        for ax in self.twins.values():
            ax.tick_params(labelsize=8)
        self.fig.canvas.draw_idle()

    @staticmethod
    def mark_zero_z(ax):
        ax.axvline(0.0, color="gray", lw=0.8, ls=":")

    def plot_heading_terms(self, data):
        ax = self.axes[0]
        total = data["u_gtg_theta"] + data["u_fw_theta"]
        ax.plot(Z_RANGE, data["u_gtg_theta"], label="u_gtg (suppressed)")
        ax.plot(Z_RANGE, data["u_fw_theta"], label="u_fw")
        ax.plot(Z_RANGE, total, "k", lw=1.6, label="u total")
        limit = max(2.0, min(10.0, 1.1 * float(np.max(np.abs(total)))))
        ax.set_ylim(-limit, limit)
        self.mark_zero_z(ax)
        ax.set_title("1. Heading terms vs z", fontsize=9)
        ax.set_xlabel("z (rad)", fontsize=8)
        ax.legend(fontsize=7, loc="best")

    def plot_heading_output(self, data, values):
        ax, twin = self.axes[1], self.twins[1]
        ax.plot(Z_RANGE, np.degrees(data["theta_vt"]), "C0", label="theta_vt (deg)")
        twin.plot(Z_RANGE, data["w"], "C3", label="omega (rad/s)")
        limit = values["max_angular_speed"]
        twin.axhline(limit, color="C3", ls="--", lw=0.7)
        twin.axhline(-limit, color="C3", ls="--", lw=0.7)
        ax.set_ylim(-95, 95)
        twin.set_ylim(-1.1 * limit * 95 / 90, 1.1 * limit * 95 / 90)
        self.mark_zero_z(ax)
        ax.set_title("2. Virtual heading and omega vs z", fontsize=9)
        ax.set_xlabel("z (rad)", fontsize=8)
        ax.set_ylabel("theta_vt (deg)", fontsize=8, color="C0")
        twin.set_ylabel("omega (rad/s)", fontsize=8, color="C3")

    def plot_distance_terms(self, data, values):
        ax, twin = self.axes[2], self.twins[2]
        gtg = data["u_gtg_distance"] * data["gtg_cosine"]
        ao = data["u_ao_distance"] * data["ao_cosine"]
        fw = data["u_fw_distance"] * data["fw_cosine"]
        ax.plot(Z_RANGE, gtg, label="GTG")
        ax.plot(Z_RANGE, ao, label="AO")
        ax.plot(Z_RANGE, fw, label="FW")
        ax.plot(Z_RANGE, gtg + ao + fw, "k", lw=1.6, label="sum")
        twin.plot(Z_RANGE, data["v"], "C3", lw=1.4, label="v (m/s)")
        limit = values["max_linear_speed"]
        twin.axhline(limit, color="C3", ls="--", lw=0.7)
        twin.axhline(-limit, color="C3", ls="--", lw=0.7)
        twin.set_ylim(-1.3 * limit, 1.3 * limit)
        ax.axhline(0.0, color="gray", lw=0.8)
        self.mark_zero_z(ax)
        ax.set_title("3. Distance terms and v vs z", fontsize=9)
        ax.set_xlabel("z (rad)", fontsize=8)
        twin.set_ylabel("v (m/s)", fontsize=8, color="C3")
        ax.legend(fontsize=7, loc="best")

    def plot_proximity(self, data, values):
        ax, twin = self.axes[3], self.twins[3]
        ax.plot(D_AO_RANGE, data["fw_proximity_weight"], label="w_fw")
        ax.plot(D_AO_RANGE, data["u_ao_distance"] / max(values["u_ao_max"], 1e-9), label="u_ao / u_ao_max")
        twin.plot(D_AO_RANGE, data["w"], "C3", label="omega")
        twin.plot(D_AO_RANGE, data["v"], "C4", label="v")
        ax.axvline(values["l_d"], color="gray", ls="--", lw=0.8)
        ax.axvline(values["d_ao"], color="k", ls=":", lw=0.8)
        ax.set_ylim(-0.05, 1.1)
        ax.set_title(f"4. Proximity terms vs d_AO (z = {values['z']:.2f})", fontsize=9)
        ax.set_xlabel("obstacle distance (m); dashed: l_d, dotted: scene", fontsize=8)
        ax.legend(fontsize=7, loc="upper right")
        twin.legend(fontsize=7, loc="center right")
        twin.set_ylabel("omega (rad/s), v (m/s)", fontsize=8)

    def plot_gtg_distance(self, data, values):
        ax, twin = self.axes[4], self.twins[4]
        ax.plot(GOAL_DISTANCE_RANGE, data["u_gtg_distance"], label="u_gtg")
        ax.plot(GOAL_DISTANCE_RANGE, data["distance_vt"], label="d_vt (m)")
        twin.plot(GOAL_DISTANCE_RANGE, data["v"], "C3", label="v (m/s)")
        ax.axvline(values["goal_tolerance"], color="gray", ls="--", lw=0.8)
        ax.set_title("5. GTG only, goal ahead: vs goal distance", fontsize=9)
        ax.set_xlabel("goal distance (m); dashed: goal_tolerance", fontsize=8)
        ax.legend(fontsize=7, loc="lower right")
        twin.set_ylabel("v (m/s)", fontsize=8, color="C3")
        twin.set_ylim(0.0, 1.2 * values["max_linear_speed"])

    def plot_gtg_bearing(self, data, values):
        ax, twin = self.axes[5], self.twins[5]
        ax.plot(BEARING_RANGE, np.degrees(data["theta_vt"]), "C0", label="theta_vt (deg)")
        twin.plot(BEARING_RANGE, data["w"], "C3", label="omega")
        twin.plot(BEARING_RANGE, data["v"], "C4", label="v")
        ax.set_ylim(-95, 95)
        ax.set_title(f"6. GTG only, goal {values['goal_distance']:.2f} m: vs bearing", fontsize=9)
        ax.set_xlabel("goal bearing (deg, positive: left)", fontsize=8)
        ax.set_ylabel("theta_vt (deg)", fontsize=8, color="C0")
        twin.legend(fontsize=7, loc="lower right")
        twin.set_ylabel("omega (rad/s), v (m/s)", fontsize=8)


def main(args=None):
    default = os.path.join(get_package_share_directory("team85_lab4"), "config", "robot.yaml")
    parser = argparse.ArgumentParser(description="Interactive tuning plots of the go_to_goal planner")
    parser.add_argument("--params", default=default, help="robot.yaml to start from")
    # Ignore ROS arguments (e.g. --ros-args) passed by ros2 run
    options, _ = parser.parse_known_args(args)

    print(f"Loaded parameters from {options.params}")
    tuner = Tuner(options.params)  # Keep a reference: widgets stop working once collected
    plt.show()
    del tuner


if __name__ == "__main__":
    main()
