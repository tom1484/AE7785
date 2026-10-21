"""Unit tests of the virtual target planner (README.md, Algorithm 4)."""

import math

import numpy as np
import pytest

from team85_lab4.scripts.go_to_goal import PlannerConfig, VirtualTargetPlanner, cross2d
from team85_lab4.scripts.obstacle_state import ObstacleState, rotate_ccw


ORIGIN = (0.0, 0.0, 0.0)
WALL_DISTANCE = 0.25


def make_config(**overrides) -> PlannerConfig:
    values = dict(
        k_gtg_theta=1.0, k_fw=1.0, s_fw=2.0, s_fw_distance=0.0, delta=0.3,
        k_gtg_distance=1.0, u_gtg_min=0.0, u_gtg_max=1.0, u_ao_max=1.5, s_ao=5.0, l_d=0.2, u_fw_distance=0.5,
        distance_scale=0.5, max_reference_distance=0.5, goal_tolerance=0.03,
        geometry_epsilon=1e-6, min_tangent_cross=0.2, max_fw_exponent=20.0,
        forward_only=True,
    )
    values.update(overrides)
    return PlannerConfig(**values)


def planner(**overrides) -> VirtualTargetPlanner:
    return VirtualTargetPlanner(make_config(**overrides))


def wall(sigma: int, z: float):
    """
    Straight wall seen from the robot at the origin heading +x, giving FW
    correction direction sigma and FW error z.

    sigma = +1: wall on the right; sigma = -1: wall on the left.
    Returns (obstacle_point, tangent).
    """
    # z = sigma * theta_FW and theta_FW = -psi_FW (robot heading 0)
    psi_fw = -sigma * z
    tangent = np.array([math.cos(psi_fw), math.sin(psi_fw)])
    # Normal towards the robot: tangent rotated so that cross(t, n) = sigma
    normal = sigma * rotate_ccw(tangent)
    return -WALL_DISTANCE * normal, tangent


def fw(p, goal, sigma, z, **kwargs):
    obstacle, tangent = wall(sigma, z)
    return p.compute(ORIGIN, goal, obstacle, tangent, enable_fw=True, enable_ao=False, **kwargs)


def total_steering(target) -> float:
    d = target.diagnostics
    return d["u_gtg_theta"] + d["u_fw_theta"]


# =============================================================
# 8.1 GTG
# =============================================================

def gtg(goal, **overrides):
    return planner(**overrides).compute(ORIGIN, goal, None, None, False, False)


def test_goal_ahead():
    t = gtg((1.0, 0.0))
    assert t.status == "OK"
    assert t.theta_vt == 0.0
    assert t.distance_vt > 0.0


def test_goal_left_and_right():
    assert gtg((1.0, 1.0)).theta_vt > 0.0
    assert gtg((1.0, -1.0)).theta_vt < 0.0


def test_goal_to_the_side():
    assert gtg((0.0, 1.0)).diagnostics["gtg_cosine"] == pytest.approx(0.0, abs=1e-12)


def test_gtg_lower_bound_near_goal():
    # k * d = 0.05 is below the bound
    t = gtg((0.05, 0.0), u_gtg_min=0.2)
    assert t.diagnostics["u_gtg_distance"] == 0.2
    assert t.distance_vt == pytest.approx(0.5 * math.atan(0.2))
    # Unbounded, the drive fades with the distance
    assert gtg((0.05, 0.0)).diagnostics["u_gtg_distance"] == pytest.approx(0.05)


def test_gtg_lower_bound_keeps_goal_reached():
    assert gtg((0.02, 0.0), u_gtg_min=0.2).status == "GOAL_REACHED"
    assert gtg((0.02, 0.0), u_gtg_min=0.2).distance_vt == 0.0


def test_gtg_lower_bound_keeps_projection_sign():
    # Goal close behind: the bounded magnitude still projects backwards
    d = gtg((-0.05, 0.0), u_gtg_min=0.2, forward_only=False).diagnostics
    assert d["u_gtg_distance"] == 0.2 and d["raw_distance_vt"] < 0.0


def test_goal_behind():
    t = gtg((-1.0, 0.0))
    d = t.diagnostics
    assert d["u_gtg_distance"] * d["gtg_cosine"] < 0.0
    assert d["raw_distance_vt"] < 0.0
    assert t.distance_vt == 0.0  # Forward-only clamp


# =============================================================
# 8.2 FW
# =============================================================

def test_fw_inactive():
    t = gtg((1.0, 1.0))
    g = -1.0 * (0.0 - math.atan2(1.0, 1.0))
    assert t.diagnostics["u_fw_theta"] == 0.0
    assert t.diagnostics["u_gtg_theta"] == pytest.approx(g)


@pytest.mark.parametrize("sigma", [1, -1])
def test_fw_correction_overrides_opposing_gtg(sigma):
    # Goal on the side opposing the correction: GTG steers towards the wall
    goal = (1.0, -sigma * 1.0)
    t = fw(planner(), goal, sigma, z=-0.4)
    d = t.diagnostics
    assert t.status == "OK"
    assert d["fw_sigma"] == sigma
    assert sigma * d["u_fw_theta"] > 0.0
    assert d["u_gtg_theta"] == 0.0
    assert sigma * total_steering(t) > 0.0
    assert sigma * t.theta_vt > 0.0


@pytest.mark.parametrize("sigma", [1, -1])
def test_gtg_assisting_fw_is_preserved(sigma):
    goal = (1.0, sigma * 1.0)
    g = -1.0 * (0.0 - math.atan2(goal[1], goal[0]))
    t = fw(planner(), goal, sigma, z=-0.4)
    assert sigma * g > 0.0
    assert t.diagnostics["u_gtg_theta"] == pytest.approx(g)


@pytest.mark.parametrize("sigma", [1, -1])
@pytest.mark.parametrize("goal_side", [1, -1])
def test_fw_transition_is_continuous(sigma, goal_side):
    p = planner()
    goal = (1.0, goal_side * 0.7)
    at_zero = total_steering(fw(p, goal, sigma, z=0.0))
    for eps in (1e-3, 1e-5, 1e-7):
        for z in (-eps, eps):
            assert total_steering(fw(p, goal, sigma, z)) == pytest.approx(at_zero, abs=20 * eps)


def fw_at(distance, sigma=1, z=-0.4, goal=(1.0, 0.0), **overrides):
    """FW with the wall at the given distance from the robot."""
    _, tangent = wall(sigma, z)
    normal = sigma * rotate_ccw(tangent)
    p = planner(**overrides)
    return p.compute(ORIGIN, goal, -distance * normal, tangent, enable_fw=True, enable_ao=False)


def test_fw_proximity_weight_saturates_inside_danger_distance():
    assert fw_at(0.1, s_fw_distance=5.0).diagnostics["fw_proximity_weight"] == 1.0
    # At l_d (up to rounding of the constructed obstacle point)
    assert fw_at(0.2, s_fw_distance=5.0).diagnostics["fw_proximity_weight"] == pytest.approx(1.0)


def test_fw_proximity_weight_decays_beyond_danger_distance():
    s, l_d = 5.0, 0.2
    near = fw_at(l_d, s_fw_distance=s).diagnostics
    far = fw_at(0.5, s_fw_distance=s).diagnostics
    assert far["fw_proximity_weight"] == pytest.approx(math.exp(-s * (0.5 - l_d)))
    assert far["u_fw_theta"] == pytest.approx(near["u_fw_theta"] * far["fw_proximity_weight"])
    # Continuous at l_d
    assert fw_at(l_d + 1e-9, s_fw_distance=s).diagnostics["fw_proximity_weight"] == pytest.approx(1.0, abs=1e-6)


def test_fw_proximity_scaling_disabled():
    assert fw_at(2.0, s_fw_distance=0.0).diagnostics["fw_proximity_weight"] == 1.0


@pytest.mark.parametrize("sigma", [1, -1])
def test_fw_priority_holds_far_from_obstacle(sigma):
    # Weak FW far away, with GTG opposing it
    t = fw_at(1.5, sigma=sigma, goal=(1.0, -sigma * 1.0), s_fw_distance=5.0)
    d = t.diagnostics
    assert 0.0 < d["fw_proximity_weight"] < 0.01
    assert d["u_gtg_theta"] == 0.0
    assert sigma * total_steering(t) > 0.0


@pytest.mark.parametrize("sigma", [1, -1])
def test_gtg_restored_beyond_delta(sigma):
    goal = (1.0, -sigma * 1.0)
    g = -1.0 * (0.0 - math.atan2(goal[1], goal[0]))
    t = fw(planner(delta=0.3), goal, sigma, z=0.35)
    assert t.diagnostics["u_gtg_theta"] == pytest.approx(g)
    assert t.diagnostics["u_fw_theta"] == 0.0
    assert t.diagnostics["fw_recovery_gamma"] == 1.0


@pytest.mark.parametrize("sigma", [1, -1])
def test_gtg_partially_restored_inside_delta(sigma):
    goal = (1.0, -sigma * 1.0)
    g = -1.0 * (0.0 - math.atan2(goal[1], goal[0]))
    t = fw(planner(delta=0.3), goal, sigma, z=0.15)
    assert t.diagnostics["fw_recovery_gamma"] == pytest.approx(0.5)
    assert t.diagnostics["u_gtg_theta"] == pytest.approx(0.5 * g)


@pytest.mark.parametrize("sigma", [1, -1])
def test_wall_alignment(sigma):
    t = fw(planner(), (1.0, -sigma * 1.0), sigma, z=0.0)
    assert t.diagnostics["u_fw_theta"] == 0.0
    assert t.diagnostics["u_gtg_theta"] == 0.0  # Opposing GTG still suppressed


def test_mirrored_geometry_is_equivalent():
    p = planner()
    right = fw(p, (1.0, -0.5), 1, z=-0.3)
    left = fw(p, (1.0, 0.5), -1, z=-0.3)
    assert right.theta_vt == pytest.approx(-left.theta_vt)
    assert right.distance_vt == pytest.approx(left.distance_vt)


# =============================================================
# 8.3 AO
# =============================================================

def ao(obstacle, goal=(1.0, 0.0), **overrides):
    return planner(**overrides).compute(ORIGIN, goal, obstacle, None, False, True)


def test_obstacle_ahead_reduces_distance():
    t = ao((0.5, 0.0))
    assert t.diagnostics["ao_cosine"] == pytest.approx(-1.0)
    assert t.diagnostics["raw_distance_vt"] < gtg((1.0, 0.0)).diagnostics["raw_distance_vt"]


def test_obstacle_behind_increases_signed_sum():
    t = ao((-0.5, 0.0), goal=(0.3, 0.0))
    assert t.diagnostics["ao_cosine"] == pytest.approx(1.0)
    assert t.diagnostics["raw_distance_vt"] > gtg((0.3, 0.0)).diagnostics["raw_distance_vt"]


def test_obstacle_beside_has_no_effect():
    t = ao((0.0, 0.5))
    assert t.diagnostics["ao_cosine"] == pytest.approx(0.0, abs=1e-12)
    assert t.diagnostics["raw_distance_vt"] == pytest.approx(gtg((1.0, 0.0)).diagnostics["raw_distance_vt"])


def test_ao_saturation_and_decay():
    u_max, l_d, s = 1.5, 0.2, 5.0
    assert ao((0.1, 0.0)).diagnostics["u_ao_distance"] == u_max   # Inside
    assert ao((l_d, 0.0)).diagnostics["u_ao_distance"] == u_max   # At
    d = 0.4
    assert ao((d, 0.0)).diagnostics["u_ao_distance"] == pytest.approx(u_max * math.exp(-s * (d - l_d)))
    # Continuous at l_d
    assert ao((l_d + 1e-9, 0.0)).diagnostics["u_ao_distance"] == pytest.approx(u_max, abs=1e-6)


def test_ao_inactive_ignores_obstacle():
    t = planner().compute(ORIGIN, (1.0, 0.0), (0.3, 0.0), None, False, False)
    assert t.diagnostics["u_ao_distance"] == 0.0
    assert t.diagnostics["raw_distance_vt"] == gtg((1.0, 0.0)).diagnostics["raw_distance_vt"]


# =============================================================
# 8.3b FW distance
# =============================================================

def fw_distance(goal, z, **overrides):
    obstacle, tangent = wall(1, z)
    return planner(**overrides).compute(ORIGIN, goal, obstacle, tangent, enable_fw=True, enable_ao=False)


def test_fw_drive_along_tangent():
    d = fw_distance((1.0, 0.0), z=0.0).diagnostics
    assert d["fw_cosine"] == pytest.approx(1.0)
    assert d["u_fw_distance"] * d["fw_cosine"] == pytest.approx(0.5)


def test_fw_drive_perpendicular_to_tangent():
    assert fw_distance((1.0, 0.0), z=-math.pi / 2).diagnostics["fw_cosine"] == pytest.approx(0.0, abs=1e-12)


def test_fw_drive_turned_away():
    d = fw_distance((1.0, 0.0), z=0.6).diagnostics
    assert d["fw_cosine"] == pytest.approx(math.cos(0.6))


def test_fw_drive_inactive():
    p = planner()
    obstacle, _ = wall(1, 0.0)
    d = p.compute(ORIGIN, (1.0, 0.0), obstacle, None, enable_fw=False, enable_ao=True).diagnostics
    assert d["u_fw_distance"] == 0.0 and d["fw_cosine"] == 0.0


def test_fw_drive_gives_progress_along_wall():
    # Goal perpendicular to the heading: GTG alone gives no distance
    goal = (0.0, 1.0)
    without = fw_distance(goal, z=0.0, u_fw_distance=0.0)
    with_drive = fw_distance(goal, z=0.0)
    assert without.distance_vt == pytest.approx(0.0, abs=1e-9)
    assert with_drive.distance_vt == pytest.approx(0.5 * math.atan(0.5))


# =============================================================
# 8.4 Interface and robustness
# =============================================================

def assert_zeroed(t, status):
    assert t.status == status
    assert (t.theta_vt, t.distance_vt) == (0.0, 0.0)


def test_missing_obstacle():
    assert_zeroed(planner().compute(ORIGIN, (1.0, 0.0), None, None, False, True), "MISSING_OBSTACLE")
    assert_zeroed(planner().compute(ORIGIN, (1.0, 0.0), None, (1.0, 0.0), True, False), "MISSING_OBSTACLE")


def test_obstacle_at_robot():
    assert_zeroed(ao((0.0, 0.0)), "INVALID_OBSTACLE_DIRECTION")


def test_missing_or_zero_tangent():
    p = planner()
    assert_zeroed(p.compute(ORIGIN, (1.0, 0.0), (0.0, -0.3), None, True, True), "MISSING_TANGENT")
    assert_zeroed(p.compute(ORIGIN, (1.0, 0.0), (0.0, -0.3), (0.0, 0.0), True, True), "INVALID_TANGENT")


def test_degenerate_wall_geometry():
    # Tangent along the obstacle-to-robot direction
    t = planner().compute(ORIGIN, (1.0, 0.0), (0.0, -0.3), (0.0, 1.0), True, True)
    assert_zeroed(t, "DEGENERATE_WALL_GEOMETRY")


def test_nonfinite_inputs():
    p = planner()
    assert_zeroed(p.compute((0.0, math.nan, 0.0), (1.0, 0.0), None, None, False, False), "NONFINITE_INPUT")
    assert_zeroed(p.compute(ORIGIN, (math.inf, 0.0), None, None, False, False), "NONFINITE_INPUT")
    assert_zeroed(p.compute(ORIGIN, (1.0, 0.0), (0.3, math.nan), None, False, True), "NONFINITE_INPUT")


def test_goal_at_robot():
    assert_zeroed(gtg((0.0, 0.0), goal_tolerance=0.0), "GOAL_REACHED")
    assert_zeroed(gtg((0.02, 0.0)), "GOAL_REACHED")


def test_extreme_inputs_stay_finite():
    p = planner(k_fw=1e6, k_gtg_theta=1e6, s_fw=1e3)
    for z in (-3.0, -1e-3, 0.0, 3.0):
        t = fw(p, (-1.0, 1e-9), 1, z)
        assert math.isfinite(t.theta_vt) and math.isfinite(t.distance_vt)
        assert -math.pi / 2 < t.theta_vt < math.pi / 2


def test_distance_clamping():
    far = gtg((100.0, 0.0), distance_scale=2.0, max_reference_distance=0.5)
    assert far.distance_vt == 0.5
    behind = gtg((-1.0, 0.0), forward_only=False)
    assert behind.distance_vt < 0.0
    assert behind.distance_vt >= -0.5


def test_frame_invariance():
    # Same scene in the robot frame and in a rotated, shifted world frame
    obstacle, tangent = wall(1, z=-0.3)
    goal = np.array([1.0, -0.4])
    robot = planner().compute(ORIGIN, goal, obstacle, tangent, True, True)

    yaw, shift = 1.1, np.array([2.0, -3.0])
    R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    world = planner().compute(
        (shift[0], shift[1], yaw), R @ goal + shift, R @ obstacle + shift, R @ tangent, True, True
    )
    assert world.theta_vt == pytest.approx(robot.theta_vt)
    assert world.distance_vt == pytest.approx(robot.distance_vt)


@pytest.mark.parametrize("name,value", [
    ("k_fw", 0.0), ("delta", math.pi), ("delta", 0.0), ("l_d", -1.0),
    ("min_tangent_cross", 1.0), ("goal_tolerance", -0.1), ("s_ao", math.nan),
    ("u_fw_distance", -0.1), ("s_fw_distance", -1.0), ("u_gtg_min", -0.1), ("u_gtg_min", 2.0),
])
def test_invalid_config_rejected(name, value):
    with pytest.raises(ValueError):
        make_config(**{name: value})


# =============================================================
# Consistency with obstacle_state (shared wall normal)
# =============================================================

@pytest.mark.parametrize("wall_normal", ["fitted", "radial"])
@pytest.mark.parametrize("wall_side", [1, -1])
def test_sigma_matches_obstacle_state_side(wall_normal, wall_side):
    # Wall ahead, slanted so that its fitted normal differs from the radial one
    position = np.array([0.2, 0.05 * wall_side])
    fitted = np.array([-0.9, -0.44 * wall_side])
    fitted /= np.linalg.norm(fitted)

    state = ObstacleState(avoid_distance=0.3, path_radius=0.2, wall_normal=wall_normal)
    wall_vector = state.update(position, fitted)
    assert state.avoiding

    t = planner().compute(ORIGIN, (1.0, 0.0), position, wall_vector, True, True,
                          wall_normal=state.normal)
    assert t.status == "OK"
    # ccw: wall on the robot's left, so the correction is clockwise
    assert t.diagnostics["fw_sigma"] == (-1.0 if state.ccw else 1.0)
    # The tangent is exactly perpendicular to the normal it was built from
    assert abs(cross2d(wall_vector, state.normal)) == pytest.approx(1.0)


def test_obstacle_state_keeps_side_during_avoidance():
    state = ObstacleState(avoid_distance=0.3, path_radius=0.2)
    position = np.array([0.2, 0.0])
    state.update(position, np.array([-0.98, -0.2]) / np.linalg.norm([-0.98, -0.2]))
    first = state.ccw
    # Normal noise that would pick the other side on entry
    wall_vector = state.update(position, np.array([-0.98, 0.2]) / np.linalg.norm([-0.98, 0.2]))
    assert state.ccw == first
    assert np.allclose(wall_vector, rotate_ccw(state.normal) if first else -rotate_ccw(state.normal))
