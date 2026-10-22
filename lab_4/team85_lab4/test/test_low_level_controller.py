"""Unit tests of the low-level controller in low_level_controller."""

import math

import pytest

from team85_lab4.scripts.go_to_goal import Target
from team85_lab4.scripts.low_level_controller import ControllerConfig, LowLevelController


def controller(**overrides) -> LowLevelController:
    values = dict(
        kp_linear=0.4, kp_angular=1.2, distance_deadband=0.005, angle_deadband=0.05,
        max_linear_speed=0.15, max_angular_speed=0.8,
    )
    values.update(overrides)
    return LowLevelController(ControllerConfig(**values))


def command(theta, distance, status="OK", **overrides):
    return controller(**overrides).compute(Target(theta, distance, status))


def test_proportional():
    assert command(0.3, 0.2) == pytest.approx((0.4 * 0.2, 1.2 * 0.3))


def test_signs():
    # theta_vt positive left -> angular.z positive (turn left)
    assert command(0.3, 0.1)[1] > 0.0
    assert command(-0.3, 0.1)[1] < 0.0
    # Negative distance -> backing up
    assert command(0.0, -0.1)[0] < 0.0


def test_turns_in_place_when_distance_is_zero():
    linear, angular = command(-1.2, 0.0)
    assert linear == 0.0
    assert angular < 0.0


def test_saturation():
    assert command(math.pi / 2, 0.5) == (0.15, 0.8)
    assert command(-math.pi / 2, -0.5) == (-0.15, -0.8)


def test_deadbands():
    assert command(0.04, 0.004) == (0.0, 0.0)
    assert command(0.06, 0.006) == pytest.approx((0.4 * 0.006, 1.2 * 0.06))


@pytest.mark.parametrize("status", ["GOAL_REACHED", "FINISHED", "STALE_INPUT", "MISSING_OBSTACLE"])
def test_stops_unless_ok(status):
    assert command(1.0, 0.3, status) == (0.0, 0.0)


@pytest.mark.parametrize("name,value", [
    ("kp_linear", 0.0), ("max_angular_speed", -1.0), ("angle_deadband", -0.1), ("kp_angular", math.inf),
])
def test_invalid_config_rejected(name, value):
    with pytest.raises(ValueError):
        controller(**{name: value})
