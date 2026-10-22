"""Unit tests of GoalState in the goal node."""

import math

import numpy as np
import pytest

from team85_lab4.scripts.goal import GoalState


@pytest.fixture
def state(tmp_path) -> GoalState:
    waypoints = tmp_path / "wayPoints.txt"
    waypoints.write_text("1.0 0.0\n1.0 1.0\n")
    return GoalState(str(waypoints), goal_tolerance=0.05, hold_time=1.0)


def test_switches_after_hold(state):
    state.update(np.zeros(2), 0.0, 0.0)
    state.update(np.array([1.0, 0.0]), 0.0, 1.0)
    assert state.goal_index == 0
    state.update(np.array([1.0, 0.0]), 0.0, 2.0)
    assert state.goal_index == 1


def test_reset_restarts_from_current_pose(state):
    state.update(np.zeros(2), 0.0, 0.0)
    state.update(np.array([1.0, 0.0]), 0.0, 1.0)
    state.update(np.array([1.0, 0.0]), 0.0, 2.0)
    assert state.goal_index == 1

    state.reset()
    assert not state.origin_set
    assert state.goal_index == 0
    assert state.current_goal is None

    # The next pose becomes the world origin: goal 1 is 1 m ahead of it
    state.update(np.array([2.0, 3.0]), math.pi / 2, 3.0)
    assert state.origin_set
    assert state.current_goal == pytest.approx([2.0, 4.0])
    assert state.reached_since is None
