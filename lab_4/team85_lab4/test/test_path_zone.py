"""Unit tests of the path zone along the goal direction."""

import math

import numpy as np
import pytest

from team85_lab4.obstacle_detector import ObstacleDetector
from team85_lab4.path_zone import HEADING, in_path_zone, path_coordinates, path_direction
from team85_lab4.scan_segmentation import ScanSegmenter
from team85_lab4.scripts.obstacle_state import ObstacleState


def unit(angle: float) -> np.ndarray:
    return np.array([math.cos(angle), math.sin(angle)])


def rotate(points: np.ndarray, angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.atleast_2d(points) @ np.array([[c, -s], [s, c]]).T


def test_direction_falls_back_to_heading():
    assert np.array_equal(path_direction(None), HEADING)
    assert np.array_equal(path_direction(np.zeros(2)), HEADING)
    assert path_direction(np.array([0.0, 2.0])) == pytest.approx([0.0, 1.0])


def test_coordinates():
    direction = unit(math.pi / 2)  # Goal to the left
    along, lateral = path_coordinates(np.array([[0.1, 0.5], [-0.3, 0.2]]), direction)
    assert along == pytest.approx([0.5, 0.2])
    assert lateral == pytest.approx([-0.1, 0.3])  # Right of the path line is negative


def test_zone_along_heading_matches_previous_strip():
    points = np.array([[0.3, 0.1], [0.3, 0.25], [-0.1, 0.0]])
    assert in_path_zone(points, HEADING, 0.2).tolist() == [True, False, False]


def test_zone_follows_goal_direction():
    # Goal 90 degrees to the left: a point straight ahead is off the path,
    # a point to the left is on it
    direction = unit(math.pi / 2)
    assert in_path_zone(np.array([[0.4, 0.0], [0.0, 0.4]]), direction, 0.2).tolist() == [False, True]


def wall_segments(points: np.ndarray):
    angles = np.arctan2(points[:, 1], points[:, 0])
    order = np.argsort(angles)
    return ScanSegmenter(min_points=5, base_threshold=0.2, k=3.0).segment_polar(
        angles[order], np.hypot(points[order, 0], points[order, 1])
    )


def test_detector_uses_goal_direction():
    # Wall ahead of the robot, and a box on the line to a goal on the left
    wall = np.array([[0.4, y] for y in np.linspace(-0.3, 0.3, 31)])
    box = np.array([[x, 0.7] for x in np.linspace(-0.05, 0.05, 6)])
    segments = wall_segments(np.vstack([wall, box]))
    detector = ObstacleDetector(path_radius=0.2)

    ahead = detector.detect(segments)
    assert ahead.point == pytest.approx([0.4, 0.0], abs=0.02)

    to_goal = detector.detect(segments, unit(math.pi / 2))
    assert to_goal.point[1] == pytest.approx(0.7)
    assert abs(to_goal.point[0]) <= 0.05


def test_obstacle_stays_on_path_while_robot_turns_along_wall():
    # World: goal straight ahead (+x), wall across the path at x = 0.35
    obstacle_world = np.array([0.35, 0.05])
    normal_world = np.array([-1.0, 0.0])
    state = ObstacleState(avoid_distance=0.5, path_radius=0.2)

    for robot_yaw in (0.0, -0.6, -1.2):  # Robot turning right along the wall
        # Robot frame: rotate world vectors by -yaw
        position = rotate(obstacle_world, -robot_yaw)[0]
        normal = rotate(normal_world, -robot_yaw)[0]
        goal_direction = unit(-robot_yaw)

        state.update(position, normal, goal_direction)
        assert state.avoiding, f"left avoidance at yaw {robot_yaw}"

        # Along the heading instead, the obstacle leaves the strip once turned
        heading_only = ObstacleState(avoid_distance=0.5, path_radius=0.2)
        heading_only.update(position, normal)
        if robot_yaw <= -0.6:
            assert not heading_only.avoiding


def test_obstacle_off_goal_line_does_not_trigger():
    # Obstacle straight ahead, but the goal is to the left
    state = ObstacleState(avoid_distance=0.5, path_radius=0.2)
    state.update(np.array([0.3, 0.0]), np.array([-1.0, 0.0]), unit(math.pi / 2))
    assert not state.avoiding


def test_zone_capped_at_goal():
    points = np.array([[0.3, 0.0], [0.6, 0.0]])
    assert in_path_zone(points, HEADING, 0.2, path_length=0.5).tolist() == [True, False]
    assert in_path_zone(points, HEADING, 0.2).tolist() == [True, True]


def test_detector_ignores_wall_beyond_goal():
    # Goal 0.3 m ahead, wall 0.4 m ahead: nothing on the way
    wall = np.array([[0.4, y] for y in np.linspace(-0.3, 0.3, 31)])
    segments = wall_segments(wall)
    detector = ObstacleDetector(path_radius=0.2)
    assert detector.detect(segments, HEADING, path_length=0.3) is None
    assert detector.detect(segments, HEADING, path_length=0.5) is not None


def test_no_avoidance_for_obstacle_beyond_goal():
    state = ObstacleState(avoid_distance=0.5, path_radius=0.2)
    state.update(np.array([0.35, 0.0]), np.array([-1.0, 0.0]), HEADING, path_length=0.3)
    assert not state.avoiding
    state.update(np.array([0.35, 0.0]), np.array([-1.0, 0.0]), HEADING, path_length=1.0)
    assert state.avoiding


def test_wall_side_chosen_towards_goal():
    # Wall straight ahead (normal towards the robot); the goal decides the side
    position, normal = np.array([0.3, 0.0]), np.array([-1.0, 0.0])
    for goal_angle, ccw, wall in ((0.4, False, [0.0, 1.0]), (-0.4, True, [0.0, -1.0])):
        state = ObstacleState(avoid_distance=0.5, path_radius=0.2)
        wall_vector = state.update(position, normal, unit(goal_angle))
        assert state.avoiding
        assert state.ccw == ccw
        assert wall_vector == pytest.approx(wall)


def test_wall_side_ignores_heading():
    # Slanted wall whose tangent along the heading points away from the goal:
    # the heading rule would pick CCW (forward-right), the goal on the left picks CW
    normal = np.array([-0.9, -0.44]) / np.linalg.norm([-0.9, -0.44])
    state = ObstacleState(avoid_distance=0.5, path_radius=0.2)
    wall_vector = state.update(np.array([0.25, 0.05]), normal, unit(0.6))
    assert state.avoiding
    assert not state.ccw
    assert wall_vector @ unit(0.6) > 0.0
    assert wall_vector[0] < 0.0  # Points backwards relative to the heading
