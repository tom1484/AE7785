import math

import pytest

from team85_final.maze import Cell, CellPose, Heading, MazeGrid


def grid3x6(**kw):
    return MazeGrid.from_dict({'rows': 3, 'cols': 6, 'cell_size': 1.0,
                               'walls': [[0, 2, 'E'], [1, 2, 'N']], **kw})


def test_heading_turns():
    assert Heading.EAST.left is Heading.NORTH
    assert Heading.EAST.right is Heading.SOUTH
    assert Heading.NORTH.back is Heading.SOUTH
    assert Heading.from_yaw(math.pi * 0.9) is Heading.WEST
    assert Heading.from_yaw(-math.pi / 2 + 0.3) is Heading.SOUTH
    assert Heading.parse('n') is Heading.NORTH


def test_walls_both_sides_and_boundary():
    g = grid3x6()
    assert g.blocked(Cell(0, 2), Heading.EAST)
    assert g.blocked(Cell(0, 3), Heading.WEST)
    assert g.blocked(Cell(2, 2), Heading.SOUTH)
    assert g.blocked(Cell(0, 0), Heading.SOUTH) and g.blocked(Cell(0, 5), Heading.EAST)
    assert not g.blocked(Cell(0, 0), Heading.EAST)


def test_drive_to_wall():
    g = grid3x6()
    assert g.drive_to_wall(CellPose(Cell(0, 0), Heading.EAST)) == CellPose(Cell(0, 2), Heading.EAST)
    assert g.drive_to_wall(CellPose(Cell(1, 0), Heading.EAST)) == CellPose(Cell(1, 5), Heading.EAST)
    p = CellPose(Cell(0, 2), Heading.EAST)
    assert g.drive_to_wall(p) == p


def test_dict_roundtrip():
    g = grid3x6()
    g2 = MazeGrid.from_dict(g.to_dict())
    assert set(g2.wall_faces()) == set(g.wall_faces())
    assert g.to_dict()['walls'] == [[0, 2, 'E'], [1, 2, 'N']]


@pytest.mark.parametrize('yaw', [0.0, 0.4, -2.0])
def test_map_frame_roundtrip(yaw):
    g = grid3x6(origin=[-0.8, 1.5], yaw=yaw)
    for cell in g.cells():
        for h in Heading:
            x, y, th = g.pose_to_map(CellPose(cell, h))
            assert g.locate(x, y, th + 0.2) == CellPose(cell, h)
    assert g.locate(-100.0, 0.0, 0.0) is None


def test_path_length_goes_around_walls():
    g = grid3x6()
    assert g.path_length(Cell(0, 2), Cell(0, 3)) == 3   # up, across, down
    assert g.path_length(Cell(0, 0), Cell(0, 0)) == 0


def test_render():
    out = grid3x6().render({Cell(0, 0): 'R'})
    assert ' R ' in out and out.count('\n') == 6
