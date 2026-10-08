import math
from types import SimpleNamespace
import numpy as np
from robot_demo.core import (
    gps_to_xy,
    xy_to_gps,
    astar,
    inflate,
    depth_points,
    above_floor,
)

DATUM = {"latitude": 55.0, "longitude": 37.0}


def test_gps_round_trip_and_axes():
    for x, y in [(0, 0), (8, 0), (-3, 4), (10, -2)]:
        assert np.allclose(gps_to_xy(*xy_to_gps(x, y, DATUM), DATUM), (x, y), atol=1e-7)
    assert gps_to_xy(55.001, 37, DATUM)[1] > 0
    assert gps_to_xy(55, 37.001, DATUM)[0] > 0


def test_astar_detours_and_rejects_unreachable_goal():
    grid = np.zeros((20, 20), dtype=bool)
    grid[3:17, 10] = True
    path = astar(grid, (3, 10), (17, 10))
    assert path[0] == (3, 10) and path[-1] == (17, 10)
    assert all(not grid[y, x] for x, y in path)
    assert any(y < 3 or y >= 17 for x, y in path)
    grid[:, 10] = True
    assert not astar(grid, (3, 10), (17, 10))


def test_no_diagonal_corner_cutting():
    grid = np.zeros((2, 2), dtype=bool)
    grid[0, 1] = True
    grid[1, 0] = True
    assert not astar(grid, (0, 0), (1, 1))


def test_robot_footprint_inflation():
    grid = np.zeros((30, 30))
    grid[15, 15] = 100
    blocked = inflate(grid, 0.5, 0.1)
    assert blocked[15, 20] and not blocked[15, 21]
    assert blocked[0, 10] and not blocked[6, 10]


def test_depth_projection_padding_units_and_invalid_values():
    arr = np.full((8, 10), 2000, dtype="<u2")
    arr[0, 0] = 0
    msg = SimpleNamespace(
        width=8,
        height=8,
        encoding="16UC1",
        is_bigendian=False,
        step=20,
        data=arr.tobytes(),
    )
    points = depth_points(msg)
    assert points.shape == (3, 3) and np.allclose(points[:, 0], 2.0)
    assert np.all(np.isfinite(points))
    arr32 = np.full((8, 8), 2, dtype=">f4")
    arr32[0, 0] = np.nan
    msg = SimpleNamespace(
        width=8,
        height=8,
        encoding="32FC1",
        is_bigendian=True,
        step=32,
        data=arr32.tobytes(),
    )
    assert len(depth_points(msg)) == 3


def test_floor_segmentation_with_pitch_and_low_box():
    x, y = np.meshgrid(np.linspace(0.7, 5, 40), np.linspace(-1, 1, 20))
    floor = np.column_stack((x.ravel(), y.ravel(), 0.03 * x.ravel() + 0.01))
    box = np.array(
        [[1.1, y, 0.16 + 0.03 * 1.1 + 0.01] for y in np.linspace(-0.3, 0.3, 20)]
    )
    points = np.vstack((floor, box))
    mask = above_floor(points)
    assert not mask[: len(floor)].any()
    assert mask[len(floor) :].all()
