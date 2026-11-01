"""Tests for the raycasting and the log-odds grid.

Runnable without ROS: `python3 -m pytest src/toio_omni_mapping/test`.

Raycasting fails quietly. An endpoint marked one cell early puts every wall
inside itself, an endpoint marked on a miss fills the horizon with phantoms, and
both produce a map that looks fine until a localiser is scored against it.
"""

import math

import numpy as np

from toio_omni_mapping.grid import UNKNOWN, LogOddsGrid, bresenham, log_odds


def make_grid(**kw):
    # 20 x 20 cells of 1 cm, origin at the corner, so a cell index is a
    # centimetre and the arithmetic in the tests stays readable.
    opts = dict(origin_x=0.0, origin_y=0.0, width_m=0.2, height_m=0.2,
                resolution=0.01)
    opts.update(kw)
    return LogOddsGrid(**opts)


def test_log_odds_sign():
    assert log_odds(0.7) > 0
    assert log_odds(0.4) < 0
    assert log_odds(0.5) == 0.0


def test_bresenham_excludes_the_endpoint():
    assert bresenham(0, 0, 3, 0) == [(0, 0), (1, 0), (2, 0)]
    assert bresenham(0, 0, 0, 0) == []


def test_bresenham_stays_inside_the_bounding_box():
    # Not symmetric under reversal -- integer Bresenham breaks ties towards one
    # side -- so the invariant worth asserting is that it never wanders. Which
    # cell a tie lands in does not matter here; the endpoint does, and
    # test_a_wall_seen_from_both_sides_lands_in_one_cell covers that.
    for x, y in bresenham(1, 2, 9, 7):
        assert 1 <= x <= 9 and 2 <= y <= 7


def test_bresenham_is_connected():
    cells = bresenham(0, 0, 11, 5) + [(11, 5)]
    for (x0, y0), (x1, y1) in zip(cells, cells[1:]):
        assert abs(x1 - x0) <= 1 and abs(y1 - y0) <= 1


def test_cell_and_bounds():
    g = make_grid(origin_x=-0.05, origin_y=-0.05)
    assert g.cols == 20 and g.rows == 20
    assert g.cell(-0.05, -0.05) == (0, 0)
    assert g.cell(0.0, 0.0) == (5, 5)
    # floor, not truncate: a negative coordinate must not round towards zero
    assert g.cell(-0.055, -0.055) == (-1, -1)
    assert not g.inside(-1, -1)
    assert g.inside(19, 19) and not g.inside(20, 19)


def test_a_hit_marks_the_endpoint_occupied_and_the_rest_free():
    g = make_grid()
    # from the centre of cell (2, 2), 5 cm along +x, so the wall is at cell (7, 2)
    g.integrate_ray(0.025, 0.025, 0.0, 0.05, hit=True)
    occ = g.occupancy()
    assert occ[2, 7] > 50, 'the endpoint should be occupied'
    for col in range(2, 7):
        assert 0 <= occ[2, col] < 50, 'the ray should clear what it passed through'
    assert occ[2, 8] == UNKNOWN, 'nothing beyond the wall was observed'


def test_a_miss_clears_the_whole_ray_including_the_horizon():
    g = make_grid()
    g.integrate_ray(0.025, 0.025, 0.0, 0.05, hit=False)
    occ = g.occupancy()
    # cells 2..7: a ray that found nothing only clears, and clears to the end,
    # or every open direction gets a ring of unknown at exactly range_max
    assert (occ[2, 2:8] < 50).all()
    assert (occ[2, 2:8] != UNKNOWN).all()
    assert occ[2, 8] == UNKNOWN


def test_untouched_cells_stay_unknown():
    g = make_grid()
    g.integrate_ray(0.025, 0.025, 0.0, 0.05, hit=True)
    assert (g.occupancy() == UNKNOWN).sum() == g.rows * g.cols - 6


def test_repeated_observations_converge_and_clamp():
    g = make_grid(l_clamp=2.0)
    for _ in range(50):
        g.integrate_ray(0.025, 0.025, 0.0, 0.05, hit=True)
    assert g.logodds[2, 7] == 2.0
    assert g.logodds[2, 3] == -2.0
    occ = g.occupancy()
    assert occ[2, 7] >= 88 and occ[2, 3] <= 12


def test_a_wall_seen_from_both_sides_lands_in_one_cell():
    # The same wall at x = 0.10 m, hit from the left and from the right. If the
    # endpoint handling were off by one these would be different cells.
    g = make_grid()
    g.integrate_ray(0.045, 0.045, 0.0, 0.06, hit=True)
    left = np.argwhere(g.occupancy()[4] > 50).ravel()
    g2 = make_grid()
    g2.integrate_ray(0.155, 0.045, math.pi, 0.05, hit=True)
    right = np.argwhere(g2.occupancy()[4] > 50).ravel()
    assert list(left) == list(right) == [10]


def test_the_sensor_outside_the_grid_is_reported():
    g = make_grid()
    assert g.integrate_ray(-1.0, -1.0, 0.0, 0.05, hit=True) is False
    assert (g.occupancy() == UNKNOWN).all()


def test_a_ray_leaving_the_grid_does_not_wrap():
    g = make_grid()
    g.integrate_ray(0.185, 0.095, 0.0, 0.10, hit=True)
    occ = g.occupancy()
    assert (occ[0:9] == UNKNOWN).all() and (occ[10:] == UNKNOWN).all()
    assert (occ[9, 0:18] == UNKNOWN).all(), 'clearing must not wrap to the far side'
