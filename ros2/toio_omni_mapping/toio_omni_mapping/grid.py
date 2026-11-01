"""The log-odds grid and the raycasting, with no ROS in sight.

Separated from the node so it can be tested on a machine with nothing but numpy.
Raycasting is the part that fails quietly: an off-by-one at the endpoint marks
the wall one cell inside itself, and the resulting map looks perfectly
reasonable while being wrong by a cell everywhere.
"""

import math

import numpy as np

UNKNOWN = -1


def log_odds(p):
    """Log-odds of a probability."""
    return math.log(p / (1.0 - p))


def bresenham(x0, y0, x1, y1):
    """Integer cells from (x0, y0) towards (x1, y1), excluding the endpoint.

    The endpoint is left out because it is the caller's business: it is a wall
    for a ray that hit something and free space for one that did not.
    """
    cells = []
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x1 > x0 else -1
    sy = 1 if y1 > y0 else -1
    err = dx - dy
    x, y = x0, y0
    while (x, y) != (x1, y1):
        cells.append((x, y))
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return cells


class LogOddsGrid:
    """Occupancy grid accumulated in log-odds, indexed [row, col] from the origin."""

    def __init__(self, origin_x, origin_y, width_m, height_m, resolution,
                 p_occ=0.7, p_free=0.4, l_clamp=4.0):
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.resolution = float(resolution)
        self.cols = int(round(width_m / resolution))
        self.rows = int(round(height_m / resolution))
        self.l_occ = log_odds(p_occ)
        self.l_free = log_odds(p_free)        # negative
        self.l_clamp = float(l_clamp)
        self.logodds = np.zeros((self.rows, self.cols), dtype=np.float32)
        self.touched = np.zeros((self.rows, self.cols), dtype=bool)

    def cell(self, x, y):
        """Cell containing a point in the map frame, as (col, row)."""
        return (int(math.floor((x - self.origin_x) / self.resolution)),
                int(math.floor((y - self.origin_y) / self.resolution)))

    def inside(self, col, row):
        """Report whether this cell is within the grid."""
        return 0 <= col < self.cols and 0 <= row < self.rows

    def known_fraction(self):
        """Fraction of cells that any ray has touched."""
        return float(self.touched.mean())

    def integrate_ray(self, sx, sy, angle, reach, hit):
        """Trace one ray from (sx, sy) and update the cells it crosses.

        `hit` says whether `reach` is a measured obstacle or just how far the
        ray was followed. A ray that found nothing still clears everything it
        passed through, which is what opens up the free space at all.
        """
        s_col, s_row = self.cell(sx, sy)
        if not self.inside(s_col, s_row):
            return False
        e_col, e_row = self.cell(sx + reach * math.cos(angle),
                                 sy + reach * math.sin(angle))
        for col, row in bresenham(s_col, s_row, e_col, e_row):
            if self.inside(col, row):
                self.logodds[row, col] = max(
                    self.logodds[row, col] + self.l_free, -self.l_clamp)
                self.touched[row, col] = True
        if self.inside(e_col, e_row):
            if hit:
                self.logodds[e_row, e_col] = min(
                    self.logodds[e_row, e_col] + self.l_occ, self.l_clamp)
            else:
                # Nothing was found at the horizon either, so that cell is free
                # as well. Leaving it out would draw a ring of unknown cells at
                # exactly range_max, all the way around every open direction.
                self.logodds[e_row, e_col] = max(
                    self.logodds[e_row, e_col] + self.l_free, -self.l_clamp)
            self.touched[e_row, e_col] = True
        return True

    def occupancy(self):
        """Render the grid as nav_msgs/OccupancyGrid values: 0-100, or -1 for unknown."""
        prob = 1.0 - 1.0 / (1.0 + np.exp(self.logodds))
        data = np.full((self.rows, self.cols), UNKNOWN, dtype=np.int8)
        known = np.rint(prob * 100.0).astype(np.int8)
        data[self.touched] = known[self.touched]
        return data
