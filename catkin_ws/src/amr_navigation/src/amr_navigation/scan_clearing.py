"""
Clearing-only scan for the costmaps (no ROS imports, unit-testable).

costmap_2d's ObstacleLayer only clears a cell when a laser ray passes through it. Two gaps leave trails behind
moving obstacles:
  * The 1 degree rays are ~5 cm (one cell) apart at 3 m and wider beyond, so once the robot has moved or turned,
    cells marked by a passing obstacle can sit between rays and are never cleared.
    -> densify(): extra rays between neighbours, reaching only as far as the nearer of the two returns.
  * The lidar sees only the front 180 degrees. Marks an obstacle leaves beside or behind the robot are never
    cleared (see docs/ghost-obstacle-problem-ticket.md).
    -> blind_ranges(): rays through the rear half, but only through cells the static map says are free, stopping
       map_margin short of mapped racks/walls so the laser marks of those are kept.

scripts/scan_clearing.py publishes the result as one 360 degree LaserScan in the laser frame. costmap_common.yaml
uses it as an observation source with marking: false, clearing: true, so it can erase but never add obstacles.
"""
import math

import numpy as np


def densify(ranges, factor):
    """Insert factor-1 rays between neighbours with the nearer neighbour's range.

    inf (no hit) stays inf only when both neighbours are inf; nan / -inf (invalid) propagate, so nothing is
    cleared next to an invalid return. Returns (n - 1) * factor + 1 ranges.
    """
    r = np.asarray(ranges, dtype=float)
    if factor <= 1 or len(r) < 2:
        return r.copy()
    out = np.empty((len(r) - 1) * factor + 1)
    out[::factor] = r
    nearer = np.minimum(r[:-1], r[1:])
    for k in range(1, factor):
        out[k::factor] = nearer
    return out


def blind_ranges(angles, pose, grid, max_range, margin):
    """Free distance along each ray according to the static map.

    angles: ray angles in the laser frame. pose: (x, y, yaw) of the laser in the map frame.
    grid: OccupancyGrid-like (resolution, origin_x, origin_y, width, height, data as row-major sequence).
    Unknown (-1), occupied (>= 50) and off-map cells block a ray. Returns the distance to the first blocked cell
    minus margin (capped at max_range), or nan when that leaves nothing to clear.
    """
    res, ox, oy, w, h, data = grid
    occ = np.asarray(data, dtype=np.int16).reshape(h, w)
    blocked = (occ < 0) | (occ >= 50)
    x, y, yaw = pose
    step = res * 0.5
    r = np.arange(0.0, max_range + step, step)
    a = np.asarray(angles, dtype=float)[:, None] + yaw
    ix = np.floor((x + r * np.cos(a) - ox) / res).astype(int)
    iy = np.floor((y + r * np.sin(a) - oy) / res).astype(int)
    inside = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
    hit = ~inside
    hit[inside] = blocked[iy[inside], ix[inside]]
    first = np.where(hit.any(axis=1), hit.argmax(axis=1), len(r))
    free = np.where(first < len(r), r[np.minimum(first, len(r) - 1)] - margin, max_range)
    free = np.minimum(free, max_range)
    return np.where(free > res, free, np.nan)


def clearing_scan(ranges, angle_min, angle_inc, factor, pose=None, grid=None, max_range=4.0, margin=0.25):
    """Build the 360 degree clearing scan.

    Returns (ranges, angle_min, angle_inc): densified front rays, extended on both sides around to the back with
    map-limited blind rays (nan without pose/grid, i.e. nothing cleared there).
    """
    front = densify(ranges, factor)
    inc = angle_inc / factor
    front_max = angle_min + (len(front) - 1) * inc
    n_low = max(0, int(math.floor((angle_min + math.pi) / inc)))
    n_high = max(0, int(math.floor((angle_min + 2 * math.pi - front_max) / inc)) - 1 - n_low)
    low = angle_min - inc * np.arange(n_low, 0, -1)
    high = front_max + inc * np.arange(1, n_high + 1)
    if pose is None or grid is None:
        low_r, high_r = np.full(n_low, np.nan), np.full(n_high, np.nan)
    else:
        rear = blind_ranges(np.concatenate([low, high]), pose, grid, max_range, margin)
        low_r, high_r = rear[:n_low], rear[n_low:]
    return np.concatenate([low_r, front, high_r]), angle_min - n_low * inc, inc
