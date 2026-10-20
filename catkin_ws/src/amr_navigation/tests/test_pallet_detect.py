#!/usr/bin/env python3
"""Load-face detection on synthetic scans: laser at the rack stage, 361 rays over 180 deg, 3 mm noise.
At 181 rays (1 deg) the edge quantisation alone reaches ~8 mm lateral, the whole fork clearance."""
import math
import os
import random
import sys
import unittest

_pkg_src = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if os.path.isdir(_pkg_src) and _pkg_src not in sys.path:
    sys.path.insert(0, _pkg_src)

from amr_navigation.pallet_detect import detect_load_face, scan_to_points

LASER = (4.0, 3.91, math.radians(90.0))  # drive_center stage (4.00, 3.58) + 0.33 m, facing the rack


def face_segment(px, py, dock_yaw_deg, width=0.35, face_offset=0.15):
    ux, uy = math.cos(math.radians(dock_yaw_deg)), math.sin(math.radians(dock_yaw_deg))
    fx, fy = px + face_offset * ux, py + face_offset * uy
    vx, vy = -uy, ux
    return (fx - 0.5 * width * vx, fy - 0.5 * width * vy), (fx + 0.5 * width * vx, fy + 0.5 * width * vy)


def ray_hit(ox, oy, dx, dy, seg):
    (ax, ay), (bx, by) = seg
    ex, ey = bx - ax, by - ay
    den = dx * ey - dy * ex
    if abs(den) < 1e-12:
        return None
    t = ((ax - ox) * ey - (ay - oy) * ex) / den
    s = ((ax - ox) * dy - (ay - oy) * dx) / den
    return t if t > 0 and 0 <= s <= 1 else None


def fake_scan(segments, noise=0.003, seed=1, rays=361):
    rng = random.Random(seed)
    ox, oy, oyaw = LASER
    amin, inc = -math.pi / 2, math.pi / (rays - 1)
    ranges = []
    for i in range(rays):
        a = oyaw + amin + i * inc
        hits = [h for h in (ray_hit(ox, oy, math.cos(a), math.sin(a), s) for s in segments) if h is not None]
        ranges.append(min(hits) + rng.gauss(0.0, noise) if hits else float("inf"))
    return scan_to_points(ranges, amin, inc, 0.05, 10.0, ox, oy, oyaw)


BACKGROUND = [((2.0, 5.6), (6.0, 5.6)),            # wall behind the rack
              ((3.55, 4.6), (3.55, 5.4)),          # rack posts either side of the bay
              ((4.45, 4.6), (4.45, 5.4))]
# Milestone_1 rack row: loaded pallets every 0.42 m, so neighbouring box faces sit 7 cm away on the same line
NEIGHBOURS = [face_segment(4.0 + k * 0.42, 5.0, 270.0) for k in (-2, -1, 1, 2)]
# Rays into the 7 cm gaps hit the 0.30 m deep side walls of the neighbouring loads
NEIGHBOUR_SIDES = [((x, 4.85), (x, 5.15)) for x in (3.335, 3.755, 4.245, 4.665)]


class LoadFaceDetectionTests(unittest.TestCase):
    def test_finds_shifted_and_rotated_pallet(self):
        for seed in range(5):
            for true_x, true_yaw in ((4.0, 270.0), (4.03, 270.0), (3.96, 268.0), (4.02, 272.5), (4.011, 269.0)):
                pts = fake_scan([face_segment(true_x, 5.0, true_yaw)] + BACKGROUND, seed=seed)
                pose, why = detect_load_face(pts, 4.0, 5.0, 270.0)
                self.assertIsNone(why, why)
                x, y, yaw = pose
                self.assertLess(abs(x - true_x), 0.006)   # lateral: inside the 8 mm entry gate
                self.assertLess(abs(y - 5.0), 0.010)
                self.assertLess(abs(yaw - true_yaw), 1.0)   # single noisy scan; 0.0 deg without noise

    def test_separates_target_from_neighbouring_loads(self):
        for seed in range(5):
            for true_x in (4.0, 4.02, 3.98):
                pts = fake_scan([face_segment(true_x, 5.0, 270.0)] + NEIGHBOURS + NEIGHBOUR_SIDES + BACKGROUND,
                                seed=seed)
                pose, why = detect_load_face(pts, 4.0, 5.0, 270.0)
                self.assertIsNone(why, why)
                self.assertLess(abs(pose[0] - true_x), 0.006)
                self.assertLess(abs(pose[2] - 270.0), 1.0)

    def test_rejects_wrong_width(self):
        pts = fake_scan([face_segment(4.0, 5.0, 270.0, width=0.20)] + BACKGROUND)
        pose, why = detect_load_face(pts, 4.0, 5.0, 270.0)
        self.assertIsNone(pose)
        self.assertIn("width", why)

    def test_rejects_missing_load(self):
        pose, why = detect_load_face(fake_scan(BACKGROUND), 4.0, 5.0, 270.0)
        self.assertIsNone(pose)
        self.assertIsNotNone(why)

    def test_rejects_pallet_far_from_expected(self):
        pts = fake_scan([face_segment(4.15, 5.0, 270.0)] + BACKGROUND)
        pose, why = detect_load_face(pts, 4.0, 5.0, 270.0)
        self.assertIsNone(pose)


if __name__ == "__main__":
    unittest.main()
