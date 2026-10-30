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

from amr_navigation.pallet_detect import detect_load_face, detect_rack_legs, detect_station, scan_to_points

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


def rack_runners(px, py, dock_yaw_deg, slots=(-2, -1, 0, 1, 2), pitch=0.42, width=0.08, length=0.40,
                 leg_offset=0.19):
    """Rack-DetectReady2 floor runners as box outlines: runner k sits at (k - 0.5) * pitch from the bay
    centre (px, py); slot 0 and 1 flank the bay. Front faces leg_offset toward the aisle."""
    ux, uy = math.cos(math.radians(dock_yaw_deg)), math.sin(math.radians(dock_yaw_deg))
    vx, vy = -uy, ux
    segs = []
    for k in slots:
        m = (k - 0.5) * pitch
        c = [(px + (m + sl * width / 2) * vx + (leg_offset - sd * length) * ux,
              py + (m + sl * width / 2) * vy + (leg_offset - sd * length) * uy)
             for sl, sd in ((-1, 0), (1, 0), (1, 1), (-1, 1))]
        segs += [(c[i], c[(i + 1) % 4]) for i in range(4)]
    return segs


BACK_WALL = [((2.0, 5.6), (6.0, 5.6))]


class RackLegDetectionTests(unittest.TestCase):
    """Laser below the bottom plank: only the floor runners and whatever stands in the bay throat are hit."""

    def test_finds_shifted_and_rotated_bay(self):
        for seed in range(5):
            for true_x, true_yaw in ((4.0, 270.0), (4.03, 270.0), (3.96, 268.0), (4.02, 272.5), (4.011, 269.0)):
                pts = fake_scan(rack_runners(true_x, 5.0, true_yaw) + BACK_WALL, seed=seed)
                pose, why = detect_rack_legs(pts, 4.0, 5.0, 270.0)
                self.assertIsNone(why, why)
                x, y, yaw = pose
                self.assertLess(abs(x - true_x), 0.006)   # lateral: inside the 8 mm entry gate
                self.assertLess(abs(y - 5.0), 0.010)
                self.assertLess(abs(yaw - true_yaw), 1.0)

    def test_empty_and_loaded_bay_give_same_pose(self):
        # Something low in the throat (e.g. a dropped board 0.25 m in) must not move the result
        clutter = [((3.9, 4.81 + 0.25), (4.1, 4.81 + 0.25))]
        for seed in range(5):
            empty, why = detect_rack_legs(fake_scan(rack_runners(4.0, 5.0, 270.0) + BACK_WALL, seed=seed),
                                          4.0, 5.0, 270.0)
            self.assertIsNone(why, why)
            full, why = detect_rack_legs(fake_scan(rack_runners(4.0, 5.0, 270.0) + clutter + BACK_WALL, seed=seed),
                                         4.0, 5.0, 270.0)
            self.assertIsNone(why, why)
            self.assertLess(math.hypot(empty[0] - full[0], empty[1] - full[1]), 0.002)

    def test_picks_target_bay_not_neighbour(self):
        # Expectation 0.12 m off toward the next bay: still the nearest-centred pair, not the neighbour
        pts = fake_scan(rack_runners(4.0, 5.0, 270.0) + BACK_WALL)
        pose, why = detect_rack_legs(pts, 4.08, 5.0, 270.0)
        self.assertIsNone(why, why)
        self.assertLess(abs(pose[0] - 4.0), 0.006)

    def test_rejects_missing_leg(self):
        pts = fake_scan(rack_runners(4.0, 5.0, 270.0, slots=(-2, -1, 0, 2)) + BACK_WALL)
        pose, why = detect_rack_legs(pts, 4.0, 5.0, 270.0)
        self.assertIsNone(pose)
        self.assertIsNotNone(why)

    def test_rejects_no_rack(self):
        pose, why = detect_rack_legs(fake_scan(BACK_WALL), 4.0, 5.0, 270.0)
        self.assertIsNone(pose)

    def test_181_rays_still_inside_gate(self):
        for seed in range(5):
            pts = fake_scan(rack_runners(4.02, 5.0, 270.0) + BACK_WALL, seed=seed, rays=181)
            pose, why = detect_rack_legs(pts, 4.0, 5.0, 270.0)
            self.assertIsNone(why, why)
            self.assertLess(abs(pose[0] - 4.02), 0.010)


def workbench(px, py, dock_yaw_deg, width=0.60, depth=0.80):
    """Workbench footprint as a box outline: `width` across the aisle-side face, `depth` along the dock axis."""
    ux, uy = math.cos(math.radians(dock_yaw_deg)), math.sin(math.radians(dock_yaw_deg))
    vx, vy = -uy, ux
    c = [(px + sl * width / 2 * vx + sd * depth / 2 * ux, py + sl * width / 2 * vy + sd * depth / 2 * uy)
         for sl, sd in ((-1, 1), (1, 1), (1, -1), (-1, -1))]
    return [(c[i], c[(i + 1) % 4]) for i in range(4)]


# Station staging: drive_center 0.54 + 0.80 m from the centre, laser 0.33 m ahead -> face 0.61 m from the laser
STATION_Y = LASER[1] + 0.61 + 0.40


class StationDetectionTests(unittest.TestCase):
    def test_finds_shifted_and_rotated_station(self):
        for seed in range(5):
            for true_x, true_yaw in ((4.0, 270.0), (4.03, 270.0), (3.96, 268.0), (4.02, 272.5)):
                pts = fake_scan(workbench(true_x, STATION_Y, true_yaw) + BACK_WALL, seed=seed)
                pose, why = detect_station(pts, 4.0, STATION_Y, 270.0)
                self.assertIsNone(why, why)
                self.assertLess(abs(pose[0] - true_x), 0.006)
                self.assertLess(abs(pose[1] - STATION_Y), 0.010)
                self.assertLess(abs(pose[2] - true_yaw), 1.0)

    def test_tolerates_amcl_error_up_to_20_cm(self):
        # station_1 / station_3 runs: AMCL put the station 10.2 cm to the side, past the rack's 10 cm limit
        for seed in range(3):
            for true_x in (4.102, 3.85, 4.18):
                pts = fake_scan(workbench(true_x, STATION_Y, 270.0) + BACK_WALL, seed=seed)
                pose, why = detect_station(pts, 4.0, STATION_Y, 270.0)
                self.assertIsNone(why, why)
                self.assertLess(abs(pose[0] - true_x), 0.006)
        pts = fake_scan(workbench(4.25, STATION_Y, 270.0) + BACK_WALL)
        self.assertIsNone(detect_station(pts, 4.0, STATION_Y, 270.0)[0])  # 25 cm off: rejected

    def test_rejects_missing_station(self):
        pose, why = detect_station(fake_scan(BACK_WALL), 4.0, STATION_Y, 270.0)
        self.assertIsNone(pose)

    def test_rejects_wrong_size_face(self):
        # The 0.80 m side of the box facing the robot would mean the map has the station turned 90 deg
        pts = fake_scan(workbench(4.0, STATION_Y, 270.0, width=0.80, depth=0.60) + BACK_WALL)
        pose, why = detect_station(pts, 4.0, STATION_Y, 270.0)
        self.assertIsNone(pose)


if __name__ == "__main__":
    unittest.main()
