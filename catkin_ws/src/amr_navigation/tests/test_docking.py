#!/usr/bin/env python3
"""
Unit tests for PalletDockingController algorithm.
Tests reverse insertion calculations, heading lock, and stopping conditions.
"""
import math
import os
import sys
import unittest

_pkg_src = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if os.path.isdir(_pkg_src) and _pkg_src not in sys.path:
    sys.path.insert(0, _pkg_src)

from amr_navigation.move_to_point import normalize_angle, deg_to_rad
from amr_navigation.docking import PalletDockingController, compute_dock_target_pose, compute_dock_base_pose, reverse_line_command


class PalletDockingTests(unittest.TestCase):
    def setUp(self):
        self.ctrl = PalletDockingController.__new__(PalletDockingController)
        self.ctrl.default_dock_speed = 0.08
        self.ctrl.default_tolerance = 0.03

    def test_compute_dock_target_pose_270_deg(self):
        # Pallet cavity at (4.0, 5.0), docking yaw 270 deg (facing south)
        # Forks are 0.35m behind drive_center (in rear -X direction)
        dx, dy = compute_dock_target_pose(pallet_x=4.0, pallet_y=5.0, dock_yaw_deg=270.0, fork_offset=0.35)
        # cos(270) = 0, sin(270) = -1 -> drive_y = 5.0 - 0.35 = 4.65
        self.assertAlmostEqual(dx, 4.0, places=4)
        self.assertAlmostEqual(dy, 4.65, places=4)

        # Backward compatibility alias test
        bx, by = compute_dock_base_pose(pallet_x=4.0, pallet_y=5.0, dock_yaw_deg=270.0, fork_offset=0.35)
        self.assertAlmostEqual(bx, 4.0, places=4)
        self.assertAlmostEqual(by, 4.65, places=4)

    def test_compute_dock_target_pose_cardinal_directions(self):
        # Facing 0 deg (East, +X), backing into pallet at (5.0, 4.0)
        dx_0, dy_0 = compute_dock_target_pose(pallet_x=5.0, pallet_y=4.0, dock_yaw_deg=0.0, fork_offset=0.35)
        self.assertAlmostEqual(dx_0, 5.35, places=4)
        self.assertAlmostEqual(dy_0, 4.0, places=4)

        # Facing 180 deg (West, -X), backing into pallet at (3.0, 4.0)
        dx_180, dy_180 = compute_dock_target_pose(pallet_x=3.0, pallet_y=4.0, dock_yaw_deg=180.0, fork_offset=0.35)
        self.assertAlmostEqual(dx_180, 2.65, places=4)
        self.assertAlmostEqual(dy_180, 4.0, places=4)

        # Facing 90 deg (North, +Y), backing into pallet at (4.0, 3.0)
        dx_90, dy_90 = compute_dock_target_pose(pallet_x=4.0, pallet_y=3.0, dock_yaw_deg=90.0, fork_offset=0.35)
        self.assertAlmostEqual(dx_90, 4.0, places=4)
        self.assertAlmostEqual(dy_90, 3.35, places=4)

    def test_reverse_docking_geometry(self):
        # Robot drive_center at (4.0, 4.0), facing 270 deg (yaw = -pi/2, pointing towards -Y)
        # Target drive_center is at (4.0, 4.65)
        rx, ry, yaw = 4.0, 4.0, -math.pi / 2
        tx, ty = 4.0, 4.65

        dx = tx - rx
        dy = ty - ry
        # Longitudinal distance along robot heading:
        d_long = dx * math.cos(yaw) + dy * math.sin(yaw)
        # Target is directly behind robot: d_long must be -0.65m
        self.assertAlmostEqual(d_long, -0.65, places=4)

        # Cross-track error (lateral):
        d_lat = -dx * math.sin(yaw) + dy * math.cos(yaw)
        self.assertAlmostEqual(d_lat, 0.0, places=4)

    def test_reverse_speed_command(self):
        # When target is 1.0m behind, speed must be negative and bounded by max speed
        d_long = -1.0
        v_max = 0.08
        v_cmd = -min(v_max, max(0.04, 0.45 * abs(d_long)))
        self.assertEqual(v_cmd, -0.08)

        # When target is close (e.g. 5 cm behind), creep speed of 0.04 m/s is maintained
        d_long_close = -0.05
        v_cmd_close = -min(v_max, max(0.04, 0.45 * abs(d_long_close)))
        self.assertEqual(v_cmd_close, -0.04)

    def test_reverse_line_command_passes_entry_gate(self):
        # Replays the measured Milestone_1 staging error (3.1 cm, 3.9 deg) and its mirror image.
        # Unity under-rotates (~0.85x commanded yaw rate), so the plant model does too.
        tx, ty, line_yaw = 4.0, 4.78, deg_to_rad(270.0)
        for x0, yaw0_deg in ((3.969, -86.13), (4.031, -93.87)):
            x, y, yaw = x0, ty + 1.2 * math.sin(line_yaw), math.radians(yaw0_deg)  # 1.2 m standoff
            gate = None
            for _ in range(2000):
                v, w, along, e_lat, e_yaw = reverse_line_command(x, y, yaw, tx, ty, line_yaw, 0.12)
                if gate is None and along <= 0.50:
                    gate = (e_lat, e_lat - 0.54 * math.sin(e_yaw), e_yaw)
                if along <= 0.0:
                    break
                yaw += 0.85 * w * 0.05
                x += v * math.cos(yaw) * 0.05
                y += v * math.sin(yaw) * 0.05
            self.assertIsNotNone(gate)
            e_lat_gate, tip_lat_gate, _ = gate
            self.assertLess(abs(e_lat_gate), 0.008)
            self.assertLess(abs(tip_lat_gate), 0.008)
            self.assertLess(abs(e_lat), 0.005)

    def test_staging_station_dropoff_clearances(self):
        # 25cm tall staging station
        z_station = 0.250

        # Transit height q = 0.28m: pallet bottom must clear station
        q_transit = 0.280
        z_pallet_bottom_transit = 0.010 + q_transit
        self.assertGreater(z_pallet_bottom_transit, z_station)
        clearance_transit = z_pallet_bottom_transit - z_station
        self.assertAlmostEqual(clearance_transit, 0.040, places=3)  # 40mm clearance

        # Deposit height q = 0.21m:
        # Pallet sits on station at z = 0.250m, pocket ceiling at 0.310m
        q_deposit = 0.210
        z_fork_bottom = 0.050 + q_deposit  # 0.260m
        z_fork_top = 0.070 + q_deposit     # 0.280m
        z_pocket_ceiling = z_station + 0.060  # 0.310m

        # Forks must float above station deck and below pocket ceiling
        self.assertGreater(z_fork_bottom, z_station)
        self.assertLess(z_fork_top, z_pocket_ceiling)
        self.assertAlmostEqual(z_fork_bottom - z_station, 0.010, places=3)  # 10mm gap to deck
        self.assertAlmostEqual(z_pocket_ceiling - z_fork_top, 0.030, places=3)  # 30mm gap to ceiling

        # Station Dock at (13.0, 0.0), facing 180 deg
        tx, ty = compute_dock_target_pose(pallet_x=13.0, pallet_y=0.0, dock_yaw_deg=180.0, fork_offset=0.22)
        self.assertAlmostEqual(tx, 12.78, places=2)
        self.assertAlmostEqual(ty, 0.0, places=2)

        # Standoff waypoint: 0.8m in front along heading (cos 180 = -1)
        stage_x = tx + 0.8 * math.cos(math.radians(180.0))
        self.assertAlmostEqual(stage_x, 11.98, places=2)
        # Front bumper at standoff: 11.98 + 0.33 = 12.31m (< 13.0m dock, ~0.7m clearance!)
        self.assertLess(stage_x + 0.33, 13.0)


if __name__ == "__main__":
    unittest.main()
