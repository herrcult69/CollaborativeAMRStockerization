#!/usr/bin/env python3
"""
Unit tests for MoveToPointController algorithm.
Tests state transitions, heading normalization, degree-to-radian conversion,
and closed-loop trajectory convergence.
"""
import math
import os
import sys
import unittest

_pkg_src = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if os.path.isdir(_pkg_src) and _pkg_src not in sys.path:
    sys.path.insert(0, _pkg_src)

from amr_navigation.move_to_point import normalize_angle, deg_to_rad, MoveToPointController


class MoveToPointControllerTests(unittest.TestCase):
    def setUp(self):
        self.ctrl = MoveToPointController.__new__(MoveToPointController)
        self.ctrl.gx = 2.0
        self.ctrl.gy = 0.0
        self.ctrl.goal_yaw = 0.0
        self.ctrl.pos_tolerance = 0.10
        self.ctrl.yaw_tolerance = 0.05
        self.ctrl.heading_align_threshold = 0.20
        self.ctrl.max_linear = 0.25
        self.ctrl.min_linear = 0.04
        self.ctrl.max_angular = 0.65
        self.ctrl.min_angular = 0.42
        self.ctrl.wheel_offset_x = 0.0
        self.ctrl.state = MoveToPointController.STATE_ALIGN_TO_GOAL

    def test_angle_normalization(self):
        self.assertAlmostEqual(normalize_angle(0.0), 0.0)
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), math.pi, places=5)
        self.assertAlmostEqual(normalize_angle(-3.0 * math.pi), -math.pi, places=5)
        self.assertAlmostEqual(normalize_angle(math.pi / 2), math.pi / 2, places=5)

    def test_deg_to_rad_conversion(self):
        self.assertAlmostEqual(deg_to_rad(0.0), 0.0)
        self.assertAlmostEqual(deg_to_rad(90.0), math.pi / 2, places=5)
        self.assertAlmostEqual(deg_to_rad(-90.0), -math.pi / 2, places=5)
        self.assertAlmostEqual(abs(deg_to_rad(180.0)), math.pi, places=5)
        self.assertAlmostEqual(deg_to_rad(360.0), 0.0, places=5)

    def test_phase1_in_place_rotation_when_facing_away(self):
        linear, angular, dist, active_err, yaw_err = self.ctrl.compute_step(0.0, 0.0, math.pi / 2)
        self.assertEqual(linear, 0.0)
        self.assertLess(angular, 0.0)
        self.assertEqual(self.ctrl.state, MoveToPointController.STATE_ALIGN_TO_GOAL)

    def test_already_at_goal_transitions_to_phase3(self):
        self.ctrl.state = MoveToPointController.STATE_ALIGN_TO_GOAL
        self.ctrl.goal_yaw = math.pi / 2
        linear, angular, dist, active_err, yaw_err = self.ctrl.compute_step(2.0, 0.0, 0.0)
        self.assertEqual(self.ctrl.state, MoveToPointController.STATE_ALIGN_FINAL_YAW)
        self.assertEqual(linear, 0.0)
        self.assertGreater(angular, 0.0)

    def test_transition_to_phase2_when_heading_aligned(self):
        self.ctrl.compute_step(0.0, 0.0, 0.05)
        self.assertEqual(self.ctrl.state, MoveToPointController.STATE_DRIVE_TO_GOAL)
        linear, angular, dist, active_err, yaw_err = self.ctrl.compute_step(0.0, 0.0, 0.05)
        self.assertGreater(linear, 0.0)

    def test_transition_to_phase3_and_docking_alignment(self):
        self.ctrl.state = MoveToPointController.STATE_DRIVE_TO_GOAL
        self.ctrl.goal_yaw = math.pi / 2
        self.ctrl.compute_step(1.95, 0.0, 0.0)
        self.assertEqual(self.ctrl.state, MoveToPointController.STATE_ALIGN_FINAL_YAW)
        linear, angular, dist, active_err, yaw_err = self.ctrl.compute_step(1.95, 0.0, 0.0)
        self.assertEqual(linear, 0.0)
        self.assertGreater(angular, 0.0)

    def test_full_convergence_simulation(self):
        self.ctrl.gx = 2.0
        self.ctrl.gy = 1.0
        self.ctrl.goal_yaw = -math.pi / 2
        self.ctrl.state = MoveToPointController.STATE_ALIGN_TO_GOAL

        x, y, yaw = 0.0, 0.0, math.pi / 4
        dt = 0.05

        for step in range(3000):
            linear, angular, dist, active_err, yaw_err = self.ctrl.compute_step(x, y, yaw)
            if self.ctrl.state == MoveToPointController.STATE_ARRIVED:
                break
            x += linear * math.cos(yaw) * dt
            y += linear * math.sin(yaw) * dt
            yaw = normalize_angle(yaw + angular * dt)

        self.assertEqual(self.ctrl.state, MoveToPointController.STATE_ARRIVED)
        final_dist = math.hypot(self.ctrl.gx - x, self.ctrl.gy - y)
        final_yaw_err = abs(normalize_angle(self.ctrl.goal_yaw - yaw))
        self.assertLessEqual(final_dist, self.ctrl.pos_tolerance)
        self.assertLessEqual(final_yaw_err, self.ctrl.yaw_tolerance)


if __name__ == "__main__":
    unittest.main()
