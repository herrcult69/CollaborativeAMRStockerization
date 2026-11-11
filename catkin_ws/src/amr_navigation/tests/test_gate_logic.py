#!/usr/bin/env python3
"""Unit tests for VelocityGate decision logic (Option B: Industrial Safety Shield + Closing-Rate Yield).
Validates ISO 3691-4 safety principles without requiring ROS runtime or TF lookups.
"""
import os
import sys
import unittest

_pkg_src = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "amr_navigation"))
if os.path.isdir(_pkg_src) and _pkg_src not in sys.path:
    sys.path.insert(0, _pkg_src)

from gate_logic import DEFAULTS, FREE, GIVEUP, HOLD, GateLogic


def make_cluster(center_x, center_y, n_pts=3, spacing=0.03):
    """Helper: create an obstacle cluster of n_pts around (center_x, center_y)."""
    half = (n_pts - 1) * spacing / 2.0
    return [(center_x, center_y - half + i * spacing) for i in range(n_pts)]


class TestGateLogic(unittest.TestCase):
    def setUp(self):
        # Instantiate gate with standard warehouse AMR parameters
        self.gate = GateLogic(
            nose_x=0.37,
            half_w=0.33,         # 0.23m half robot + 0.10m margin
            d_stop=0.35,         # 0.35m protective stop shield
            d_slow=1.50,         # 1.50m warning zone
            yield_dist=1.50,     # Head-on yield threshold
            closing_rate_thr=-0.20, # Gap closing faster than 0.2 m/s -> yield
            clear_time=0.80,     # Hysteresis release duration
            hold_timeout=20.0,
            min_cluster_pts=3    # Noise rejection: require >=3 points
        )

    def test_clear_corridor_full_speed(self):
        """Points outside the aisle or behind drive_center should not impede motion."""
        t = 0.0
        # Points far to the side (racking / walls outside corridor)
        pts = [(1.0, 0.8), (1.0, -0.8), (-0.2, 0.0)]
        self.gate.update_scan(pts, t)
        v_out, w_out = self.gate.step(0.35, 0.0, t)

        self.assertEqual(self.gate.state, FREE)
        self.assertAlmostEqual(v_out, 0.35, places=2)
        self.assertEqual(w_out, 0.0)

    def test_static_obstacle_proportional_slowdown_and_stop(self):
        """A stationary obstacle in front should cause proportional deceleration down to 0 at d_stop."""
        nose_x = self.gate.p["nose_x"]
        d_stop = self.gate.p["d_stop"]

        # Case 1: Obstacle at 1.0m (inside warning zone < 1.5m, but > d_stop)
        pts = make_cluster(nose_x + 1.0, 0.0, n_pts=5)
        self.gate.update_scan(pts, 0.0)
        v_out_1, _ = self.gate.step(0.35, 0.0, 0.0)
        self.assertEqual(self.gate.state, FREE)
        self.assertGreater(v_out_1, 0.0)
        self.assertLess(v_out_1, 0.35, "Velocity should be scaled down inside warning zone")

        # Case 2: Obstacle right at or inside protective shield (d_stop = 0.35m)
        pts_close = make_cluster(nose_x + d_stop, 0.0, n_pts=5)
        self.gate.update_scan(pts_close, 0.1)
        v_out_2, _ = self.gate.step(0.35, 0.0, 0.1)
        self.assertAlmostEqual(v_out_2, 0.0, places=2, msg="Must stop at or inside d_stop")

    def test_head_on_closing_yield(self):
        """An oncoming obstacle closing in at > 0.2 m/s must trigger an early HOLD (ISO yield)."""
        nose_x = self.gate.p["nose_x"]

        # Time series of an oncoming vehicle approaching at ~0.5 m/s relative
        trajectory = [
            (0.00, 1.40),
            (0.10, 1.35),
            (0.20, 1.30),
            (0.30, 1.25),
        ]

        for t, gap in trajectory:
            pts = make_cluster(nose_x + gap, 0.0, n_pts=5)
            self.gate.update_scan(pts, t)
            v_out, w_out = self.gate.step(0.35, 0.0, t)

        self.assertEqual(self.gate.state, HOLD, "Must enter HOLD on closing head-on obstacle")
        self.assertEqual(self.gate.reason, "moving")
        self.assertEqual(v_out, 0.0)
        self.assertEqual(w_out, 0.0)

    def test_receding_obstacle_no_panic_hold(self):
        """A vehicle ahead moving away or at equal speed must NOT trigger a HOLD."""
        nose_x = self.gate.p["nose_x"]

        # Distance is constant or increasing
        trajectory = [
            (0.00, 1.00),
            (0.10, 1.02),
            (0.20, 1.05),
            (0.30, 1.08),
        ]

        for t, gap in trajectory:
            pts = make_cluster(nose_x + gap, 0.0, n_pts=5)
            self.gate.update_scan(pts, t)
            v_out, _ = self.gate.step(0.35, 0.0, t)

        self.assertEqual(self.gate.state, FREE, "Receding traffic should not trigger HOLD")
        self.assertGreater(v_out, 0.0, "AMR should continue following at controlled speed")

    def test_noise_spike_rejection(self):
        """A single noisy ray (dust / glare) inside the safety zone must be ignored."""
        nose_x = self.gate.p["nose_x"]
        t = 0.0

        # Only 1 single ray at 0.1m from nose (normally a crash zone)
        single_dust_ray = [(nose_x + 0.10, 0.0)]
        self.gate.update_scan(single_dust_ray, t)
        v_out, _ = self.gate.step(0.35, 0.0, t)

        self.assertEqual(self.gate.state, FREE, "Single dust ray must not trigger emergency stop")
        self.assertAlmostEqual(v_out, 0.35, places=2)

        # But a real cluster of 3 rays DOES trigger stop
        real_obstacle = make_cluster(nose_x + 0.10, 0.0, n_pts=3)
        self.gate.update_scan(real_obstacle, t + 0.05)
        v_out_real, _ = self.gate.step(0.35, 0.0, t + 0.05)
        self.assertAlmostEqual(v_out_real, 0.0, places=2, msg="Real cluster must trigger stop")

    def test_hysteresis_release(self):
        """After entering HOLD, clearing the corridor must wait clear_time before resuming."""
        nose_x = self.gate.p["nose_x"]

        # Trigger HOLD
        for t, gap in [(0.0, 1.3), (0.1, 1.2), (0.2, 1.1)]:
            self.gate.update_scan(make_cluster(nose_x + gap, 0.0), t)
            self.gate.step(0.35, 0.0, t)
        self.assertEqual(self.gate.state, HOLD)

        # Clear the corridor completely at t = 0.3
        self.gate.update_scan([], 0.3)
        self.gate.step(0.35, 0.0, 0.3)
        self.assertEqual(self.gate.state, HOLD, "Must not release immediately at t=0.3")

        # Advance time to t = 0.8 (0.5s clear < clear_time 0.8s)
        self.gate.update_scan([], 0.8)
        self.gate.step(0.35, 0.0, 0.8)
        self.assertEqual(self.gate.state, HOLD, "Must still hold while under clear_time")

        # Advance time to t = 1.2 (0.9s clear >= clear_time 0.8s)
        self.gate.update_scan([], 1.2)
        v_out, _ = self.gate.step(0.35, 0.0, 1.2)
        self.assertEqual(self.gate.state, FREE, "Should release to FREE after full clear_time")
        self.assertGreater(v_out, 0.0)

    def test_hold_timeout_giveup(self):
        """Holding for > hold_timeout transitions to GIVEUP, unblocking angular commands."""
        nose_x = self.gate.p["nose_x"]

        # Trigger HOLD
        for t, gap in [(0.0, 1.3), (0.1, 1.2), (0.2, 1.1)]:
            self.gate.update_scan(make_cluster(nose_x + gap, 0.0), t)
            self.gate.step(0.35, 0.0, t)

        # Obstacle remains blocked for 25s (> 20s timeout)
        self.gate.update_scan(make_cluster(nose_x + 1.1, 0.0), 25.0)
        v_out, w_out = self.gate.step(0.0, 0.4, 25.0)

        self.assertEqual(self.gate.state, GIVEUP, "Must transition to GIVEUP after hold_timeout")
        self.assertEqual(v_out, 0.0, "Forward drive remains blocked in GIVEUP")
        self.assertEqual(w_out, 0.4, "Angular steering unblocked in GIVEUP so robot can reroute")


if __name__ == "__main__":
    unittest.main()
