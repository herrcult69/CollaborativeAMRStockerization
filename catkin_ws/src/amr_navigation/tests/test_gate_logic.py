#!/usr/bin/env python3
"""Unit tests for VelocityGate decision logic: Dual-Shield Linear Heading Guard Architecture.
Validates Inner Protective Shield, Outer Warning Shield, Linear Heading Lock, and In-Place Pivot Rotation.
Runs in pure Python without requiring ROS runtime or TF lookups (< 0.005s).
"""
import os
import sys
import unittest

_pkg_src = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "amr_navigation"))
if os.path.isdir(_pkg_src) and _pkg_src not in sys.path:
    sys.path.insert(0, _pkg_src)

from gate_logic import DEFAULTS, FREE, GIVEUP, HOLD, GateLogic, path_heading_error


def make_cluster(center_x, center_y, n_pts=3, spacing=0.03):
    """Helper: create an obstacle cluster of n_pts around (center_x, center_y)."""
    half = (n_pts - 1) * spacing / 2.0
    return [(center_x, center_y - half + i * spacing) for i in range(n_pts)]


class TestDualShieldGateLogic(unittest.TestCase):
    def setUp(self):
        # Instantiate gate with measured warehouse AMR parameters
        self.gate = GateLogic(
            nose_x=0.37,              # 0.37m from drive_center to front bumper
            half_w=0.33,              # 0.23m half robot + 0.10m margin
            d_inner=0.35,             # 0.35m inner protective stop shield
            d_outer=1.20,             # 1.20m outer warning slowdown shield
            d_hold_static=0.45,       # 0.45m in-line stationary hold threshold
            omega_corridor_max=0.05,  # 0.05 rad/s linear heading lock clamp
            clear_time=0.80,          # 0.80s hysteresis release duration
            hold_timeout=20.0,
            min_cluster_pts=3         # Noise rejection: require >= 3 points
        )

    def test_clear_corridor_full_speed(self):
        """Points outside the aisle or behind drive_center should not impede motion."""
        t = 0.0
        # Points far to the side (racking at y = +/-0.8m)
        pts = [(1.0, 0.8), (1.0, -0.8), (-0.2, 0.0)]
        self.gate.update_scan(pts, t)
        v_out, w_out = self.gate.step(0.35, 0.0, t)

        self.assertEqual(self.gate.state, FREE)
        self.assertAlmostEqual(v_out, 0.35, places=2)
        self.assertEqual(w_out, 0.0)

    def test_outer_shield_proportional_deceleration(self):
        """Obstacle in Outer Shield causes smooth proportional deceleration down toward inner shield."""
        nose_x = self.gate.p["nose_x"]
        d_inner = self.gate.p["d_inner"]
        d_outer = self.gate.p["d_outer"]

        # Obstacle at 0.80m ahead of nose (midway between 0.35m and 1.20m)
        pts = make_cluster(nose_x + 0.80, 0.0, n_pts=5)
        self.gate.update_scan(pts, 0.0)
        v_out, _ = self.gate.step(0.35, 0.0, 0.0)

        self.assertEqual(self.gate.state, FREE)
        self.assertLess(v_out, 0.35)
        self.assertGreater(v_out, 0.0)

    def test_outer_shield_linear_heading_lock_at_cruising_speed(self):
        """When slowing in outer shield, w scales proportionally with v, preserving planned curvature w / v."""
        nose_x = self.gate.p["nose_x"]

        # Obstacle at 1.00m ahead in Outer Shield (v_out will be > 0.12 m/s)
        pts = make_cluster(nose_x + 1.00, 0.0, n_pts=5)
        self.gate.update_scan(pts, 0.0)

        # TEB commands full angular swerve (w = 0.45 rad/s)
        v_out, w_out = self.gate.step(0.35, 0.45, 0.0)

        self.assertGreater(v_out, 0.12)
        # Angular velocity follows v scaling: w_out / v_out == w / v
        self.assertAlmostEqual(w_out / v_out, 0.45 / 0.35, places=3)
        self.assertAlmostEqual(w_out, 0.45 * (v_out / 0.35), places=3)

    def test_inner_shield_category0_stop(self):
        """Obstacle within d_inner (0.35m ahead of nose) must produce zero linear velocity."""
        nose_x = self.gate.p["nose_x"]

        # Obstacle at 0.30m ahead of nose (breaches Inner Shield)
        pts = make_cluster(nose_x + 0.30, 0.0, n_pts=5)
        self.gate.update_scan(pts, 0.0)
        v_out, _ = self.gate.step(0.35, 0.0, 0.0)

        self.assertAlmostEqual(v_out, 0.0, places=2, msg="Must stop linear drive completely")

    def test_in_line_stationary_hold_allows_pivot_turn(self):
        """When stopped at a wall/obstacle ahead, HOLD outputs (0, 0): no pivot (avoids pivot-release loop)."""
        nose_x = self.gate.p["nose_x"]

        # Obstacle at 0.40m ahead of nose (<= d_hold_static 0.45m)
        pts = make_cluster(nose_x + 0.40, 0.0, n_pts=5)
        self.gate.update_scan(pts, 0.0)

        # TEB commands zero forward velocity and a pivot turn away from the wall (w = 0.50 rad/s)
        v_out, w_out = self.gate.step(0.0, 0.50, 0.0)

        self.assertEqual(self.gate.state, HOLD)
        self.assertEqual(self.gate.reason, "blocked")
        self.assertEqual(v_out, 0.0, "Forward drive into obstacle must remain blocked")
        self.assertEqual(w_out, 0.0, "HOLD never rotates (inner shield is a full stop)")

    def test_flank_protective_shield_stops_all_motion(self):
        """Obstacle beside chassis in flank zone must trigger immediate HOLD [flank] and stop both v and w."""
        # Obstacle at px = 0.10m (along chassis), py = 0.30m (flank boundary is 0.23m to 0.35m)
        flank_pts = make_cluster(0.10, 0.30, n_pts=3, spacing=0.02)
        self.gate.update_scan(flank_pts, 0.0)
        v_out, w_out = self.gate.step(0.35, 0.40, 0.0)

        self.assertEqual(self.gate.state, HOLD, "Flank breach must trigger immediate HOLD")
        self.assertEqual(self.gate.reason, "flank")
        self.assertEqual(v_out, 0.0)
        self.assertEqual(w_out, 0.0, "Flank breach must stop rotation to avoid rubbing chassis against obstacle")

    def test_noise_spike_rejection(self):
        """A single noisy ray (dust / glare) inside the safety zone must be ignored."""
        nose_x = self.gate.p["nose_x"]
        t = 0.0

        # Only 1 single ray at 0.10m from nose (normally a crash zone)
        single_dust_ray = [(nose_x + 0.10, 0.0)]
        self.gate.update_scan(single_dust_ray, t)
        v_out, _ = self.gate.step(0.35, 0.0, t)

        self.assertEqual(self.gate.state, FREE, "Single dust ray must not trigger emergency stop")
        self.assertAlmostEqual(v_out, 0.35, places=2)

        # Real cluster of 3 rays DOES trigger stop
        real_obstacle = make_cluster(nose_x + 0.10, 0.0, n_pts=3)
        self.gate.update_scan(real_obstacle, t + 0.05)
        v_out_real, _ = self.gate.step(0.35, 0.0, t + 0.05)
        self.assertAlmostEqual(v_out_real, 0.0, places=2, msg="Real cluster must trigger stop")

    def test_hysteresis_release(self):
        """After entering HOLD, clearing the corridor must wait clear_time before resuming."""
        nose_x = self.gate.p["nose_x"]

        # Trigger HOLD
        self.gate.update_scan(make_cluster(nose_x + 0.30, 0.0), 0.0)
        self.gate.step(0.35, 0.0, 0.0)
        self.assertEqual(self.gate.state, HOLD)

        # Clear corridor at t = 0.10s
        self.gate.update_scan([], 0.10)
        self.gate.step(0.35, 0.0, 0.10)
        self.assertEqual(self.gate.state, HOLD, "Must not release immediately")

        # Advance to t = 0.50s (0.4s clear < clear_time 0.8s)
        self.gate.update_scan([], 0.50)
        self.gate.step(0.35, 0.0, 0.50)
        self.assertEqual(self.gate.state, HOLD, "Must still hold while under clear_time")

        # Advance to t = 1.00s (0.9s clear >= clear_time 0.8s)
        self.gate.update_scan([], 1.00)
        v_out, _ = self.gate.step(0.35, 0.0, 1.00)
        self.assertEqual(self.gate.state, FREE, "Should release to FREE after full clear_time")
        self.assertGreater(v_out, 0.0)

    def test_hold_timeout_giveup(self):
        """Holding for > hold_timeout transitions to GIVEUP, unblocking angular commands for replanning."""
        nose_x = self.gate.p["nose_x"]

        # Trigger HOLD
        self.gate.update_scan(make_cluster(nose_x + 0.30, 0.0), 0.0)
        self.gate.step(0.35, 0.0, 0.0)

        # Obstacle remains blocked for 25s (> 20s timeout)
        self.gate.update_scan(make_cluster(nose_x + 0.30, 0.0), 25.0)
        v_out, w_out = self.gate.step(0.0, 0.65, 25.0)

        self.assertEqual(self.gate.state, GIVEUP, "Must transition to GIVEUP after hold_timeout")
        self.assertEqual(v_out, 0.0, "Forward drive remains blocked in GIVEUP")
        self.assertAlmostEqual(w_out, 0.30, places=3, msg="GIVEUP rotation capped at giveup_w_max")

    def test_aisle_with_side_racks_no_false_stops(self):
        """Driving past multiple static racks at y = +/-0.60m must never trigger slowdown or hold."""
        # Aisle with racks on both sides at y = +0.60m and y = -0.60m
        v_robot = 0.35
        for step in range(15):
            t = step * 0.05
            dx = v_robot * t
            rack_left = make_cluster(1.50 - dx, 0.60, n_pts=5, spacing=0.04)
            rack_right = make_cluster(1.50 - dx, -0.60, n_pts=5, spacing=0.04)
            box_side = make_cluster(1.80 - dx, 0.70, n_pts=4, spacing=0.03)

            all_pts = rack_left + rack_right + box_side
            self.gate.update_scan(all_pts, t)
            v_out, w_out = self.gate.step(0.35, 0.0, t)

            self.assertEqual(self.gate.state, FREE, f"Step {step} falsely triggered {self.gate.state}")
            self.assertAlmostEqual(v_out, 0.35, places=2)
            self.assertEqual(w_out, 0.0)

    def test_open_space_unconstrained_steering(self):
        """When the corridor ahead is clear (gap > 1.20m), TEB must have 100% full steering freedom."""
        # No obstacle ahead (gap = inf)
        self.gate.update_scan([], 0.0)

        # TEB commands strong turn (e.g. 0.45 rad/s at an intersection or turn)
        v_out, w_out = self.gate.step(0.35, 0.45, 0.0)

        self.assertEqual(self.gate.state, FREE)
        self.assertAlmostEqual(v_out, 0.35, places=2)
        self.assertAlmostEqual(w_out, 0.45, places=2, msg="TEB must have full unconstrained turning in open space")


class TestCrossingYield(unittest.TestCase):
    def setUp(self):
        # Pin baseline crossing parameters from initial implementation
        self.gate = GateLogic(
            min_cluster_pts=3,
            d_outer=1.20,
            cross_confirm=2,
            cross_max_extent=999.0,
            cross_w_max=999.0,
        )

    def _run(self, path, v=0.0, w=0.0, n=None, t0=0.0, dt=0.1):
        """Feed one cluster per scan; path is a list of (x, y). Returns list of (state, v_out, w_out)."""
        out = []
        for i, (x, y) in enumerate(path[:n]):
            t = t0 + i * dt
            # Provide boundary points so single test cluster does not touch scan edges
            pts = [(0.0, -3.0)] + make_cluster(x, y) + [(0.0, 3.0)]
            self.gate.update_scan(pts, t)
            vo, wo = self.gate.step(v, w, t)
            out.append((self.gate.state, vo, wo))
        return out

    def test_crossing_toward_centreline_holds(self):
        path = [(1.2, 1.0 - 0.05 * i) for i in range(6)]
        res = self._run(path, v=0.35, n=3)
        self.assertEqual(res[-1][0], HOLD)
        self.assertEqual(self.gate.reason, "crossing")
        self.assertEqual(res[-1][1:], (0.0, 0.0))
        self.assertIn("yield: crossing", self.gate.trigger_info)

    def test_moving_away_stays_free(self):
        path = [(1.2, 1.0 + 0.05 * i) for i in range(6)]
        res = self._run(path, v=0.35)
        self.assertTrue(all(r[0] == FREE for r in res))

    def test_static_beside_path_driving_no_crossing(self):
        # Robot drives 0.35 m/s: static obstacle's x shrinks 0.035/scan in robot frame
        path = [(1.0 + 0.1 * 0.35 * (5 - i), 0.8) for i in range(6)]
        # prime last_v with a driving step so ego compensation is active
        res = self._run(path, v=0.35)
        self.assertTrue(all(r[0] == FREE for r in res))
        self.assertFalse(self.gate.crossing)

    def test_static_while_pivoting_no_crossing(self):
        import math
        w, dt = 0.65, 0.1
        pts = []
        x, y = 1.5, 0.8
        for _ in range(6):
            pts.append((x, y))
            a = -w * dt
            x, y = x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)
        res = self._run(pts, v=0.0, w=w)
        self.assertFalse(self.gate.crossing)

    def test_releases_after_clear_time(self):
        cross = [(1.2, 1.0 - 0.1 * i) for i in range(12)]  # 1.0 m/s sweep through to y=-0.1
        self._run(cross[:4], v=0.0)
        self.assertEqual(self.gate.state, HOLD)
        # obstacle exits on the other side, then scene empties
        t = 0.4
        exit_path = [(1.2, -0.8 - 0.1 * i) for i in range(3)]
        for x, y in exit_path:
            pts = [(0.0, -3.0)] + make_cluster(x, y) + [(0.0, 3.0)]
            self.gate.update_scan(pts, t)
            self.gate.step(0.35, 0.0, t)
            t += 0.1
        self.assertEqual(self.gate.state, HOLD)  # clear_time not yet elapsed
        for _ in range(12):
            self.gate.update_scan([], t)
            self.gate.step(0.35, 0.0, t)
            t += 0.1
        self.assertEqual(self.gate.state, FREE)

    def test_single_scan_jitter_ignored(self):
        path = [(1.2, 1.0), (1.2, 1.0), (1.2, 0.7), (1.2, 1.0), (1.2, 1.0)]
        res = self._run(path, v=0.35)
        self.assertTrue(all(r[0] == FREE for r in res))


class TestShieldTuning(unittest.TestCase):
    """Tests corresponding to .agents/feature-plan.md Test Plan for shield-tuning task."""
    def setUp(self):
        self.gate = GateLogic()

    def _pad(self, cluster):
        """Helper to include non-corridor boundary points so cluster does not touch scan edges."""
        return [(-3.0, -3.0)] + cluster + [(-3.0, 3.0)]

    def test_long_static_wall_while_rotating_never_crossing(self):
        """1. Long static wall (span 2 m) ahead at x=1.0 while robot rotates w=0.5 -> never crossing."""
        wall = [(1.0, -1.0 + i * 0.1) for i in range(21)]  # 2.0m extent
        for i in range(5):
            t = i * 0.1
            self.gate.update_scan(self._pad(wall), t)
            self.gate.step(0.0, 0.5, t)
        self.assertFalse(self.gate.crossing)
        self.assertNotEqual(self.gate.reason, "crossing")

    def test_wide_cluster_shifting_laterally_ignored(self):
        """2. Wide cluster (span 1.0 m) shifting laterally 0.3 m/s -> ignored."""
        for i in range(5):
            t = i * 0.1
            y0 = 1.0 - 0.03 * i
            wide_cluster = [(1.0, y0 + j * 0.1) for j in range(11)]  # 1.0m extent > cross_max_extent (0.80m)
            self.gate.update_scan(self._pad(wide_cluster), t)
            self.gate.step(0.3, 0.0, t)
        self.assertFalse(self.gate.crossing)

    def test_compact_cluster_moving_toward_centreline_holds(self):
        """3. Compact cluster (0.3 m) at (1.0, 1.0) moving toward centreline at 0.5 m/s, robot v=0.3, w=0 -> HOLD crossing within 3 scans."""
        for i in range(4):
            t = i * 0.1
            c = [(1.0, 1.0 - 0.05 * i + j * 0.05) for j in range(6)]  # ~0.25m compact extent
            self.gate.update_scan(self._pad(c), t)
            vo, wo = self.gate.step(0.3, 0.0, t)
        self.assertEqual(self.gate.state, HOLD)
        self.assertEqual(self.gate.reason, "crossing")
        self.assertEqual(vo, 0.0)
        self.assertEqual(wo, 0.0)

    def test_cluster_touching_scan_boundary_ignored(self):
        """4. Cluster touching the scan's first/last point -> ignored."""
        for i in range(5):
            t = i * 0.1
            # Unpadded cluster touches first and last index of scan points
            c = [(1.0, 1.0 - 0.05 * i + j * 0.05) for j in range(6)]
            self.gate.update_scan(c, t)
            self.gate.step(0.3, 0.0, t)
        self.assertFalse(self.gate.crossing)

    def test_crossing_clears_tight_gap_allows_pivot(self):
        """5. Crossing HOLD, then the crossing clears with gap 0.35 m -> HOLD stays a full stop: step(0, 0.4) returns (0, 0)."""
        # Enter crossing HOLD
        for i in range(4):
            t = i * 0.1
            c = [(1.0, 1.0 - 0.05 * i + j * 0.05) for j in range(6)]
            self.gate.update_scan(self._pad(c), t)
            self.gate.step(0.3, 0.0, t)
        self.assertEqual(self.gate.state, HOLD)
        self.assertEqual(self.gate.reason, "crossing")

        # Crossing clears, obstacle remains in corridor at gap = 0.35m (nose_x = 0.37m -> x = 0.72m)
        t = 0.5
        static_obs = [(0.72, -0.05), (0.72, 0.0), (0.72, 0.05)]
        self.gate.update_scan(self._pad(static_obs), t)
        vo, wo = self.gate.step(0.0, 0.4, t)
        self.assertEqual(self.gate.state, HOLD)
        self.assertEqual(vo, 0.0)
        self.assertEqual(wo, 0.0)

    def test_new_defaults_values(self):
        self.assertEqual(DEFAULTS["d_outer"], 1.00)
        self.assertEqual(DEFAULTS["omega_corridor_max"], 0.10)
        self.assertEqual(DEFAULTS["hold_timeout"], 10.0)
        self.assertEqual(DEFAULTS["giveup_w_max"], 0.30)
        self.assertEqual(DEFAULTS["scale_min"], 0.25)

    def test_crawl_speed_heading_lock(self):
        """Curvature rule: gap 0.5 (v_out ~0.175) and gap 0.35 (crawl path) preserve w / v curvature."""
        nose_x = self.gate.p["nose_x"]
        for gap, exp_v in ((0.5, 0.175), (0.35, 0.35 * 0.35)):
            gate = GateLogic()
            obs = [(nose_x + gap, -0.05), (nose_x + gap, 0.0), (nose_x + gap, 0.05)]
            gate.update_scan(self._pad(obs), 0.0)
            vo, wo = gate.step(0.35, 0.65, 0.0)
            self.assertEqual(gate.state, FREE)
            self.assertAlmostEqual(vo, exp_v, places=3)
            self.assertAlmostEqual(wo / vo, 0.65 / 0.35, places=3)
            self.assertAlmostEqual(wo, 0.65 * (vo / 0.35), places=3)

    def test_slowdown_scale_floor(self):
        """Gap 0.5 -> scale 0.50; gap 0.25 -> 0 (hold)."""
        nose_x = self.gate.p["nose_x"]
        obs = [(nose_x + 0.5, -0.05), (nose_x + 0.5, 0.0), (nose_x + 0.5, 0.05)]
        self.gate.update_scan(self._pad(obs), 0.0)
        self.gate.step(0.35, 0.0, 0.0)
        self.assertAlmostEqual(self.gate.scale, 0.50, places=3)
        gate = GateLogic()
        obs = [(nose_x + 0.25, -0.05), (nose_x + 0.25, 0.0), (nose_x + 0.25, 0.05)]
        gate.update_scan(self._pad(obs), 0.0)
        vo, wo = gate.step(0.35, 0.0, 0.0)
        self.assertEqual(gate.state, HOLD)
        self.assertEqual((vo, wo), (0.0, 0.0))

    def test_giveup_after_timeout_caps_w(self):
        nose_x = self.gate.p["nose_x"]
        obs = [(nose_x + 0.28, -0.05), (nose_x + 0.28, 0.0), (nose_x + 0.28, 0.05)]
        self.gate.update_scan(self._pad(obs), 0.0)
        self.gate.step(0.35, 0.0, 0.0)
        self.assertEqual(self.gate.state, HOLD)
        self.gate.update_scan(self._pad(obs), 11.0)
        vo, wo = self.gate.step(0.0, 0.65, 11.0)
        self.assertEqual(self.gate.state, GIVEUP)
        self.assertEqual(vo, 0.0)
        self.assertLessEqual(abs(wo), 0.30 + 1e-9)

    def test_new_defaults_reduced_speed_and_blocked(self):
        """6. With new defaults: obstacle at gap 0.35 -> FREE with reduced speed (not HOLD); gap 0.28 -> HOLD blocked."""
        nose_x = self.gate.p["nose_x"]
        # gap = 0.35m: between d_hold_static (0.30m) and d_outer (0.80m) -> FREE with proportional deceleration
        obs_35 = [(nose_x + 0.35, -0.05), (nose_x + 0.35, 0.0), (nose_x + 0.35, 0.05)]
        self.gate.update_scan(self._pad(obs_35), 0.0)
        v35, w35 = self.gate.step(0.35, 0.0, 0.0)
        self.assertEqual(self.gate.state, FREE)
        self.assertGreater(v35, 0.0)
        self.assertLess(v35, 0.35)

        # gap = 0.28m: below d_hold_static (0.30m) -> HOLD [blocked]
        obs_28 = [(nose_x + 0.28, -0.05), (nose_x + 0.28, 0.0), (nose_x + 0.28, 0.05)]
        self.gate.update_scan(self._pad(obs_28), 0.1)
        v28, w28 = self.gate.step(0.35, 0.0, 0.1)
        self.assertEqual(self.gate.state, HOLD)
        self.assertEqual(self.gate.reason, "blocked")
        self.assertEqual(v28, 0.0)


class TestTebSameRouteBlocker(unittest.TestCase):
    """Tests corresponding to .agents/feature-plan.md Test Plan for teb-same-route task."""

    def test_new_defaults_blocker_keys(self):
        """Defaults test covers the new keys: blocker_half_width, blocker_ttl, blocker_spacing."""
        from gate_logic import DEFAULTS
        self.assertEqual(DEFAULTS["blocker_half_width"], 0.60)
        self.assertEqual(DEFAULTS["blocker_ttl"], 60.0)
        self.assertEqual(DEFAULTS["blocker_spacing"], 0.05)

    def test_blocker_points_normal_gap(self):
        """blocker_points(0.29, DEFAULTS): 25 points, all at x = 0.37 + 0.29, y symmetric from -0.60 to +0.60."""
        from gate_logic import DEFAULTS, blocker_points
        pts = blocker_points(0.29, DEFAULTS)
        self.assertEqual(len(pts), 25)
        nose_x = DEFAULTS.get("nose_x", 0.37)
        exp_x = nose_x + 0.29
        for x, y in pts:
            self.assertAlmostEqual(x, exp_x, places=4)
        ys = [p[1] for p in pts]
        self.assertAlmostEqual(min(ys), -0.60, places=4)
        self.assertAlmostEqual(max(ys), 0.60, places=4)
        for y in ys:
            self.assertTrue(any(abs(y + other) < 1e-4 for other in ys), f"Point {y} not symmetric")
        for i in range(len(ys) - 1):
            self.assertAlmostEqual(ys[i + 1] - ys[i], 0.05, places=4)

    def test_blocker_points_clamped_gap(self):
        """blocker_points(0.0, ...) clamps x to nose_x + 0.10."""
        from gate_logic import DEFAULTS, blocker_points
        nose_x = DEFAULTS.get("nose_x", 0.37)
        exp_x = nose_x + 0.10
        pts = blocker_points(0.0, DEFAULTS)
        self.assertEqual(len(pts), 25)
        for x, y in pts:
            self.assertAlmostEqual(x, exp_x, places=4)

        pts_neg = blocker_points(-0.50, DEFAULTS)
        self.assertEqual(len(pts_neg), 25)
        for x, y in pts_neg:
            self.assertAlmostEqual(x, exp_x, places=4)

    def test_blocker_points_custom_params(self):
        """blocker_points respects custom blocker_half_width, blocker_spacing, and nose_x."""
        from gate_logic import blocker_points
        custom_p = {
            "nose_x": 0.50,
            "blocker_half_width": 0.30,
            "blocker_spacing": 0.10,
        }
        pts = blocker_points(0.40, custom_p)
        self.assertEqual(len(pts), 7)
        for x, y in pts:
            self.assertAlmostEqual(x, 0.50 + 0.40, places=4)
        ys = [p[1] for p in pts]
        self.assertAlmostEqual(ys[0], -0.30, places=4)
        self.assertAlmostEqual(ys[-1], 0.30, places=4)


class TestNavAssessment(unittest.TestCase):
    """Tests corresponding to .agents/feature-plan.md Test Plan for nav-assessment task."""

    def _pad(self, cluster):
        return [(-3.0, -3.0)] + cluster + [(-3.0, 3.0)]

    def test_new_defaults_shim_keys(self):
        """DEFAULTS contains shim_lookahead=0.60, shim_tol=0.15, shim_plan_age=3.0."""
        self.assertEqual(DEFAULTS["shim_lookahead"], 0.60)
        self.assertEqual(DEFAULTS["shim_tol"], 0.15)
        self.assertEqual(DEFAULTS["shim_plan_age"], 3.0)

    def test_free_curvature_preserved_during_slowdown(self):
        """FREE, gap 0.5, v 0.35, w 0.45: w_out / v_out equals 0.45 / 0.35 (curvature kept)."""
        gate = GateLogic()
        nose_x = gate.p["nose_x"]
        obs = [(nose_x + 0.5, -0.05), (nose_x + 0.5, 0.0), (nose_x + 0.5, 0.05)]
        gate.update_scan(self._pad(obs), 0.0)
        vo, wo = gate.step(0.35, 0.45, 0.0)

        self.assertEqual(gate.state, FREE)
        self.assertAlmostEqual(vo, 0.175, places=3)
        self.assertAlmostEqual(wo / vo, 0.45 / 0.35, places=3)
        self.assertAlmostEqual(wo, 0.45 * (vo / 0.35), places=3)

    def test_free_curvature_preserved_at_crawl_floor(self):
        """FREE at crawl floor (v*scale < 0.10): v_out is 0.10 and w_out / v_out still equals w / v."""
        gate = GateLogic()
        nose_x = gate.p["nose_x"]
        # gap 0.35 gives scale = 0.35. For v = 0.20, v * scale = 0.07 < min_crawl_speed (0.10)
        obs = [(nose_x + 0.35, -0.05), (nose_x + 0.35, 0.0), (nose_x + 0.35, 0.05)]
        gate.update_scan(self._pad(obs), 0.0)
        vo, wo = gate.step(0.20, 0.40, 0.0)

        self.assertEqual(gate.state, FREE)
        self.assertAlmostEqual(vo, 0.10, places=3)
        self.assertAlmostEqual(wo / vo, 0.40 / 0.20, places=3)
        self.assertAlmostEqual(wo, 0.20, places=3)

    def test_free_pure_rotation_unclamped_with_outer_shield_obstacle(self):
        """FREE, v 0, w 0.4, gap 0.5: output (0, 0.4)."""
        gate = GateLogic()
        nose_x = gate.p["nose_x"]
        obs = [(nose_x + 0.5, -0.05), (nose_x + 0.5, 0.0), (nose_x + 0.5, 0.05)]
        gate.update_scan(self._pad(obs), 0.0)
        vo, wo = gate.step(0.0, 0.4, 0.0)

        self.assertEqual(gate.state, FREE)
        self.assertEqual((vo, wo), (0.0, 0.4))

    def test_giveup_rotation_shim(self):
        """GIVEUP with heading_err +1.0: (0, +0.30); -1.0: (0, -0.30); 0.05: (0, 0); None with TEB w 0.65: (0, 0.30)."""
        gate = GateLogic()
        nose_x = gate.p["nose_x"]
        obs = [(nose_x + 0.28, -0.05), (nose_x + 0.28, 0.0), (nose_x + 0.28, 0.05)]
        gate.update_scan(self._pad(obs), 0.0)
        gate.step(0.35, 0.0, 0.0)
        self.assertEqual(gate.state, HOLD)

        # Advance past hold_timeout (10.0s)
        t = 11.0
        gate.update_scan(self._pad(obs), t)

        # heading_err = +1.0 -> turn positive at giveup_w_max (0.30)
        vo1, wo1 = gate.step(0.0, 0.50, t, heading_err=1.0)
        self.assertEqual(gate.state, GIVEUP)
        self.assertEqual(vo1, 0.0)
        self.assertAlmostEqual(wo1, 0.30, places=3)

        # heading_err = -1.0 -> turn negative at giveup_w_max (-0.30)
        vo2, wo2 = gate.step(0.0, 0.50, t + 0.1, heading_err=-1.0)
        self.assertEqual(gate.state, GIVEUP)
        self.assertEqual(vo2, 0.0)
        self.assertAlmostEqual(wo2, -0.30, places=3)

        # heading_err = 0.05 (|err| <= shim_tol 0.15) -> stop turning (0.0)
        vo3, wo3 = gate.step(0.0, 0.50, t + 0.2, heading_err=0.05)
        self.assertEqual(gate.state, GIVEUP)
        self.assertEqual(vo3, 0.0)
        self.assertAlmostEqual(wo3, 0.0, places=3)

        # heading_err = None -> fallback to TEB w clamped to giveup_w_max
        vo4, wo4 = gate.step(0.0, 0.65, t + 0.3, heading_err=None)
        self.assertEqual(gate.state, GIVEUP)
        self.assertEqual(vo4, 0.0)
        self.assertAlmostEqual(wo4, 0.30, places=3)

    def test_path_heading_error(self):
        """path_heading_error: straight ahead gives 0; behind gives about +/-pi; left gives positive."""
        import math
        # Straight plan ahead: robot at origin facing +x (yaw=0)
        pose_fwd = (0.0, 0.0, 0.0)
        plan_fwd = [(0.0, 0.0), (0.3, 0.0), (0.6, 0.0), (1.0, 0.0)]
        err_fwd = path_heading_error(pose_fwd, plan_fwd, lookahead=0.60)
        self.assertAlmostEqual(err_fwd, 0.0, places=3)

        # Plan behind: robot at origin facing +x (yaw=0), plan goes along -x
        plan_back = [(0.0, 0.0), (-0.3, 0.0), (-0.6, 0.0), (-1.0, 0.0)]
        err_back = path_heading_error(pose_fwd, plan_back, lookahead=0.60)
        self.assertAlmostEqual(abs(err_back), math.pi, places=3)

        # Plan to the left (+y direction): bearing error must be +pi/2
        plan_left = [(0.0, 0.0), (0.0, 0.3), (0.0, 0.6), (0.0, 1.0)]
        err_left = path_heading_error(pose_fwd, plan_left, lookahead=0.60)
        self.assertAlmostEqual(err_left, math.pi / 2, places=3)

        # Plan to the right (-y direction): bearing error must be -pi/2
        plan_right = [(0.0, 0.0), (0.0, -0.3), (0.0, -0.6), (0.0, -1.0)]
        err_right = path_heading_error(pose_fwd, plan_right, lookahead=0.60)
        self.assertAlmostEqual(err_right, -math.pi / 2, places=3)

        # Empty plan returns None
        self.assertIsNone(path_heading_error((0.0, 0.0, 0.0), [], 0.60))


if __name__ == "__main__":
    unittest.main()
