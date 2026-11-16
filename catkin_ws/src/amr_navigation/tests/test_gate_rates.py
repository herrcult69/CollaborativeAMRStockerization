#!/usr/bin/env python3
"""Rate-sensitivity tests for GateLogic: the gate must not depend on the odom rate or on step() jitter.

Why: the sim clock and odom rates are being raised (20 -> 100 Hz clock, 20 -> 50 Hz odom) and ROS timers jitter with
the clock. Scans stay at 10 Hz, so decisions measured in scans (cross_confirm) keep their meaning.
Pure Python; no ROS runtime.
"""
import os
import sys
import types
import unittest

_pkg_src = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "amr_navigation"))
if os.path.isdir(_pkg_src) and _pkg_src not in sys.path:
    sys.path.insert(0, _pkg_src)
if "amr_navigation" not in sys.modules:
    _m = types.ModuleType("amr_navigation")
    _m.__path__ = [_pkg_src]
    sys.modules["amr_navigation"] = _m
import gate_logic
sys.modules.setdefault("amr_navigation.gate_logic", gate_logic)

from gate_logic import FREE, HOLD, GateLogic

SCAN_DT = 0.10   # 10 Hz lidar
STEP_DT = 0.05   # 20 Hz gate timer
CROSSER_X = 0.80  # inside cross_check_dist of the nose
CROSSER_Y0 = 0.75
CROSSER_VY = -0.50   # m/s toward the centreline


def cluster_scan(x, y, n_pts=3, spacing=0.03):
    """Boundary-padded cluster so the test cluster never touches the scan edges."""
    half = (n_pts - 1) * spacing / 2.0
    return [(0.0, -3.0)] + [(x, y - half + i * spacing) for i in range(n_pts)] + [(0.0, 3.0)]


def robot_speed(t):
    """Robot accelerates 0 -> 0.3 m/s over 2 s, so a stale odom sample really differs from the current one."""
    return min(0.15 * t, 0.30)


class TestGateRates(unittest.TestCase):
    def _crossing_run(self, odom_hz):
        """Scan 10 Hz, gate step 20 Hz, odom at odom_hz (sample-and-hold). Returns scan index where HOLD [crossing] latched."""
        gate = GateLogic()
        odom_dt = 1.0 / odom_hz
        n_ticks = 400                       # 5 ms grid: exact for every rate used here
        held_v, next_odom, scan_idx, hold_idx = 0.0, 0.0, -1, None
        for k in range(n_ticks):
            t = k * 0.005
            if t >= next_odom - 1e-9:
                held_v = robot_speed(t)
                next_odom += odom_dt
            if k % round(SCAN_DT / 0.005) == 0:
                scan_idx += 1
                gate.update_scan(cluster_scan(CROSSER_X, CROSSER_Y0 + CROSSER_VY * t), t, vw=(held_v, 0.0))
            if k % round(STEP_DT / 0.005) == 0:
                gate.step(robot_speed(t), 0.0, t)
                if hold_idx is None and gate.state == HOLD and gate.reason == "crossing":
                    hold_idx = scan_idx
        return hold_idx

    def test_crossing_latency_independent_of_odom_rate(self):
        at_20 = self._crossing_run(20.0)
        at_50 = self._crossing_run(50.0)
        self.assertIsNotNone(at_20, "crossing HOLD must latch with 20 Hz odom")
        self.assertIsNotNone(at_50, "crossing HOLD must latch with 50 Hz odom")
        self.assertEqual(at_20, at_50, "HOLD latched at scan %s (20 Hz odom) vs %s (50 Hz odom)" % (at_20, at_50))

    def _state_per_scan(self, step_intervals):
        """Crosser sweeps in, leaves, scene empties. step() runs at the given (cycled) intervals; scans are fixed 10 Hz.
        Returns the gate state after the last step before each next scan, so runs with different step counts compare."""
        gate = GateLogic()
        scans = []
        for i in range(30):                                         # 3.0 s
            t = i * SCAN_DT
            y = CROSSER_Y0 + CROSSER_VY * t
            scans.append((t, cluster_scan(CROSSER_X, y) if 0.0 <= y <= 1.0 else []))
        states, t_step, n = [], 0.0, 0
        for idx, (t_scan, pts) in enumerate(scans):
            gate.update_scan(pts, t_scan, vw=(0.20, 0.0))
            t_next_scan = t_scan + SCAN_DT
            t_step = max(t_step, t_scan)
            last = gate.state
            while t_step < t_next_scan - 1e-9:
                gate.step(0.20, 0.0, t_step)
                last = gate.state
                t_step += step_intervals[n % len(step_intervals)]
                n += 1
            states.append(last)
        return states

    def test_gate_step_jitter_tolerant(self):
        steady = self._state_per_scan([0.050])
        jittered = self._state_per_scan([0.040, 0.060, 0.045, 0.055, 0.050])
        self.assertIn(HOLD, steady, "scenario must exercise a HOLD, otherwise this test proves nothing")
        self.assertIn(FREE, steady[steady.index(HOLD):], "scenario must also release the HOLD")
        self.assertEqual(steady, jittered)


if __name__ == "__main__":
    unittest.main()
