"""
Velocity-gate decision logic (Option B: Industrial Safety Shield + Closing-Rate Yield).
Adheres to ISO 3691-4 safety principles with zero external ROS dependencies.

Architecture:
1. Inner Protective Shield (d_stop): Deterministic zero-velocity stop if an obstacle breaches the corridor.
2. Outer Warning Zone (d_slow): Proportional velocity scaling based on corridor gap.
3. Closing-Rate Yield Detector: Triggers early HOLD (wait-and-proceed) only when an obstacle is actively
   closing in on the robot along the corridor, preventing deadlocks on static walls or receding traffic.
4. Multi-ray Noise Rejection: Requires contiguous clusters to prevent dust/glare false trips.
5. Release Hysteresis & Deadlock Timeout: Ensures chatter-free release and enables rerouting on timeout.
"""
import math

FREE, HOLD, GIVEUP = "free", "hold", "giveup"

DEFAULTS = dict(
    nose_x=0.37,              # Footprint front edge ahead of drive_center (m)
    laser_x=0.33,             # Laser mount ahead of drive_center (m)
    tail_x=0.54,              # Footprint rear edge behind drive_center (m)
    half_w=0.33,              # Half corridor width: half robot (0.23m) + side margin (0.10m)
    d_stop=0.35,              # Protective stop shield depth ahead of nose (m)
    d_slow=1.50,              # Warning zone slowdown depth ahead of nose (m)
    yield_dist=1.50,          # Distance ahead to trigger head-on yield (m)
    closing_rate_thr=-0.15,   # Obstacle world-velocity threshold toward robot (m/s)
    seg_gap=0.20,             # Max distance between adjacent rays in same obstacle cluster (m)
    min_cluster_pts=3,        # Min rays to confirm obstacle (noise rejection)
    closing_window=0.50,      # Sliding time window for rate estimation (s)
    closing_min_dt=0.08,      # Minimum dt before computing closing rate (s)
    release_rate=1.0,         # Acceleration ramp-up (scale change per second)
    scan_timeout=0.50,        # Drop out to stop if scan is delayed (s)
    hold_timeout=20.0,        # Max duration to hold before giving up to allow reroute (s)
    clear_margin=0.15,        # Gap must exceed d_stop + clear_margin to count as clear (m)
    clear_time=0.80,          # Duration corridor must stay clear before releasing HOLD (s)
)


class FreeSpaceDetector:
    """Lightweight stub kept for backward-compatibility with velocity_gate.py imports."""
    def __init__(self, ref_age=0.4, tol=0.12):
        pass

    def update(self, ranges, angle_min, angle_increment, range_min, range_max, pose, now):
        return None


class GateLogic:
    def __init__(self, **overrides):
        self.p = dict(DEFAULTS)
        unknown = set(overrides) - set(DEFAULTS)
        if unknown:
            raise KeyError("unknown gate params: %s" % sorted(unknown))
        self.p.update(overrides)

        self.gap = float("inf")
        self.dyn_gap = float("inf")
        self.dyn_hold = None
        self.reason = None
        self.action = None
        self.pass_note = None
        self.tracks = []

        self.state = FREE
        self.scale = 1.0
        self.last_v = 0.0
        self.last_t = None
        self.scan_t = None
        self.hold_t0 = None
        self.clear_since = None
        self.closing_rate = 0.0
        self.gap_hist = []

    def _cluster_points(self, points):
        """Groups points in scan order into clusters separated by > seg_gap."""
        if not points:
            return []
        p = self.p
        clusters, cur = [], [points[0]]
        for pt in points[1:]:
            prev = cur[-1]
            if math.hypot(pt[0] - prev[0], pt[1] - prev[1]) <= p["seg_gap"]:
                cur.append(pt)
            else:
                if len(cur) >= p["min_cluster_pts"]:
                    clusters.append(cur)
                cur = [pt]
        if len(cur) >= p["min_cluster_pts"]:
            clusters.append(cur)
        return clusters

    def update_scan(self, points, now, moving=None, pose=None):
        """Process incoming LiDAR scan points in robot drive_center frame."""
        p = self.p
        self.scan_t = now

        # Filter noise: only clusters with >= min_cluster_pts are valid physical obstacles
        clusters = self._cluster_points(points)

        # Find closest point inside robot's corridor
        gap = float("inf")
        for cluster in clusters:
            for px, py in cluster:
                if px >= 0.0 and abs(py) <= p["half_w"]:
                    gap = min(gap, px - p["nose_x"])
        self.gap = gap

        # Update gap history for closing-rate calculation
        self.gap_hist.append((now, gap))
        self.gap_hist = [h for h in self.gap_hist if now - h[0] <= p["closing_window"]]

        # Calculate closing rate (relative closing speed + robot forward velocity = obstacle world speed)
        obs_v = 0.0
        if len(self.gap_hist) >= 2 and gap < float("inf"):
            for t0, g0 in self.gap_hist:
                dt = now - t0
                if dt >= p["closing_min_dt"]:
                    if g0 < float("inf"):
                        gap_dot = (gap - g0) / dt
                        obs_v = gap_dot + max(0.0, self.last_v)
                    break
        self.closing_rate = obs_v

        # Head-on oncoming vehicle detection
        if gap <= p["yield_dist"] and obs_v <= p["closing_rate_thr"]:
            self.dyn_hold = "moving"
            self.dyn_gap = gap
        else:
            self.dyn_hold = None
            self.dyn_gap = float("inf")

    def step(self, v, w, now):
        """Computes gated (v_out, w_out) at time now."""
        p = self.p
        dt = min(max(now - self.last_t, 1e-3), 0.2) if self.last_t is not None else 0.05
        self.last_t = now

        # 1. Proportional target scale based on corridor gap
        blind = (self.scan_t is None) or (now - self.scan_t > p["scan_timeout"])
        if blind or self.gap <= p["d_stop"]:
            target = 0.0
        elif self.gap <= p["d_slow"]:
            target = (self.gap - p["d_stop"]) / (p["d_slow"] - p["d_stop"])
            target = max(0.0, min(1.0, target))
        else:
            target = 1.0

        corridor_clear = (not blind) and (self.gap > p["d_stop"] + p["clear_margin"]) and (self.dyn_hold is None)

        # 2. State Machine: FREE <-> HOLD -> GIVEUP
        if self.state == FREE:
            if self.dyn_hold:
                self.state = HOLD
                self.reason = self.dyn_hold
                self.hold_t0 = now
                self.clear_since = None
            elif v > 0.0 and target <= 0.0:
                self.state = HOLD
                self.reason = "blocked"
                self.hold_t0 = now
                self.clear_since = None
        elif self.state in (HOLD, GIVEUP):
            if corridor_clear:
                if self.clear_since is None:
                    self.clear_since = now
                elif now - self.clear_since >= p["clear_time"]:
                    self.state = FREE
                    self.reason = None
                    self.clear_since = None
            else:
                self.clear_since = None

            if self.state == HOLD and (now - self.hold_t0 > p["hold_timeout"]):
                self.state = GIVEUP

        # 3. Output Generation
        if self.state == HOLD:
            self.scale = 0.0
            self.last_v = 0.0
            return 0.0, 0.0

        if self.state == GIVEUP:
            # Allow angular rotation so TEB can pivot away and reroute, but keep forward blocked
            self.scale = 0.0 if not corridor_clear else target
            v_out = 0.0 if not corridor_clear else v * self.scale
            self.last_v = v_out
            return v_out, w

        # FREE State
        if target < self.scale:
            self.scale = target  # Immediate braking
        else:
            self.scale = min(target, self.scale + p["release_rate"] * dt)

        v_out = v * self.scale
        self.last_v = v_out
        return v_out, w
