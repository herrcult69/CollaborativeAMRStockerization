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
    d_hold_static=0.50,       # Distance ahead to trigger in-line HOLD for stationary obstacle (m)
    d_slow=1.50,              # Warning zone slowdown depth ahead of nose (m)
    yield_dist=1.50,          # Distance ahead to trigger head-on yield (m)
    closing_rate_thr=-0.15,   # Obstacle world-velocity threshold toward robot (m/s)
    flank_x_min=-0.20,        # Rear boundary along chassis for side flank protection (m)
    flank_w=0.35,             # Lateral boundary from centerline for flank protection (m)
    cross_y_max=2.20,         # Outer lateral monitoring window for crossing traffic (m) - 220 deg FOV
    cross_rate_thr=-0.15,     # Lateral inward velocity threshold to trigger crossing yield (m/s)
    cross_rate_max=-2.50,     # Max plausible physical speed (rejects multi-object jumps) (m/s)
    cross_x_min=-0.10,        # Longitudinal bounds of intersection crossing zone (m)
    cross_x_max=1.80,         # Longitudinal bounds of intersection crossing zone (m)
    cross_clear_w=1.00,       # Lateral corridor must clear to this width before unlocking crossing (m)
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
        self.flank_breach = False
        self.flank_obs = None
        self.cross_hold = None
        self.cross_rate = 0.0
        self.lat_hist = []
        self.lat_obs = None
        self.intersection_occupied = False
        self.trigger_info = ""

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

        # 1. Head-on oncoming vehicle detection
        if gap <= p["yield_dist"] and obs_v <= p["closing_rate_thr"]:
            self.dyn_hold = "moving"
            self.dyn_gap = gap
        else:
            self.dyn_hold = None
            self.dyn_gap = float("inf")

        # 2. Flank Protection Shield (detects objects along the side chassis using 220-270 deg FOV)
        flank_breach = False
        flank_obs = None
        for cluster in clusters:
            for px, py in cluster:
                if p["flank_x_min"] <= px <= p["nose_x"] and 0.23 < abs(py) <= p["flank_w"]:
                    flank_breach = True
                    flank_obs = (px, py)
                    break
            if flank_breach:
                break
        self.flank_breach = flank_breach
        self.flank_obs = flank_obs

        # 3. Lateral Crossing Anticipation (detects crossing traffic entering intersection)
        min_lat = float("inf")
        lat_obs = None
        for cluster in clusters:
            for px, py in cluster:
                if p["cross_x_min"] <= px <= p["cross_x_max"] and p["half_w"] < abs(py) <= p["cross_y_max"]:
                    if abs(py) < min_lat:
                        min_lat = abs(py)
                        lat_obs = (px, py)

        self.lat_obs = lat_obs
        self.lat_hist.append((now, min_lat))
        self.lat_hist = [h for h in self.lat_hist if now - h[0] <= p["closing_window"]]

        cross_v = 0.0
        if len(self.lat_hist) >= 2 and min_lat < float("inf"):
            for t0, l0 in self.lat_hist:
                dt = now - t0
                if dt >= p["closing_min_dt"]:
                    if l0 < float("inf"):
                        cross_v = (min_lat - l0) / dt
                    break
        self.cross_rate = cross_v

        # Only legitimate physical speeds (-1.80 <= cross_v <= -0.15 m/s) trigger crossing hold.
        # Faster jumps (< -1.80 m/s) are step discontinuities from new static objects entering FOV.
        if min_lat <= p["cross_y_max"] and (p["cross_rate_max"] <= cross_v <= p["cross_rate_thr"]):
            self.cross_hold = "crossing"
        else:
            self.cross_hold = None

        # 4. Intersection Occupancy Zone: Check if any obstacle is currently inside intersection envelope
        intersection_occupied = False
        for cluster in clusters:
            for px, py in cluster:
                if p["cross_x_min"] <= px <= p["cross_x_max"] and abs(py) <= p["cross_clear_w"]:
                    intersection_occupied = True
                    break
            if intersection_occupied:
                break
        self.intersection_occupied = intersection_occupied

    def step(self, v, w, now):
        """Computes gated (v_out, w_out) at time now."""
        p = self.p
        dt = min(max(now - self.last_t, 1e-3), 0.2) if self.last_t is not None else 0.05
        self.last_t = now

        # Proportional target scale based on forward corridor gap
        blind = (self.scan_t is None) or (now - self.scan_t > p["scan_timeout"])
        if blind or self.gap <= p["d_stop"]:
            target = 0.0
        elif self.gap <= p["d_slow"]:
            target = (self.gap - p["d_stop"]) / (p["d_slow"] - p["d_stop"])
            target = max(0.0, min(1.0, target))
        else:
            target = 1.0

        # Assess dynamic threats (head-on oncoming, lateral crossing, or flank breach)
        dyn_threat = self.dyn_hold or self.cross_hold
        if self.flank_breach:
            dyn_threat = "flank"

        # Update human-readable diagnostic trigger description
        if dyn_threat == "flank" and self.flank_obs:
            self.trigger_info = "flank breach at (x=%.2fm, y=%.2fm)" % self.flank_obs
        elif dyn_threat == "crossing" and self.lat_obs:
            self.trigger_info = "crossing at (x=%.2fm, y=%.2fm), vy=%.2fm/s" % (self.lat_obs[0], self.lat_obs[1], self.cross_rate)
        elif dyn_threat == "moving":
            self.trigger_info = "head-on at gap %.2fm, vx=%.2fm/s" % (self.gap, self.closing_rate)
        elif target <= 0.0 or self.gap <= p["d_hold_static"]:
            self.trigger_info = "blocked ahead at gap %.2fm" % self.gap
        else:
            self.trigger_info = "clear"

        hold_threshold = max(p["d_stop"], p["d_hold_static"])
        corridor_clear = (not blind) and (self.gap > hold_threshold + p["clear_margin"]) and (dyn_threat is None) and (not self.flank_breach)

        # Intersection Occupancy Lock: While yielding to crossing traffic, stay locked until intersection is clear
        if self.state == HOLD and self.reason == "crossing" and self.intersection_occupied:
            corridor_clear = False
            self.clear_since = None

        # 2. State Machine: FREE <-> HOLD -> GIVEUP
        if self.state == FREE:
            if dyn_threat:
                self.state = HOLD
                self.reason = dyn_threat
                self.hold_t0 = now
                self.clear_since = None
            elif v > 0.0 and (target <= 0.0 or self.gap <= p["d_hold_static"]):
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
        # Suppress aggressive swerving into racking when slowed down by obstacle ahead
        w_out = w * self.scale if v > 0.0 else w
        self.last_v = v_out
        return v_out, w_out
