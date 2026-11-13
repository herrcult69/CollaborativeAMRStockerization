"""
Velocity-gate decision logic: Dual-Shield Linear Heading Guard Architecture.
Adheres to ISO 3691-4 industrial safety principles with zero external ROS dependencies.

Architecture:
1. Inner Protective Shield (d_inner = 0.35m): Deterministic zero-velocity stop if an obstacle breaches close range.
2. Outer Warning Shield (d_outer = 1.00m): Proportional velocity scaling based on forward corridor gap.
3. Linear Heading Lock (Aisle Swerve Protection): When an obstacle is detected in the outer shield (gap <= d_outer = 1.00m)
   AND driving forward at cruising speed (v > 0.02 m/s), angular velocity is clamped (|w| <= omega_corridor_max = 0.10 rad/s)
   to prevent erratic swerving arcs into racking.
4. Pivot Rotation: In FREE with a clear outer shield, in-place rotation (v = 0) is unclamped. In HOLD the AMR never
   moves or rotates; only GIVEUP (after hold_timeout = 10 s) rotates, capped at giveup_w_max = 0.30 rad/s.
5. Clean In-Line Stop (d_hold_static = 0.45m): Enforces a crisp stop ahead of obstacles without crawling at < 0.01 m/s.
6. Flank Protective Shield: Immediate HOLD [flank] if an object breaches the side chassis clearance zone.
7. Open Space Freedom: When the corridor ahead is clear (gap > d_outer), the gate is 100% transparent, giving
   TEB unconstrained steering freedom for turns, intersections, and docking.
8. Release Hysteresis & Deadlock Timeout: Ensures chatter-free resumption after clearance and allows reroute on timeout.
"""
import math

FREE, HOLD, GIVEUP = "free", "hold", "giveup"

DEFAULTS = dict(
    nose_x=0.37,              # Footprint front edge ahead of drive_center (m)
    laser_x=0.33,             # Laser mount ahead of drive_center (m)
    tail_x=0.54,              # Footprint rear edge behind drive_center (m)
    half_w=0.33,              # Driving corridor half-width: robot (0.23m) + side margin (0.10m)
    d_inner=0.25,             # Inner protective shield depth ahead of nose (m) [Category 0 Stop]
    d_outer=1.00,             # Outer warning shield depth ahead of nose (m) [Deceleration horizon]
    d_hold_static=0.30,       # Distance ahead to trigger in-line HOLD for stationary obstacle (m)
    omega_corridor_max=0.10,  # Max angular velocity allowed when cruising in corridor (rad/s)
    min_crawl_speed=0.10,     # Minimum forward speed during slowdown (prevents stall on PhysX friction) (m/s)
    flank_x_min=-0.20,        # Rear boundary along chassis for side flank protection (m)
    flank_w=0.35,             # Lateral boundary from centerline for flank protection (m)
    seg_gap=0.20,             # Max distance between adjacent rays in same obstacle cluster (m)
    min_cluster_pts=3,        # Min rays to confirm obstacle (noise rejection)
    release_rate=1.0,         # Acceleration ramp-up (scale change per second)
    scan_timeout=0.50,        # Drop out to stop if scan is delayed (s)
    hold_timeout=10.0,        # Max duration to hold before giving up to allow reroute (s)
    giveup_w_max=0.30,        # Max |w| while in GIVEUP rotation escape (rad/s)
    scale_min=0.25,           # Slow-down scale just outside d_inner (floor of the ramp)
    clear_margin=0.10,        # Gap must exceed d_inner + clear_margin to count as clear (m)
    clear_time=0.80,          # Duration corridor must stay clear before releasing HOLD (s)
    cross_zone_w=1.50,        # Lateral half-width of crossing watch zone ahead (m)
    cross_v_min=0.15,         # Min lateral closing speed to count as crossing (m/s)
    cross_t_horizon=2.5,      # Max time-to-enter-corridor that triggers yield (s)
    cross_assoc_gap=0.35,     # Max centroid jump to associate a cluster across scans (m)
    cross_confirm=3,          # Consecutive scans a crossing threat must persist
    cross_max_extent=0.80,    # Ignore clusters longer than this (walls, rack faces) (m)
    cross_w_max=0.30,         # Skip crossing detection while |commanded w| exceeds this (rad/s)
    # Backward compatibility aliases:
    d_stop=0.35,              # Maps to d_inner
    d_slow=1.20,              # Maps to d_outer
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
        self.closest_obs = None
        self.reason = None
        self.action = None

        self.state = FREE
        self.scale = 1.0
        self.last_v = 0.0
        self.last_w = 0.0
        self.last_t = None
        self.scan_t = None
        self.hold_t0 = None
        self.clear_since = None

        self.flank_breach = False
        self.flank_obs = None
        self.trigger_info = ""

        self.crossing = False
        self._last_points = []
        self.crossing_obs = None   # (x, y, vy) of the threatening cluster
        self._cross_hits = 0
        self._prev_centroids = []
        self._prev_scan_t = None

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

    def _update_crossing(self, clusters, now):
        """Detect obstacles moving across the corridor ahead (ego-motion compensated)."""
        p = self.p
        # Only compact, fully-seen clusters are trackable: skip long extents and FOV-edge-truncated ones.
        n_pts = len(self._last_points)
        edge_pts = {self._last_points[0], self._last_points[-1]} if n_pts else set()
        good = []
        for c in clusters:
            if math.hypot(c[-1][0] - c[0][0], c[-1][1] - c[0][1]) > p["cross_max_extent"]:
                continue
            if edge_pts and any(q in edge_pts for q in c):
                continue
            good.append(c)
        clusters = good
        centroids = [(sum(q[0] for q in c) / len(c), sum(q[1] for q in c) / len(c)) for c in clusters]
        dt = (now - self._prev_scan_t) if self._prev_scan_t is not None else None
        prev, self._prev_centroids, self._prev_scan_t = self._prev_centroids, centroids, now

        threat = None
        turning = abs(self.last_w) > p["cross_w_max"]
        if dt is not None and not turning and 1e-3 < dt <= 0.5 and prev:
            # Express previous centroids in the current robot frame (commanded motion over dt)
            ang = -self.last_w * dt
            ca, sa = math.cos(ang), math.sin(ang)
            comp = []
            for x, y in prev:
                x -= self.last_v * dt
                comp.append((x * ca - y * sa, x * sa + y * ca))
            x_hi = p["nose_x"] + p.get("d_outer", p.get("d_slow", 1.20))
            for cx, cy in centroids:
                if not (p["nose_x"] <= cx <= x_hi and abs(cy) <= p["cross_zone_w"]):
                    continue
                best, best_d = None, p["cross_assoc_gap"]
                for qx, qy in comp:
                    d = math.hypot(cx - qx, cy - qy)
                    if d <= best_d:
                        best, best_d = (qx, qy), d
                if best is None:
                    continue
                vy = (cy - best[1]) / dt
                if abs(vy) < p["cross_v_min"]:
                    continue
                if abs(cy) <= p["half_w"]:
                    hit = True   # already inside corridor and still moving laterally
                else:
                    hit = (cy * vy < 0) and ((abs(cy) - p["half_w"]) / abs(vy) <= p["cross_t_horizon"])
                if hit and (threat is None or abs(cy) < abs(threat[1])):
                    threat = (cx, cy, vy)

        if threat:
            self._cross_hits += 1
            self.crossing_obs = threat
        else:
            self._cross_hits = 0
            self.crossing_obs = None
        self.crossing = self._cross_hits >= p["cross_confirm"]

    def update_scan(self, points, now, moving=None, pose=None):
        """Process incoming LiDAR scan points in robot drive_center frame."""
        p = self.p
        self.scan_t = now

        # 1. Filter noise: group valid returns into physical clusters
        self._last_points = [tuple(q) for q in points]
        clusters = self._cluster_points(points)

        # 2. Forward Driving Corridor: closest point within |py| <= half_w and px >= nose_x
        gap = float("inf")
        closest_obs = None
        for cluster in clusters:
            for px, py in cluster:
                if px >= p["nose_x"] and abs(py) <= p["half_w"]:
                    d = px - p["nose_x"]
                    if d < gap:
                        gap = d
                        closest_obs = (px, py)
        self.gap = gap
        self.closest_obs = closest_obs

        # 3. Flank Protective Shield (side chassis clearance: 220-270 deg FOV)
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

        # 4. Crossing-obstacle detection (yield before it enters the corridor)
        self._update_crossing(clusters, now)

    def step(self, v, w, now):
        """Computes gated (v_out, w_out) enforcing Inner/Outer shields and in-place pivot freedom."""
        p = self.p
        dt = min(max(now - self.last_t, 1e-3), 0.2) if self.last_t is not None else 0.05
        self.last_t = now

        d_inner = p.get("d_inner", p.get("d_stop", 0.35))
        d_outer = p.get("d_outer", p.get("d_slow", 1.20))
        d_hold_static = p.get("d_hold_static", 0.45)
        omega_max = p.get("omega_corridor_max", 0.05)
        min_crawl = p.get("min_crawl_speed", 0.10)

        blind = (self.scan_t is None) or (now - self.scan_t > p["scan_timeout"])

        # 1. Proportional Deceleration Scale in Outer Shield
        if blind or self.gap <= d_inner:
            target_scale = 0.0
        elif self.gap <= d_outer:
            scale_min = p.get("scale_min", 0.25)
            target_scale = scale_min + (1.0 - scale_min) * (self.gap - d_inner) / (d_outer - d_inner)
            target_scale = max(0.0, min(1.0, target_scale))
        else:
            target_scale = 1.0

        # Diagnostic trigger string for logging
        if self.flank_breach and self.flank_obs:
            self.trigger_info = "flank breach at (x=%.2fm, y=%.2fm)" % self.flank_obs
        elif self.crossing and self.crossing_obs:
            self.trigger_info = "yield: crossing obstacle at (%.2f, %.2f) vy=%.2f" % self.crossing_obs
        elif target_scale <= 0.0 or self.gap <= d_hold_static:
            self.trigger_info = "blocked ahead at gap %.2fm" % self.gap
        elif target_scale < 1.0:
            self.trigger_info = "slowdown ahead at gap %.2fm (scale %.2f)" % (self.gap, target_scale)
        else:
            self.trigger_info = "clear"

        hold_threshold = max(d_inner, d_hold_static)
        corridor_clear = (not blind) and (self.gap > hold_threshold + p["clear_margin"]) and (not self.flank_breach) and (not self.crossing)

        # 2. State Machine: FREE <-> HOLD -> GIVEUP
        if self.state == FREE:
            if self.flank_breach:
                self.state = HOLD
                self.reason = "flank"
                self.hold_t0 = now
                self.clear_since = None
            elif self.crossing:
                self.state = HOLD
                self.reason = "crossing"
                self.hold_t0 = now
                self.clear_since = None
            elif target_scale <= 0.0 or self.gap <= d_hold_static:
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
            # Inner shield is a full stop: no translation, no rotation (avoids pivot-release-pivot loop)
            self.scale = 0.0
            self.last_v = 0.0
            self.last_w = 0.0
            return 0.0, 0.0

        if self.state == GIVEUP:
            # Forward blocked, angular steering unblocked for replanning
            self.scale = 0.0 if not corridor_clear else target_scale
            v_out = 0.0 if not corridor_clear else v * self.scale
            self.last_v = v_out
            gmax = p.get("giveup_w_max", 0.30)
            w_out = max(-gmax, min(gmax, w))
            self.last_w = w_out
            return v_out, w_out

        # FREE State
        if target_scale < self.scale:
            self.scale = target_scale  # Immediate braking
        else:
            self.scale = min(target_scale, self.scale + p["release_rate"] * dt)

        # Prevent velocity from dropping into an unworkable trickle (< min_crawl_speed) while in motion
        if self.scale > 0.0 and v > 0.0:
            v_out = max(min_crawl, v * self.scale)
        else:
            v_out = v * self.scale

        # 4. Angular Velocity Control:
        # Only clamp w when an obstacle is detected in the outer warning shield (gap <= d_outer or scale < 1.0)
        # AND driving forward at cruising speed (v_out > 0.02 m/s).
        # In clear open space (gap > d_outer), TEB has 100% full steering freedom!
        if (self.gap <= d_outer or self.scale < 1.0) and v_out > 0.02 and abs(w) > omega_max:
            w_out = math.copysign(omega_max, w)
        else:
            w_out = w

        self.last_v = v_out
        self.last_w = w_out
        return v_out, w_out
