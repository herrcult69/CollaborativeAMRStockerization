"""
Velocity-gate decision logic: Dual-Shield architecture with spin-first rotation.
Adheres to ISO 3691-4 industrial safety principles with zero external ROS dependencies.

All shield geometry is in the drive_center frame, measured from the footprint nose (nose_x):
1. Inner Protective Shield: rectangle |y| <= inner_half_w (chassis 0.23 m + margin), x in [nose, nose + d_inner].
   The only hard stop: a point inside it (gap <= d_inner / d_hold_static) means HOLD [blocked].
2. Outer Warning Shield: half-ellipse centred on the nose, forward semi-axis d_outer, lateral semi-axis outer_half_w.
   Speed scale ramps from scale_min (at the inner depth) to 1.0 on the normalised elliptical distance
   d_outer * sqrt((dx / d_outer)^2 + (y / outer_half_w)^2). It only slows, it never HOLDs.
3. Flank Shield: optional (flank_enable, default off). Side clearance is TEB's job (footprint + min_obstacle_dist).
4. Crossing yield: HOLD [crossing] for compact obstacles moving laterally into the corridor ahead (watch zone depth
   cross_x_max). Ego-motion comes from measured odometry; lateral velocity is taken over a cross_v_window baseline.
5. Spin-first: in FREE, a path heading error above spin_enter makes the gate rotate in place toward the plan
   (v = 0, |w| = spin_w) until it falls below spin_exit. In HOLD [blocked], the same rotation is allowed after
   spin_hold_delay. HOLD [crossing] / [flank] never rotate.
6. Open Space Freedom: with nothing in the outer shield the gate is transparent to TEB (curvature preserved).
7. Release Hysteresis & Deadlock Timeout: chatter-free resumption, GIVEUP (|w| <= giveup_w_max) after hold_timeout.
"""
import math

FREE, HOLD, GIVEUP = "free", "hold", "giveup"

DEFAULTS = dict(
    nose_x=0.37,              # Footprint front edge ahead of drive_center (m)
    laser_x=0.33,             # Laser mount ahead of drive_center (m)
    tail_x=0.54,              # Footprint rear edge behind drive_center (m)
    half_w=0.33,              # Crossing-corridor half-width (robot 0.23m + side margin 0.10m); NOT the inner shield
    inner_half_w=0.26,        # Inner shield half-width: chassis (0.23m) + 0.03m
    d_inner=0.20,             # Inner protective shield depth ahead of nose (m) [Category 0 Stop]
    outer_half_w=0.50,        # Outer shield (half-ellipse) lateral semi-axis (m)
    d_outer=1.00,             # Outer shield (half-ellipse) forward semi-axis ahead of nose (m)
    d_hold_static=0.20,       # Distance ahead to trigger in-line HOLD for stationary obstacle (m)
    min_crawl_speed=0.10,     # Minimum forward speed during slowdown (prevents stall on PhysX friction) (m/s)
    flank_enable=False,       # Flank shield off by default (side clearance is TEB's job)
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
    cross_x_max=1.50,         # Depth of crossing watch zone ahead of the nose (m), independent of d_outer
    cross_v_window=0.40,      # Baseline over which a track's lateral velocity is measured (s)
    cross_v_min_span=0.25,    # Min track age before its velocity is trusted (s)
    odom_timeout=0.30,        # Measured odom older than this falls back to commanded v/w (s)
    cross_v_min=0.15,         # Min lateral closing speed to count as crossing (m/s)
    cross_t_horizon=2.5,      # Max time-to-enter-corridor that triggers yield (s)
    cross_assoc_gap=0.35,     # Max centroid jump to associate a cluster across scans (m)
    cross_confirm=3,          # Consecutive scans a crossing threat must persist
    cross_max_extent=0.80,    # Ignore clusters longer than this (walls, rack faces) (m)
    cross_w_max=0.45,         # Skip crossing detection while |w| exceeds this (rad/s); 0.45 = max_vel_theta = off
    blocker_half_width=0.60,  # Half-length of the GIVEUP blocker line across the passage (m)
    blocker_ttl=60.0,         # How long the global-costmap blocker stays published (s)
    blocker_spacing=0.05,     # Point spacing along the blocker line (m)
    shim_lookahead=0.60,      # GIVEUP rotation shim: distance walked along the new plan (m)
    shim_tol=0.15,            # GIVEUP rotation shim: heading error below this stops turning (rad)
    shim_plan_age=3.0,        # GIVEUP rotation shim: max age of the plan used (s)
    spin_enable=True,         # Spin-first: rotate in place toward the plan when facing away from it
    spin_enter=1.05,          # Heading error that starts a spin (rad, 60 deg)
    spin_exit=0.35,           # Heading error that ends a spin (rad, 20 deg)
    spin_w=0.45,              # Spin angular speed (rad/s)
    spin_hold_delay=2.0,      # HOLD [blocked] allows the spin only after this long (s)
    spin_min_remaining=0.50,  # No spin when less plan than this is left (goal-yaw rotation belongs to TEB) (m)
    reinit_enable=True,       # Band monitor: ask TEB to rebuild its band when it looks wrong (see band_health)
    reinit_bulge=0.50,        # Local plan this far from the global plan = bad band (m)
    reinit_ratio=1.60,        # Local plan this many times longer than the global stretch it covers = bad band
    reinit_cusp_seg=0.03,     # Segments shorter than this are ignored by the cusp test (m)
    reinit_confirm=2,         # Consecutive bad local plans before a reinit is requested
    reinit_cooldown=3.0,      # Min time between detector-triggered reinits (s)
    reinit_hold=0.40,         # How long TEB re-initialises every cycle (and the gate outputs zero) (s)
    reinit_min_override=0.50,  # An override (HOLD / spin / GIVEUP) shorter than this does not trigger a reinit (s)
)


def blocker_points(gap, p):
    """Base-frame points of a line across the heading, just past the blocked gap."""
    x = p["nose_x"] + max(gap, 0.10)
    n = int(round(2.0 * p["blocker_half_width"] / p["blocker_spacing"]))
    return [(x, -p["blocker_half_width"] + i * p["blocker_spacing"]) for i in range(n + 1)]


def path_heading_error(pose, plan_xy, lookahead):
    """Bearing error (rad, [-pi, pi], +left) from pose (x, y, yaw) to a point `lookahead` m along plan_xy.

    Starts at the plan point nearest the robot. Returns None for an empty plan.
    """
    if not plan_xy:
        return None
    x, y, yaw = pose
    i = min(range(len(plan_xy)), key=lambda k: (plan_xy[k][0] - x) ** 2 + (plan_xy[k][1] - y) ** 2)
    tx, ty = plan_xy[i]
    walked = 0.0
    while i + 1 < len(plan_xy) and walked < lookahead:
        nx, ny = plan_xy[i + 1]
        walked += math.hypot(nx - tx, ny - ty)
        tx, ty = nx, ny
        i += 1
    err = math.atan2(ty - y, tx - x) - yaw
    return math.atan2(math.sin(err), math.cos(err))


def path_remaining(pose, plan_xy):
    """Plan length (m) from the point nearest to pose (x, y, yaw) to the end of plan_xy; 0.0 for an empty plan."""
    if not plan_xy:
        return 0.0
    x, y = pose[0], pose[1]
    i = min(range(len(plan_xy)), key=lambda k: (plan_xy[k][0] - x) ** 2 + (plan_xy[k][1] - y) ** 2)
    return sum(math.hypot(plan_xy[k + 1][0] - plan_xy[k][0], plan_xy[k + 1][1] - plan_xy[k][1])
               for k in range(i, len(plan_xy) - 1))


def _seg_dist(px, py, ax, ay, bx, by):
    """Distance from point (px, py) to segment (a, b)."""
    dx, dy = bx - ax, by - ay
    n = dx * dx + dy * dy
    t = 0.0 if n <= 1e-12 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / n))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def band_health(local_xy, global_xy, p):
    """Is TEB's local plan sane next to the global plan? Both lists are in the ROBOT frame (robot at the origin).

    Returns (bad, why). Bad = sideways bulge, detour (local much longer than the global stretch it covers) or a cusp.
    Never bad near the goal, where TEB legitimately rotates and curves.
    """
    if len(local_xy) < 3 or len(global_xy) < 2:
        return False, ""
    if path_remaining((0.0, 0.0, 0.0), global_xy) < p["spin_min_remaining"]:
        return False, ""

    segs = list(zip(global_xy[:-1], global_xy[1:]))
    bulge = max(min(_seg_dist(x, y, a[0], a[1], b[0], b[1]) for a, b in segs) for x, y in local_xy)
    if bulge > p["reinit_bulge"]:
        return True, "bulge %.2fm" % bulge

    local_len = sum(math.hypot(local_xy[k + 1][0] - local_xy[k][0], local_xy[k + 1][1] - local_xy[k][1])
                    for k in range(len(local_xy) - 1))
    if local_len > 0.8:
        def nearest(pt):
            return min(range(len(global_xy)), key=lambda k: (global_xy[k][0] - pt[0]) ** 2 + (global_xy[k][1] - pt[1]) ** 2)
        i0, i1 = sorted((nearest(local_xy[0]), nearest(local_xy[-1])))
        g_len = sum(math.hypot(global_xy[k + 1][0] - global_xy[k][0], global_xy[k + 1][1] - global_xy[k][1])
                    for k in range(i0, i1))
        if local_len > p["reinit_ratio"] * max(g_len, 0.30):
            return True, "detour x%.1f" % (local_len / max(g_len, 0.30))

    prev = None
    for k in range(len(local_xy) - 1):
        dx, dy = local_xy[k + 1][0] - local_xy[k][0], local_xy[k + 1][1] - local_xy[k][1]
        if math.hypot(dx, dy) < p["reinit_cusp_seg"]:
            continue
        if prev is not None and prev[0] * dx + prev[1] * dy < 0.0:
            return True, "cusp"
        prev = (dx, dy)
    return False, ""


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

        self.gap = float("inf")          # inner-shield (rectangle) distance ahead of the nose
        self.gap_outer = float("inf")    # outer-shield (half-ellipse) normalised distance
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
        self._tracks = []          # [{"pos": (X, Y), "hist": [(t, X, Y), ...]}] in a gate-local odom frame
        self._prev_scan_t = None
        self._gx = self._gy = self._gyaw = 0.0   # gate-local odom pose, integrated from measured v/w
        self.spinning = False

        self.reinit_count = 0            # TEB band re-init requests so far
        self.reinit_reason = ""
        self._reinit_until = -1.0
        self._last_reinit = -1e9
        self._bad_hits = 0
        self._override = False           # state != FREE or spinning at the end of the last step
        self._override_t0 = 0.0

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

    def _update_crossing(self, clusters, now, vw=None):
        """Detect obstacles moving across the corridor ahead (ego-motion compensated).

        Clusters are tracked in a gate-local odom frame integrated from the measured (v, w), so a static object stays
        static while the robot yaws. Lateral velocity is measured over a cross_v_window baseline, not one scan.
        """
        p = self.p
        v, w = vw if vw is not None else (self.last_v, self.last_w)
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
        centroids = [(sum(q[0] for q in c) / len(c), sum(q[1] for q in c) / len(c)) for c in good]

        dt = (now - self._prev_scan_t) if self._prev_scan_t is not None else None
        self._prev_scan_t = now
        if dt is None or not (1e-3 < dt <= 0.5):
            # First scan, duplicate stamp or scan gap: restart the local frame and drop all tracks
            self._gx = self._gy = self._gyaw = 0.0
            self._tracks = []
        else:
            mid = self._gyaw + 0.5 * w * dt
            self._gx += v * math.cos(mid) * dt
            self._gy += v * math.sin(mid) * dt
            self._gyaw += w * dt

        cg, sg = math.cos(self._gyaw), math.sin(self._gyaw)
        old, tracks, used = self._tracks, [], set()
        for cx, cy in centroids:
            pos = (self._gx + cx * cg - cy * sg, self._gy + cx * sg + cy * cg)
            best, best_d = None, p["cross_assoc_gap"]
            for i, tr in enumerate(old):
                if i in used:
                    continue
                d = math.hypot(pos[0] - tr["pos"][0], pos[1] - tr["pos"][1])
                if d <= best_d:
                    best, best_d = i, d
            hist = []
            if best is not None:
                used.add(best)
                hist = [h for h in old[best]["hist"] if now - h[0] <= p["cross_v_window"] + 1e-6]
            hist.append((now, pos[0], pos[1]))
            tracks.append({"pos": pos, "hist": hist, "local": (cx, cy)})
        self._tracks = tracks

        threat = None
        turning = abs(w) > p["cross_w_max"]
        x_hi = p["nose_x"] + p["cross_x_max"]
        for tr in tracks:
            cx, cy = tr["local"]
            if turning or not (p["nose_x"] <= cx <= x_hi and abs(cy) <= p["cross_zone_w"]):
                continue
            t0, x0, y0 = tr["hist"][0]
            span = now - t0
            if span < p["cross_v_min_span"] - 1e-6:
                continue
            # World-frame track velocity projected on the robot's lateral axis (ego motion already removed)
            vx_w, vy_w = (tr["pos"][0] - x0) / span, (tr["pos"][1] - y0) / span
            vy = -sg * vx_w + cg * vy_w
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

    def update_scan(self, points, now, moving=None, pose=None, vw=None):
        """Process incoming LiDAR scan points in robot drive_center frame.

        vw (optional): measured (v, w) from odometry, used for crossing ego-motion; falls back to the last output.
        """
        p = self.p
        self.scan_t = now

        # 1. Filter noise: group valid returns into physical clusters
        self._last_points = [tuple(q) for q in points]
        clusters = self._cluster_points(points)

        # 2. Shield distances for points ahead of the nose:
        #    gap       - inner shield rectangle (|py| <= inner_half_w): the stop / HOLD distance
        #    gap_outer - half-ellipse (forward d_outer, lateral outer_half_w): normalised distance that drives the slow-down
        gap = gap_outer = float("inf")
        closest_obs = None
        d_outer, ow = p["d_outer"], p["outer_half_w"]
        for cluster in clusters:
            for px, py in cluster:
                if px < p["nose_x"]:
                    continue
                dx = px - p["nose_x"]
                if abs(py) <= p["inner_half_w"] and dx < gap:
                    gap = dx
                    closest_obs = (px, py)
                e = d_outer * math.sqrt((dx / d_outer) ** 2 + (py / ow) ** 2)
                if e < gap_outer:
                    gap_outer = e
        self.gap = gap
        self.gap_outer = gap_outer
        self.closest_obs = closest_obs

        # 3. Flank Protective Shield (optional): side chassis clearance
        flank_breach = False
        flank_obs = None
        if p["flank_enable"]:
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
        self._update_crossing(clusters, now, vw)

    def _spin_target(self, heading_err):
        """Spin-first hysteresis: True while the robot should rotate in place toward the plan."""
        p = self.p
        if not p["spin_enable"] or heading_err is None:
            self.spinning = False
        elif self.spinning:
            self.spinning = abs(heading_err) >= p["spin_exit"]
        else:
            self.spinning = abs(heading_err) > p["spin_enter"]
        return self.spinning

    def _request_reinit(self, now, why):
        self._reinit_until = now + self.p["reinit_hold"]
        self._last_reinit = now
        self.reinit_count += 1
        self.reinit_reason = why

    def reinit_active(self, now):
        """True while TEB should re-initialise its band every cycle."""
        return now < self._reinit_until

    def band_update(self, local_xy, global_xy, now):
        """Band monitor: feed one TEB local plan (robot frame) with the global plan (robot frame)."""
        p = self.p
        if not p["reinit_enable"]:
            return
        in_cooldown = now - self._last_reinit < p["reinit_cooldown"]
        if self.state != FREE or self.spinning or in_cooldown:
            self._bad_hits = 0     # override bends the band on purpose; the end of it is handled in step()
            return
        bad, why = band_health(local_xy, global_xy, p)
        if not bad:
            self._bad_hits = 0
            return
        self._bad_hits += 1
        if self._bad_hits >= p["reinit_confirm"]:
            self._bad_hits = 0
            self._request_reinit(now, why)

    def step(self, v, w, now, heading_err=None):
        """Gated (v_out, w_out): shields + spin-first (see _step), plus the band-reinit pause.

        An override (HOLD / spin / GIVEUP) bends TEB's warm-started band while the gate holds the wheels. When it ends
        after >= reinit_min_override, request a TEB re-init and output zero for reinit_hold so no stale command is used.
        """
        out = self._step(v, w, now, heading_err)
        override = self.state != FREE or self.spinning
        if override and not self._override:
            self._override_t0 = now
        elif self._override and not override and self.p["reinit_enable"] and now - self._override_t0 >= self.p["reinit_min_override"]:
            self._request_reinit(now, "override end")
        self._override = override
        if not override and self.reinit_active(now):
            self.last_v = self.last_w = 0.0
            return 0.0, 0.0
        return out

    def _step(self, v, w, now, heading_err=None):
        """Computes gated (v_out, w_out) enforcing Inner/Outer shields and spin-first rotation.

        heading_err (rad, optional): bearing error to the global path. Drives spin-first in FREE and HOLD [blocked],
        and the rotation shim in GIVEUP. None (no fresh plan) disables all three.
        """
        p = self.p
        dt = min(max(now - self.last_t, 1e-3), 0.2) if self.last_t is not None else 0.05
        self.last_t = now

        d_inner = p["d_inner"]
        d_outer = p["d_outer"]
        d_hold_static = p["d_hold_static"]
        min_crawl = p["min_crawl_speed"]

        blind = (self.scan_t is None) or (now - self.scan_t > p["scan_timeout"])

        # 1. Proportional deceleration: stop from the inner rectangle, ramp from the outer half-ellipse
        if blind or self.gap <= d_inner:
            target_scale = 0.0
        elif self.gap_outer <= d_outer:
            scale_min = p["scale_min"]
            target_scale = scale_min + (1.0 - scale_min) * (self.gap_outer - d_inner) / (d_outer - d_inner)
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
            self.trigger_info = "slowdown ahead at outer distance %.2fm (scale %.2f)" % (self.gap_outer, target_scale)
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
            self.scale = 0.0
            self.last_v = 0.0
            # HOLD [blocked] may rotate toward the plan after spin_hold_delay (turn out of the dead end).
            # Rotation follows the plan heading, not TEB's w, so there is no pivot-release-pivot loop.
            # HOLD [crossing] / [flank] stay a full freeze.
            if self.reason == "blocked" and now - self.hold_t0 >= p["spin_hold_delay"] and self._spin_target(heading_err):
                self.last_w = math.copysign(p["spin_w"], heading_err)
            else:
                self.spinning = False
                self.last_w = 0.0
            return 0.0, self.last_w

        if self.state == GIVEUP:
            self.spinning = False
            # Forward blocked, angular steering unblocked for replanning
            self.scale = 0.0 if not corridor_clear else target_scale
            v_out = 0.0 if not corridor_clear else v * self.scale
            self.last_v = v_out
            gmax = p["giveup_w_max"]
            if heading_err is None:
                w_out = max(-gmax, min(gmax, w))
            elif abs(heading_err) > p["shim_tol"]:
                w_out = math.copysign(gmax, heading_err)
            else:
                w_out = 0.0
            self.last_w = w_out
            return v_out, w_out

        # FREE State
        if target_scale < self.scale:
            self.scale = target_scale  # Immediate braking
        else:
            self.scale = min(target_scale, self.scale + p["release_rate"] * dt)

        # Spin-first: facing too far away from the plan -> rotate in place toward it instead of a wide forward arc
        if self._spin_target(heading_err):
            self.trigger_info = "spin toward plan (heading error %.0f deg)" % math.degrees(heading_err)
            self.last_v = 0.0
            self.last_w = math.copysign(p["spin_w"], heading_err)
            return 0.0, self.last_w

        # Prevent velocity from dropping into an unworkable trickle (< min_crawl_speed) while in motion
        if self.scale > 0.0 and v > 0.0:
            v_out = max(min_crawl, v * self.scale)
        else:
            v_out = v * self.scale

        # 4. Angular Velocity Control: keep the curvature TEB planned (w / v unchanged).
        # w follows the same factor as v (including the crawl floor); pure rotation passes unchanged.
        if v > 0.02:
            w_out = w * (v_out / v)
        else:
            w_out = w

        self.last_v = v_out
        self.last_w = w_out
        return v_out, w_out
