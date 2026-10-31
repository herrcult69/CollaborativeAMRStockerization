"""
Velocity-gate decision logic (no ROS imports, unit-testable).

Sits between move_base (/cmd_vel_nav) and the robot (/cmd_vel). Works in the drive_center frame with the scan
already converted to (px, py) points, plus the drive_center pose in odom for tracking moving obstacles.

Static obstacles (anything not tracked as moving)
  * Obstacle ahead in the swept corridor: slow, then stop at d_stop from the nose.
  * Once a forward command has been stopped, HOLD ("blocked"): fully zero Twist, also zero angular, so TEB cannot
    weave. Release after the corridor has been clear for clear_time.

Moving obstacles. Returns flagged by FreeSpaceDetector are clustered and tracked in odom (visible bounding box,
median velocity). Each track is classified by its own motion, seen from the robot:
  * HEAD-ON  (moving mostly along -x, toward the robot)
  * SIDE     (moving mostly sideways, |uy| >= |ux|): it is crossing the robot's path
  * ignored  (slower than min_track_speed, or moving away along +x): handled by the static rules
For every track three robot plans are predicted over predict_s, robot driving straight:
  continue = keep TEB's speed;  pass = drive at boost_v;  stop = brake now (stopping distance model).
A plan's clearance is the smallest gap between the footprint and the obstacle's visible bounding box along it
(negative = overlap depth); the plan is safe when that exceeds margin = obs_pad + obstacle speed * time_margin.

Decision, per track whose CONTINUE plan collides:
  * HEAD-ON within yield_dist ahead of the nose -> HOLD ("moving"): yield completely still.
  * SIDE: 0. a pass in progress continues until the obstacle is behind the fork tips by the margin (or has
             crossed and is moving away), as long as passing keeps any clearance
          1. PASS if passing is safe (speed up to boost_v, even if TEB slowed down, as long as TEB still commands
             forward: a zero, rotate-in-place or reverse command is never turned into a pass)
          2. else keep going while stopping defer_s later would still be safe (no premature stop)
          3. else HOLD ("crossing") if stopping now is safe: it will cross in front of the stopped robot
          4. else whichever of pass / stop keeps more clearance. Passing then is an ESCAPE: standing still would
             leave the robot in its path (an escape also ends a HOLD).
  The pass commitment is physical (fork tips past the obstacle), not "TEB's speed is predicted to be enough now".
  A SIDE track never causes a "moving" hold, even when its edge enters the corridor while the robot passes.
  Tracks that go out of the lidar's view beside the robot keep coasting at their last velocity (coast_timeout),
  so a pass is not abandoned halfway.

Holds and timeouts
  * A hold lasts at most hold_timeout. It must stay below move_base's oscillation_timeout / controller_patience
    (see warehouse_navigation.launch), otherwise move_base aborts the goal. After the timeout the gate GIVES UP
    holding: rotation and reversing pass through again so TEB can reroute; forward stays blocked.

The 180 degree scan only sees points at or ahead of the laser (px >= laser_x). Nothing beside the body or behind
the laser is visible.
"""

import math

FREE, HOLD, GIVEUP = "free", "hold", "giveup"
HEAD_ON, SIDE = "head-on", "side"
PASS, ESCAPE = "pass", "escape"

DEFAULTS = dict(
    nose_x=0.37,           # footprint front edge ahead of drive_center
    laser_x=0.33,          # laser ahead of drive_center (edge of the lidar's view)
    tail_x=0.54,           # footprint rear edge behind drive_center (forks)
    half_w=0.30,           # half corridor width = half robot width + side margin
    d_stop=0.30,           # full stop at this gap from the nose
    horizon_s=2.5,         # slow zone = d_stop + v * horizon_s
    release_rate=0.9,      # max scale change per second when speeding up after a slowdown / hold
    scan_timeout=0.5,
    hold_timeout=20.0,     # keep below move_base oscillation_timeout and controller_patience
    clear_margin=0.15,     # corridor counts as clear beyond d_stop + clear_margin
    clear_time=1.0,        # ...for this long, before the hold is released
    yield_dist=1.5,        # head-on moving obstacle this close ahead of the nose -> HOLD
    dyn_min_points=2,      # returns flagged as moving needed for a cluster to count (rejects single-ray noise)
    seg_gap=0.25,          # neighbouring returns closer than this belong to the same cluster
    track_ahead=5.0,       # moving clusters tracked up to this far ahead of drive_center...
    track_side=3.5,        # ...and this far to either side
    assoc_dist=0.6,        # max distance between a track's predicted centre and a new cluster
    vel_window=0.6,        # seconds of centre history used for the velocity estimate
    vel_min_span=0.25,     # a track needs this much history before it is used for decisions
    min_track_speed=0.15,  # slower tracks are treated as static
    pose_timeout=1.0,      # without a pose (TF) for this long, all tracks are dropped
    lost_timeout=0.5,      # drop a track not seen for this long while it should be visible
    coast_timeout=4.0,     # ...or for this long while it is predicted beside/behind the laser (out of view)
    predict_s=7.0,         # plan prediction horizon
    predict_dt=0.1,
    obs_pad=0.1,           # clearance required around the visible box (its far side is not seen)
    time_margin=0.3,       # obstacle radius grows by its speed * this, as a timing margin
    boost_v=0.5,          # absolute speed used to pass a side obstacle
    pass_max_w=0.2,        # |angular| allowed while passing: drive through, do not swerve
    defer_s=0.5,           # a side yield waits until stopping defer_s later would no longer be safe
    stop_latency=0.3,      # stopping distance = v * stop_latency + v^2 / (2 * stop_decel)
    stop_decel=0.5,
)


class FreeSpaceDetector:
    """Flags scan returns that lie in space an earlier scan saw as free, i.e. something moved there.

    Compared in the odom frame, so the robot's own driving and turning cancel out. Receding obstacles and newly
    revealed static surfaces lie in space that was occluded before, so they are not flagged.
    """

    def __init__(self, ref_age=0.4, tol=0.12, history=1.5):
        self.ref_age, self.tol, self.history = ref_age, tol, history
        self.scans = []      # (t, pose, ranges, angle_min, angle_inc, range_min, range_max)

    def update(self, ranges, angle_min, angle_inc, range_min, range_max, pose, now):
        """pose: (x, y, yaw) of the laser in odom. Returns one bool per ray (True = moving)."""
        ref = None
        for scan in self.scans:
            if now - scan[0] >= self.ref_age:
                ref = scan
        self.scans = [s for s in self.scans if now - s[0] <= self.history] + [
            (now, pose, list(ranges), angle_min, angle_inc, range_min, range_max)]
        if ref is None:
            return [False] * len(ranges)
        _, (rx, ry, ryaw), rranges, rmin_a, rinc, rmin, rmax = ref
        x, y, yaw = pose
        c, s = math.cos(ryaw), math.sin(ryaw)
        flags = []
        a = angle_min
        for r in ranges:
            moving = False
            if range_min < r < range_max:
                wx, wy = x + r * math.cos(yaw + a), y + r * math.sin(yaw + a)
                dx, dy = wx - rx, wy - ry
                lx, ly = c * dx + s * dy, -s * dx + c * dy
                j = int(round((math.atan2(ly, lx) - rmin_a) / rinc))
                if 1 <= j < len(rranges) - 1:
                    # Neighbouring rays too, so a point beside a static edge is not mistaken for motion.
                    seen_free = min(self._free_range(rranges[k], rmin, rmax) for k in (j - 1, j, j + 1))
                    moving = seen_free > math.hypot(lx, ly) + self.tol
            flags.append(moving)
            a += angle_inc
        return flags

    @staticmethod
    def _free_range(r, rmin, rmax):
        if r == float("inf"):
            return rmax                  # no hit: free up to max range
        if rmin < r < rmax:
            return r
        return 0.0                       # nan / too close / max-range: unknown


class _Track:
    """A moving obstacle in odom: history of its visible bounding-box centre, half sizes, velocity.

    Velocity is the median of the scan-to-scan steps over vel_window, so a one-off jump of the box centre (another
    face of the obstacle coming into view) does not corrupt it.
    """

    def __init__(self, x, y, hx, hy, t):
        self.hist = [(t, x, y)]
        self.hx, self.hy = hx, hy
        self.seen = t
        self.vx = self.vy = 0.0
        self.passing = False      # the robot is passing this obstacle: keep going until the forks are past it

    def span(self):
        return self.hist[-1][0] - self.hist[0][0]

    def predict(self, now):
        t, x, y = self.hist[-1]
        return x + self.vx * (now - t), y + self.vy * (now - t)

    def add(self, x, y, hx, hy, t, window):
        self.hist.append((t, x, y))
        self.hist = [h for h in self.hist if t - h[0] <= window]
        self.hx, self.hy, self.seen = hx, hy, t
        steps = [((x1 - x0) / (t1 - t0), (y1 - y0) / (t1 - t0))
                 for (t0, x0, y0), (t1, x1, y1) in zip(self.hist, self.hist[1:]) if t1 - t0 > 1e-3]
        if steps:
            self.vx, self.vy = _median([v[0] for v in steps]), _median([v[1] for v in steps])


def _median(xs):
    xs = sorted(xs)
    n = len(xs)
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


class GateLogic:
    def __init__(self, **overrides):
        self.p = dict(DEFAULTS)
        unknown = set(overrides) - set(DEFAULTS)
        if unknown:
            raise KeyError("unknown gate params: %s" % sorted(unknown))
        self.p.update(overrides)
        self.gap = float("inf")
        self.dyn_gap = float("inf")    # nearest head-on moving obstacle that would collide, from the nose
        self.reason = None             # why the current HOLD started: "blocked", "moving", "crossing"
        self.action = None             # side obstacle response this step: None, "pass", "escape"
        self.pass_note = None          # why the last pass ended (for the log), e.g. "forks 0.30 m past it"
        self.scan_t = None
        self.pose = None               # drive_center (x, y, yaw) in odom at the last scan
        self.pose_t = None             # when that pose was last measured (not dead-reckoned)
        self.pose_dr_t = None          # when it was last advanced
        self.state = FREE
        self.scale = 1.0
        self.hold_t0 = None
        self.clear_since = None
        self.last_t = None
        self.last_v = 0.0              # last output speed (for the stopping distance)
        self.tracks = []

    # ------------------------------------------------------------------ scan
    def update_scan(self, points, now, moving=None, pose=None):
        """points: (px, py) valid returns in drive_center, in scan order. now: seconds.
        moving: optional parallel bools from FreeSpaceDetector. pose: drive_center (x, y, yaw) in odom.
        Without moving + pose, only the static rules apply."""
        p = self.p
        gap = float("inf")
        for px, py in points:
            if px >= 0.0 and abs(py) <= p["half_w"]:
                gap = min(gap, px - p["nose_x"])
        self.gap, self.scan_t = gap, now
        if moving is None or pose is None:
            # TF dropout: keep the tracks coasting and dead-reckon the pose on our own output speed, so one missed
            # lookup does not throw away a pass in progress. Give up only after pose_timeout.
            if self.pose is None or now - self.pose_t > p["pose_timeout"]:
                self.tracks, self.pose = [], None
            else:
                x, y, yaw = self.pose
                d = self.last_v * max(0.0, now - self.pose_dr_t)
                self.pose, self.pose_dr_t = (x + d * math.cos(yaw), y + d * math.sin(yaw), yaw), now
            return
        self.pose, self.pose_t, self.pose_dr_t = pose, now, now
        self._update_tracks(self._moving_clusters(points, moving), now)

    def _moving_clusters(self, points, moving):
        """Split the scan into segments of neighbouring returns; keep segments with enough moving returns.
        Returns [(cx, cy, hx, hy, clipped)]: visible bounding box in drive_center. clipped = the box touches the
        edge of the lidar's view (px ~ laser_x), so its position is incomplete."""
        p = self.p
        segs, cur = [], []
        for i, (px, py) in enumerate(points):
            if cur and math.hypot(px - points[cur[-1]][0], py - points[cur[-1]][1]) > p["seg_gap"]:
                segs.append(cur)
                cur = []
            cur.append(i)
        if cur:
            segs.append(cur)
        clusters = []
        for seg in segs:
            mov = [i for i in seg if moving[i]]
            if len(mov) < p["dyn_min_points"]:
                continue
            first, last = points[seg[0]], points[seg[-1]]
            if math.hypot(first[0] - last[0], first[1] - last[1]) > 1.5:
                seg = mov                     # a long (wall-like) segment: only its moving part
            xs = [points[i][0] for i in seg]
            ys = [points[i][1] for i in seg]
            x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
            cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
            if not (0.0 <= cx <= p["track_ahead"] and abs(cy) <= p["track_side"]):
                continue
            clipped = x0 < p["laser_x"] + 0.05
            clusters.append((cx, cy, 0.5 * (x1 - x0), 0.5 * (y1 - y0), clipped))
        return clusters

    def _to_world(self, px, py):
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        return x + c * px - s * py, y + s * px + c * py

    def _to_robot(self, wx, wy, pose=None):
        x, y, yaw = pose or self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = wx - x, wy - y
        return c * dx + s * dy, -s * dx + c * dy

    def _update_tracks(self, clusters, now):
        p = self.p
        free = list(self.tracks)
        for cx, cy, hx, hy, clipped in clusters:
            wx, wy = self._to_world(cx, cy)
            best, best_d = None, p["assoc_dist"] + max(hx, hy)
            for tr in free:
                px, py = tr.predict(now)
                d = math.hypot(px - wx, py - wy)
                if d < best_d:
                    best, best_d = tr, d
            if best is not None:
                free.remove(best)
                if clipped:
                    # Cut off at the edge of the view: its sideways position is still measured, its extent
                    # along x is not, so keep x on the prediction.
                    ox, _ = self._to_robot(*best.predict(now))
                    best.add(*self._to_world(ox, cy), hx=best.hx, hy=hy, t=now, window=p["vel_window"])
                else:
                    best.add(wx, wy, hx, hy, now, p["vel_window"])
            else:
                self.tracks.append(_Track(wx, wy, hx, hy, now))
        keep = []
        for tr in self.tracks:
            ox, _ = self._to_robot(*tr.predict(now))
            in_view = ox - tr.hx > p["laser_x"] + 0.05
            unseen = now - tr.seen
            if unseen <= (p["lost_timeout"] if in_view else p["coast_timeout"]) and ox + tr.hx > -p["tail_x"] - 0.5:
                keep.append(tr)
        self.tracks = keep

    # ------------------------------------------------------------------ prediction
    def _clearance(self, ox, oy, ux, uy, hx, hy, plan):
        """Smallest gap over predict_s between the footprint (corridor width), driving straight with s(t) = plan(t),
        and the obstacle box (centre ox + ux*t, oy + uy*t, half sizes hx, hy). Negative = overlap depth."""
        p = self.p
        hw = p["half_w"]
        n = int(round(p["predict_s"] / p["predict_dt"]))
        best = float("inf")
        for k in range(n + 1):
            t = k * p["predict_dt"]
            s = plan(t)
            x, y = ox + ux * t, oy + uy * t
            gx = max(s - p["tail_x"] - (x + hx), (x - hx) - (s + p["nose_x"]))
            gy = abs(y) - hy - hw
            if gx > 0.0 and gy > 0.0:
                gap = math.hypot(gx, gy)
            else:
                gap = max(gx, gy)            # one axis separated: that gap; both overlapping: -(smaller depth)
            best = min(best, gap)
        return best

    def _stop_dist(self, v):
        p = self.p
        return v * p["stop_latency"] + v * v / (2.0 * p["stop_decel"])

    def _assess(self, v, now, holding=False):
        """Moving-obstacle decision for the commanded speed v: (hold_reason, action, pass_v).

        A plan is safe when its clearance exceeds margin = obs_pad + obstacle speed * time_margin.
        Side obstacle, first safe option wins:
          continue -> pass -> defer (continue defer_s longer, then stop; not while holding) -> stop
        and if none is safe, whichever of pass / stop keeps more clearance (pass = ESCAPE; ties pass). A pass in
        progress continues until the obstacle is behind the fork tips (or has crossed), as long as it keeps any
        clearance, so the robot does not slow down or stop with the forks still in its path."""
        p = self.p
        self.dyn_gap = float("inf")
        pose, tracks = self.pose, list(self.tracks)    # snapshot: the scan callback may replace them meanwhile
        if pose is None or not tracks:
            return None, None, 0.0
        _, _, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        v_cont = max(v, 0.0)
        v_pass = p["boost_v"] * self._front_target(p["boost_v"])
        v_now = self.last_v
        b = self._stop_dist(v_now)
        b_cont = self._stop_dist(v_cont)
        hold, side, votes = None, [], []
        self.pass_note = None
        for tr in tracks:
            if tr.span() < p["vel_min_span"]:
                continue
            ux, uy = c * tr.vx + s * tr.vy, -s * tr.vx + c * tr.vy
            speed = math.hypot(ux, uy)
            ox, oy = self._to_robot(*tr.predict(now), pose=pose)
            margin = p["obs_pad"] + max(speed, p["min_track_speed"]) * p["time_margin"]
            clear = lambda plan: self._clearance(ox, oy, ux, uy, tr.hx, tr.hy, plan)
            if tr.passing:
                # Committed: finish the pass physically, not as soon as TEB's speed is predicted to be enough.
                past = -p["tail_x"] - (ox + tr.hx)              # > 0: the obstacle is behind the fork tips
                crossed = oy * uy > 0.0 and abs(oy) - tr.hy > p["half_w"]   # on the far side, moving away
                if past > margin or crossed:
                    tr.passing = False
                    self.pass_note = "forks %.2f m past it" % past if past > margin else "it has crossed"
                elif v_pass > 0.0 and clear(lambda t: max(v_pass, v_cont) * t) > 0.0:
                    side.append(PASS)
                    votes.append((tr, PASS))
                    continue
            if speed < p["min_track_speed"]:
                continue
            kind = SIDE if abs(uy) >= abs(ux) else (HEAD_ON if ux < 0.0 else None)
            if kind is None:
                continue
            if clear(lambda t: v_cont * t) > margin:
                continue                                   # continuing at TEB's speed is fine
            if kind == HEAD_ON:
                ahead = ox - tr.hx - p["nose_x"]
                self.dyn_gap = min(self.dyn_gap, ahead)
                if ahead <= p["yield_dist"]:
                    hold = hold or "moving"
                continue
            pass_c = clear(lambda t: v_pass * t) if v_pass > v_cont else -float("inf")
            if pass_c > margin:
                side.append(PASS)
                votes.append((tr, PASS))
                continue
            if not holding and clear(lambda t: min(v_cont * t, v_cont * p["defer_s"] + b_cont)) > margin:
                continue                                   # it can still wait: stopping later is just as safe
            stop_c = clear(lambda t: min(v_now * t, b))
            if stop_c > margin or stop_c > pass_c:
                side.append("crossing")
            else:
                side.append(ESCAPE)                        # ties go to driving through, not standing in its path
                votes.append((tr, ESCAPE))
        if hold:
            result = hold, None, 0.0
        elif "crossing" in side:
            result = "crossing", None, 0.0
        elif side:
            result = None, (ESCAPE if ESCAPE in side else PASS), v_pass
        else:
            result = None, None, 0.0
        passing_now = {id(tr) for tr, _ in votes} if result[1] else set()
        for tr in tracks:
            tr.passing = id(tr) in passing_now
        return result

    # ------------------------------------------------------------------ step
    def _front_target(self, v):
        p = self.p
        d_slow = p["d_stop"] + max(v, 0.05) * p["horizon_s"]
        return max(0.0, min(1.0, (self.gap - p["d_stop"]) / (d_slow - p["d_stop"])))

    def step(self, v, w, now):
        """Return (v_out, w_out) for the commanded (v, w) at time `now`."""
        v_out, w_out = self._gate(v, w, now)
        self.last_v = v_out
        return v_out, w_out

    def _gate(self, v, w, now):
        p = self.p
        dt = 0.05 if self.last_t is None else min(max(now - self.last_t, 1e-3), 0.2)
        self.last_t = now
        blind = self.scan_t is None or now - self.scan_t > p["scan_timeout"]
        target = 0.0 if blind else self._front_target(max(v, 0.0))
        dyn_hold, action, v_pass = (None, None, 0.0) if blind else self._assess(v, now, self.state != FREE)
        self.action = action if v > 0.0 else None     # a pass only speeds up a forward command, never a stop
        corridor_clear = (not blind) and self.gap > p["d_stop"] + p["clear_margin"] and dyn_hold is None

        if self.state == FREE:
            reason = dyn_hold
            if reason is None and v > 0.0 and not blind and target <= 0.0:
                reason = "blocked"
            if reason:
                self.state, self.hold_t0, self.clear_since, self.reason = HOLD, now, None, reason
        elif self.state in (HOLD, GIVEUP):
            if self.action == ESCAPE and v_pass > 0.0:
                self.state, self.clear_since = FREE, None      # standing still would get us hit
            else:
                if corridor_clear:
                    if self.clear_since is None:
                        self.clear_since = now
                else:
                    self.clear_since = None
                if self.state == HOLD and now - self.hold_t0 > p["hold_timeout"]:
                    self.state = GIVEUP
                if self.clear_since is not None and now - self.clear_since >= p["clear_time"]:
                    self.state, self.clear_since = FREE, None

        if self.state == HOLD:
            self.scale = 0.0
            return 0.0, 0.0

        if self.action in (PASS, ESCAPE) and v_pass > 0.0:
            self.scale = 1.0
            return max(v, v_pass), max(-p["pass_max_w"], min(p["pass_max_w"], w))

        if v <= 0.0:
            return v, w                              # rotation / reversing: not gated

        if self.state == GIVEUP:
            target = 0.0 if not corridor_clear else target

        if target < self.scale:
            self.scale = target                      # brake immediately
        else:
            self.scale = min(target, self.scale + p["release_rate"] * dt)
        return v * self.scale, w
