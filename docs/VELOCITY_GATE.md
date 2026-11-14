# Velocity Gate — algorithm and log reference

Code: `catkin_ws/src/amr_navigation/scripts/velocity_gate.py` (ROS node) and
`catkin_ws/src/amr_navigation/src/amr_navigation/gate_logic.py` (pure logic, tested by `tests/test_gate_logic.py` and
`tests/test_gate_rates.py`). Companion node for costmap trails: `scripts/scan_clearing.py` (see [§8](#8-scan_clearing-companion-node)).
Every parameter, with units and tuning recipes: [`TUNING.md`](TUNING.md).

> **Current design.** Dual shield (narrow inner stop rectangle + half-ellipse outer slow-down), optional flank shield (off),
> crossing yield with odom-compensated tracking, spin-first rotation toward the plan, TEB band monitor (S3b), and GIVEUP with a
> global-costmap blocker. The older predictive machinery (`FreeSpaceDetector`, head-on / PASS / ESCAPE plans) is retired and
> no longer in the code.

---

## 1. What it is

The velocity gate is a safety filter between move_base and the robot:

```
move_base (TEB) ──/cmd_vel_nav──▶ velocity_gate ──/cmd_vel──▶ Unity robot
                                      ▲   ▲   ▲
                     /scan ───────────┘   │   └── /odom (measured v, w), Navfn plan, TEB local_plan
                                          └────── /velocity_gate/shields (RViz), /velocity_gate/state, /velocity_gate/blocker
```

TEB decides *where* to go. The gate only decides *how fast* that command may be executed, given what the lidar sees right now.

| Situation | Gate reaction |
|---|---|
| Static obstacle inside the outer half-ellipse | Slow down in proportion to the distance (never stops by itself) |
| Obstacle inside the inner rectangle (<= 0.20 m from the nose) | **HOLD [blocked]**: `v = 0`, `w = 0` |
| Compact obstacle moving sideways into the corridor ahead | **HOLD [crossing]**: `v = 0`, `w = 0`, whatever TEB asks |
| Robot faces more than 60 deg away from the plan | **Spin-first**: rotate in place toward the plan |
| HOLD lasts longer than `hold_timeout` (10 s) | **GIVEUP**: rotation allowed, forward blocked, blocker line sent to the global costmap |
| TEB's trajectory is bent after an override | **S3b reinit**: TEB rebuilds its band from the Navfn plan |
| Scan older than `scan_timeout` (0.5 s) | gate is blind, treated as blocked (HOLD [blocked]) |

Important properties:

- It only publishes while move_base is actively commanding (a `/cmd_vel_nav` message within `cmd_timeout`, 0.5 s). When
  move_base goes idle and the last output was not zero, it publishes **one zero Twist**, then stays silent, so the
  robot is never left driving on a stale command. Docking/lift scripts publish `/cmd_vel` directly and are never touched by the gate.
- The scan callback and the 20 Hz timer share the logic under a lock. If the timer step raises, the error is
  logged and a zero Twist is published; the timer keeps running.
- During HOLD the output is a fully zero Twist (except the optional HOLD [blocked] rotation, §3). TEB keeps planning and sending
  swerve commands, but none of them reach the robot. That is what stops "stopped but weaving left/right".
- Decisions are counted in scans (`cross_confirm` = 3 scans = 0.3 s at 10 Hz) or in seconds, never in odom samples, so the odom
  rate (20 or 50 Hz) and `step()` jitter do not change when a HOLD latches (`tests/test_gate_rates.py`).
- The lidar is a 220 deg arc (361 rays, 10 Hz) mounted 0.33 m ahead of `drive_center`. Nothing behind or directly beside the
  laser is visible; a crosser that leaves the arc is simply no longer seen.

---

## 2. Geometry (drive_center frame, x forward, y left)

Everything is measured from the footprint nose (`nose_x` = 0.37 m). RViz draws it from the same parameters
(`publish_shields`, topic `/velocity_gate/shields`), so the drawing is the definition.

```
                          outer shield: half-ellipse centred on the nose
                          forward semi-axis d_outer = 1.00 m, lateral semi-axis outer_half_w = 0.50 m
   y                         .--------------------------.
   ^                   .--'                                '--.
 +0.50                .'                                       '.
 +0.26 ---+-------+---+                                           \
   |      | robot |inner (HOLD)                                    |
   0 -----+-------+---+  depth 0.20 m                              |--> x
   |      |       |   |                                           /
 -0.26 ---+-------+---+                                          .'
 -0.50                '.                                       .'
                         '--.                              .--'
                             '--------------------------'
        tail -0.54       nose 0.37
   crossing watch zone (thin outline): x in [nose, nose + 1.50 m], |y| <= 1.50 m
```

| Shield | Shape | Effect |
|---|---|---|
| **Inner** | rectangle `\|y\| <= inner_half_w` (0.26 = chassis 0.23 + 0.03), depth `d_inner` / `d_hold_static` (0.20) | a point inside -> **HOLD [blocked]**, `v = w = 0` |
| **Outer** | half-ellipse centred on the nose; normalised distance `e = d_outer * sqrt((dx/d_outer)^2 + (y/outer_half_w)^2)` | `e <= d_outer`: speed scale ramps from `scale_min` (at `d_inner`) to 1.0 (at the rim). Slow only, never HOLDs. An aisle wall 0.42 m abeam costs about 15 % speed |
| **Flank** | boxes beside the chassis, `flank_enable` (default **off**) | HOLD [flank] when enabled. Side clearance is TEB's job (footprint + `min_obstacle_dist`) |
| **Crossing zone** | rectangle ahead, depth `cross_x_max` (1.5 m, own parameter, not tied to `d_outer`), half-width `cross_zone_w` (1.5 m); threats only count within `cross_check_dist` (0.9 m) of the nose centre | compact clusters moving laterally into the corridor -> HOLD [crossing]. RViz: cyan outline + fill while watching, magenta while yielding; thin arc = check distance |

**Crossing detection.** Clusters (gap 0.20 m, >= 3 points, <= 0.80 m long, not touching the FOV edge) are tracked in a
gate-local odom frame integrated from the measured `/odom` twist (falls back to the commanded `v`/`w` if odom is older than
`odom_timeout`). A track's lateral velocity is measured over `cross_v_window` (0.40 s, trusted after `cross_v_min_span`
0.25 s), so a 1.5 cm centroid shift does not read as 0.15 m/s. A threat is `|vy| >= cross_v_min` inside the corridor or
entering it within `cross_t_horizon` (1.8 s); it must persist `cross_confirm` scans. Every cluster in the zone is tracked, but
only one within `cross_check_dist` of the nose centre can be a threat (set it >= `cross_x_max + cross_zone_w` to disable).

*Known limit:* crossers wider than `cross_max_extent` (0.80 m, e.g. a wide cart) are not tracked as crossers; they fall
back to the inner/outer shield.

**Spin-first.** With a fresh Navfn plan (and more than `spin_min_remaining` of it left), a heading error to the plan point
`shim_lookahead` ahead above `spin_enter` (60 deg) makes the gate output `v = 0, w = +/- spin_w` until the error drops
below `spin_exit` (20 deg). In HOLD [blocked] the same rotation is allowed after `spin_hold_delay` (2 s); HOLD [crossing]
and [flank] never rotate. Log lines: `[gate] spin start|stop (heading error N deg)`, `(spin)` in the telemetry line.

**TEB band monitor.** TEB warm-starts its trajectory and does not look at the robot heading when deciding to rebuild it, so
a band bent during a gate override (spin, HOLD, GIVEUP) stays bent afterwards (a sideways loop or V cusps). The gate
subscribes to TEB's `local_plan`, converts it and the Navfn plan to the robot frame and calls `band_health`: bad if a local
pose is more than `reinit_bulge` (0.50 m) from the global plan, the local plan is more than `reinit_ratio` (1.6x) longer than
the global stretch it covers, or it has a cusp. Two bad plans in a row (`reinit_confirm`), or the end of an override that
lasted at least `reinit_min_override` (0.5 s), request a re-init: for `reinit_hold` (0.4 s) the gate sets
`force_reinit_new_goal_dist` to 0.0 on `/move_base/TebLocalPlannerROS` (dynamic_reconfigure; TEB then rebuilds from the Navfn
plan every cycle) and outputs zero, then restores the old value. Detector triggers wait `reinit_cooldown` (3 s) and are
ignored during an override and near the goal. Log lines: `[gate] teb reinit (<reason>)`, `[gate] teb reinit end`.
`reinit_enable: false` switches it off; without the reconfigure server the gate only warns.

Each scan, every valid return is converted to a point `(px, py)` in drive_center
(`px = laser_x + r*cos a`, `py = r*sin a`). Returns of `0`, `nan`, `inf` or `>= range_max` are dropped. The laser is
220 deg, 361 rays, 10 Hz.

---

## 3. Per-command decision — `GateLogic.step` (20 Hz)

```
              flank breach / crossing / blocked
   FREE ───────────────────────────────────────▶ HOLD ──── hold_timeout (10 s) ────▶ GIVEUP
     ▲                                             │                                    │
     └──────── corridor clear for clear_time ──────┴────────────────────────────────────┘
```

**Front speed target** (every step, before the state machine). With `gap` = nose-to-nearest-return distance inside the inner
rectangle and `e` = normalised outer-shield distance (§2):

| Condition | Target scale |
|---|---|
| scan stale, or `gap <= d_inner` | 0 |
| `e <= d_outer` | `scale_min + (1 - scale_min) * (e - d_inner) / (d_outer - d_inner)`, clamped to 0..1 |
| otherwise | 1 |

**FREE.** Braking is immediate; speeding up is limited to `release_rate` per second (about 1 s from 0 to full speed). While
moving forward the output never drops below `min_crawl_speed` (0.10 m/s, avoids stalling on PhysX friction), and `w` is scaled by
the same factor as `v`, so TEB's curvature is kept; a pure rotation (`v <= 0.02`) passes unchanged. With nothing in the outer
shield the gate is transparent. Leaving FREE, in this priority: flank breach, then crossing, then blocked (`target_scale <= 0`
or `gap <= d_hold_static`).

**HOLD.** Output `v = 0`. `w = 0` as well, with one exception: **HOLD [blocked]** may rotate toward the plan at `spin_w` after
`spin_hold_delay` (2 s), so the robot can turn out of a dead end; the rotation follows the plan heading, not TEB's `w`, so there
is no pivot-release-pivot loop. HOLD [crossing] and [flank] are a full freeze.

**Release.** HOLD or GIVEUP returns to FREE when, for `clear_time` (0.8 s) without interruption, the scan is fresh, `gap >
max(d_inner, d_hold_static) + clear_margin` (0.30 m), there is no flank breach and no crossing threat. Any interruption restarts
the 0.8 s.

**Spin-first (FREE).** A fresh Navfn plan with more than `spin_min_remaining` left and a heading error above `spin_enter` (60 deg)
makes the gate output `v = 0`, `w = +/- spin_w` until the error drops below `spin_exit` (20 deg). Details in §2.

**S3b TEB band monitor.** Any override (HOLD, spin, GIVEUP) bends TEB's warm-started band while the wheels are held. After an
override of at least `reinit_min_override` (0.5 s), or after `reinit_confirm` bad local plans, the gate sets TEB's
`force_reinit_new_goal_dist` to 0.0 for `reinit_hold` (0.4 s) and outputs zero. Details in §2.

---

## 4. GIVEUP and the timeout rule

While the gate holds, move_base sees no progress. If the hold outlasted move_base's patience or oscillation timeout it would abort
the goal, so after `hold_timeout` the gate stops holding:

- **Forward stays blocked** (`v = 0`) until the corridor is clear; rotation passes through again so TEB can find another way.
  Angular output is limited to `giveup_w_max` (0.30 rad/s). With a fresh plan the gate rotates toward it until the heading error
  is below `shim_tol` (0.15 rad), then outputs `w = 0`; without a plan it passes TEB's `w`, clamped.
- On entering GIVEUP the node publishes a blocker line (`blocker_half_width` 0.60 m, spacing 0.05 m) across the passage on
  `/velocity_gate/blocker` for `blocker_ttl` (60 s). The global costmap's `blocker_layer` marks it, so Navfn plans around
  whatever blocks the aisle. When the blocker expires the costmaps are cleared (`[gate] blocker expired, costmaps cleared`).
- GIVEUP ends (`-> free`) when the corridor has been clear for `clear_time`.

**Timeout rule:** `gate_hold_timeout` < `nav_patience` (controller and planner patience) < `oscillation_timeout`.
Defaults: 10 s < 30 s < 60 s, set in `warehouse_navigation.launch`. If you raise `gate_hold_timeout`, raise the other two with it.

---

## 5. Typical scenarios

| Scenario | What happens |
|---|---|
| Person walks across the aisle ahead, inside 0.9 m of the nose | About 0.25 s of track baseline plus 3 confirming scans, then `free -> hold [crossing]`, completely still. After they leave and the corridor stays clear for 0.8 s: `hold -> free`, ramping back up |
| Person walks along the aisle, away from the robot | lateral speed below `cross_v_min`: nothing |
| Person stands still in the aisle ahead | Slows inside the outer ellipse, `hold [blocked]` at 0.20 m; after 2 s the gate may rotate toward the plan; after 10 s `giveup`, blocker line, TEB goes around |
| Aisle wall 0.42 m abeam | Outer shield slows the robot by about 15 %; no HOLD |
| Rack beside the robot while it turns or drives | Not a crosser: tracks are compensated with measured odom, and detection is skipped while `\|w\| > cross_w_max` |
| Cluster longer than 0.80 m (wall, rack face, wide cart) | Not tracked as a crosser; handled by the shields only |
| Robot ends a goal facing away from the next plan | `spin start`, rotates in place, `spin stop` |
| TEB band bent after a HOLD or spin | `teb reinit (override end)` or `(bulge ...)` / `(detour ...)` / `(cusp)` |
| Lidar stops or lags | After 0.5 s the gate counts as blind: `hold [blocked]` until scans resume and the corridor is clear for 0.8 s |

---

## 6. Log reference

All gate logs are prefixed `[gate]` and come from `velocity_gate` (`output="screen"` in `warehouse_navigation.launch`). The current
state is also published, latched, on `/velocity_gate/state` (`free` / `hold` / `giveup`).

### 6.1 State change — INFO, once per change

```
[gate] <old> -> <new>[ [reason]][ (spin)][ | <trigger>] (fwd_gap: <G>m)
```

| Field | Meaning |
|---|---|
| `<old>` | previous state; `None` on the first command after the node starts |
| `<new>` | `free`, `hold` or `giveup` |
| `[reason]` | only on transitions into `hold`: `[blocked]`, `[crossing]` or `[flank]` |
| `<trigger>` | what caused it: `clear`, `blocked ahead at gap ...`, `slowdown ahead at outer distance ...`, `yield: crossing obstacle at (x, y) vy=...`, `flank breach at ...`, `spin toward plan ...` |
| `fwd_gap` | nose to nearest return in the inner rectangle; `infm` = nothing there |

| Log line | Meaning |
|---|---|
| `[gate] None -> free \| clear (fwd_gap: infm)` | first command received; nothing ahead |
| `[gate] free -> hold [blocked] \| blocked ahead at gap 0.19m (fwd_gap: 0.19m)` | something reached the inner rectangle |
| `[gate] free -> hold [crossing] \| yield: crossing obstacle at (0.80, 0.45) vy=-0.50 (fwd_gap: infm)` | lateral mover entering the corridor; `(x, y)` is in `drive_center`, `vy` in m/s |
| `[gate] hold -> free \| clear (...)` | clear for 0.8 s; ramping back up |
| `[gate] hold -> giveup \| ...` | held longer than `hold_timeout`; rotation allowed, forward blocked |
| `[gate] giveup -> free \| ...` | corridor finally clear for 0.8 s |

States persist while move_base is idle: after one zero Twist the gate goes silent but does not reset.

### 6.2 Telemetry — INFO, at most once per second (when the state did not change)

```
[gate] gap: 0.60m | outer: 0.74m | state: free | scale: 0.48 | v_out: 0.17m/s, w_out: 0.00rad/s
```

`gap` is the inner-rectangle distance, `outer` the normalised outer-shield distance, `scale` the fraction of TEB's speed sent.
`(spin)` follows the state while spinning. `scale` rising from a low value with nothing close is the ramp-up after a hold.

### 6.3 Spin, reinit and blocker lines

| Log | Meaning |
|---|---|
| `[gate] spin start (heading error 94 deg)` / `spin stop (...)` | spin-first began / ended; `n/a` means no fresh plan |
| `[gate] teb reinit (override end)` | an override of >= 0.5 s ended; TEB is told to rebuild its band |
| `[gate] teb reinit (bulge 0.62m)` / `(detour x1.9)` / `(cusp)` | the band monitor found a bad local plan (twice in a row) |
| `[gate] teb reinit end` | reinit window over, TEB's old setting restored |
| `[gate] blocker set at (x, y) for 60 s` | GIVEUP put a blocker line in the global costmap |
| `[gate] blocker expired, costmaps cleared` | blocker removed, costmaps cleared |

### 6.4 Warnings and errors

| Log | Meaning / action |
|---|---|
| `[gate] TEB reinit unavailable (<ns>): <error>` (WARN, every 10 s) | the TEB dynamic_reconfigure server is not there yet; the band monitor only warns. Check move_base is running, or set `reinit_enable: false` |
| `[gate] TEB reinit reconfigure failed: <error>` (WARN) | the reconfigure call failed; TEB keeps its previous setting |
| `[gate] blocker skipped, tf failed: <error>` (WARN) | no TF for the blocker pose; GIVEUP still rotates, but the global costmap is not told |
| `[gate] clear_costmaps failed: <error>` (WARN) | the costmap clear service call failed after the blocker expired |
| `[gate] error, publishing zero: <exception>` (ERROR, every 1 s) | a bug or bad input raised inside the timer step. Zero is sent every tick while it persists; report the exception |

### 6.5 Silent cases worth knowing

| Symptom | Likely cause |
|---|---|
| No gate output at all | move_base isn't publishing `/cmd_vel_nav` (idle, or the goal was aborted) |
| Robot stuck in `hold [blocked]` with nothing visible | `/scan` older than 0.5 s; check `rostopic hz /scan` |
| Robot yields to a rack or wall | crossing false positive; see the recipes in `TUNING.md` (`cross_check_dist`, `cross_v_min`, `cross_max_extent`) |
| Robot does not yield to a person | person is outside `cross_check_dist`, slower than `cross_v_min`, wider than `cross_max_extent`, or not seen (lidar layer / height); see `TROUBLESHOOTING.md` |

---

## 7. RViz marker legend (`/velocity_gate/shields`, namespace `shields`)

The drawing is generated from the same parameters as the logic. Colours are RGB(A) as published.

| Id | Type | What it shows | Colours |
|---|---|---|---|
| 0 | text billboard | state summary above the robot | green `CLEAR`, amber `SLOW (n%)`, red `HOLD [...]` / `YIELD (crossing)`, cyan `SPIN`, purple `GIVEUP / REROUTE` |
| 1 | cube | inner shield rectangle (`d_inner` deep) | bright red = blocked, magenta = crossing hold, faint red otherwise |
| 2 | triangle fan | outer half-ellipse fill | orange = slowing, dark amber = HOLD, yellow = idle |
| 3, 4 | cubes | flank boxes, only when `flank_enable` | magenta = breach, blue otherwise; removed when disabled |
| 5 | - | **unused on purpose** (renumbering would leave stale markers in saved RViz sessions) | - |
| 6 | sphere | the crossing object, only while HOLD [crossing] | magenta |
| 7 | line strip | crossing watch zone outline (`cross_x_max` x `cross_zone_w`) | cyan = watching, magenta = yielding |
| 8 | line strip | outer ellipse rim | same colour as marker 2 |
| 9 | flat cube | translucent fill of the watch zone | same colours as marker 7 |
| 10 | line strip | check arc: threats only count inside `cross_check_dist` of the nose centre | same colours as marker 7; removed when the radius is beyond the zone |

---

## 8. `scan_clearing` (companion node)

Separate from the gate, but part of the same dynamic-obstacle fix. costmap_2d erases an obstacle cell only when a
laser ray passes through it. That leaves trails in two places: between the 1° rays (more than one 5 cm cell apart
beyond about 3 m), and in the blind rear half of the lidar. `scan_clearing.py` publishes `/scan_clearing`, a
360° scan in the laser frame. Both costmaps use it as the `laser_clearing` source with `marking: false`, so it can
**only erase**:

- **Front:** 4× denser rays. Each inserted ray reaches only as far as the nearer of its two neighbours. A
  `nan` neighbour means "clear nothing here".
- **Rear blind arc:** rays cast through the static `/map` from the laser pose (TF `map ← laser`), stopping
  `map_margin` (0.25 m) before any occupied, unknown or off-map cell, capped at `blind_range` (4 m). Racks
  stay; trails beside and behind the robot are erased.

| Log | Meaning |
|---|---|
| `[scan_clearing] no map pose, blind arc not cleared: <tf error>` (WARN, every 10 s) | no AMCL `map → odom` yet; front densification still works |
| `[scan_clearing] rotated map origin not supported; blind arc clearing off` | the map yaml has a non-zero origin yaw |

Trade-off: objects that are **not on the static map** (a pallet left in the aisle) are forgotten once they are
beside or behind the robot. If AMCL is off and rack marks next to the robot get erased, raise `~map_margin`.

---

## 9. Parameters

Gate parameters are set on the node as `~<name>`. Logic parameters come from `gate_logic.DEFAULTS`.
Full list with units, side effects and tuning recipes: [`TUNING.md`](TUNING.md).

| Param | Default | Effect |
|---|---|---|
| `inner_half_w` | 0.26 m | inner shield half-width |
| `d_inner`, `d_hold_static` | 0.20, 0.20 m | inner shield depth / HOLD distance from the nose |
| `outer_half_w`, `d_outer` | 0.50, 1.00 m | outer half-ellipse semi-axes |
| `scale_min`, `min_crawl_speed` | 0.25, 0.10 m/s | slow-down floor at the inner depth, minimum forward speed |
| `half_w` (`~half_width` + `~side_margin`) | 0.33 m | crossing-corridor half-width only |
| `flank_enable`, `flank_x_min`, `flank_w` | false, -0.20, 0.35 | optional flank shield |
| `cross_x_max`, `cross_zone_w`, `cross_check_dist` | 1.50, 1.50, 0.90 m | crossing watch zone and threat check radius |
| `cross_v_window`, `cross_v_min_span`, `cross_v_min` | 0.40 s, 0.25 s, 0.15 m/s | lateral velocity baseline / threshold |
| `cross_confirm`, `cross_t_horizon`, `cross_assoc_gap`, `cross_max_extent` | 3, 1.8 s, 0.35 m, 0.80 m | crossing confirmation and tracking |
| `cross_w_max` | 0.45 rad/s | skip crossing detection while turning faster (= max_vel_theta, effectively off) |
| `odom_timeout` | 0.30 s | measured odom older than this -> commanded v/w |
| `spin_enable`, `spin_enter`, `spin_exit`, `spin_w` | true, 1.05, 0.35 rad, 0.45 rad/s | spin-first hysteresis and speed |
| `spin_hold_delay`, `spin_min_remaining` | 2.0 s, 0.50 m | HOLD [blocked] rotation delay; no spin near the goal |
| `reinit_enable`, `reinit_bulge`, `reinit_ratio`, `reinit_cusp_seg` | true, 0.50 m, 1.60, 0.03 m | TEB band monitor: bad-band tests |
| `reinit_confirm`, `reinit_cooldown`, `reinit_hold`, `reinit_min_override` | 2, 3.0 s, 0.40 s, 0.50 s | reinit trigger and window |
| `~teb_ns`, `~local_plan_topic` | /move_base/TebLocalPlannerROS, `<teb_ns>/local_plan` | node wiring for the band monitor |
| `release_rate`, `clear_margin`, `clear_time` | 1.0 /s, 0.10 m, 0.80 s | speed-up ramp, release condition |
| `scan_timeout` | 0.5 s | older scan = blind |
| `hold_timeout` | 10 s (launch arg `gate_hold_timeout`) | HOLD -> GIVEUP |
| `giveup_w_max`, `shim_lookahead`, `shim_tol`, `shim_plan_age` | 0.30 rad/s, 0.60 m, 0.15 rad, 3 s | GIVEUP rotation shim |
| `blocker_half_width`, `blocker_ttl`, `blocker_spacing` | 0.60 m, 60 s, 0.05 m | GIVEUP costmap blocker line |
| `~laser_x`, `~base_frame`, `~cmd_timeout`, `~odom_topic`, `~plan_topic` | 0.33, drive_center, 0.5 s, /odom, Navfn plan | node wiring |
| `scan_clearing ~densify`, `~blind_range`, `~map_margin`, `~map_frame` | 4, 4.0 m, 0.25 m, `map` | clearing scan |
