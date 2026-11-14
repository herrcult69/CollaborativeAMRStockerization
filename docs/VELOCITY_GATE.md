# Velocity Gate — algorithm and log reference

Code: `catkin_ws/src/amr_navigation/scripts/velocity_gate.py` (ROS node) and
`catkin_ws/src/amr_navigation/src/amr_navigation/gate_logic.py` (pure logic, tested by `tests/test_velocity_gate.py`).
Companion node for costmap trails: `scripts/scan_clearing.py` (see [§8](#8-scan_clearing-companion-node)).

> **Status (current design).** The live gate is the dual-shield + spin-first design described in **section 2
> (geometry)** and **section 9 (parameters)**: narrow inner stop rectangle, half-ellipse outer slow-down, optional flank
> shield (off), crossing yield with odom-compensated tracking, and spin-first rotation toward the plan. The predictive
> moving-obstacle machinery in sections 1 and 3-7 (`FreeSpaceDetector`, head-on / PASS / ESCAPE plans) was retired and is
> kept only as history. Tests: `tests/test_gate_logic.py`.

---

## 1. What it is

The velocity gate is a safety filter between move_base and the robot:

```
move_base (TEB) ──/cmd_vel_nav──▶ velocity_gate ──/cmd_vel──▶ Unity robot
                                      ▲
                     /scan ───────────┤   (+ TF odom ← laser_link, for moving-obstacle detection)
```

TEB decides *where* to go. The gate only decides *how fast* that command may be executed, given what the lidar
sees right now. It reacts as follows:

| Situation | Gate reaction |
|---|---|
| Static obstacle ahead in the robot's path | Slow down in proportion to the gap, stop 0.30 m from the nose, then **HOLD** |
| **Head-on** moving obstacle on a collision course, within 1.5 m | **HOLD** immediately: `v = 0` **and** `w = 0`, whatever TEB asks |
| **Side** (crossing) obstacle that keeping TEB's speed would avoid | nothing, keep going |
| Side obstacle that would hit us, and passing at 0.32 m/s avoids it | **PASS**: drive at 0.32 m/s, even if TEB slowed down |
| Side obstacle we can't pass, and stopping lets it cross in front | **HOLD** (`crossing`), but only once waiting any longer would be unsafe |
| Side obstacle where stopping would leave us in its path | **ESCAPE**: pass anyway (also ends a HOLD) |

The key distinction: **what kind of motion** (head-on vs side), and for side obstacles **which robot plan is
predicted to be safe**. Head-on means yield. Side means continue, then pass, then stop, in that order. Stop is
chosen only when it really keeps the robot out of the obstacle's path.

Important properties:

- It only publishes while move_base is actively commanding (a `/cmd_vel_nav` message in the last 0.3 s). When
  move_base goes idle and the last output was not zero, it publishes **one zero Twist**, then stays silent, so the
  robot is never left driving on a stale command.
- The scan callback and the 20 Hz timer share the logic under a lock. If the timer step raises, the error is
  logged and a zero Twist is published; the timer keeps running.
  Docking/lift scripts publish `/cmd_vel` directly and are never touched by the gate.
- During HOLD the output is a fully zero Twist. TEB keeps planning and keeps sending swerve commands, but
  none of them reach the robot. That is what stops the "stopped but weaving left/right" behaviour.
- The lidar is a 180° arc mounted 0.33 m ahead of `drive_center`. Nothing beside or behind the laser is visible.
  Moving obstacles that go out of view beside the robot are carried on their last velocity (coasting, §4).

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
| **Crossing zone** | rectangle ahead, depth `cross_x_max` (own parameter, not tied to `d_outer`) | compact clusters moving laterally into the corridor -> HOLD [crossing] |

**Crossing detection.** Clusters (gap 0.20 m, >= 3 points, <= 0.80 m long, not touching the FOV edge) are tracked in a
gate-local odom frame integrated from the measured `/odom` twist (falls back to the commanded `v`/`w` if odom is older than
`odom_timeout`). A track's lateral velocity is measured over `cross_v_window` (0.40 s, trusted after `cross_v_min_span`
0.25 s), so a 1.5 cm centroid shift does not read as 0.15 m/s. A threat is `|vy| >= cross_v_min` inside the corridor or
entering it within `cross_t_horizon`; it must persist `cross_confirm` scans.

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

## 3. Moving-obstacle detection — `FreeSpaceDetector`

The question is "is this return something that moved here?". The answer is a **free-space violation test**:

1. Keep the last 1.5 s of scans together with the laser pose in `odom` at each scan's timestamp (from TF).
2. For the current scan, pick a reference scan at least `moving_ref_age` = 0.4 s old.
3. For each current return:
   - Convert it to a world (odom) point.
   - Look at it from the reference scan's laser pose: angle and distance `ρ`.
   - Find the reference ray at that angle and its two neighbours, and take the **smallest** free distance among
     the three (an `inf` reference ray counts as free up to `range_max`).
   - If that free distance is `> ρ + moving_tol` (0.12 m), then 0.4 s ago the laser saw **through** this spot.
     Something is there now that wasn't, so the return is flagged **moving**.

Why this works:

| Case | Reference ray at that spot | Flagged? |
|---|---|---|
| Static wall, robot driving or turning | hits the same spot (world frame cancels own motion) | no |
| Obstacle coming toward the robot | used to pass through to the obstacle's old, farther position | **yes** |
| Obstacle crossing the path | used to pass through to the wall behind | **yes** (leading edge) |
| Obstacle moving away | the spot was hidden behind the obstacle itself | no |
| Static surface newly revealed (passing a rack end) | the spot was hidden | no |

Taking the minimum over three neighbouring reference rays stops a point right next to a static edge from being
mistaken for motion. Sensitivity: anything faster than about `0.12 m / 0.4 s ≈ 0.3 m/s` is detected.

If TF `odom ← laser_link` is missing for a scan, that scan only updates the static gap. Existing tracks keep
coasting, and the pose is dead-reckoned on the gate's own output speed, so a single missed lookup doesn't abandon a
pass. After `pose_timeout` (1.0 s) without a pose, all tracks are dropped and the gate is a distance-only gate
(static rules only). It logs a warning ([§7](#7-log-reference)).

---

## 4. Tracking moving obstacles — `GateLogic.update_scan`

Each scan (needs the moving flags **and** the drive_center pose in odom):

- **gap** = min `(px − nose_x)` over points with `|py| ≤ half_w`. This includes moving points, and drives the static rules.
- **Clusters:** consecutive returns (scan order) closer than `seg_gap` = 0.25 m form a segment. A segment with
  ≥ 2 moving returns is a moving obstacle. Its **visible bounding box** (centre, half sizes) is kept, and must lie
  within 5 m ahead and 3.5 m to the side. For long (wall-like) segments, only the moving part is used.
- **Tracks** live in odom. Each cluster is matched to the nearest predicted track within
  `assoc_dist` (0.6 m) + its size, otherwise it starts a new track.
  - **Velocity** is the **median** of the scan-to-scan steps of the box centre over the last 0.6 s. A one-off jump
    (another face of the obstacle coming into view) therefore doesn't corrupt it.
  - A box cut off at the edge of the lidar's view (`px ≈ laser_x`) only updates the track's **sideways**
    position; its x position is kept on the prediction.
  - **Coasting:** a track that should be visible but isn't seen for 0.5 s is dropped. A track predicted to be
    beside or behind the laser (out of view) keeps coasting at its last velocity for up to 4 s. So a pass isn't
    abandoned when the obstacle disappears beside the robot.
- A track is used for decisions once it has ≥ 0.25 s of history and moves ≥ 0.15 m/s.
- **Classification** by its own velocity `(ux, uy)` in the robot frame:
  - `|uy| ≥ |ux|`: **SIDE** (crossing the path)
  - `ux < 0`: **HEAD-ON** (coming toward the robot)
  - otherwise (moving away): ignored, the static rules handle it

## 5. Per-command decision — `GateLogic.step` (20 Hz)

Inputs: TEB's `(v, w)`. Output: `(v_out, w_out)`.

### 5.1 Front speed target (static obstacles)

```
d_slow = d_stop + max(v, 0.05) · horizon_s           (v = 0.25 → 0.30 + 0.625 = 0.925 m)
target = clamp((gap − d_stop) / (d_slow − d_stop), 0, 1)
```

Example at v = 0.25: gap 0.925 m or more → 1.0 (full speed); gap 0.60 → 0.48; gap ≤ 0.30 → 0.

If the scan is older than 0.5 s (**blind**), `target = 0`: forward motion is blocked, but there is no HOLD.

### 5.2 Moving obstacles — predict three plans

For each usable track, the robot is predicted to drive straight for 7 s (`predict_s`) under three plans:

| Plan | Robot travel `s(t)` |
|---|---|
| **continue** | `v · t` (TEB's speed) |
| **pass** | `v_pass · t`, with `v_pass = 0.32 · front_target(0.32)` (no passing into a static obstacle) |
| **stop** | `min(v_now · t, b)`, with `b = v_now·0.3 + v_now² / (2·0.5)` (latency + braking) |

The obstacle box moves at its velocity. The **clearance** of a plan is the smallest gap between the footprint
(corridor width) and the box along the whole horizon; a negative value is the overlap depth. A plan is **safe**
when

```
clearance > margin = obs_pad (0.1 m) + obstacle speed · time_margin (0.3 s)
```

**Decision**, only for tracks where *continue* is not safe:

```
HEAD-ON:  within yield_dist (1.5 m) ahead of the nose     → HOLD "moving"
          further away                                    → nothing yet (static rules slow us)

SIDE:     0. already passing it: until it is behind the fork tips by the margin
             (or has crossed and moves away), keep PASS while pass clearance > 0
          1. pass safe                                               → PASS
          2. continue 0.5 s more, then stop, is still safe         → nothing yet (defer the stop)
          3. stop safe                                             → HOLD "crossing"
          4. none safe: stop clearance > pass clearance            → HOLD "crossing"
                        otherwise (ties too)                       → ESCAPE (pass anyway)
```

The ingredients that fix "stops and gets hit":

- **Pass is absolute** (only while TEB commands forward, `v > 0`). The gate drives at `max(TEB v, v_pass)`, with
  `|w|` clipped to 0.2 rad/s. A zero (goal reached or cancelled), rotate-in-place or reverse command is never turned
  into a pass
  (drive through, don't swerve). TEB slowing down for the obstacle can't stall the robot in its path.
- **Commitment is physical.** Once passing an obstacle, the robot keeps passing until that obstacle is behind the
  fork tips (x = −0.54) by the margin, or has crossed the path and is moving away. It doesn't stop when TEB's speed
  is merely *predicted* to be enough, so the chassis and the forks both get past. It never stops halfway, as long
  as passing doesn't collide.
- **Stop must be earned.** A HOLD for a side obstacle needs a stop plan that keeps clearance. If stopping would leave
  the robot in the path, it escapes instead, and an ESCAPE also ends a HOLD that is already running.
- **No premature stop.** It only yields when stopping 0.5 s later would no longer be safe.
- **Side obstacles never trigger the head-on rule**, even when their edge enters the corridor during a pass.

Example (closed-loop sim, `tests/gate_sim.py`): a box crossing at world x = 1.2 m, starting 3.5 m to the side at
0.4 m/s, robot at 0.25 m/s. Keeping 0.25 m/s collides. The gate predicts the pass plan to be safe, drives 0.32 m/s,
and clears the box without stopping.

### 5.3 State machine

```
                     moving obstacle ≤ yield_dist           ─┐
          ┌───────── or forward command with target ≤ 0      ├─▶ reason: moving / blocked / crossing
          │          or side conflict must-yield           ─┘
          ▼
  ┌──────────┐                       ┌──────────┐   held > hold_timeout (20 s)   ┌──────────┐
  │   FREE   │──────────────────────▶│   HOLD   │───────────────────────────────▶│  GIVEUP  │
  └──────────┘                       └──────────┘                                └──────────┘
       ▲   ▲   corridor clear for clear_time (1 s)      │                              │
       │   └────────────────────────────────────────────┘                              │
       └──────────────── corridor clear for clear_time (1 s) ──────────────────────────┘
```

**Entering HOLD** (only from FREE):

| Reason | Condition | Note |
|---|---|---|
| `moving` | head-on track on a collision course within `yield_dist` (1.5 m) | any command, including rotate-in-place |
| `crossing` | side track, the decision says stop | |
| `blocked` | `v > 0`, not blind, `target ≤ 0` (gap ≤ 0.30 m) | static; only when TEB drives forward into it |

**"Corridor clear"** means all of the following:

- the scan is fresh;
- `gap > d_stop + clear_margin` (0.45 m);
- no moving-obstacle hold reason, evaluated without deferring (while holding, *continue* itself must be safe).

The clear timer restarts whenever any of these fails. **ESCAPE** leaves HOLD/GIVEUP immediately.

**Output per state:**

| State | `v_out` | `w_out` |
|---|---|---|
| HOLD | 0 | **0** |
| PASS / ESCAPE (`v > 0`) | `max(v, v_pass)` | `w` clipped to ±0.2 |
| any, `v ≤ 0` (rotate / reverse) | `v` | `w` (not gated) |
| GIVEUP | 0 unless the corridor is clear, then `v·scale` | `w` (TEB may rotate and reroute) |
| FREE | `v·scale` | `w` |

**Scale ramp:**

- Braking (target < 1) is applied **immediately**.
- Speeding up is limited to `release_rate` = 0.8 per second. From 0 to full speed takes about 1.25 s.

### 5.4 Why GIVEUP exists

While the gate holds, move_base sees no progress. If the hold outlasted move_base's `oscillation_timeout` (60 s)
or `controller_patience` / `planner_patience` (`nav_patience`, 30 s), move_base would abort the goal. So after
`gate_hold_timeout` (20 s) the gate stops holding:

- rotation and reversing pass through again, so TEB can find another way;
- forward motion stays blocked until the corridor is clear.

`tests/test_velocity_gate.py::LaunchContractTest` enforces `gate_hold_timeout + 5 < oscillation_timeout` and
`< nav_patience`.

---

## 6. Typical scenarios

| Scenario | What happens |
|---|---|
| Person walks toward the robot | Head-on track, collision course, ≤ 1.5 m → `free -> hold [moving]`, completely still. They pass beside the robot → after 1 s `hold -> free` |
| Person walks toward the robot but offset, passing beside | continue is safe → nothing, no stop |
| Person walks up and stops 1 m in front | HOLD `[moving]`. Once they stand still they're not tracked as moving → static rules: slow, `hold [blocked]` at 0.30 m → after 20 s `giveup`, TEB may go around |
| Cart crosses far ahead, or will be past before we get there | continue is safe → nothing |
| Cart crosses where we would meet it; passing at 0.32 clears | `side obstacle: pass at 0.32 m/s`, no stop |
| Cart crosses close ahead, can't be passed | keeps driving until stopping is due, then `hold [crossing]`; it crosses in front, then release |
| Cart would hit the robot even if it stopped | `side obstacle: escape ...`: drive through |
| End of an aisle (wall) | static: slows, TEB's turns still pass; HOLD only if it drives forward into 0.30 m |

## 7. Log reference

All gate logs are prefixed `[gate]` and come from `velocity_gate` (`output="screen"` in
`warehouse_navigation.launch`). The current state is also published, latched, on `/velocity_gate/state`
(`free` / `hold` / `giveup`), e.g. `rostopic echo /velocity_gate/state`.

### 7.1 State change — INFO, once per change

```
[gate] <old> -> <new>[ [reason]] (gap <G>m, head-on <H>m, tracks <N>)
```

| Field | Meaning |
|---|---|
| `<old>` | previous state; `None` on the first command after the node starts |
| `<new>` | `free`, `hold` or `giveup` |
| `[reason]` | only on transitions **into** `hold`: `[moving]`, `[blocked]` or `[crossing]` |
| `gap <G>m` | nose → nearest return in the corridor at that moment; `infm` = nothing in the corridor |
| `head-on <H>m` | nose → nearest head-on obstacle on a collision course; `infm` = none |
| `tracks <N>` | number of moving obstacles currently tracked (including coasting ones) |

Examples and how to read them:

| Log line | Meaning |
|---|---|
| `[gate] None -> free (gap 3.63m, head-on infm, tracks 0)` | first command received; corridor clear for 3.6 m |
| `[gate] free -> hold [moving] (gap 1.53m, head-on 1.50m, tracks 1)` | head-on obstacle on a collision course at 1.5 m → yield completely still |
| `[gate] free -> hold [blocked] (gap 0.29m, head-on infm, tracks 0)` | static obstacle reached the 0.30 m stop line while TEB was driving forward |
| `[gate] free -> hold [crossing] (gap infm, head-on infm, tracks 1)` | side obstacle can't be passed; stopping now lets it cross in front |
| `[gate] hold -> free (...)` | clear for 1 s (or an escape); ramping back up |
| `[gate] hold -> giveup (gap 0.28m, ...)` | held for 20 s; TEB may rotate/reverse now, forward still blocked |
| `[gate] giveup -> free (...)` | corridor finally clear for 1 s |

States persist while move_base is idle: after one zero Twist the gate goes silent, but it does not reset. So after a pause, the next line
starts from whatever state it was in before.

### 7.2 Slowing — INFO, at most once per second

```
[gate] gap <G>m -> scale <S>
```

Printed while the output speed is reduced (`scale < 0.99`) and the robot still moves forward. `S` is the fraction of
TEB's speed actually sent. It is **not** printed during HOLD, during GIVEUP with forward blocked, or when
the scan is stale, because the output speed is 0 then.

| Example | Meaning |
|---|---|
| `[gate] gap 0.60m -> scale 0.48` | slow zone: 0.30 m left before the stop line, half speed |
| `[gate] gap 3.20m -> scale 0.55` | nothing close: this is the **ramp-up** after a hold or slowdown (0.8 per second) |

### 7.3 Side obstacle decision — INFO, once per change

```
[gate] side obstacle: pass at <V> m/s (TEB asked <T>)
[gate] side obstacle: escape at <V> m/s (TEB asked <T>) - stopping would leave us in its path
[gate] side obstacle: pass done (<why>)
```

| Example | Meaning |
|---|---|
| `side obstacle: pass at 0.32 m/s (TEB asked 0.25)` | continuing would collide, passing is predicted safe → speeding up |
| `side obstacle: pass at 0.32 m/s (TEB asked 0.05)` | TEB slowed for the obstacle; the gate overrides that so the robot doesn't stall in its path |
| `side obstacle: escape at 0.32 m/s ...` | neither passing nor stopping is safe; standing still is worse |
| `side obstacle: pass done (forks 0.25 m past it)` | the obstacle is behind the fork tips by the margin; back to TEB's speed |
| `side obstacle: pass done (it has crossed)` | it finished crossing the path and is moving away |
| `side obstacle: pass done (TEB stopped)` | TEB commanded zero / rotate / reverse; a pass never overrides that |
| `side obstacle: pass done (holding)` / `(no longer needed)` | another obstacle made it hold, or the track vanished |

A `hold [crossing]` while a pass is running never happens: a pass continues as long as it keeps any clearance.

### 7.4 Warnings

| Log | Meaning / action |
|---|---|
| `[gate] no odom pose, moving-obstacle yield off: <tf error>` (WARN, every 5 s) | TF `odom → laser_link` at the scan stamp is missing. Tracks coast on a dead-reckoned pose for up to 1 s, then moving-obstacle tracking (head-on hold, side pass/yield) is off; the static rules still work. Check Unity odom/TF, `laser_tf.launch`, `/use_sim_time` |
| `[gate] error, publishing zero: <exception>` (ERROR, every 1 s) | a bug or bad input raised inside the timer step. The robot is sent zero every tick while it persists; report the exception |

### 7.5 Silent cases worth knowing

| Symptom | Likely cause |
|---|---|
| Robot doesn't move forward, no `[gate]` line at all | `/scan` older than 0.5 s (blind): forward is blocked without HOLD or log. Check `rostopic hz /scan` |
| No gate output at all | move_base isn't publishing `/cmd_vel_nav` (idle, or the goal was aborted) |
| Robot weaves while `/velocity_gate/state` is `free` | the obstacle is static, or slower than about 0.3 m/s, so not flagged moving (`tracks 0`); or it is beyond 1.5 m; or it will pass beside us. Tune `yield_dist`, `~moving_tol`, `~moving_ref_age` |
| Robot stops for a side obstacle that would have missed | the clearance margin is too big for that obstacle: lower `obs_pad` / `time_margin` |

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

| Param | Default | Effect |
|---|---|---|
| `inner_half_w` | 0.26 m | inner shield half-width |
| `d_inner`, `d_hold_static` | 0.20, 0.20 m | inner shield depth / HOLD distance from the nose |
| `outer_half_w`, `d_outer` | 0.50, 1.00 m | outer half-ellipse semi-axes |
| `scale_min`, `min_crawl_speed` | 0.25, 0.10 m/s | slow-down floor at the inner depth, minimum forward speed |
| `half_w` (`~half_width` + `~side_margin`) | 0.33 m | crossing-corridor half-width only |
| `flank_enable`, `flank_x_min`, `flank_w` | false, -0.20, 0.35 | optional flank shield |
| `cross_x_max`, `cross_zone_w` | 1.50, 1.50 m | crossing watch zone |
| `cross_v_window`, `cross_v_min_span`, `cross_v_min` | 0.40 s, 0.25 s, 0.15 m/s | lateral velocity baseline / threshold |
| `cross_confirm`, `cross_t_horizon`, `cross_assoc_gap`, `cross_max_extent` | 3, 2.5 s, 0.35 m, 0.80 m | crossing confirmation and tracking |
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
