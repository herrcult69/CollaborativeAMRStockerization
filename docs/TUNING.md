# Velocity Gate & Navigation Stack Tuning Reference

Parameter dictionary (§1), rates and perf profiles (§2), crossing-zone recipes (§3), the timeout rule (§4) and how to switch and verify a
perf profile (§5). How the gate works: [`VELOCITY_GATE.md`](VELOCITY_GATE.md). Defaults below match `gate_logic.py:DEFAULTS`.

---

## 1. Velocity Gate Parameter Dictionary (`gate_logic.py:DEFAULTS`)

All parameters are loaded from `gate_logic.py:DEFAULTS` and can be overridden via node private parameters `~<name>` or in navigation launch files.

| Parameter | Default | Unit | Meaning (Source Code Comments) |
| :--- | :--- | :--- | :--- |
| `nose_x` | 0.37 | m | Footprint front edge ahead of drive_center |
| `laser_x` | 0.33 | m | Laser mount ahead of drive_center |
| `tail_x` | 0.54 | m | Footprint rear edge behind drive_center |
| `half_w` | 0.33 | m | Crossing-corridor half-width (robot 0.23m + side margin 0.10m); NOT the inner shield |
| `inner_half_w` | 0.26 | m | Inner shield half-width: chassis (0.23m) + 0.03m |
| `d_inner` | 0.20 | m | Inner protective shield depth ahead of nose [Category 0 Stop] |
| `outer_half_w` | 0.50 | m | Outer shield (half-ellipse) lateral semi-axis |
| `d_outer` | 1.00 | m | Outer shield (half-ellipse) forward semi-axis ahead of nose |
| `d_hold_static` | 0.20 | m | Distance ahead to trigger in-line HOLD for stationary obstacle |
| `min_crawl_speed` | 0.10 | m/s | Minimum forward speed during slowdown (prevents stall on PhysX friction) |
| `flank_enable` | False | bool | Flank shield off by default (side clearance is TEB's job) |
| `flank_x_min` | -0.20 | m | Rear boundary along chassis for side flank protection |
| `flank_w` | 0.35 | m | Lateral boundary from centerline for flank protection |
| `seg_gap` | 0.20 | m | Max distance between adjacent rays in same obstacle cluster |
| `min_cluster_pts` | 3 | count | Min rays to confirm obstacle (noise rejection) |
| `release_rate` | 1.0 | 1/s | Acceleration ramp-up (scale change per second) |
| `scan_timeout` | 0.50 | s | Drop out to stop if scan is delayed |
| `hold_timeout` | 10.0 | s | Max duration to hold before giving up to allow reroute |
| `giveup_w_max` | 0.30 | rad/s | Max \|w\| while in GIVEUP rotation escape |
| `scale_min` | 0.25 | ratio | Slow-down scale just outside d_inner (floor of the ramp) |
| `clear_margin` | 0.10 | m | Gap must exceed d_inner + clear_margin to count as clear |
| `clear_time` | 0.80 | s | Duration corridor must stay clear before releasing HOLD |
| `cross_zone_w` | 1.50 | m | Lateral half-width of crossing watch zone ahead |
| `cross_x_max` | 1.50 | m | Depth of crossing watch zone ahead of the nose, independent of d_outer |
| `cross_check_dist` | 0.90 | m | A tracked crosser only counts as threat within this distance of the nose centre; >= x_max + zone_w = off |
| `cross_v_window` | 0.40 | s | Baseline over which a track's lateral velocity is measured |
| `cross_v_min_span` | 0.25 | s | Min track age before its velocity is trusted |
| `odom_timeout` | 0.30 | s | Measured odom older than this falls back to commanded v/w |
| `cross_v_min` | 0.15 | m/s | Min lateral closing speed to count as crossing |
| `cross_t_horizon` | 1.8 | s | Max time-to-enter-corridor that triggers yield |
| `cross_assoc_gap` | 0.35 | m | Max centroid jump to associate a cluster across scans |
| `cross_confirm` | 3 | scans | Consecutive scans a crossing threat must persist |
| `cross_max_extent` | 0.80 | m | Ignore clusters longer than this (walls, rack faces) |
| `cross_w_max` | 0.45 | rad/s | Skip crossing detection while \|w\| exceeds this; 0.45 = max_vel_theta = off |
| `blocker_half_width` | 0.60 | m | Half-length of the GIVEUP blocker line across the passage |
| `blocker_ttl` | 60.0 | s | How long the global-costmap blocker stays published |
| `blocker_spacing` | 0.05 | m | Point spacing along the blocker line |
| `shim_lookahead` | 0.60 | m | GIVEUP rotation shim: distance walked along the new plan |
| `shim_tol` | 0.15 | rad | GIVEUP rotation shim: heading error below this stops turning |
| `shim_plan_age` | 3.0 | s | GIVEUP rotation shim: max age of the plan used |
| `spin_enable` | True | bool | Spin-first: rotate in place toward the plan when facing away from it |
| `spin_enter` | 1.05 | rad | Heading error that starts a spin (60 deg) |
| `spin_exit` | 0.35 | rad | Heading error that ends a spin (20 deg) |
| `spin_w` | 0.45 | rad/s | Spin angular speed |
| `spin_hold_delay` | 2.0 | s | HOLD [blocked] allows the spin only after this long |
| `spin_min_remaining` | 0.50 | m | No spin when less plan than this is left (goal-yaw rotation belongs to TEB) |
| `reinit_enable` | True | bool | Band monitor: ask TEB to rebuild its band when it looks wrong (see band_health) |
| `reinit_bulge` | 0.50 | m | Local plan this far from the global plan = bad band |
| `reinit_ratio` | 1.60 | ratio | Local plan this many times longer than the global stretch it covers = bad band |
| `reinit_cusp_seg` | 0.03 | m | Segments shorter than this are ignored by the cusp test |
| `reinit_confirm` | 2 | plans | Consecutive bad local plans before a reinit is requested |
| `reinit_cooldown` | 3.0 | s | Min time between detector-triggered reinits |
| `reinit_hold` | 0.40 | s | How long TEB re-initialises every cycle (and the gate outputs zero) |
| `reinit_min_override` | 0.50 | s | An override (HOLD / spin / GIVEUP) shorter than this does not trigger a reinit |

---

## 2. Rates and perf profiles

Why the clock matters: with `use_sim_time`, every ROS timer and every `rospy.Rate` ticks on `/clock`. A 20 Hz clock means the
20 Hz gate loop can only fire on 50 ms ticks and a 10 Hz controller cannot be steady. Unity now publishes `/clock` at 100 Hz from
`FixedUpdate`, on its own timer (`clockHz`), separate from `/odom` + `/tf` (`publishHz`). All three share one fixed sim-time source,
so the clock is never behind the odom it stamps.

| Item | `laptop` | `smooth` (default) | Where | Why |
| :--- | :--- | :--- | :--- | :--- |
| `/clock` | 100 Hz | 100 Hz | Unity `PlanarOdometryPublisher.clockHz` (capped at 1 / Fixed Timestep = 100 Hz) | Tiny message; removes timer jitter. Use 50 if the ROS-TCP link lags |
| `/odom` + `/tf` | 50 Hz | 50 Hz | Unity `PlanarOdometryPublisher.publishHz` | Smoother measured twist for the gate's ego-motion |
| `/scan` | 10 Hz | 10 Hz (**unchanged**) | Unity `PlanarLaserScanPublisher` | Matches a real lidar; keeps `cross_confirm = 3` meaning 0.3 s |
| `controller_frequency` | 5 Hz | 10 Hz | `config/perf_*.yaml` | Main CPU cost; paid for by the rows below |
| Local costmap size | 8 x 8 m | 5 x 5 m | `config/perf_*.yaml` | Without costmap_converter every occupied cell is a TEB obstacle; about 60 % fewer cells |
| Local costmap `obstacle_range` / `raytrace_range` | 3.5 / 4.0 m | 2.5 / 3.0 m | `config/perf_*.yaml` | Still above TEB lookahead (2.0 m) plus braking distance at 0.35 m/s. The **global** costmap keeps 3.5 / 4.0 |
| TEB `costmap_obstacles_behind_robot_dist` | 1.5 m | 0.5 m | `config/perf_*.yaml` | Drops obstacles behind the robot |
| TEB `obstacle_poses_affected` | 30 | 15 | `config/perf_*.yaml` | Fewer graph edges |
| Local costmap update / publish | 3 / 1 Hz | 5 / 2 Hz | `config/perf_*.yaml` | Cheap once the map is 5 x 5 m |
| Kept as is | | | | TEB iterations 3/3, homotopy off, converter off, gate 20 Hz, shields 10 Hz, RViz light profile |

The Unity rates are the same in both profiles (they are Inspector fields on the robot prefab, not ROS parameters). `perf:=` only
changes the move_base side. The gate logic and its thresholds are identical in both profiles.

---

## 3. Crossing-zone recipes

Change one parameter at a time and re-run the same crossing. The zone and check arc are drawn in RViz (cyan = watching, magenta =
yielding), so you can see what a change does. Gate parameters are node parameters; set them in `warehouse_navigation.launch`
inside the `velocity_gate` node, for example `<param name="cross_check_dist" value="0.7" />`.

### Yields too early (stops while the person is still far away)

| Change | Effect | Side effect |
| :--- | :--- | :--- |
| Lower `cross_check_dist` (0.90 -> 0.70) | only crossers closer than this to the nose centre count | the robot yields later; keep it above the distance it travels in a few scans (0.35 m/s is about 3.5 cm per scan) plus any margin you want |
| Lower `cross_t_horizon` (1.8 -> 1.2 s) | a crosser must reach the corridor sooner to count | a slow walker aimed at the corridor is ignored for longer |
| Raise `cross_v_min` (0.15 -> 0.25 m/s) | slow lateral drift no longer counts | a person who slowly shuffles into the aisle is missed |

### Yields too late, or not at all

| Check or change | Why |
| :--- | :--- |
| Raise `cross_check_dist` (up to `cross_x_max + cross_zone_w` = 3.0 m, which switches the check off) | the person is outside the check radius |
| Raise `cross_t_horizon` (1.8 -> 2.5 s) | longer look-ahead for crossers that will enter the corridor |
| Lower `cross_confirm` (3 -> 2) | latches one scan (0.1 s) sooner; more false triggers |
| Lower `cross_v_min`, but not below about 0.10 m/s | centroid noise of +/- 1.5 cm over the 0.4 s baseline already reads as roughly 0.05-0.08 m/s |
| `cross_max_extent` (0.80 m) | a wide cart or a person carrying a box is longer than this and is not tracked as a crosser; raise it for wide moving objects, at the cost of rack faces being tracked |
| Person visible at all? | the cluster needs `min_cluster_pts` (3) rays; the Human must be on a lidar-visible layer and tall enough (costmap `min_obstacle_height` 0.05 m); see `TROUBLESHOOTING.md` |

### False yields near racks or walls

| Check or change | Why |
| :--- | :--- |
| `rostopic hz /odom` | stale odom makes the gate fall back to the commanded `v`/`w` (after `odom_timeout`, 0.30 s). A static rack then appears to move sideways as the real motion differs from the command |
| Lower `cross_max_extent` (0.80 -> 0.60) | rack faces and walls are long; a person is about 0.3-0.5 m |
| Raise `cross_v_min` (0.15 -> 0.25) or `cross_confirm` (3 -> 4) | rejects slow apparent drift |
| Lower `cross_w_max` (0.45 -> 0.30 rad/s) | skips crossing detection earlier while the robot turns (during turns the whole scene sweeps sideways) |
| Lower `cross_check_dist` | racks beside the aisle fall outside the check radius |

### Robot stays in YIELD

The gate releases after the corridor has been clear for `clear_time` (0.8 s). If it never releases, the thing it yields to is still
in the corridor or still tracked (check the magenta sphere in RViz). After `hold_timeout` (10 s) the gate moves to GIVEUP and
TEB may rotate and replan; see `VELOCITY_GATE.md` §4.

---

## 4. The timeout rule

```
gate_hold_timeout  <  nav_patience  <  oscillation_timeout        (defaults: 10 s < 30 s < 60 s)
```

While the gate holds, move_base sees no progress. If the hold outlasted `controller_patience` / `planner_patience`
(`nav_patience`) or `oscillation_timeout`, move_base would abort the goal before the gate gives up. All three are launch args of
`warehouse_navigation.launch`; if you raise `gate_hold_timeout`, raise the other two with it. Raising `hold_timeout` makes the robot wait
longer for a crosser before it tries to go around; lowering it makes it try to reroute sooner (and place a costmap blocker sooner).

---

## 5. Switching and verifying a perf profile

```bash
roslaunch amr_navigation warehouse_autonav.launch launch_rviz:=true                 # perf:=smooth (default)
roslaunch amr_navigation warehouse_autonav.launch launch_rviz:=true perf:=laptop    # the earlier low-CPU values
```

`warehouse_navigation.launch` takes the same `perf:=` argument when started on its own.

Check what was actually loaded (the profile is loaded after the base configs and overrides them):

```bash
rosparam get /move_base/controller_frequency                          # 10.0 smooth, 5.0 laptop
rosparam get /move_base/local_costmap/width                           # 5.0 smooth, 8.0 laptop
rosparam get /move_base/TebLocalPlannerROS/obstacle_poses_affected    # 15 smooth, 30 laptop
```

Acceptance check, with the sim running the same route (Human crossing plus a rack aisle), CPU only, RViz light profile:

```bash
rosrun amr_navigation perf_probe.sh        # 60 s; prints PASS/FAIL
```

| Criterion | Target |
| :--- | :--- |
| `/clock` | >= 95 Hz |
| `/cmd_vel_nav` | >= 9 Hz |
| move_base `Control loop missed its desired rate` | <= 1 per minute |
| move_base CPU (mean) | < 60 % of one core |
| Unity | holds 100 Hz FixedUpdate, no "Maximum Allowed Timestep" spiral (check the Unity console) |

If `smooth` fails on a machine, use `perf:=laptop` there; nothing else needs to change. `rostopic hz` measures in ROS time, so under
`use_sim_time` a slow Unity (real-time factor below 1) does not show up in the rates; watch the Unity frame time as well.
