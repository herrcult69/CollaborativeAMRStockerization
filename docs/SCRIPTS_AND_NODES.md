# AMR Navigation — Scripts, Nodes & Libraries Reference

> Last updated: 2026-10-03  
> Package: `amr_navigation` (`catkin_ws/src/amr_navigation/`)

---

## Table of Contents

1. [System Architecture Overview](#1-system-architecture-overview)
2. [Library Modules (`src/amr_navigation/`)](#2-library-modules)
   - [move_to_point.py](#21-move_to_pointpy)
   - [docking.py](#22-dockingpy)
   - [lift.py](#23-liftpy)
3. [ROS Nodes (`scripts/`)](#3-ros-nodes)
   - [move_to_point_node.py](#31-move_to_point_nodepy)
   - [docking_node.py](#32-docking_nodepy)
   - [lift_node.py](#33-lift_nodepy)
4. [Mission Scripts (`scripts/`)](#4-mission-scripts)
   - [milestone1_mission.py](#41-milestone1_missionpy)
   - [test_dock_reverse.py](#42-test_dock_reversepy)
   - [test_undock_reverse.py](#43-test_undock_reversepy)
5. [Launch Files](#5-launch-files)
6. [Unity Simulator Components](#6-unity-simulator-components)
7. [Repo Cleanup Assessment](#7-repo-cleanup-assessment)
8. [Git Commit Strategy](#8-git-commit-strategy)

---

## 1. System Architecture Overview

```
Unity Simulator (amr_ware_house)
        │
        │  /cmd_vel (geometry_msgs/Twist)      ← Robot receives motion commands
        │  /lift_cmd (std_msgs/Float32)         ← Mast elevation commands
        │  /odom (nav_msgs/Odometry)            → Robot publishes drive_center pose
        ↓
   ROS (Noetic) — amr_navigation package
   ┌────────────────────────────────────────────────────┐
   │  Libraries (src/amr_navigation/)                   │
   │  ┌──────────────────┐  ┌──────────────┐           │
   │  │ MoveToPointCtrl  │  │ LiftCtrl     │           │
   │  │ (move_to_point)  │  │  (lift.py)   │           │
   │  └──────────────────┘  └──────────────┘           │
   │  ┌───────────────────────────────────┐            │
   │  │  PalletDockingController (docking)│            │
   │  │  compute_dock_target_pose()       │            │
   │  └───────────────────────────────────┘            │
   │                                                    │
   │  Standalone Nodes (scripts/)      Mission Scripts  │
   │  move_to_point_node.py            milestone1_      │
   │  docking_node.py                  mission.py       │
   │  lift_node.py                     test_dock_       │
   │                                   reverse.py       │
   │                                   test_undock_     │
   │                                   reverse.py       │
   └────────────────────────────────────────────────────┘
```

**ROS Topics:**

| Topic | Direction | Msg Type | Description |
|-------|-----------|----------|-------------|
| `/cmd_vel` | Published by ROS, consumed by Unity | `geometry_msgs/Twist` | Linear X (forward/backward) + angular Z (rotation) |
| `/lift_cmd` | Published by ROS, consumed by Unity | `std_msgs/Float32` | Target mast height in meters |
| `/odom` | Published by Unity, consumed by ROS | `nav_msgs/Odometry` | `drive_center` position + quaternion orientation |

**Odometry Frame:**  
Unity's `PlanarOdometryPublisher.cs` tracks the `drive_centre` game object and publishes it with `child_frame_id = "drive_center"`. The drive center is located **+0.10m** in front of `base_link` (i.e., 0.10m toward the back from the geometric center of the chassis), directly between the two drive wheels.

---

## 2. Library Modules

### 2.1 `move_to_point.py`
**Path:** `src/amr_navigation/move_to_point.py`

The core point-to-point navigation library. Implements a **3-phase state machine** that guides the AMR from its current position to any `(x, y, yaw)` target.

#### How It Works

```
Phase 1: ALIGN_TO_GOAL
  └─ Rotate in place until heading error < 12 deg
Phase 2: DRIVE_TO_GOAL  
  └─ Drive forward with proportional speed (max capped) + continuous heading correction
     └─ If heading drifts > 37 deg (0.65 rad) mid-drive → return to Phase 1
Phase 3: ALIGN_FINAL_YAW
  └─ Arrived at position, rotate to final yaw target
     └─ If position drifts > 1.6× tolerance during spin → recover to Phase 1 or 2
```

**Anti-stiction kick:** If the robot is commanded angular velocity but yaw doesn't change for >12 consecutive cycles (0.6s at 20Hz), it applies an additional breakaway kick `+0.18 rad/s`. Overcomes Unity PhysX tire scrub static friction.

#### Key Parameters (ROS params, all settable from launch file)

| Param | Default | Description |
|-------|---------|-------------|
| `~goal_x` | `2.0` | Target X position (m) |
| `~goal_y` | `0.0` | Target Y position (m) |
| `~goal_yaw` / `~goal_yaw_deg` | `0.0` | Final orientation (degrees) |
| `~pos_tolerance` | `0.05` | Arrival radius (m) |
| `~yaw_tolerance` | `4.0` | Final yaw accuracy (degrees) |
| `~max_linear` | `0.55` | Max forward speed (m/s) |
| `~min_linear` | `0.06` | Min driving speed (m/s) |
| `~max_angular` | `0.65` | Max turn speed (rad/s) |
| `~min_angular` | `0.42` | Min turn speed — overcomes Unity stiction (rad/s) |
| `~receipt_timeout` | `1.5` | Odometry staleness limit (s) — emergency stop |

#### Speed Ramp Formula
```
target_v = clamp(0.45 * dist_to_goal, min_linear, max_linear)
linear = target_v * cos(heading_error)   # reduces speed when pointing off-axis
```

#### Public API

```python
# Constructor: pass max speeds to override defaults
nav = MoveToPointController(max_linear=0.35, max_angular=0.65)

# Blocking navigation call
ok = nav.navigate_to(gx, gy, goal_yaw=270.0, pos_tolerance=0.05, label="WP1")
# Returns True on success, False if odometry lost or ros shutdown

# Single-goal node runner (reads goal from ROS params)
nav.run()
```

> [!NOTE] `ThreePhaseDockingController` is an alias kept for backward compatibility. Use `MoveToPointController` in all new code.

---

### 2.2 `docking.py`
**Path:** `src/amr_navigation/docking.py`

Precision pallet insertion / extraction library. Works differently from `MoveToPointController` — **no turning is allowed at all** during docking. The robot must arrive pre-aligned; the controller only commands longitudinal (linear) velocity plus a small heading-correction term to prevent yaw drift.

#### Key Design Principle
Pallet legs have only ~80mm lateral clearance per tine. Any rotation during insertion would laterally strike the legs. The controller clamps angular velocity to ±0.18 rad/s with a gain of `1.6` on yaw error — it corrects micro-drift but cannot steer around obstacles.

#### `compute_dock_target_pose(pallet_x, pallet_y, dock_yaw_deg, fork_offset=0.22)`
Helper function to calculate where `drive_center` must stop so the forks are positioned over the pallet cavity.

```
drive_center_target_x = pallet_x + fork_offset * cos(yaw_rad)
drive_center_target_y = pallet_y + fork_offset * sin(yaw_rad)
```

| `fork_offset` | Effect |
|---|---|
| `0.35m` (old default) | Fork tine midpoint over pallet cavity (~50% insertion) |
| `0.22m` (current default) | Fork heel at cavity entry → **~85-90% tine insertion** |

**Example:** Pallet at `(4.0, 5.0)`, yaw `270°` (south-facing):
```python
tx = 4.0 + 0.22 * cos(270°) = 4.0 + 0.0 = 4.0
ty = 5.0 + 0.22 * sin(270°) = 5.0 - 0.22 = 4.78
# drive_center must stop at (4.0, 4.78) for correct tine depth
```

#### `dock_reverse()` — Reverse Insertion

```python
dock.dock_reverse(
    target_x, target_y,    # drive_center stop pose (from compute_dock_target_pose)
    dock_yaw=270.0,        # locked orientation (degrees)
    speed=0.12,            # max reverse speed (m/s)
    pos_tolerance=0.04,    # arrival tolerance (m)
    label="Pallet Insertion"
)
```

**Velocity law:**
```python
v_cmd = -min(speed, max(0.07, 0.70 * abs(d_long)))
```
- `d_long` = signed distance along robot's heading axis (negative = target is behind robot)
- `0.70 * d_long` = proportional slow-down as it approaches (70% of remaining distance)
- `0.07 m/s` minimum = prevents stalling against Unity PhysX wheel stiction

**Termination conditions:** `dist_to_dock <= tol` OR `d_long >= 0` (robot has passed/reached target)

#### `undock()` — Forward Extraction

Mirror of `dock_reverse` but drives forward. Used to pull forks clear of pallet legs.

```python
dock.undock(
    target_x, target_y,   # exit waypoint (usually pre-dock staging point)
    dock_yaw=270.0,
    speed=0.10,
    pos_tolerance=0.05,
    label="Rack Extraction"
)
```

**Termination:** `dist <= tol` OR `d_long <= 0` (passed target)

---

### 2.3 `lift.py`
**Path:** `src/amr_navigation/lift.py`

Controls the mast vertical elevation with a velocity-ramped trajectory. Publishes a continuous stream of height setpoints to `/lift_cmd` (latched `std_msgs/Float32`). Unity's `StackerController.cs` reads this and drives the mast joint using its own PID.

> [!IMPORTANT] The LiftController tracks height **internally** (dead-reckoning from speed × time). It does **not** read any encoder feedback — there is no `/lift_state` topic. Always initialize `LiftController` with the correct `initial_height` if the robot already has a non-zero mast elevation.

#### `set_height(target_height_m, speed=None, settling_time=0.3)`

Blocking call — returns only after mast reaches target.

```python
lift = LiftController(default_speed=0.06, initial_height=0.0)
lift.set_height(0.015, speed=0.04)   # Slow careful lift inside rack
lift.set_height(0.28, speed=0.06)    # Transit height
lift.set_height(-0.02, speed=0.06)   # Fork entry position (30mm off floor)
```

**Key height reference points:**

| Height | Meaning |
|--------|---------|
| `-0.02m` | Fork entry: bottom of tines ~30mm above floor → slides into 60mm pallet pocket |
| `+0.015m` | Under-rack pick: lifts pallet 15mm, 17mm headroom below 592mm beam |
| `+0.15m` | Aisle transit (single pallet) |
| `+0.21m` | Station deposit: forks inside pocket, pallet weight on station surface |
| `+0.24m` | Pallet bottom just touches 25cm station deck |
| `+0.28m` | Station transit: 40mm clearance over 25cm station top |

---

## 3. ROS Nodes

Thin wrapper nodes that initialize ROS and delegate to the library controllers. Each is launched via its associated `.launch` file.

### 3.1 `move_to_point_node.py`

```
Node name: move_to_point_node
Launch:    roslaunch amr_navigation move_to_point.launch goal_x:=4.0 goal_y:=5.0 goal_yaw:=270.0
```

Reads goal from ROS params and calls `MoveToPointController.run()`. Single-shot: node exits when goal is reached.

### 3.2 `docking_node.py`

```
Node name: docking_node
Launch:    roslaunch amr_navigation docking.launch dock_x:=4.0 dock_y:=5.0 dock_yaw:=270.0
```

Reads pallet target from ROS params, calls `dock_reverse()` once, then stops. Does **not** perform navigation or lift operations — raw docking only. Useful for testing insertion without the full mission script.

**Key params:**

| Param | Default | Description |
|-------|---------|-------------|
| `~dock_x` | `4.0` | Pallet cavity X |
| `~dock_y` | `5.0` | Pallet cavity Y |
| `~dock_yaw` | `270.0` | Insertion heading (degrees) |
| `~dock_speed` | `0.08` | Max creep speed (m/s) |
| `~dock_tolerance` | `0.03` | Stop tolerance (m) |
| `~fork_offset` | `0.22` | Tine penetration depth (m) |

### 3.3 `lift_node.py`

```
Node name: lift_node
Launch:    rosrun amr_navigation lift_node.py 0.35
       or: rosrun amr_navigation lift_node.py _height:=0.35 _speed:=0.06
```

One-shot lift command. Useful for manual testing of mast elevation without running a full mission.

---

## 4. Mission Scripts

Full end-to-end orchestration scripts that combine navigation + docking + lift into multi-step sequences.

### 4.1 `milestone1_mission.py`

**Purpose:** Full end-to-end autonomous warehouse replenishment pipeline. Connects rack docking, under-rack lifting, rack extraction, Manhattan corridor navigation, staging station reverse docking, deposit, and undock extraction into one seamless mission.

**Mission steps (11 phases):**
```
1. Approach rack pre-dock staging pose: (pallet_x, ~3.98), yaw 270°
2. Position lift mast to entry height (-0.020m for bottom; +0.240m for upper pallet; +0.640m for shelf)
3. Precision straight-line reverse docking into pallet cavity at (pallet_x, pallet_y)
4. Elevate lift mast to pick height (+0.010m for bottom; +0.290m for upper pallet; +0.685m for shelf)
5. Pull straight forward out of rack (rack extraction / undock)
6. Elevate / position lift mast to transit height (0.28m-0.29m, 40-50mm clearance above 25cm station)
7. Follow Manhattan orthogonal corridor: (4.0, 5.0) -> (4.0, 0.0) -> (11.98, 0.0), align 180°
8. Precision reverse docking over the 25cm staging station to stop pose (12.78, 0.0)
9. Lower mast to deposit height (0.21m) - pallet settles on station deck, forks float free
10. Pull straight forward to staging standoff (11.98, 0.0), cleanly extracting forks
11. Lower mast to idle/transit height (0.00m) - Mission Complete!
```

**Key configurable params:**

| Param | Default | Description |
|---|---|---|
| `~pick_level` | `bottom` | Target pallet level: `"bottom"` / `"1"` (ground double stack), `"top"` / `"2"` (upper pallet), or `"shelf"` / `"3"` (tier 2 shelf) |
| `~skip_pickup` | `false` | If true, assumes pallet already loaded and starts directly at transit phase |
| `~pallet_x/y` | `4.0, 5.0` | Target rack pallet cavity coordinates (m) |
| `~dock_yaw` | `270.0°` | Approach orientation for rack pickup |
| `~fork_offset` | `0.22m` | Tine penetration depth offset from drive_center |
| `~station_x/y` | `13.0, 0.0` | Staging station target dock coordinates (m) |
| `~station_yaw` | `180.0°` | Station docking orientation (facing West, forks East) |
| `~standoff` | `0.8m` | Standoff distance in front of dock stop pose |
| `~transit_lift_height`| `0.28m` | Mast height during corridor transit (40mm above station) |
| `~deposit_lift_height`| `0.21m` | Mast height during deposit (forks float inside pocket) |
| `~dock_speed` | `0.12 m/s` | Reverse docking creep speed limit |
| `~transit_speed` | `0.45 m/s` | Corridor linear cruising speed limit |
| `~lift_speed` | `0.08 m/s` | Mast vertical elevation speed limit |

---

### 4.2 `test_dock_reverse.py`

**Purpose:** Isolated rack staging, reverse insertion, under-rack lift, and extraction test. Engages the pallet from the rack, lifts it, pulls straight forward into the aisle, and sets the mast to open-aisle transit height. Supports picking from the ground double-stack (`bottom`), upper pallet (`top`), or second-tier rack shelf (`shelf`) via `~pick_level`.

**Mission steps (5 phases):**

```
STEP 1: Navigate to pre-dock staging pose (standoff 0.8m in front of dock target)
        → Yaw: 270° | Tolerance: 5cm
STEP 2: Position lift mast to pocket entry height
        → "bottom": -0.020m (forks at 30-50mm off floor into 0-60mm cavity)
        → "top":    +0.240m (forks at 290-310mm into 280-340mm cavity)
        → "shelf":  +0.640m (forks centered at 690-710mm into shelf cavity)
STEP 3: dock_reverse() straight into pallet cavity
        → drive_center stops at compute_dock_target_pose(4.0, 5.0, 270°, 0.22) = (4.0, 4.78)
        → Speed: 0.12 m/s | Tolerance: 4cm
STEP 4: Lift mast to pick height (Standardized Option 1: Uniform 20mm Pallet Lift)
        → "bottom": +0.010m (lifts pallet 20mm off floor; preserves safe 12mm headroom below 0.592m beam)
        → "top":    +0.290m (lifts pallet 20mm off lower box; preserves safe 12mm headroom below 0.592m beam)
        → "shelf":  +0.685m (lifts pallet 20mm off shelf beam; preserves safe margin below upper ceiling)
STEP 5: undock() straight forward back to staging pose
        → Speed: 0.10 m/s | Tolerance: 5cm
        → Position mast to open-aisle transit height (0.150m for bottom, 0.290m for top, 0.280m for shelf)
```

**Key configurable params:**

| Param | Default | Description |
|-------|---------|-------------|
| `~pick_level` | `bottom` | Target level: `"bottom"` / `"1"` (ground double stack), `"top"` / `"2"` (upper pallet), or `"shelf"` / `"3"` (tier 2 shelf) |
| `~pallet_x/y` | `4.0, 5.0` | Pallet cavity center |
| `~dock_yaw` | `270.0°` | Insertion heading |
| `~fork_offset` | `0.22m` | Tine depth (85-90% penetration) |
| `~dock_speed` | `0.12 m/s` | Insertion creep speed |
| `~entry_lift_height` | *(auto)* | Optional explicit mast entry height override (m) |
| `~pick_lift_height` | *(auto)* | Optional explicit mast pick height override (m) |

---

### 4.3 `test_undock_reverse.py`

**Purpose:** Isolated staging station test at `(13.0, 0.0)`. Tests station approach, transit elevation, reverse docking over the 25cm station, lowering to deposit height, forward undock extraction, and homing the mast. Does **not** perform rack pickup or corridor transit (can run even if nothing was picked up yet).

**Mission steps (6 phases):**

```
STEP 1: Navigate directly to Station Pre-Dock Staging Pose (11.98m, 0.0m)
        → Standoff at 0.8m in front of stop pose; yaw 180.0°
STEP 2: Elevate mast to safe transit height (0.28m)
        → 40mm clearance above 25cm station top
STEP 3: dock_reverse() backwards over 25cm staging station
        → drive_center stops at (12.78m, 0.0m)
STEP 4: Lower lift to deposit height (0.21m)
        → Pallet lands at 0.24m (if loaded); forks float at 0.21m (10mm above deck)
STEP 5: undock() forward from 12.78m to 11.98m
        → Cleanly withdraws forks back into open aisle
STEP 6: Lower mast to 0.00m (idle/travel height)
```

**Key configurable params:**

| Param | Default | Description |
|---|---|---|
| `~station_x/y` | `13.0, 0.0` | Staging station target dock cavity (m) |
| `~station_yaw` | `180.0°` | Station approach orientation (degrees) |
| `~fork_offset` | `0.22m` | Tine depth offset from drive_center |
| `~standoff` | `0.8m` | Distance in front of dock stop pose for staging (m) |
| `~transit_lift_height` | `0.28m` | Approach elevation clearing 25cm station |
| `~deposit_lift_height` | `0.21m` | Fork float height for pallet release |
| `~dock_speed` | `0.12 m/s` | Docking reverse creep speed |
| `~transit_speed` | `0.45 m/s` | Linear speed limit to station staging pose |
| `~lift_speed` | `0.06 m/s` | Mast elevation speed limit |

---

## 5. Launch Files

| Launch File | Node Started | Purpose |
|-------------|-------------|---------|
| `move_to_point.launch` | `move_to_point_node.py` | Single-shot navigate to any waypoint (x, y, yaw) |
| `docking.launch` | `docking_node.py` | Single-shot raw fork insertion (no nav/lift) |
| `test_dock_reverse.launch` | `test_dock_reverse.py` | Isolated rack approach, reverse dock, under-rack lift & extraction test |
| `test_undock_reverse.launch` | `test_undock_reverse.py` | Isolated station approach, reverse dock & undock extraction test |
| `milestone1.launch` | `milestone1_mission.py` | Full end-to-end pick-and-place replenishment mission |
| `unity_bridge.launch` | *(ros_tcp_endpoint)* | Starts the ROS↔Unity TCP bridge on port 10000 |

> [!IMPORTANT] Always start `unity_bridge.launch` **first** (or ensure Unity is running and connected) before any navigation launch, or odometry will time out immediately.

> [!IMPORTANT] Always start `unity_bridge.launch` **first** (or ensure Unity is running and connected) before any navigation launch, or odometry will time out immediately.

---

## 6. Unity Simulator Components

### `PlanarOdometryPublisher.cs`
Tracks the `drive_centre` game object, computes planar position relative to spawn, converts Unity → ROS coordinates, and publishes `/odom`.

**Coordinate mapping:**
```
ROS X = Unity relative.z   (forward in Unity = +X in ROS)
ROS Y = -Unity relative.x  (left in Unity = +Y in ROS)
Yaw   = -Unity Y rotation  (Unity CCW = ROS CW → negate)
child_frame_id = "drive_center"
```

### `StackerController.cs`
Translates `/cmd_vel` Twist messages into wheel torques and reads `/lift_cmd` Float32 for mast PID setpoint.

---

## 7. Repo Cleanup Assessment

### ✅ Healthy / Keep As-Is
- `src/amr_navigation/docking.py` — Clean, well-commented, no dead code
- `src/amr_navigation/move_to_point.py` — Clean, alias `ThreePhaseDockingController` maintained intentionally
- `src/amr_navigation/lift.py` — Clean
- `scripts/test_dock_reverse.py` — Clean after comment sync
- `scripts/test_undock_reverse.py` — Clean; `~custom_staging_x/y` override params are not dead code — they are an intentional escape hatch

### ⚠️ Minor Issues to Address
| File | Issue | Action |
|------|-------|--------|
| `scripts/milestone1_mission.py` | Legacy script, no docking — could confuse newcomers | Add `# LEGACY - superseded by test_dock_reverse.launch` warning in header |
| `scripts/docking_node.py` | Only does raw `dock_reverse`, no undock/nav — limited standalone use | Already well-documented; consider deprecation if not used in demos |
| `tests/test_move_to_point.py` | Not seen in detail — should be reviewed for staleness | Verify unit test values match current defaults |
| Launch files | No `milestone1.launch` or `move_to_point.launch` — check if they exist | Confirm `CMakeLists.txt` install targets are current |

### 🗑️ Potentially Remove
- Any old `*.pyc` cache files in `scripts/` or `src/` — should be in `.gitignore`
- Check for any lingering `staging_x` / `staging_y` param blocks in **old launch files** from before the fix

### ✅ `.gitignore` Should Contain
```gitignore
catkin_ws/build/
catkin_ws/devel/
catkin_ws/logs/
**/__pycache__/
**/*.pyc
```

---

## 8. Git Commit Strategy

Commit changes as small, focused atomic units so the git log tells a clear story. Suggested sequence:

```bash
# Stage and commit files in this order:

# 1. Odometry frame migration (Unity side)
git add Unity/amr_ware_house/Assets/PlanarOdometryPublisher.cs
git commit -m "feat(odom): track drive_centre object, publish child_frame_id=drive_center"

# 2. Core library: docking minimum creep fix (most impactful bug fix)
git add catkin_ws/src/amr_navigation/src/amr_navigation/docking.py
git commit -m "fix(docking): raise min creep speed to 0.07m/s to overcome Unity wheel stiction"

# 3. Core library: fork_offset default 0.35→0.22 for deep tine penetration
git add catkin_ws/src/amr_navigation/src/amr_navigation/docking.py
git commit -m "fix(docking): change default fork_offset to 0.22m for 85-90% tine insertion depth"

# 4. Test script: dock reverse (fork offset, tolerance, param exposure)
git add catkin_ws/src/amr_navigation/scripts/test_dock_reverse.py \
        catkin_ws/src/amr_navigation/launch/test_dock_reverse.launch
git commit -m "feat(nav): parameterise fork_offset in test_dock_reverse, tolerance 0.03→0.04m"

# 5. docking_node: fix stale fork_offset default
git add catkin_ws/src/amr_navigation/scripts/docking_node.py \
        catkin_ws/src/amr_navigation/launch/docking.launch
git commit -m "fix(docking_node): sync fork_offset default 0.35→0.22m across node and launch"

# 6. New feature: undock reverse full mission
git add catkin_ws/src/amr_navigation/scripts/test_undock_reverse.py \
        catkin_ws/src/amr_navigation/launch/test_undock_reverse.launch \
        catkin_ws/src/amr_navigation/CMakeLists.txt
git commit -m "feat(nav): add test_undock_reverse 7-phase pick-and-deliver mission script"

# 7. milestone1: sync stale comments with actual values
git add catkin_ws/src/amr_navigation/scripts/milestone1_mission.py
git commit -m "docs(milestone1): sync stale speed/waypoint comments with actual code values"

# 8. Unit tests
git add catkin_ws/src/amr_navigation/tests/
git commit -m "test(docking): add staging station clearance unit tests, verify 12.78m stop pose"

# 9. Documentation
git add docs/
git commit -m "docs: add warehouse metrics, schematics, and scripts reference documentation"
```

**Commit message format used:**
```
<type>(<scope>): <short description>

Types: feat | fix | docs | test | refactor | chore
```

> [!TIP] Use `git log --oneline` to verify your history looks clean before pushing. Use `git diff HEAD~1` to double-check each commit contains exactly what you expect.
