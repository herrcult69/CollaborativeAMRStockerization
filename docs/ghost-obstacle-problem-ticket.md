# 🎫 TICKET: Ghost Obstacle & Inflation Trap — AMR Freezes When Bumped Near Walls

**Priority:** 🔴 Critical  
**Component:** `amr_navigation` — costmap / recovery / dynamic obstacle handling  
**Branch:** `dynamic-milestone3-an`  
**Robot:** xstack profile ($0.91\,\text{m}$ long, asymmetric: $0.37\,\text{m}$ front / $0.54\,\text{m}$ tail+forks)  
**Sensor:** 2D LiDAR mounted at $x = +0.33\,\text{m}$, 181 rays across $180^\circ$ forward arc only  
**Environment:** Warehouse narrow aisles ($\approx 2.2\,\text{m} - 2.5\,\text{m}$ wide) flanked by heavy pallet racking  

---

## 1. Problem Statement

When a dynamic obstacle (e.g., an automated cart, pallet truck, or pedestrian) makes side-contact with the AMR near a rack/wall, the robot **gets pushed into the wall's inflation zone, the obstacle leaves the $180^\circ$ LiDAR field of view, and the robot permanently freezes and oscillates**.

The mission must be aborted manually. This failure mode prevents autonomous warehouse operations.

---

## 2. Step-by-Step Anatomy of the Failure

```
[1. Dynamic Obstacle Approaching]
       │
       ▼
[2. Side Impact & Lateral Displacement]
       │  Dynamic obstacle brushes or hits AMR flank
       │  AMR is pushed sideways toward warehouse rack
       ▼
[3. Obstacle Enters 180° LiDAR Blind Spot]
       │  Obstacle moves behind the forward 180° arc
       │  No LiDAR rays pass through those cells anymore
       ▼
[4. Ghost Obstacle Forms in costmap_2d]
       │  ROS ObstacleLayer only clears cells when a ray TRACES THROUGH them
       │  With 180° front LiDAR, blind spot cells remain LETHAL forever
       ▼
[5. Combined Inflation Trap]
       │  Rack inflation (0.50m) + Ghost obstacle inflation (0.50m)
       │  Robot footprint is submerged in high/lethal cost cells
       ▼
[6. TEB Planner Trapping & Oscillation]
       │  TEB finds all trajectory candidates in collision
       │  move_base commands jitter back and forth (±0.02 m/s, ±0.03 rad/s)
       ▼
[7. Timeout & Abort]
       │  oscillation_timeout (60s) expires
       └─► Goal ABORTED (no recovery behavior enabled)
```

---

## 3. Root Cause Breakdown

| Subsystem | Root Cause |
|---|---|
| **Sensor Geometry** | The 2D LiDAR is mounted forward at $x = +0.33\,\text{m}$. Due to the front mast, drive wheels, chassis, and rear forks, rear rays are blocked. Blind spot extends $180^\circ$ from robot flank to rear. |
| **`costmap_2d` Clearing Logic** | Cells marked as `LETHAL_OBSTACLE` (254) are **never cleared by time**; they are *only* cleared if a laser ray travels *through* the cell to free space beyond. Because no rays reach the blind spot, the cell becomes a permanent **ghost**. |
| **Disabled Recovery** | `recovery_behavior_enabled: false` was set in [`warehouse_navigation.launch`](file:///p:/AntiGravity/FinalYear/Project_Project/CollaborativeAMRStockerization/catkin_ws/src/amr_navigation/launch/warehouse_navigation.launch#L37). While disabling 360° clearing rotation (`clearing_rotation_allowed: false`) was correct to protect racks, disabling recovery entirely stripped `move_base` of the safe `clear_costmap_recovery` step. |

---

## 4. Evaluated & Rejected Approaches

### ❌ [DISCARDED] Solution D: Object-Lock Partial Turning
- **Concept:** Rotate the robot chassis toward the moving obstacle to keep it in the front $180^\circ$ LiDAR field of view until it passes safely.
- **Why Discarded:**
  1. **Asymmetric Tail Sweep:** The xstack AMR has a $0.54\,\text{m}$ rear tail with forks. In a narrow aisle ($\approx 2.2\,\text{m}$ wide) where the robot is already near a rack ($< 0.3\,\text{m}$ clearance), rotating even $15^\circ - 30^\circ$ causes the forks to swing outward and strike the steel rack uprights.
  2. **Unnecessary Complexity:** Requires real-time continuous swept-volume polygon collision checking against static map geometry before executing any turn.
  3. **Alternative Sufficiency:** Deterministic costmap clearing and breadcrumb reversing solve the unsticking problem without any chassis rotation.

### ❌ [DISCARDED] Reinforcement Learning (RL) Policy
- **Why Discarded:**
  1. The ghost obstacle phenomenon is a **deterministic state-estimation bug** in `costmap_2d`, not a complex dynamic control problem.
  2. RL policies are black boxes that cannot guarantee safety bounds (e.g., maximum reverse distance, certified non-contact under **ISO 3691-4** standards).

---

## 5. Candidate Deterministic Solutions

### ✅ Solution 1: Costmap Auto-Clear on Stall (Primary Fix)
* **Mechanism:** A supervisor watchdog monitors robot odometry during active `move_base` goals. If translation is $< 0.08\,\text{m}$ for $> 3.0\,\text{s}$ while commands are being issued:
  1. Call `/move_base/clear_costmaps` service.
  2. The ghost cells in the blind spot are wiped immediately.
  3. Real obstacles (the rack wall) are re-marked by the forward LiDAR on the very next scan ($0.1\,\text{s}$).
  4. TEB replans with a clean local costmap.
* **Acceptance Criteria:**
  - [ ] Stall detected within $3.0 - 4.0\,\text{s}$.
  - [ ] Ghost obstacle removed without dropping static map layers.
  - [ ] Cooldown period ($\ge 5.0\,\text{s}$) prevents clearing loops.

### ✅ Solution 2: Breadcrumb Safe Reverse Nudge (Escape Maneuver)
* **Mechanism:** If the robot is physically pressed against the wall's inflation layer, clearing the ghost obstacle may not be enough because the rack's own inflation still penalizes the footprint.
  1. The supervisor tracks the robot's historical odometry ("wake trail").
  2. Robot executes a reverse creep of $\le 0.25\,\text{m}$ at $\le 0.05\,\text{m/s}$ strictly along the corridor it occupied $1.0 - 3.0\,\text{s}$ ago (proven clear).
  3. Followed by a fresh costmap clear to give TEB clearance to replan.
* **Acceptance Criteria:**
  - [ ] Reverse motion strictly limited to $\le 0.25\,\text{m}$ and $\le 0.05\,\text{m/s}$.
  - [ ] Reverses only into historical footprint space, never unknown/unvisited space.
  - [ ] Maximum 2 attempts before escalating to operator alert.

### ✅ Solution 3: Proactive Dynamic Velocity Gate (Collision Prevention)
* **Mechanism:** Upgrade [`velocity_gate.py`](file:///p:/AntiGravity/FinalYear/Project_Project/CollaborativeAMRStockerization/catkin_ws/src/amr_navigation/scripts/velocity_gate.py) from a static distance corridor check to an active Time-to-Collision (TTC) estimator.
  1. Cluster dynamic obstacle returns from consecutive `/scan` messages.
  2. Estimate obstacle velocity vector $\vec{v}_{\text{obs}}$.
  3. Predict intersection point with AMR path:
     - $\text{TTC} > 3.0\,\text{s}$: Full operating speed ($0.25\,\text{m/s}$).
     - $1.5\,\text{s} < \text{TTC} \le 3.0\,\text{s}$: Proportional speed reduction.
     - $\text{TTC} \le 1.5\,\text{s}$: Complete halt before the dynamic obstacle makes contact.
* **Acceptance Criteria:**
  - [ ] Halts AMR ahead of intersecting crossing traffic.
  - [ ] Eliminates side collisions before the robot can be pushed into racks.

---

## 6. Implementation Scope

1. **Launch File Tuning:**
   - Keep `clearing_rotation_allowed: false` to protect forks and rack uprights.
   - Separate costmap clearing recovery from rotational recovery if using built-in `move_base` recovery behaviors, OR run `recovery_supervisor.py`.
2. **Supervisor Node:**
   - Standalone node watching `/odom` and `/move_base/status`.
   - Dispatches `/move_base/clear_costmaps` and breadcrumb reverse commands safely.
