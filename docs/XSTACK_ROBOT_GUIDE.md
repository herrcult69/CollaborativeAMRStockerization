# XStack AMR: System Architecture, Kinematics & Operations Guide

> Consolidated Authoritative Reference for the XStack AMR Platform  
> Replaces and supersedes `XSTACK_MIGRATION.md`, `XSTACK_NEWCOMER_REPORT.md`, and `XSTACK_VERIFICATION.md`.

---

## 1. Platform Overview & Migration Summary

The **XStack Pallet Stacker AMR** is the primary differential-drive robot for the Collaborative AMR Stockerization project. It features:
* **Differential-drive mobile base** with independent drive wheels and rear casters.
* **1-DOF vertical prismatic mast** commanding fork elevation via `/lift_cmd`.
* **2D planar LiDAR** publishing 181 raycasts at 10 Hz across a $180^\circ$ forward arc.
* **Ground-truth odometry** tracking the ground projection of the drive axle (`drive_center`).

The legacy robot model remains supported in parallel via `robot_profile:=legacy` for backward compatibility and comparative regression tests.

---

## 2. Coordinate Frame & TF Architecture

### 2.1 The `drive_center` Reference Contract
In differential-drive kinematics, the Instantaneous Center of Rotation (ICR / fulcrum) lies on the axis passing through the drive wheels. 
* **`drive_center`**: The ground-projected midpoint of the drive axle ($Z = 0.0\text{ m}$). In Unity, this is tracked by the helper object `drive_centre` and published over `/odom`.
* **`base_link`**: The structural front chassis origin, located **$0.10\text{ m}$ ahead** ($+X$) of `drive_center`.
* **Why this matters**: In early iterations, odometry tracked `base_link`. Rotating in place caused `base_link` to swing in a circle of radius $0.10\text{ m}$, corrupting odometry with artificial translational drift. Tracking `drive_center` ensures **pure in-place rotations produce zero translational drift** ($\Delta x = 0, \Delta y = 0$).

### 2.2 Complete Coordinate Transform (TF) Tree

$$\text{map} \xrightarrow{\text{AMCL}} \text{odom} \xrightarrow{\text{Unity}} \text{drive\_center} \xrightarrow{\text{laser\_tf}} \text{base\_link} \xrightarrow{\text{laser\_tf}} \text{laser\_link}$$

| Frame | Origin Description | Owner / Publisher |
| :--- | :--- | :--- |
| **`map`** | Global fixed warehouse map frame | `amcl` / `map_server` |
| **`odom`** | Continuous dead-reckoning local navigation frame | Unity `PlanarOdometryPublisher.cs` |
| **`drive_center`** | Ground projection of drive axle midpoint | Unity `/odom` `child_frame_id` |
| **`base_link`** | Structural chassis origin ($X = +0.10\text{ m}, Z = 0.05\text{ m}$) | `laser_tf.launch` (static transform) |
| **`laser_link`** | Optical center of 2D LiDAR scanner ($X = +0.33\text{ m}, Z = 0.13\text{ m}$) | `laser_tf.launch` (static transform) |

```text
       +---------------------------------------------------------+
       | [Front Bumper / LiDAR] (X = +0.33m from drive_center)  |
       |                      base_link (X = +0.10m)             |
       |                                                         |
===O===+==================== drive_center =======================+===O=== (X = 0.00m)
(Left Wheel)                [Track Width = 0.40m]               (Right Wheel)
       |                                                         |
       |                      Mast Link (X = -0.09m)             |
       |                      Fork Carriage (X = -0.12m)         |
       |                      Fork Base (X = -0.16m)             |
       |                                                         |
       |                      FORK REFERENCE (X = -0.22m) <====== [Dock Target Offset]
       |                                                         |
       |                      Fork Tips (X = -0.535m)            |
       +---------------------------------------------------------+
```

---

## 3. Physical Footprint & Sensor Specifications

### 3.1 Collision & Costmap Footprint
Measured directly from the 10 active physical ArticulationBody colliders in Unity (excluding the dormant demonstration pallet):
* **Bounding Box**: $X \in [-0.54\text{ m}, +0.37\text{ m}]$, $Y \in [-0.23\text{ m}, +0.23\text{ m}]$
* **Chassis Length**: $0.91\text{ m}$ (from fork tips to front bumper)
* **Chassis Width**: $0.46\text{ m}$ (track width $0.40\text{ m}$)
* **Costmap Polygon (with $2\text{ cm}$ safety padding)**:
  ```yaml
  footprint: [[0.37, 0.23], [0.37, -0.23], [-0.54, -0.23], [-0.54, 0.23]]
  ```

### 3.2 2D LiDAR Specifications
* **Topic**: `/scan` (`sensor_msgs/LaserScan`)
* **Ray Count**: 181 rays across $180.0^\circ$ ($1.0^\circ$ angular resolution)
* **Update Rate**: 10 Hz
* **Range Limits**: $0.05\text{ m} \to 10.0\text{ m}$
* **Mount Offset from `drive_center`**: $(X = +0.33\text{ m}, Y = 0.00\text{ m}, Z = +0.13\text{ m})$
* **Raycast Layer Mask**: Robot body colliders are strictly excluded from raycasting to prevent self-reflection ghost obstacles.

---

## 4. Robot Profile Selection (`xstack` vs `legacy`)

The launch system parameterizes robot geometry via the `robot_profile` argument:

| Property | `robot_profile:=xstack` (Default) | `robot_profile:=legacy` |
| :--- | :--- | :--- |
| **Odometry Base Frame** | `drive_center` | `base_footprint` |
| **Chassis Origin (`base_link`)** | $(0.10, 0.0, 0.05)$ | $(0.0, 0.0, 0.05)$ |
| **LiDAR Relative to Base** | $(0.23, 0.0, 0.08)$ | $(0.25, 0.0, 0.225)$ |
| **Default Map File** | `maps/milestone_1_01.yaml` | `maps/warehouse_training_01.yaml` |
| **Unity Scene Match** | `Milestone_1` / `Milestone_2` | `WarehouseTraining` / `SampleScene` |

---

## 5. Measured Physical Characteristics & Verification Data

### 5.1 PhysX Wheel Stiction & Actuator Dynamics
* **Under-Rotation Factor**: In Unity PhysX, tire scrub friction causes differential-drive robots to under-rotate by approximately $\sim 0.85\times$ relative to commanded angular velocity $\omega$.
* **Anti-Stiction Solutions Applied**:
  * In `docking.py` (`reverse_line_command`): Clamps minimum reverse creep velocity to **$\ge 0.07\text{ m/s}$**, preventing stiction stalls when backing into pallet pockets.
  * In `move_to_point.py`: Detects yaw stalls ($>0.6\text{s}$ at 20 Hz without progress) and injects an active **$+0.18\text{ rad/s}$ breakaway pulse**.
  * Rack Entry Standoff: Rack docking uses a **$1.2\text{ m}$ runway** (`rack_standoff:=1.2`), giving the control law adequate distance to damp out initial $3\text{ cm}$ lateral stiction offset before reaching the $\pm 8\text{ mm}$ fork pocket gate.

### 5.2 Navigation Stack Verification Results
* **Map**: `milestone_1_01.yaml` ($800 \times 800$ cells @ $0.05\text{ m/pix}$, measured from simulation scans).
* **AMCL Initialization**: Automated seed at $(0.0, 0.0, 0.0)$ converges within $0.0009\text{ m}$ continuity error.
* **DWA Local Planner Goals**: Successfully tested over multiple $8\text{ m}$ aisle traverses, terminating within position tolerance $<0.20\text{ m}$ and heading tolerance $<0.18\text{ rad}$.

---

## 6. How to Run & Test the Robot

### 1-Click Autonomous Stack (Recommended)
```bash
# 1. Press Play in Unity (Milestone_1 or Milestone_2 scene)
# 2. Inside Docker, launch master stack:
roslaunch amr_navigation warehouse_autonav.launch launch_rviz:=true
```

### Milestone 1 Replenishment Mission
```bash
roslaunch amr_navigation milestone1.launch
```

### Milestone 2 Hybrid Mission (move_base + Precision Docking)
```bash
roslaunch amr_navigation milestone2.launch
```

### Automated Unit Test Suite
```bash
python3 -m unittest discover -s /catkin_ws/src/amr_navigation/tests -p "test_*.py"
```
