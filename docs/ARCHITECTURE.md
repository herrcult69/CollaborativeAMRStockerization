# System Architecture & Project Blueprint

> **Course**: 61CSE326 – Robot Obstacle-Avoidance Planning Software (WS2026)  
> **Institution**: Vietnamese-German University (VGU) — Computer Science & Engineering  
> **Supervisor**: Dr.-Ing. Quang Huan Dong (`huan.dq@vgu.edu.vn`)  
> **Use Case**: **UC 6** (Human-Robot Collaborative Logistics Runner / Autonomous Pallet Stacker AMR)

---

## 1. Executive Summary & Defined Scope

The project focuses on back-of-house warehouse logistics for pallet/tote replenishment:
1. **Autonomous Pallet Stacker AMR**:
   - Differential-drive kinematics for high indoor maneuverability.
   - 1-DOF vertical prismatic mast to lift, dock, and transport pallets/totes.
   - Autonomous obstacle avoidance navigating narrow aisles and shared corridors.
2. **Human Worker Staging Station**:
   - Receives totes from the AMR at an ergonomic workstation.
   - Eliminates complex retail shelf manipulation in favor of **80% software focus on static and dynamic obstacle avoidance**.

---

## 2. Technical Stack & Architecture

```text
+-----------------------------------------------------------------------------------+
|                              WINDOWS 11 WORKSTATION                               |
|                                                                                   |
|   +------------------------------------+   +----------------------------------+   |
|   |         UNITY 3D (Native)          |   |          VS CODE / IDE           |   |
|   |   - Warehouse 3D Environment       |   |   - Edits P:\... on Windows      |   |
|   |   - AMR ArticulationBody Physics   |   |   - Git repository management    |   |
|   |   - 2D LiDAR Raycasting            |   +----------------------------------+   |
|   |   - ROS-TCP-Connector (C#)         |                    | (Mounted Volume)    |
|   +-----------------+------------------+                    v                     |
|                     ^               +-----------------------------------------+   |
|     TCP Loopback    |               |         DOCKER CONTAINER (ROS 1)        |   |
|   127.0.0.1:10000   |               |                                         |   |
|                     v               |   - ROS 1 Noetic Desktop Full           |   |
|   +-----------------+----------+    |   - ros_tcp_endpoint (Port 10000)       |   |
|   |   ros_tcp_endpoint Node    |--->|   - move_base (DWA / TEB Planners)      |   |
|   +----------------------------+    |   - Costmap2D (Static & Dynamic)        |   |
|                                     |   - rosserial_python (Hardware Link)    |   |
|                                     +-------------------+---------------------+   |
|                                                         | (WSLg Graphics)         |
|   +------------------------------------+                v                         |
|   |          RVIZ (Native GUI)         |<---------------+                         |
|   |   - 2D Obstacle Costmaps           |                                          |
|   |   - Global Path & Local Trajectory |                                          |
|   +------------------------------------+                                          |
+-----------------------------------------------------------------------------------+
```

### Why ROS 1 Noetic + Docker on Windows?
* **DirectX GPU Acceleration**: Unity runs natively on Windows at 60+ FPS without Linux GPU passthrough issues.
* **Deterministic Navigation**: ROS 1 `move_base` provides battle-tested monolithic YAML configurations (`costmap_2d`, `dwa_local_planner`, `teb_local_planner`) without the complexity of ROS 2 lifecycle states.
* **Zero Host Pollution**: ROS 1 runs entirely inside a Docker container with Catkin workspace mounted live from Windows.

---

## 3. Communication & Standard Topic Interfaces

| Topic | Type | Source | Destination | Purpose |
| :--- | :--- | :--- | :--- | :--- |
| `/odom` | `nav_msgs/Odometry` | Unity | ROS (`go_to_point`, `move_base`) | Ground-truth planar odometry from `base_link` |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS | Unity | Linear and angular velocity commands |
| `/lift_cmd` | `std_msgs/Float32` | ROS | Unity | Target mast lift height ($0.25\text{ m}$) |
| `/scan` | `sensor_msgs/LaserScan` | Unity | ROS (`costmap_2d`) | 2D LiDAR raycast point cloud for obstacle avoidance |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | ROS / Unity | RViz, Planners | Coordinate transforms (`map` $\rightarrow$ `odom` $\rightarrow$ `base_link` $\rightarrow$ `laser_link`) |

---

## 4. Course Milestones Roadmap

| Milestone | Session | Deliverable in ROS 1 | Validation Criterion |
| :--- | :--- | :--- | :--- |
| **Milestone 1** | Session 5 | 3-Phase Precision Docking & Lift Trigger | Unity $\leftrightarrow$ Docker loopback, stopping within 5 cm, final yaw within 3.0°, lift triggered |
| **Milestone 2** | Session 10 | Static Obstacle Avoidance (`move_base`) | 2D Costmap clears/marks obstacles in RViz; DWA steers AMR around static clutter |
| **Milestone 3** | Session 14 | Dynamic Obstacle Avoidance (Walking Pedestrians) | TEB Local Planner yields or re-routes in real-time without collision |
| **Milestone 4** | Session 15 | Final Technical Report & Defense | IMRaD report ($\le 20$ pages), quantitative metrics (clearance, speed, zero collisions) |

---

## 5. Team Division of Labor

| Role | Member Responsibilities |
| :--- | :--- |
| **System Architect** | Docker containerization, `docker-compose`, Catkin workspace, ROS-TCP bridge, hardware abstraction. |
| **Algorithm Engineer** | ROS 1 `move_base` tuning, Costmap2D inflation layers, local trajectory planning (DWA / TEB). |
| **Simulation Engineer** | Unity 3D warehouse environment, ArticulationBody physics calibration, C# LiDAR raycasting. |
| **Mission Logic & QA Lead** | High-level mission state machine in Python (`rospy`), Moodle weekly progress reports, test validation. |
