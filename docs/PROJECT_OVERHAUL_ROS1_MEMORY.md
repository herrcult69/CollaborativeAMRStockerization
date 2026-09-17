# Project Memory & Architecture Blueprint: ROS 1 Noetic Docker Overhaul
## Course: 61CSE326 – Robot Obstacle-Avoidance Planning Software (WS2026)
- **Institution**: Vietnamese-German University (VGU) — Computer Science & Engineering
- **Supervisor**: Dr.-Ing. Quang Huan Dong (huan.dq@vgu.edu.vn)
- **Use Case**: **UC 6** (Autonomous Mobile Pallet/Tote Stacker AMR for Collaborative Order Replenishment)
- **Document Status**: Active Reference & Memory Architecture

---

## 1. Executive Summary of the Architectural Overhaul

Based on supervisory feedback and hardware-convenience constraints, the project underwent a complete strategic overhaul:
1. **Middleware Migration (ROS 2 Humble -> ROS 1 Noetic)**:
   - **Reason**: ROS 2 Nav2 introduces unnecessary complexity for academic evaluation (Behavior Tree XML schemas, fragile Lifecycle Nodes `unconfigured -> inactive -> active`, and DDS QoS multicast dropouts over student Wi-Fi).
   - **Advantage of ROS 1**: Battle-tested `move_base` navigation stack. Monolithic, deterministic parameter YAML files (`costmap_common_params.yaml`, `dwa_local_planner_params.yaml`, `teb_local_planner_params.yaml`). Clean XML launch files instead of convoluted Python launch scripts.
2. **Environment Migration (Linux Dual-Boot -> Windows Host + Dockerized ROS 1)**:
   - Eliminates disruptive dual-booting; preserves Windows battery management, laptop touch/pen features, and native Wi-Fi drivers.
   - **Unity runs natively on Windows**: Leverages direct DirectX 11/12 hardware GPU acceleration at 60+ FPS.
   - **ROS 1 runs inside a Docker Container**: Isolated, reproducible, and zero system pollution on the host.
   - **WSL2 as an Invisible Engine**: Docker Desktop on Windows automatically utilizes the WSL2 lightweight Linux micro-kernel in the background. The user never needs to manually operate inside WSL2.
3. **Visualization Demystification (Unity vs. RViz)**:
   - **Unity 3D**: *The Physical World / Simulator*. Simulates rigid-body physics, wheel friction, LiDAR laser raycasts, warehouse racks, and human pedestrian agents.
   - **RViz**: *The Robot's Mind / Algorithmic Diagnostic Display*. Draws what the navigation algorithm computes: 2D Costmaps (forbidden zones vs. inflation buffers), global A*/Dijkstra paths, local DWA/TEB avoidance trajectories, and raw laser scans.
   - **WSLg GUI**: Windows 11 WSLg automatically displays RViz as a native Windows desktop application window alongside Unity.
4. **Embedded Sim-to-Real Hardware Roadmap (Rosserial vs. Micro-ROS)**:
   - **Clarification**: `micro-ROS` is built for ROS 2 DDS. In **ROS 1**, the universal industry and academic standard for microcontrollers (STM32 Nucleo, ESP32, Arduino) is **`rosserial`** (`rosserial_python`).
   - `rosserial` requires under 20 lines of C++ on the microcontroller to stream `/odom` and receive `/cmd_vel` over USB Serial or Wi-Fi, without complex DDS agent overhead.
   - Windows 11 forwards USB microcontroller boards or physical 2D LiDARs (e.g., RPLiDAR A1) into Docker seamlessly using `usbipd-win`.

---

## 2. System Architecture & Communication Pipeline

```
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

### Standard Topic Interfaces:
- `/scan` (`sensor_msgs/LaserScan`): Published by Unity LiDAR -> Subscribed by `costmap_2d`.
- `/odom` (`nav_msgs/Odometry`): Published by Unity kinematics -> Subscribed by `move_base` / `robot_state_publisher`.
- `/cmd_vel` (`geometry_msgs/Twist`): Published by `move_base` local planner -> Subscribed by Unity wheel controller script.
- `/tf`, `/tf_static` (`tf2_msgs/TFMessage`): Coordinate transforms: `map` -> `odom` -> `base_footprint` -> `base_link` -> `laser_link`.
- `/move_base_simple/goal` (`geometry_msgs/PoseStamped`): Navigation target dispatched from RViz "2D Nav Goal" tool or mission coordinator.
- `/lift_cmd` (`std_msgs/Bool` or `std_srvs/SetBool`): Triggers vertical tote lift UP / DOWN in Unity.

---

## 3. Directory Layout Blueprint

```
Project_Project/
├── PROJECT_OVERHAUL_ROS1_MEMORY.md       # This core memory blueprint
├── DOCKER_ROS1_SETUP_GUIDE.md            # Turnkey step-by-step setup guide for Windows
├── docker/
│   ├── Dockerfile                        # ROS 1 Noetic + Navigation + TEB + ROS-TCP + Rosserial
│   ├── docker-compose.yml                # Port 10000 mapping, volume mounting, WSLg display
│   └── entrypoint.sh                     # Auto-sourcing setup scripts
├── catkin_ws/                            # ROS 1 Catkin Workspace (Mounted into Docker)
│   └── src/
│       ├── ROS-TCP-Endpoint/             # Unity-Robotics-Hub ROS 1 endpoint (cloned)
│       └── amr_navigation/               # Project custom ROS 1 package
│           ├── CMakeLists.txt
│           ├── package.xml
│           ├── config/
│           │   ├── costmap_common_params.yaml
│           │   ├── local_costmap_params.yaml
│           │   ├── global_costmap_params.yaml
│           │   ├── base_local_planner_params.yaml
│           │   ├── dwa_local_planner_params.yaml
│           │   └── teb_local_planner_params.yaml
│           ├── launch/
│           │   ├── unity_bridge.launch   # Starts ROS-TCP endpoint on 0.0.0.0:10000
│           │   ├── move_base.launch      # Starts move_base with costmaps & local planner
│           │   └── rviz.launch           # Opens RViz with preconfigured navigation displays
│           └── rviz/
│               └── amr_navigation.rviz   # Saved RViz configuration
└── unity_sim/                            # Unity 3D Project (On Windows)
```

---

## 4. Course Milestones Mapped to ROS 1

| Milestone | Session | Deliverable in ROS 1 | Validation Criterion |
|---|---|---|---|
| **Milestone 1** | Session 5 | Demo tutorial adaptation for UC6: AMR spawns in Unity, connects to Docker ROS 1 via TCP, navigates A -> B, executes lift. | Unity <-> Docker loopback verified, `/cmd_vel` drives AMR, lift triggers. |
| **Milestone 2** | Session 10 | Demo **Static Obstacle Avoidance**: 2D LiDAR raycast costmaps in `move_base`, avoiding blocked aisles. | Costmap clears/marks obstacles in RViz; DWA steers AMR around static clutter. |
| **Milestone 3** | Session 14 | Demo **Dynamic Obstacle Avoidance**: Real-time avoidance/yielding to walking pedestrians (Unity NavMesh). | TEB Local Planner dynamically modifies trajectory or yields without collision. |
| **Milestone 4** | Session 15 | Final Technical Report (IMRaD, <= 20 pages) & Oral Presentation/Defense. | Quantitative evaluation (clearance, speed, path efficiency, collision rate = 0%). |

---

## 5. Division of Responsibilities (4-Person Team)

1. **Lead / System Architect (YOU)**:
   - Docker containerization, `docker-compose` orchestration, and Windows-Docker networking.
   - ROS 1 Catkin workspace configuration, TF coordinate tree, and Unity-Robotics-Hub bridge.
   - Hardware abstraction and `rosserial` microcontroller integration.
2. **SWE Member 1 (Algorithms & Path Planning)**:
   - ROS 1 `move_base` parameter tuning (`costmap_2d`, inflation layers).
   - Local reactive planners: benchmarking DWA vs. TEB Local Planner for dynamic avoidance.
   - Quantitative collision rate, time-to-goal, and trajectory smoothness logging.
3. **SWE Member 2 (Unity 3D Simulation Lead)**:
   - Unity warehouse scene (storage racks, tote pallets, staging desks).
   - URDF import and ArticulationBody wheel physics calibration.
   - C# LiDAR raycast publisher (`sensor_msgs/LaserScan`) and motor subscriber (`geometry_msgs/Twist`).
   - Unity NavMesh pedestrian dynamic obstacle paths.
4. **Member 4 (Mission Logic & QA Lead)**:
   - Weekly Moodle status reports (preventing -5% late penalty!).
   - High-level mission coordinator state machine in Python (`rospy`): `IDLE -> NAV_TO_RACK -> DOCK -> LIFT -> NAV_TO_DESK -> LOWER -> RETREAT`.
   - Slide decks and demo video recording.
