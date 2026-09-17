# Project Kickoff: Autonomous Collaborative Warehouse Stacker AMR
## Course: 61CSE326 – Project Module (WS2026)
**Topic:** Robot Obstacle-Avoidance Planning Software  
**Supervising Lecturer:** Dr.-Ing. Quang Huan Dong (huan.dq@vgu.edu.vn)  
**Use Case Selection:** UC 6 (Collaborative Logistics & Order Replenishment)  
**Institution:** Vietnamese-German University (VGU) — Computer Science & Engineering  

---

## 1. Executive Summary & Topic Overview
During autonomous warehouse and factory operations, the motion of mobile robots is frequently obstructed by static structures (misplaced cargo, pillars, racking) and dynamic obstacles (warehouse personnel, moving carts, other autonomous robots). 

For **61CSE326**, our team is developing an **Autonomous Pallet/Tote Stacker AMR (Autonomous Mobile Robot)** designed for **human-robot collaborative logistics replenishment (Use Case 6)**. The system couples a differential-drive mobile base equipped with 2D LiDAR with a 1-degree-of-freedom (1-DOF) vertical lift mechanism to retrieve standardized totes/pallets from storage racks and ferry them to human-operated staging desks.

The core engineering objective is the design, implementation, and rigorous evaluation of **real-time static and dynamic obstacle-avoidance path planning software** in a high-fidelity 3D simulation environment (**Unity 6 / 2022.3 LTS on Windows 11** coupled to **ROS 1 Noetic in Docker** via Unity-Robotics-Hub), architected with a strict hardware abstraction layer for seamless transfer to embedded microcontrollers (**Nucleo STM32 / ESP32** running `rosserial`).

---

## 2. Industry Context, Status Quo & Problem Statement

### 2.1 Status Quo & The Practical Dilemma
In contemporary retail fulfillment centers and warehouse backrooms, manual material transport constitutes **over 50% of labor hours**. Workers spend hours walking miles daily pushing heavy rolling carts between bulk storage aisles and sorting/packing benches.
- **The Retail-Facing Scope Trap**: While robots can move reliably in 2D space, attempting fine-grained robotic retail shelf stocking (e.g., placing consumer packages neatly on customer-facing grocery shelves) requires multi-axis dexterous manipulation, soft grippers, tactile sensing, and costly 3D vision. This is technically fragile and commercially impractical within a 1-semester obstacle-avoidance course.
- **The Optimal Industry Solution**: Modern automated facilities (e.g., Amazon Robotics, Geek+, Hai Robotics) adopt a **collaborative model**:
  1. The **Robot** performs the heavy, repetitive hauling of standardized bulk bins/totes across long corridors.
  2. The **Human** remains at ergonomic consolidation workstations, performing dexterous inspection, packing, and individual shelf-stocking.

### 2.2 The Engineering Problem Statement
Deploying autonomous stacker robots into shared human-robot warehouse corridors introduces critical safety and routing challenges. Fixed global paths are constantly invalidated by transient obstacles (dropped boxes, misplaced equipment) and unpredictable dynamic agents (walking warehouse staff, crossing manual carts). 

The software system must guarantee:
1. **Collision-free navigation** under kinematic constraints of a loaded stacker AMR.
2. **Real-time reactive obstacle avoidance** without freezing or oscillating in narrow warehouse aisles.
3. **Graceful dynamic interaction**: decelerating, yielding, or dynamically replanning when human trajectories intersect the robot's operational envelope.

---

## 3. Project Scope: In-Scope vs. Out-of-Scope

### In-Scope:
- **Simulation**: High-fidelity Unity warehouse scene (storage aisles, pallets, totes, human staging bench, dynamic pedestrian agents driven by NavMesh).
- **Robot Model**: Differential-drive pallet stacker AMR with vertical lift mast represented in URDF and Unity ArticulationBody.
- **Communication Bridge**: Unity-Robotics-Hub (`com.unity.robotics.ros-tcp-connector` <-> `ros_tcp_endpoint`).
- **Obstacle Avoidance Stack**: ROS 1 `move_base`, Global Planner (A* / Dijkstra), and Local Reactive Planners (DWA / Timed-Elastic-Band - TEB Local Planner).
- **Sim-to-Real Architecture**: Standard ROS 1 message compliance (`/scan`, `/cmd_vel`, `/odom`, `/tf`) ensuring identical software execution when ported to STM32/ESP32 via `rosserial`.

### Out-of-Scope (Justified Exclusions):
- Multi-DOF articulated arm manipulation (deferred to human workers at staging area to avoid scope creep).
- High-payload physical hydraulics (desktop-scale prototype payload for subsequent hardware phase).
- SLAM mapping of unknown environments (known warehouse floorplan with predefined landmark costmaps).

---

## 4. Course Milestones & Deliverables Roadmap

| Milestone | Target Session | Week | Core Deliverable | Grading Weight |
|---|---|---|---|---|
| **Milestone 1** | Session 5 | Week 5 | Demo tutorial adaptation for UC6: Robot spawns in Unity warehouse, connects via ROS-TCP, navigates waypoint A -> B, triggers lift service. | 16.7% (Part of 50% MS) |
| **Milestone 2** | Session 10 | Week 10 | Demo **Static Obstacle Avoidance**: Navigating cluttered aisles with dropped boxes, misplaced pallets, and static equipment using 2D LiDAR costmaps. | 16.7% (Part of 50% MS) |
| **Milestone 3** | Session 14 | Week 14 | Demo **Dynamic Obstacle Avoidance**: Real-time collision avoidance with walking humans and crossing carts in shared aisles using TEB local planner. | 16.7% (Part of 50% MS) |
| **Final Report** | Session 15 | Week 15 | Comprehensive technical documentation following Moodle checklist and template (IMRaD, <= 20 pages). | 35% |
| **Oral Demo** | Session 15 | Week 15 | Live demonstration, video presentation, and technical defense. | 15% |
| **Weekly Reports** | Sessions 1–14 | Weekly | Concise weekly progress update submitted to Moodle. (Strict -5% penalty per late submission!) | Continuous |

---

## 5. Team Roles & Responsibility Matrix (4 Members)

| Role | Member | Primary Domain | Core Responsibilities |
|---|---|---|---|
| **Lead / System Architect** | **YOU** | Embedded, Systems & ROS 1 | - Docker containerization, Git repository management, and Windows-Docker integration.<br>- Catkin workspace configuration, TF transforms, and coordinate frames.<br>- Hardware abstraction layer and STM32 / ESP32 `rosserial` migration plan.<br>- Reviewing pull requests and integration testing. |
| **SWE Member 1** | **SWE 1** | Algorithms & Path Planning | - ROS 1 `move_base` integration and costmap parameter tuning.<br>- Local obstacle-avoidance planner benchmarking (DWA vs. TEB Local Planner).<br>- Dynamic obstacle trajectory tracking and velocity scaling logic.<br>- Benchmark evaluations (clearance distance, path smoothness). |
| **SWE Member 2** | **SWE 2** | Unity 3D Simulation | - Warehouse 3D environment setup (racks, totes, lighting, staging desks).<br>- URDF import and ArticulationBody physics configuration.<br>- C# Virtual LiDAR raycasting publisher (`sensor_msgs/LaserScan`).<br>- C# Wheel drive controller (`geometry_msgs/Twist` subscriber).<br>- Dynamic pedestrian obstacle setup using Unity NavMesh agents. |
| **Member 4** | **Member 4** | Mission Logic, QA & Documentation | - **Weekly Progress Reports** drafting for Moodle submissions (Grade Shield).<br>- High-level mission coordinator node in Python (`rospy`) (Shelf -> Pick -> Desk -> Drop state machine).<br>- Scenario test-case definitions and quantitative metric logging (collisions, time-to-goal).<br>- Slide deck and demo video recording preparation. |

---

## 6. Development Environment & Toolchain Setup

### 6.1 Workstation Architecture
1. **Host OS**: **Windows 11** (preserving native battery management, audio/touchscreen hardware, and GPU drivers).
2. **Robotics Middleware**: **ROS 1 Noetic Desktop Full** containerized via Docker Desktop (WSL2 backend).
3. **Simulation Engine**: **Unity 6 / 2022.3 LTS** running natively on Windows host with DirectX 11/12 GPU acceleration.
4. **Unity Packages**:
   - `com.unity.robotics.ros-tcp-connector` (ROS 1 protocol)
   - `com.unity.robotics.urdf-importer`
5. **Git Repository Setup**:
   - `catkin_ws/`: Catkin workspace containing ROS 1 packages (`amr_navigation`, `ROS-TCP-Endpoint`).
   - `unity_warehouse_amr/`: Unity simulation project.
   - `docker/`: Turnkey Dockerfile and compose configuration.
