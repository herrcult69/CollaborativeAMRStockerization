# 61CSE326 Project & Meeting Context Briefing (For LLM / Chatbot Context)

## System Prompt / AI Persona Instructions for the Chatbot:
> You are an engineering assistant helping a 4th-year Computer Science & Engineering student at Vietnamese-German University (VGU) prepare a Weekly Progress Report and a 4-Slide Presentation for the course **61CSE326: Robot Obstacle-Avoidance Planning Software** supervised by **Dr.-Ing. Quang Huan Dong**. Maintain a professional, concise, academic engineering tone (IMRaD structure, systems-engineering mindset).

---

## 1. Course & Administrative Metadata
- **Course**: 61CSE326 Project Module – WS2026
- **Topic**: Robot Obstacle-Avoidance Planning Software
- **Supervisor**: Dr.-Ing. Quang Huan Dong (huan.dq@vgu.edu.vn)
- **Institution**: Vietnamese-German University (VGU) – Computer Science Study Program
- **Selected Use Case**: **UC 6** (*A mobile robot navigates to storage locations, retrieves specified items from shelves, and transports them to designated drop-off points*).
- **Meeting Format**: Strict 10-minute slot in meeting room Cluster 3-4-B3-404.
- **Reporting Constraints**:
  - Weekly progress reports submitted to Moodle are mandatory (**-5% total course grade penalty for each missed or late submission**).
  - Final report must follow the **IMRaD structure** (Introduction, Methods, Results, Discussion) and APA referencing (<= 20 pages).
  - Course Milestones:
    - **Milestone 1 (Session 5)**: Demo tutorial adaptation for selected use case (A -> B navigation + lift).
    - **Milestone 2 (Session 10)**: Demo Static Obstacle Avoidance (cluttered aisles, LiDAR costmap).
    - **Milestone 3 (Session 14)**: Demo Dynamic Obstacle Avoidance (walking humans, crossing carts).
    - **Session 15**: Final Report (35%) & Oral Presentation/Demo (15%).

---

## 2. Project Concept & Defined Scope

### 2.1 The Concept: Human-Robot Collaborative Logistics Runner (Autonomous Pallet/Tote Stacker AMR)
Instead of attempting complex robotic manipulation at the retail shelf (which requires multi-axis dexterous arms and costly vision that is not graded in this course), the project focuses on **back-of-house logistics**:
1. **The Robot (Autonomous Pallet Stacker AMR)**:
   - Uses differential-drive kinematics for high maneuverability.
   - Equipped with a **1-DOF vertical prismatic lift mast** to raise and lower standardized warehouse totes/pallets.
   - Navigates bulk storage aisles, docks under a tote rack, lifts the tote, and transports it across shared corridors.
2. **The Human Worker**:
   - Stationed at an ergonomic consolidation/staging workbench.
   - Receives the tote from the AMR and performs the dexterous, fine-grained inspection, sorting, and shelf-stocking.
3. **Why this scope was chosen (Engineering Rationale)**:
   - **Maximizes Obstacle Avoidance Focus**: 80% of software development remains on navigation, path planning, and obstacle avoidance (the core syllabus requirement).
   - **Authentic Justification for Dynamic Avoidance**: The shared corridor between storage racks and human staging desks naturally introduces moving personnel and manual carts that the robot must dynamically detect, avoid, or yield to.

---

## 3. Technology Stack & Architecture (ROS 1 Noetic + Docker on Windows)

- **Simulation Engine**: **Unity (Unity 6 / 2022.3 LTS on Windows 11 Host)**
  - Utilizes Unity-Robotics-Hub (`com.unity.robotics.ros-tcp-connector` ROS 1 protocol, URDF Importer).
  - Simulates differential-drive physics via ArticulationBody.
  - Simulates 2D LiDAR using raycasts and publishes `sensor_msgs/LaserScan`.
  - Simulates dynamic obstacles (walking humans) using Unity NavMesh agents.
- **Robotics Middleware**: **ROS 1 Noetic Ninjemys (Dockerized on Windows 11 via WSL2 Backend)**
  - Navigation Stack: **`move_base`** (Global Planner: NavFn / GlobalPlanner, Local Planners: DWA / TEB Local Planner).
  - Visualization / Diagnostics: **RViz** displayed on Windows desktop via VcXsrv X11 server.
  - Standard Topics: `/cmd_vel` (`geometry_msgs/Twist`), `/scan` (`sensor_msgs/LaserScan`), `/odom` (`nav_msgs/Odometry`), `/tf`, `/lift_cmd`.
- **Hardware Sim-to-Real Readiness**:
  - The software layer strictly adheres to standard ROS 1 topics.
  - Physical microcontroller (**Nucleo STM32 / ESP32**) communicates with ROS 1 via **`rosserial`** (`rosserial_python`) over USB serial (forwarded via `usbipd-win` on Windows) or TCP Wi-Fi.

---

## 4. Team Structure & Division of Labor (4 Members)

| Role | Member | Responsibilities |
|---|---|---|
| **Team Lead / System Architect** | **[Name 1]** | Dockerized ROS 1 architecture, Catkin workspace configuration, Git repository management, coordinate frames (TF2), and hardware abstraction (`rosserial`). |
| **Algorithm Engineer** | **[Name 2]** | ROS 1 `move_base` integration, Costmap2D parameter tuning, local reactive path planning (DWA / TEB), and dynamic obstacle collision avoidance. |
| **Simulation Engineer** | **[Name 3]** | Unity 3D warehouse environment setup, URDF robot importing, ArticulationBody physics calibration, C# LiDAR raycast publisher, and NavMesh dynamic human obstacles. |
| **Mission Logic & QA Lead** | **[Name 4]** | Weekly status reports drafting for Moodle, high-level mission coordinator state machine in Python (`rospy`), test scenario definitions, and evaluation metrics logging. |

---

## 5. Week 1 Status & Immediate Sprint Goals

### What Was Accomplished in Week 1:
1. **Topic Definition & Scope Finalization**: Selected UC6 and defined the Collaborative Stacker AMR concept to maximize focus on obstacle avoidance.
2. **Team Organization**: Assigned explicit roles and boundaries for all 4 members.
3. **Environment Toolchain Overhaul**:
   - Host setup: Windows 11 host with Docker Desktop running containerized ROS 1 Noetic Desktop Full with `move_base` and `teb_local_planner`.
   - Simulation toolchain: Unity 6 installed on Windows host with `com.unity.robotics.ros-tcp-connector` and URDF Importer.
   - Communication Loopback: Fully tested and verified bidirectional communication between Unity and Docker ROS 1 over port 10000 (`/unity_heartbeat` and `/cmd_vel`).
4. **Project Infrastructure**: Initialized team Git repository, development guidelines, and weekly report template.

### Sprint Plan for Week 2 (Toward Milestone 1 at Session 5):
- Import baseline stacker robot URDF model into Unity using URDF Importer.
- Connect Unity ArticulationBody wheel motors to incoming `/cmd_vel` messages from ROS 1.
- Implement virtual 2D LiDAR raycast publisher (`sensor_msgs/LaserScan`) to `/scan`.
- Lay out the baseline warehouse storage racks and staging workstation in Unity.

---

## 6. Structure of the 4 Slides for Supervisor Meeting

1. **Slide 1: Title & Team Organization**: Course title, project name, supervisor, and the 4-person role division matrix.
2. **Slide 2: Concept & Defined Scope (Core Slide)**: Concept diagram (`warehouse_amr_concept.jpg`) showing storage rack, Stacker AMR with lift mast, and human staging desk; explanation of why retail shelf manipulation was eliminated in favor of 80% focus on static/dynamic obstacle avoidance in shared corridors.
3. **Slide 3: Toolchain & Sim-to-Real Readiness**: High-level tools (Unity 6 + Unity-Robotics-Hub on Windows, Dockerized ROS 1 Noetic + `move_base`, `rosserial` STM32/ESP32 hardware roadmap).
4. **Slide 4: Week 1 Deliverables & Sprint 1 Roadmap**: Completed setup tasks, verified Unity <-> Docker bidirectional loopback, and clear next steps leading up to Milestone 1 (Session 5).
