# 61CSE326 Project Briefing & Docker Workflow Guide (For Team Members)
## Course: 61CSE326 – Robot Obstacle-Avoidance Planning Software (WS2026)
- **Topic:** Autonomous Pallet/Tote Stacker AMR for Collaborative Order Replenishment (**UC 6**)
- **Supervisor:** Dr.-Ing. Quang Huan Dong (huan.dq@vgu.edu.vn)
- **Institution:** Vietnamese-German University (VGU) — Computer Science & Engineering

---

## 1. Important Update: Why We Pivoted to ROS 1 + Docker

If you were setting up native Ubuntu dual-boot and ROS 2 Humble, **stop and read this first**. We have upgraded our architecture to save everyone dozens of hours of headaches:

### 1.1 Why We Switched from ROS 2 to ROS 1 Noetic
- **Teacher's Feedback**: Our supervisor noted that ROS 2 Nav2 creates massive friction for a 1-semester project (fragile Behavior Tree XMLs, lifecycle node transitions `unconfigured -> inactive -> active`, and silent DDS QoS network dropouts where `/scan` or `/tf` randomly stop communicating).
- **The ROS 1 Advantage**: ROS 1 uses **`move_base`**, the industry's most stable, battle-tested navigation stack. Configuring obstacle avoidance is as simple as editing straightforward YAML files (`costmap_common_params.yaml`, `teb_local_planner_params.yaml`).

### 1.2 Why We Ditch Dual-Booting (Windows + Docker Method)
- **No More Re-booting**: You stay 100% on Windows. No broken Wi-Fi drivers, no battery drain on Linux, no GRUB bootloader issues.
- **Native Unity Performance**: Unity 2022.3 LTS runs natively on Windows with direct DirectX 11/12 GPU acceleration.
- **Dockerized ROS 1**: ROS 1 runs in an isolated container. You never have to install Ubuntu on your laptop.
- **Live Code Sync**: Your code lives on your regular Windows drive in `catkin_ws/`. You edit it in VS Code on Windows, and Docker sees the changes instantly.
- **RViz Works Out-of-the-Box**: On Windows 11, when you type `rviz` inside Docker, an RViz window pops up directly onto your Windows desktop via WSLg.

---

## 2. How the Complete System Works (The 30-Second Mental Model)

```
+-------------------------------------------------------------------------------+
|                            YOUR WINDOWS 11 LAPTOP                             |
|                                                                               |
|   +------------------------------------+   +------------------------------+   |
|   |         UNITY 3D (Native)          |   |        VS CODE / IDE         |   |
|   |  - Warehouse 3D environment        |   |  - Edit ROS configs / C#     |   |
|   |  - Robot physics (wheels/lift)     |   |    directly on Windows drive |   |
|   |  - Virtual 2D LiDAR raycasts       |   +--------------+---------------+   |
|   +-----------------+------------------+                  | (Live Mounted)    |
|                     ^                                     v                   |
|      TCP Loopback   |                      +------------------------------+   |
|    127.0.0.1:10000  |                      |   DOCKER CONTAINER (ROS 1)   |   |
|                     v                      |                              |   |
|   +-----------------+------------------+   |  - ROS 1 Noetic Core         |   |
|   |  ros_tcp_endpoint (Port 10000)     |-->|  - move_base (TEB / DWA)     |   |
|   +------------------------------------+   |  - 2D Obstacle Costmaps      |   |
|                                            +--------------+---------------+   |
|                                                           | (Native Windows   |
|   +------------------------------------+                  |  GUI via WSLg)    |
|   |       RVIZ (Diagnostic Window)     |<-----------------+                   |
|   |  - Shows what robot "thinks"       |                                      |
|   |  - Costmaps, path trajectories     |                                      |
|   +------------------------------------+                                      |
+-------------------------------------------------------------------------------+
```

### The Communication Loop:
1. **Unity** shoots virtual LiDAR laser rays and sends the numbers to ROS as `/scan`.
2. **ROS 1 (`move_base`)** calculates a 2D Costmap (red = obstacles, blue = safe space) and computes steering speeds.
3. **ROS 1** sends `/cmd_vel` back to Unity over port `10000`.
4. **Unity** rotates the robot's virtual wheels, moving it around the obstacle.
5. **RViz** lets you see the robot's internal mind (costmaps and trajectories) in real-time.

---

## 3. The 5-Minute Setup Guide for Teammates

Follow these 4 steps to get the entire robotics stack running on your machine:

### Step 1: Install Docker Desktop
1. Download and install [Docker Desktop for Windows](https://www.docker.com/products/docker-desktop/).
2. During installation, leave the checkbox **"Use the WSL 2 based engine"** checked.
3. Open the Docker Desktop app and make sure the icon in your taskbar shows the engine is running (green).

### Step 2: Clone the Project Repository
Open **PowerShell** on your Windows laptop:
```powershell
cd P:\AntiGravity\FinalYear\Project_Project   # (or your cloned folder path)
```

Make sure the Unity endpoint package is inside `catkin_ws/src`:
```powershell
cd catkin_ws\src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
```

### Step 3: Build & Start the ROS 1 Container
Inside PowerShell:
```powershell
cd ..\..\docker
docker compose build
docker compose up -d
```
*(The first build takes 2–3 minutes to download the ROS base and navigation packages. Future starts take less than 1 second!)*

### Step 4: Jump Inside and Test ROS
Open a terminal inside your container:
```powershell
docker exec -it ros1_amr_core bash
```

Now inside the container:
```bash
# Compile packages (only needed once or when adding new ROS packages)
catkin_make
source devel/setup.bash

# Test that RViz opens on your Windows desktop!
rviz
```
*(Close RViz with `Ctrl + C` when done).*

---

## 4. How to Connect Unity to Docker ROS 1

1. Open our Unity project on Windows.
2. In the top toolbar, click **Robotics** -> **ROS Settings**.
3. Set the fields:
   - **ROS IP Address**: `127.0.0.1` (or `localhost`)
   - **ROS Port**: `10000`
   - **Protocol**: **ROS 1**
4. Hit **Play** in Unity! Unity will talk directly to the container over port 10000.

---

## 5. Team Roles & Division of Labor (4 Members)

| Role | Member | Responsibilities |
|---|---|---|
| **System Architect / Lead** | **[Lead Name]** | Docker containerization, ROS-TCP bridge, Catkin workspace, hardware abstraction (`rosserial`), coordination. |
| **Algorithm Engineer** | **SWE 1** | Tuning `move_base` costmaps (`costmap_common_params.yaml`), testing **DWA vs. TEB Local Planner** for dynamic human obstacle avoidance. |
| **Simulation Engineer** | **SWE 2** | Unity 3D warehouse scene (racks, totes, staging desk), URDF robot physics (`ArticulationBody`), LiDAR raycaster script, NavMesh walking human obstacles. |
| **Mission Logic & QA** | **Member 4** | Weekly Moodle progress reports (mandatory to protect against -5% grade penalties!), high-level mission state machine in Python (`rospy`), logging test metrics. |

---

## 6. Project Milestones & Grading Deadlines

- **Milestone 1 (Session 5)**: AMR spawns in Unity warehouse, connects via ROS-TCP, navigates A -> B, executes tote lift.
- **Milestone 2 (Session 10)**: Static Obstacle Avoidance (navigating cluttered aisles, dropped boxes, costmap clearing).
- **Milestone 3 (Session 14)**: Dynamic Obstacle Avoidance (avoiding/yielding to walking humans using TEB local planner).
- **Session 15**: Final Report (IMRaD structure, <= 20 pages) & Live Demo / Technical Defense.

---

## 7. Daily Commands Cheat Sheet

```powershell
# 1. Start the container before working:
docker compose up -d

# 2. Enter the ROS terminal:
docker exec -it ros1_amr_core bash

# 3. Stop the container when you are done:
docker compose down
```
