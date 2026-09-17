# 61CSE326: Complete System Setup, Troubleshooting & Development Workflow Guide
## Autonomous Pallet/Tote Stacker AMR for Collaborative Order Replenishment (UC 6)
- **Course**: 61CSE326 – Robot Obstacle-Avoidance Planning Software (WS2026)
- **Institution**: Vietnamese-German University (VGU) — Computer Science & Engineering
- **Supervisor**: Dr.-Ing. Quang Huan Dong (huan.dq@vgu.edu.vn)
- **Document Purpose**: Complete reference manual for system setup, troubleshooting history, and standard development iteration workflows. (Optimized for team sharing and Google Docs import).

---

## 1. Executive Summary & Architectural Rationale

### 1.1 The Pivot: Why ROS 1 Noetic over ROS 2 Humble
During initial planning, our supervisor advised pivoting away from ROS 2 Nav2 to ROS 1 Noetic for this course module. The rationale is based on systems engineering trade-offs:
- **ROS 2 Nav2 Complexity**: Nav2 relies on Behavior Tree XML engines, lifecycle states (`unconfigured -> inactive -> active`), and DDS QoS policy matching (e.g. `Reliable` vs `Best Effort` mismatches silently drop sensor topics like `/scan` or `/tf`).
- **ROS 1 `move_base` Reliability**: ROS 1 navigation is deterministic, rock-solid, and battle-tested. It uses clean, monolithic YAML configuration files (`costmap_common_params.yaml`, `teb_local_planner_params.yaml`) and XML launch files, allowing 100% of engineering effort to focus on obstacle avoidance algorithms rather than middleware bugs.

### 1.2 The Environment: Why Docker on Windows over Dual-Booting
- **Hardware Preservation**: Eliminates the risk of bootloader (GRUB) corruption, preserves laptop battery management, audio/touchscreen drivers, and Wi-Fi stability.
- **Native Unity Performance**: Unity 6 / 2022.3 runs directly on Windows with hardware DirectX 11/12 GPU acceleration at 60+ FPS.
- **Zero Host Pollution**: All ROS 1 Noetic libraries, planners, and compilers live in a self-contained Docker container backed by WSL2.
- **Live Code Sync (Bind Mounts)**: The workspace folder (`catkin_ws/`) lives on the Windows drive and is mirrored into the container in real-time. Code edited in Windows (VS Code) takes effect immediately without rebuilding Docker images.

---

## 2. High-Level Architecture Diagram

```
+-------------------------------------------------------------------------------+
|                            WINDOWS 11 WORKSTATION                             |
|                                                                               |
|   +------------------------------------+   +------------------------------+   |
|   |         UNITY 6 (Native)           |   |       VS CODE / WINDOWS      |   |
|   |  - Warehouse 3D Environment        |   |  - Edits code directly on    |   |
|   |  - Pallet Stacker AMR Physics      |   |    P:\... drive              |   |
|   |  - 2D LiDAR Raycasting (/scan)     |   +--------------+---------------+   |
|   |  - ROS-TCP-Connector (C#)          |                  | (Live Volume)     |
|   +-----------------+------------------+                  v                   |
|                     ^                      +------------------------------+   |
|      TCP Loopback   |                      |   DOCKER CONTAINER (ROS 1)   |   |
|    127.0.0.1:10000  |                      |                              |   |
|                     v                      |  - ROS 1 Noetic Desktop Full |   |
|   +-----------------+------------------+   |  - ros_tcp_endpoint (:10000) |   |
|   |  ros_tcp_endpoint Node             |-->|  - move_base (TEB / DWA)     |   |
|   +------------------------------------+   |  - Costmap2D Obstacle Layers |   |
|                                            +--------------+---------------+   |
|                                                           | (X11 Display)     |
|   +------------------------------------+                  v                   |
|   |       RVIZ (via VcXsrv / WSLg)     |<-----------------+                   |
|   |  - 2D Costmap Visualization        |                                      |
|   |  - Planned Trajectory Inspection   |                                      |
|   +------------------------------------+                                      |
+-------------------------------------------------------------------------------+
```

---

## 3. Step-by-Step Installation & Turnkey Setup

### Step 3.1: Windows Prerequisites
1. **Windows 11** with WSL2 enabled.
2. **Docker Desktop for Windows**:
   - Download from docker.com and check **"Use the WSL 2 based engine"** during installation.
3. **VcXsrv Windows X Server** (For RViz GUI display):
   - Download from SourceForge and install with default settings.
4. **Unity 6 / Unity 2022.3 LTS**:
   - Installed via Unity Hub with 3D (Built-in or URP) template.

### Step 3.2: Workspace & Repository Layout
Clone the Unity ROS-TCP-Endpoint into your Catkin workspace source folder:
```powershell
# Open Windows PowerShell
cd P:\AntiGravity\FinalYear\Project_Project
mkdir -Force catkin_ws\src
cd catkin_ws\src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
```

### Step 3.3: Build & Launch Docker Container
From your project's `docker/` folder:
```powershell
cd P:\AntiGravity\FinalYear\Project_Project\docker
docker compose build
docker compose up -d
```

### Step 3.4: Compile Catkin Workspace
Enter the container:
```powershell
docker exec -it ros1_amr_core bash
```
Inside the container terminal:
```bash
catkin_make
source devel/setup.bash
echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
```

### Step 3.5: Unity ROS Connector Installation
1. Open your Unity project on Windows.
2. Go to **Window** -> **Package Manager** -> **`+`** -> **Add package from git URL...**
3. Add: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
4. In Unity top menu: **Robotics** -> **ROS Settings**:
   - Set **ROS IP Address**: `127.0.0.1`
   - Set **ROS Port**: `10000`
   - Set **Protocol**: `ROS 1`

---

## 4. Comprehensive Troubleshooting Log (Real Issues Encountered & Solved)

| Issue # | Symptom / Error Message | Root Cause | Permanent Solution |
|:---|:---|:---|:---|
| **1** | `fatal: Remote branch main-ros1 not found` | Older Unity tutorials referenced separate branch names; Unity has consolidated both ROS 1 and ROS 2 support into `main`. | Run `git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git` (defaults to `main`). |
| **2** | `bash: catkin_make: command not found` | A fresh non-login bash shell inside Docker had not sourced `/opt/ros/noetic/setup.bash`. | Added `source /opt/ros/noetic/setup.bash` to `/root/.bashrc` in the Dockerfile. |
| **3** | `/usr/bin/env: 'python\r': No such file or directory` | Git on Windows automatically converted line endings to CRLF (`\r\n`). Linux looked for interpreter `python\r`. | Installed `dos2unix` and `python-is-python3`, ran `dos2unix` on all `.py` scripts. |
| **4** | `qt.qpa.xcb: could not connect to display :0` | Linux GUI applications inside Docker require an active Windows X-display server to draw windows on desktop. | Launched **VcXsrv (XLaunch)** on Windows with **"Disable access control"** checked, and exported `DISPLAY=host.docker.internal:0.0`. |
| **5** | `OSError: [Errno 98] Address already in use` | A previous instance of `unity_bridge.launch` was already running and holding port 10000. | Run `rosnode list` to verify, or kill old processes with `killall -9 python3`. |
| **6** | Windows Security: `Part of this app has been blocked (Unity.SourceGenerators.dll)` | Windows 11 Smart App Control flagged fresh .NET source generators from Unity 6. | Opened Windows Security -> Protection History -> Clicked **Actions** -> **Allow on device**. |
| **7** | Pressing Play in Unity showed no connection / no HUD overlay | Unity's `ROSConnection` is lazy-loaded: it does not open a network socket if no GameObject in the scene requests a publisher or subscriber. | Created an empty GameObject `ROS_Manager` in the Hierarchy and attached a bridge script that registers topics. |

---

## 5. Development Iteration Workflow (Standard Operating Procedure)

Whenever you begin a development session, follow this 4-step sequence:

### Phase 1: Startup
1. Open **Docker Desktop** on Windows.
2. In PowerShell:
   ```powershell
   cd P:\AntiGravity\FinalYear\Project_Project\docker
   docker compose up -d
   ```
3. Enter the container and start the bridge:
   ```powershell
   docker exec -it ros1_amr_core bash
   roslaunch amr_navigation unity_bridge.launch
   ```

### Phase 2: Active Development & Code Editing
- **Where to edit**: Open VS Code directly on Windows pointing to `P:\AntiGravity\FinalYear\Project_Project`.
- **Python nodes & YAML configs**: Changes take effect **immediately** on next run (no compilation needed).
- **C++ nodes**: After editing, run `catkin_make` inside the Docker terminal.
- **Unity C# scripts**: Edit in VS Code; Unity auto-recompiles on window focus.

### Phase 3: Algorithm Testing & Visualization
1. Launch **XLaunch (VcXsrv)** on Windows (Multiple windows -> Start no client -> Disable access control -> Finish).
2. In a second Docker terminal (`docker exec -it ros1_amr_core bash`):
   ```bash
   rviz
   ```
3. Press **Play** in Unity.
4. Verify `/scan` point clouds, costmaps, and `/cmd_vel` velocity trajectories in RViz while the robot moves in Unity.

### Phase 4: Clean Shutdown
1. Press **Stop** (`■`) in Unity.
2. Press **`Ctrl + C`** in your Docker terminals to stop running nodes.
3. Exit the shell with `exit`.
4. Shut down the container:
   ```powershell
   docker compose down
   ```

---

## 6. Sim-to-Real Hardware Roadmap: Rosserial

When transitioning from Unity simulation to physical embedded hardware:
1. **The Microcontroller Protocol**:
   - In ROS 1, **`rosserial`** (`rosserial_python` + `rosserial_client`) is the universal standard for STM32 Nucleo and ESP32 boards.
   - It requires fewer than 20 lines of C++ code to publish wheel encoder ticks on `/odom` and subscribe to motor velocities on `/cmd_vel`.
2. **Windows USB Passthrough via `usbipd-win`**:
   - Install `usbipd-win` on Windows (`winget install dorssel.usbipd-win`).
   - Forward your plugged-in microcontroller or 2D LiDAR into Docker:
     ```powershell
     usbipd list
     usbipd attach --wsl --busid <bus-id>
     ```
   - The device appears inside Docker as `/dev/ttyUSB0` or `/dev/ttyACM0`, where `rosrun rosserial_python serial_node.py /dev/ttyACM0` connects it immediately!
