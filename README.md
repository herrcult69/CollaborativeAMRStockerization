# Autonomous Pallet Stacker AMR (Collaborative Warehouse Replenishment)

> **Course**: 61CSE326 – Robot Obstacle-Avoidance Planning Software (WS2026)  
> **Institution**: Vietnamese-German University (VGU) — Computer Science & Engineering  
> **Supervisor**: Dr.-Ing. Quang Huan Dong (`huan.dq@vgu.edu.vn`)  
> **Use Case**: UC 6 (Human-Robot Collaborative Logistics Runner / Autonomous Pallet Stacker AMR)  
> **Robot Architecture Guide**: See [`docs/XSTACK_ROBOT_GUIDE.md`](docs/XSTACK_ROBOT_GUIDE.md) for full kinematics, coordinate frames (`drive_center`), footprint, and physical verification data.

---

## 1. System Architecture

The project connects a high-fidelity **Unity 3D** simulation running natively on Windows with a containerized **ROS 1 Noetic** brain running inside Docker via a high-performance TCP loopback socket on port `10000`:

```text
+--------------------------------------------------------------------------------+
|                             YOUR WINDOWS COMPUTER                              |
|                                                                                |
|   +------------------------------------+    +------------------------------+   |
|   |          UNITY 3D (Native)         |    |        VS CODE / EDIT        |   |
|   |  - Warehouse Scene (PhysX 3D)      |    |  - Edit ROS code on Windows  |   |
|   |  - Stacker AMR (ArticulationBody)  |    +--------------+---------------+   |
|   |  - PlanarOdometryPublisher (/odom) |                   | (Live Volume)     |
|   |  - StackerController (/cmd_vel)    |                   v                   |
|   +-----------------+------------------+    +------------------------------+   |
|                     ^                       |   DOCKER CONTAINER (ROS 1)   |   |
|     TCP Loopback    |                       |                              |   |
|   127.0.0.1:10000   |                       |  - ROS 1 Noetic Desktop Full |   |
|                     v                       |  - ros_tcp_endpoint (:10000) |   |
|   +-----------------+------------------+    |  - amr_navigation package    |   |
|   |  ros_tcp_endpoint Node (:10000)    |--->|  - move_base (DWA Planner)   |   |
|   +------------------------------------+    +--------------+---------------+   |
|                                                            | (X11 / WSLg)      |
|   +------------------------------------+                   v                   |
|   |              RVIZ                  |<------------------+                   |
|   |  - Visualizes /odom, TF, Costmaps  |                                       |
|   +------------------------------------+                                       |
+--------------------------------------------------------------------------------+
```

---

## 2. Quickstart: 3 Steps to Run

### Step 0: First-Time Setup (Build Workspace Once)
```powershell
# In PowerShell, enter the Docker container:
docker exec -it ros1_amr_core bash

# Build the Catkin workspace:
cd /catkin_ws && catkin_make && source devel/setup.bash
```

---

### Step 1: Start Docker Container
Make sure Docker Desktop is running. In your terminal:
```powershell
cd docker
docker compose -f docker-compose.windows.yml up -d
docker exec -it ros1_amr_core bash
```
*(On native Linux, use `docker-compose.ubuntu.yml`).*

---

### Step 2: Start Unity Simulation
1. Open **Unity Hub** and launch the project **`amr_ware_house`** (Unity 2022.3 LTS).
2. Open scene `Assets/Scenes/Milestone_X.unity`.
3. Press **Play (`▶`)** at the top center of Unity.

---

### Step 3: Launch Autonomous Navigation (1-Click Startup)
Inside your Docker terminal, launch the master navigation stack:
```bash
# Bundles Unity Bridge, Laser TF, AMCL Localization (auto-initialized at 0,0,0), and move_base:
roslaunch amr_navigation warehouse_autonav.launch launch_rviz:=true
```

To run the full autonomous pick-and-deliver replenishment mission automatically on startup:
```bash
roslaunch amr_navigation warehouse_autonav.launch launch_rviz:=true launch_mission:=true
```

---

## 3. Operations & Mission Launch Catalog

Detailed arguments, physical parameter tolerances, and copy-paste CLI examples are embedded as **XML documentation blocks inside each `.launch` file header** and cataloged in [`docs/SCRIPTS_AND_NODES.md`](docs/SCRIPTS_AND_NODES.md).

| Launch File | Command | Description |
| :--- | :--- | :--- |
| **Master 1-Click Autonav** | `roslaunch amr_navigation warehouse_autonav.launch` | Bundles bridge, mounts, AMCL with automated $(0,0,0)$ pose, `move_base`, and optional RViz / mission. |
| **Milestone 1 Mission** | `roslaunch amr_navigation milestone1.launch` | Full 11-step autonomous replenishment: rack staging $\to$ entry $\to$ lift $\to$ corridor $\to$ station deposit $\to$ extraction. |
| **Milestone 2 Mission** | `roslaunch amr_navigation milestone2.launch` | Hybrid mission: `move_base` DWA planner drives long corridors, precision controllers execute docking. |
| **Rack Docking Test** | `roslaunch amr_navigation test_dock_reverse.launch` | 5-phase rack reverse docking, uniform $20\text{ mm}$ lift, and extraction test (`pick_level:=bottom\|top\|shelf`). |
| **Station Deposit Test** | `roslaunch amr_navigation test_undock_reverse.launch` | 6-phase test approaching the $25\text{ cm}$ staging station at $(13.12, 0.0)$, depositing, and undocking. |
| **Waypoint Navigation** | `roslaunch amr_navigation move_to_point.launch` | 3-phase state machine driving chassis to any $(x, y, \text{yaw})$ with anti-stiction dither pulses. |
| **Raw Reverse Docking** | `roslaunch amr_navigation docking.launch` | Standalone straight reverse insertion with active heading lock and stiction clamp ($\ge 0.07\text{ m/s}$). |
| **Localization Stack** | `roslaunch amr_navigation warehouse_localization.launch` | Standalone `map_server` + AMCL with automated initial pose matching the Unity spawn point. |

> [!TIP]
> **Opposite-Side Shelves ($180^\circ$ across the aisle)**: The reverse docking algorithm is completely direction-agnostic. To pick from shelves on the South side of the aisle (facing North), simply pass `dock_yaw:=90.0` (e.g. `roslaunch amr_navigation test_dock_reverse.launch dock_yaw:=90.0 pallet_y:=-5.0`).

---

## 4. RViz Visualization

To inspect live coordinate frames, LiDAR costmaps, global plans, and AMCL particle clouds:

* **Windows 11 (WSLg)**: Run `rviz` directly inside your container terminal.
* **Windows 10 / VcXsrv (XLaunch)**: Ensure XLaunch is running with **"Disable access control"** checked, then run:
  ```bash
  export DISPLAY=host.docker.internal:0.0
  rviz -d /catkin_ws/src/amr_navigation/rviz/navigation.rviz
  ```

---

## 5. Automated Testing & Verification

All unit tests run directly inside the Docker container without requiring Unity to be running:

```bash
# Source ROS environment:
source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash

# Run ALL automated unit tests:
python3 -m unittest discover -s /catkin_ws/src/amr_navigation/tests -p "test_*.py"
```

### What the Test Suite Verifies (33 Automated Tests):
* **`test_move_to_point.py`**: State machine transitions (`ALIGN_TO_GOAL` $\to$ `DRIVE_TO_GOAL` $\to$ `ALIGN_FINAL_YAW`), angle normalization ($[-\pi, \pi]$), and drift recovery.
* **`test_docking.py`**: Stop pose calculations with lever arm offsets ($d_{\text{fork}} = 0.22\text{ m}$), minimum creep velocity clamps ($\ge 0.07\text{ m/s}$), and cardinal direction stability ($0^\circ, 90^\circ, 180^\circ, 270^\circ$).
* **`test_robot_profiles.py`**: Validates launch file parameter contracts, frame consistency (`drive_center`), footprint matrices, and map loading for both `xstack` and `legacy` profiles.
* **`test_unification_params.py`**: Validates unified parameter defaults across launch files (`station_x: 13.12`, `lift_speed: 0.08`, `rack_standoff: 1.2`, `standoff: 0.8`).

### Topic Introspection
```bash
rostopic list              # List active ROS topics
rostopic echo /odom        # Inspect live odometry (drive_center frame)
rostopic echo /cmd_vel     # Inspect commanded wheel velocities
rostopic echo /lift_cmd    # Inspect commanded mast elevation height
```

---

## 6. Documentation Index

| Document | Description |
| :--- | :--- |
| **[`docs/XSTACK_ROBOT_GUIDE.md`](docs/XSTACK_ROBOT_GUIDE.md)** | **Primary Robot Reference**: Kinematics, `drive_center` frame contract, collision envelope, PhysX verification data, and profile switching. |
| **[`docs/SCRIPTS_AND_NODES.md`](docs/SCRIPTS_AND_NODES.md)** | **Complete Software Reference**: Detailed architectural breakdown of every Python library, ROS node, mission coordinator, and launch file. |
| **[`docs/WAREHOUSE_METRICS_AND_CLEARANCE.md`](docs/WAREHOUSE_METRICS_AND_CLEARANCE.md)** | **Physical Clearances & Kinematics**: Warehouse dimensions, pallet geometries, and standardized Option 1 uniform $20\text{ mm}$ lift table. |
| **[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)** | High-level system architecture, loopback networking, and design rationale. |
| **[`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md)** | Diagnostic steps for Docker networking, VcXsrv display setup, and common errors. |
