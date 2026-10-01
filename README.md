# Autonomous Pallet Stacker AMR (Collaborative Warehouse Replenishment)

> **Course**: 61CSE326 – Robot Obstacle-Avoidance Planning Software (WS2026)  
> **Institution**: Vietnamese-German University (VGU) — Computer Science & Engineering  
> **Supervisor**: Dr.-Ing. Quang Huan Dong (`huan.dq@vgu.edu.vn`)  
> **Use Case**: UC 6 (Human-Robot Collaborative Logistics Runner)

---

## 1. System Architecture

The project connects a high-fidelity **Unity 3D** simulation running natively on Windows with a containerized **ROS 1 Noetic** brain running inside Docker:

```text
+--------------------------------------------------------------------------------+
|                             YOUR WINDOWS COMPUTER                              |
|                                                                                |
|   +------------------------------------+    +------------------------------+   |
|   |          UNITY 3D (Native)         |    |        VS CODE / EDIT        |   |
|   |  - Warehouse Scene                 |    |  - Edit ROS code on Windows  |   |
|   |  - Stacker AMR (ArticulationBody)  |    +--------------+---------------+   |
|   |  - PlanarOdometryPublisher (/odom) |                   | (Live Volume)     |
|   |  - StackerController (/cmd_vel)    |                   v                   |
|   +-----------------+------------------+    +------------------------------+   |
|                     ^                       |   DOCKER CONTAINER (ROS 1)   |   |
|     TCP Loopback    |                       |                              |   |
|   127.0.0.1:10000   |                       |  - ROS 1 Noetic Desktop Full |   |
|                     v                       |  - ros_tcp_endpoint (:10000) |   |
|   +-----------------+------------------+    |  - amr_navigation package    |   |
|   |  ros_tcp_endpoint Node (:10000)    |--->|  - go_to_point_3phase node   |   |
|   +------------------------------------+    +--------------+---------------+   |
|                                                            | (X11 / WSLg)      |
|   +------------------------------------+                   v                   |
|   |              RVIZ                  |<------------------+                   |
|   |  - Visualizes /odom & TF tree      |                                       |
|   +------------------------------------+                                       |
+--------------------------------------------------------------------------------+
```

---

## 2. Quickstart: How to Run the Project Right Now

To run the full simulation and watch the robot autonomously navigate to a point:

### Step 0: First-Time Setup (Build Once After Cloning)
When you clone or download this repository for the first time, your custom packages (`amr_navigation` and `ROS-TCP-Endpoint`) must be compiled so ROS registers them:

1. In PowerShell, enter the container:
   ```powershell
   docker exec -it ros1_amr_core bash
   ```
2. Run `catkin_make` to compile:
   ```bash
   cd /catkin_ws
   source /opt/ros/noetic/setup.bash
   catkin_make
   source devel/setup.bash
   ```
3. *(Optional, Recommended)* Make it permanent for all future terminals:
   ```bash
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
   ```

---

### Step 1: Start Docker & The Unity Bridge Server
1. Make sure **Docker** is open and running (Docker Desktop on Windows or Docker Engine on Ubuntu).
2. Open your terminal (**PowerShell** on Windows or **Bash** on Ubuntu) and navigate to the `docker/` folder:
   ```bash
   cd docker
   ```
3. Build and launch the container for your operating system:
   ```bash
   # Build the container image:
   docker compose -f docker-compose.[ubuntu/windows].yml build

   # Launch the container in the background:
   docker compose -f docker-compose.[ubuntu/windows].yml up -d

   # Enter the container terminal:
   docker exec -it ros1_amr_core bash
   ```
   *(Use `docker-compose.windows.yml` on Windows, or `docker-compose.ubuntu.yml` on native Ubuntu).*

4. Inside the container, start the Unity communication bridge:
   ```bash
   roslaunch amr_navigation unity_bridge.launch
   ```
   *(Keep this terminal open — it should log `Starting server on 0.0.0.0:10000`)*.

---

### Step 2: Start Unity Simulation
1. Open **Unity Hub** and launch the project **`amr_ware_house`** (Unity 2022.3 LTS).
2. In the Project panel, open: `Assets/Scenes/SampleScene.unity`.
3. Press **Play (`▶`)** at the top center of Unity.
4. Verify: In Terminal 1, you will see `Connection from 172.18.0.1 established!`.

> **Manual Drive Test (Optional)**: You can use `W`, `A`, `S`, `D` on your keyboard to manually steer the robot and test physics.

---

### Step 3: Run the Autonomous 3-Phase Precision Docking Controller (Milestone 1)
Open a **second PowerShell window** on Windows:
```powershell
docker exec -it ros1_amr_core bash
```

Inside this second terminal, launch the 3-phase precision docking controller with your target $(x, y)$ in meters and docking orientation in **degrees**:
```bash
roslaunch amr_navigation go_to_point_3phase.launch goal_x:=2.0 goal_y:=0.0 goal_yaw:=90
```

#### What happens:
1. **Phase 1 (`ALIGN_TO_GOAL`)**: AMR rotates in place to face target position $(2.0, 0.0)$.
2. **Phase 2 (`DRIVE_TO_GOAL`)**: Drives forward with smooth deceleration and active steering until within **5 cm** of the goal.
3. **Phase 3 (`ALIGN_FINAL_YAW`)**: Rotates in place until its forks are aligned with `goal_yaw` (e.g. $90^\circ$) within **3.0 degrees**.
4. **Phase 4 (`ARRIVED`)**: Publishes `lift_height` (0.25 m) to `/lift_cmd` to trigger the pallet mast lift!

---

### Step 4: (Optional) Open RViz to See the Robot's Mind
To inspect coordinate frames (`odom -> base_footprint`) and odometry vectors:

- **WSLg (Windows 11 Native GUI)**: Simply run:
  ```bash
  rviz
  ```
- **Windows with VcXsrv / XLaunch**: Ensure XLaunch is running with **"Disable access control"** checked, then run:
  ```bash
  export DISPLAY=host.docker.internal:0.0
  rviz
  ```

#### Configuring RViz:
1. Set **Fixed Frame** to `odom`.
2. Click **Add** (bottom left), select **TF** (shows coordinate frame axes).
3. Click **Add**, select **Odometry**, and set Topic to `/odom` (shows robot position arrows).

---

## 3. Directory Layout

```text
CollaborativeAMRStockerization/
├── README.md                            # This quickstart reference
├── docker/                              # Docker containerization files
│   ├── Dockerfile                       # ROS 1 Noetic + Navigation stack recipe
│   ├── docker-compose.windows.yml       # Docker Compose for Windows (WSLg / VcXsrv)
│   ├── docker-compose.ubuntu.yml        # Docker Compose for native Linux / Ubuntu
│   └── entrypoint.sh                    # Startup environment script
│
├── catkin_ws/                           # ROS 1 Workspace (Mirrored into Docker)
│   └── src/
│       ├── ROS-TCP-Endpoint/            # Official Unity-Robotics-Hub bridge
│       └── amr_navigation/              # Custom project package
│           ├── launch/
│           │   ├── unity_bridge.launch  # Starts TCP endpoint on port 10000
│           │   ├── go_to_point.launch   # Baseline 2-phase controller
│           │   └── go_to_point_3phase.launch # Upgraded 3-phase precision docking
│           └── scripts/
│               ├── go_to_point.py       # Baseline 2-phase node
│               ├── go_to_point_3phase.py # 3-phase precision docking + lift node
│               ├── test_go_to_point.py  # Baseline tests
│               └── test_go_to_point_3phase.py # 7 unit tests for 3-phase controller
│
└── docs/                                # Documentation & Guides
    ├── ARCHITECTURE.md                  # Unified system architecture & blueprint
    ├── TROUBLESHOOTING.md               # Turnkey setup & troubleshooting guide
    ├── warehouse_amr_concept.jpg        # Architectural concept diagram
    ├── lessons/                         # All 13 sequential lab tutorials (01-13)
    └── Reports/                         # Weekly PDF progress reports for Moodle
```

---

## 4. Useful Development Commands

```bash
# Check running ROS topics:
rostopic list

# Echo live odometry data:
rostopic echo /odom

# Echo motor velocity commands:
rostopic echo /cmd_vel

# Echo lift commands:
rostopic echo /lift_cmd

# Run the 3-phase controller unit tests offline:
python3 /catkin_ws/src/amr_navigation/scripts/test_go_to_point_3phase.py

# Recompile workspace after code changes:
cd /catkin_ws && catkin_make && source devel/setup.bash
```

---

## 5. Environment & Startup Mechanism (`entrypoint.sh`)

The container relies on [`docker/entrypoint.sh`](docker/entrypoint.sh) to prepare the ROS environment.

### What `entrypoint.sh` Does:
1. Sources the core ROS Noetic base: `source /opt/ros/noetic/setup.bash` (enables `rostopic`, `roslaunch`, `catkin_make`).
2. Checks if your custom workspace is built: `if [ -f /catkin_ws/devel/setup.bash ]; then source /catkin_ws/devel/setup.bash; fi` (enables `amr_navigation` and `ros_tcp_endpoint`).
3. Executes your command (`exec "$@"`).

### Pro-Tip: Entering Terminals
- When the container first boots via `docker compose up`, Docker automatically executes `/entrypoint.sh`.
- When you open a *new terminal* using `docker exec -it ros1_amr_core bash`, Docker starts a fresh shell.
- **Shortcut**: If any ROS command is ever not recognized in a new terminal, you can load the entire environment in **one single command**:
  ```bash
  source /entrypoint.sh
  ```
  *(Or add `source /opt/ros/noetic/setup.bash` and `source /catkin_ws/devel/setup.bash` to `/root/.bashrc` to load automatically).*