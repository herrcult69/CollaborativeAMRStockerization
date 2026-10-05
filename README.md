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

### Step 3: Run Autonomous Navigation, Docking & Missions
Open a **second PowerShell window** on Windows:
```powershell
docker exec -it ros1_amr_core bash
```

Inside this second terminal, you can run any of the navigation, precision docking, or full replenishment missions:

---

#### A. Point-to-Point Waypoint Navigation (`move_to_point.launch`)
Navigates the robot chassis to any `(x, y)` coordinate and aligns to the requested `yaw` angle using a 3-phase state machine (`ALIGN_TO_GOAL` $\to$ `DRIVE_TO_GOAL` $\to$ `ALIGN_FINAL_YAW` $\to$ `ARRIVED`) with anti-stiction dither and closed-loop position drift recovery.

```bash
# Default target (x=2.0m, y=0.0m, yaw=0.0 deg):
roslaunch amr_navigation move_to_point.launch

# Custom target waypoint with specified orientation and speed:
roslaunch amr_navigation move_to_point.launch goal_x:=4.0 goal_y:=3.85 goal_yaw:=270.0 max_linear:=0.45 pos_tolerance:=0.05
```

**Configurable Arguments:**
| Argument | Default | Type | Description |
|---|---|---|---|
| `goal_x` | `2.0` | float | Target X coordinate in meters (`odom` / `drive_center` frame) |
| `goal_y` | `0.0` | float | Target Y coordinate in meters (`odom` / `drive_center` frame) |
| `goal_yaw` | `0.0` | float | Final heading in degrees (e.g. `0.0`, `90.0`, `180.0`, `270.0`) |
| `pos_tolerance` | `0.05` | float | Linear arrival tolerance in meters ($5\text{ cm}$) |
| `yaw_tolerance` | `4.0` | float | Final orientation accuracy tolerance in degrees |
| `max_linear` | `0.35` | float | Maximum forward cruising velocity ($\text{m/s}$) |
| `max_angular` | `0.65` | float | Maximum turning angular velocity ($\text{rad/s}$) |

---

#### B. Pallet Rack Reverse Docking & Extraction Test (`test_dock_reverse.launch`)
Executes an isolated 5-phase reverse docking, lift, and extraction test from a pallet storage rack:
1. Navigates to pre-dock standoff ($0.8\text{ m}$ in front of dock target)
2. Positions mast to entry height ($-0.020\text{ m}$ for bottom; $+0.240\text{ m}$ for top; $+0.640\text{ m}$ for shelf)
3. Reverses straight into pallet cavity (`dock_reverse()`) with locked heading and stiction clamp ($\ge 0.07\text{ m/s}$)
4. Elevates mast to pick height ($+0.010\text{ m}$ for bottom; $+0.290\text{ m}$ for top; $+0.685\text{ m}$ for shelf) — standardizing a uniform **$20\text{ mm}$ pallet lift** across all tiers
5. Pulls straight forward (`undock()`) to extract pallet into the aisle, then positions mast to open-aisle transit height ($0.15\text{ m}$ for bottom, $0.29\text{ m}$ for top, $0.28\text{ m}$ for shelf)

```bash
# Run default rack docking test at (4.0, 5.0) facing 270 deg (bottom level):
roslaunch amr_navigation test_dock_reverse.launch

# Pick upper pallet (second height) of a double stack:
roslaunch amr_navigation test_dock_reverse.launch pick_level:=top

# Pick pallet from second-tier rack shelf:
roslaunch amr_navigation test_dock_reverse.launch pick_level:=shelf

# Custom pallet location, penetration depth, and creep speed:
roslaunch amr_navigation test_dock_reverse.launch pallet_x:=4.0 pallet_y:=5.0 dock_yaw:=270.0 fork_offset:=0.22 dock_speed:=0.12
```

**Configurable Arguments:**
| Argument | Default | Type | Description |
|---|---|---|---|
| `pick_level` | `bottom` | string | Target pallet height: `"bottom"` / `"1"` (ground double stack), `"top"` / `"2"` (upper pallet), or `"shelf"` / `"3"` (rack tier 2) |
| `pallet_x` | `4.0` | float | Target pallet cavity center X coordinate (meters) |
| `pallet_y` | `5.0` | float | Target pallet cavity center Y coordinate (meters) |
| `dock_yaw` | `270.0` | float | Locked reverse docking orientation in degrees |
| `fork_offset` | `0.22` | float | Distance from `drive_center` to fork reference point ($0.22\text{ m} = 85\text{--}90\%$ tine penetration) |
| `dock_speed` | `0.12` | float | Maximum reverse creep speed ($\text{m/s}$, minimum clamp $\ge 0.07\text{ m/s}$ prevents stiction stall) |

> **Standalone Docking Node (`docking.launch`)**: For testing raw reverse docking without navigation or mast lift:
> ```bash
> roslaunch amr_navigation docking.launch dock_x:=4.0 dock_y:=5.0 dock_yaw:=270.0 fork_offset:=0.22 dock_speed:=0.12
> ```

---

#### C. Staging Station Reverse Dock & Undock Extraction Test (`test_undock_reverse.launch`)
Executes an isolated 6-phase test of approaching the $25\text{ cm}$ staging station at $(13.0, 0.0)$, lowering the mast to deposit height, and performing forward undock extraction (even if no pallet has been picked up yet):
1. Navigates directly to station pre-dock standoff pose ($11.98\text{ m}, 0.0\text{ m}$, yaw $180.0^\circ$)
2. Elevates mast to safe transit height ($0.28\text{ m}$, clears $0.25\text{ m}$ station by $40\text{ mm}$)
3. Reverses over the $25\text{ cm}$ staging station to stop pose ($12.78\text{ m}, 0.0\text{ m}$)
4. Lowers mast to deposit height ($0.21\text{ m}$): forks float $10\text{ mm}$ above station surface
5. Undocks forward to standoff pose ($11.98\text{ m}$) with locked yaw $180.0^\circ$, extracting forks cleanly
6. Lowers mast to idle/transit height ($0.00\text{ m}$)

```bash
# Run isolated staging station undock test at (13.0, 0.0):
roslaunch amr_navigation test_undock_reverse.launch

# Custom station position, standoff, and speeds:
roslaunch amr_navigation test_undock_reverse.launch station_x:=13.0 station_y:=0.0 station_yaw:=180.0 standoff:=0.8 transit_speed:=0.45 dock_speed:=0.12 lift_speed:=0.06
```

**Configurable Arguments:**
| Argument | Default | Type | Description |
|---|---|---|---|
| `station_x` | `13.0` | float | Staging station / workbench dock cavity X (meters) |
| `station_y` | `0.0` | float | Staging station / workbench dock cavity Y (meters) |
| `station_yaw` | `180.0` | float | Station approach heading in degrees (robot faces $180^\circ$ West, forks point East) |
| `fork_offset` | `0.22` | float | Tine penetration offset from `drive_center` ($0.22\text{ m}$) |
| `standoff` | `0.8` | float | Frontal standoff distance ($\text{m}$) before dock stop pose (yields $\sim 1.0\text{ m}$ clearance to station) |
| `transit_lift_height`| `0.28` | float | Mast height ($\text{m}$) during approach ($40\text{ mm}$ clearance above $0.25\text{ m}$ station) |
| `deposit_lift_height`| `0.21` | float | Mast height ($\text{m}$) for deposit (pallet settles on deck, forks float free) |
| `dock_speed` | `0.12` | float | Docking reverse creep speed limit ($\text{m/s}$) |
| `transit_speed` | `0.45` | float | Linear speed limit to station staging standoff ($\text{m/s}$) |
| `lift_speed` | `0.06` | float | Mast elevation vertical speed limit ($\text{m/s}$) |

---

#### D. Full Milestone 1 Replenishment Mission (`milestone1.launch`)
Executes the complete, end-to-end autonomous warehouse workflow combining rack pickup, orthogonal corridor transit, and staging station deposit:
1. **Rack Approach & Staging**: Navigates to rack pre-dock standoff $(4.00, 3.98, 270.0^\circ)$
2. **Mast Lowering/Positioning**: Positions mast to entry height (`-0.020m` for bottom pallet; `+0.240m` for upper pallet; `+0.640m` for shelf pallet)
3. **Rack Docking**: Reverses straight into pallet cavity at $(4.0, 5.0)$ -> `drive_center` stops at $(4.0, 4.78)$
4. **Under-Rack Lift**: Elevates mast to pick height (`+0.010m` for bottom; `+0.290m` for upper; `+0.685m` for shelf) — lifting every pallet by an identical **$20\text{ mm}$** off its support, preserving safe $12\text{ mm}$ headroom below under-rack beams and safe clearance below upper shelf ceilings
5. **Rack Extraction**: Undocks forward out of rack back to $(4.0, 3.98)$
6. **Transit Elevation**: Elevates / positions mast to transit height ($0.28\text{--}0.29\text{ m}$, providing $40\text{--}50\text{ mm}$ clearance over $0.25\text{ m}$ station)
7. **Corridor Navigation (Manhattan)**: Navigates to junction $(4.0, 0.0)$, then down corridor highway to station staging pose $(11.98, 0.0)$ facing $180.0^\circ$
8. **Station Reverse Docking**: Reverses over $25\text{ cm}$ station to stop pose $(12.78, 0.0)$
9. **Station Deposit**: Lowers mast to $0.21\text{ m}$ (pallet settles onto station at $0.24\text{ m}$, forks sink into cavity)
10. **Station Undock Extraction**: Pulls forward to standoff $(11.98, 0.0)$, leaving pallet resting on the station
11. **Mast Home**: Lowers mast to $0.00\text{ m}$ (idle height) — Mission Complete!

```bash
# Run full end-to-end replenishment mission (bottom pallet):
roslaunch amr_navigation milestone1.launch

# Pick upper pallet (second height in double stack) and deliver to station:
roslaunch amr_navigation milestone1.launch pick_level:=top

# Pick from second-tier rack shelf:
roslaunch amr_navigation milestone1.launch pick_level:=shelf

# Pick from a different rack bay location (e.g. bay at x=6.0, y=5.0):
roslaunch amr_navigation milestone1.launch pallet_x:=6.0 pallet_y:=5.0 dock_yaw:=270.0

# Skip rack pickup if AMR already has pallet loaded:
roslaunch amr_navigation milestone1.launch skip_pickup:=true

# High-speed demo run:
roslaunch amr_navigation milestone1.launch transit_speed:=0.75 dock_speed:=0.15 lift_speed:=0.10
```

#### Pallet Pickup Level Kinematic Specifications (Standardized Option 1: Uniform 20mm Lift)

| `pick_level` | Entry Mast $q$ | Fork Tines at Entry ($Z_{\text{bottom}} \to Z_{\text{top}}$) | Pick Mast $q$ | Pick $Z_{\text{top}}$ | Net Pallet Lift | Overhead Clearance |
| :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| **`bottom`** / `1` | **$-0.020\text{ m}$** | $30\text{ mm} \to 50\text{ mm}$ | **$+0.010\text{ m}$** | $0.080\text{ m}$ | **$+20\text{ mm}$ off floor** | $12\text{ mm}$ headroom below $0.592\text{ m}$ rack beam |
| **`top`** / `2` | **$+0.240\text{ m}$** | $290\text{ mm} \to 310\text{ mm}$ | **$+0.290\text{ m}$** | $0.360\text{ m}$ | **$+20\text{ mm}$ off lower box** | $12\text{ mm}$ headroom below $0.592\text{ m}$ rack beam |
| **`shelf`** / `3` | **$+0.640\text{ m}$** | $690\text{ mm} \to 710\text{ mm}$ | **$+0.685\text{ m}$** | $0.755\text{ m}$ | **$+20\text{ mm}$ off shelf** | Safe clearance below upper shelf ceiling |

*(Fork tine thickness = $20\text{ mm}$; Kinematic relation: $Z_{\text{fork\_bottom}} = 0.050 + q\text{ m}$, $Z_{\text{fork\_top}} = 0.070 + q\text{ m}$).*

**Configurable Arguments:**
| Argument | Default | Type | Description |
|---|---|---|---|
| `pick_level` | `bottom` | string | Target pickup level: `"bottom"` / `"1"` (ground double stack), `"top"` / `"2"` (upper pallet), or `"shelf"` / `"3"` (tier 2 shelf) |
| `skip_pickup` | `false` | bool | If `true`, assumes pallet is already on forks and starts directly at transit phase |
| `pallet_x` | `4.0` | float | Rack pallet pickup cavity X coordinate (meters) |
| `pallet_y` | `5.0` | float | Rack pallet pickup cavity Y coordinate (meters) |
| `dock_yaw` | `270.0` | float | Heading for rack pickup (degrees) |
| `fork_offset` | `0.22` | float | Tine penetration offset from `drive_center` ($0.22\text{ m}$) |
| `corridor_x` | `4.0` | float | Manhattan transit junction waypoint X (meters) |
| `corridor_y` | `0.0` | float | Manhattan transit junction waypoint Y (meters) |
| `station_x` | `13.0` | float | Staging station / workbench dock cavity X (meters) |
| `station_y` | `0.0` | float | Staging station / workbench dock cavity Y (meters) |
| `station_yaw` | `180.0` | float | Station approach heading in degrees (robot faces $180^\circ$ West, forks point East) |
| `standoff` | `0.8` | float | Frontal standoff distance ($\text{m}$) before dock stop pose |
| `transit_lift_height`| `0.28` | float | Mast height ($\text{m}$) during transit ($40\text{ mm}$ clearance above $0.25\text{ m}$ station) |
| `deposit_lift_height`| `0.21` | float | Mast height ($\text{m}$) for deposit (pallet settles on deck, forks float free) |
| `dock_speed` | `0.12` | float | Docking reverse creep speed limit ($\text{m/s}$) |
| `transit_speed` | `0.45` | float | Corridor cruising linear speed limit ($\text{m/s}$) |
| `lift_speed` | `0.08` | float | Mast elevation vertical speed limit ($\text{m/s}$) |

---

### Step 4: (Optional) Open RViz to See the Robot's Mind
To inspect coordinate frames (`odom -> drive_center`) and odometry vectors:

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
├── README.md                            # Quickstart & operational reference
├── docker/                              # Docker containerization files
│   ├── Dockerfile                       # ROS 1 Noetic + Navigation stack recipe
│   ├── docker-compose.windows.yml       # Docker Compose for Windows (WSLg / VcXsrv)
│   ├── docker-compose.ubuntu.yml        # Docker Compose for native Linux / Ubuntu
│   └── entrypoint.sh                    # Startup environment script
│
├── catkin_ws/                           # ROS 1 Catkin Workspace (Live-mounted)
│   └── src/
│       ├── ROS-TCP-Endpoint/            # Official Unity-Robotics-Hub bridge
│       └── amr_navigation/              # Custom AMR Navigation Package
│           ├── setup.py                 # Standard Catkin Python package setup
│           ├── CMakeLists.txt           # Build configuration
│           ├── package.xml              # Package dependencies
│           ├── src/amr_navigation/      # Reusable Python Library Modules
│           │   ├── __init__.py          # Package exports
│           │   ├── move_to_point.py     # 3-Phase Waypoint Navigation Controller
│           │   ├── docking.py           # Precision Reverse Docking & Extraction Module
│           │   └── lift.py              # Velocity-Ramped Mast Lift Controller
│           ├── scripts/                 # Executable ROS Nodes & Missions
│           │   ├── move_to_point_node.py# Point-to-point waypoint CLI node
│           │   ├── docking_node.py      # Standalone reverse docking CLI node
│           │   ├── lift_node.py         # Pallet mast elevation CLI node
│           │   ├── milestone1_mission.py# Multi-waypoint mission coordinator
│           │   ├── test_dock_reverse.py # Pallet staging, reverse docking & rack extraction test
│           │   └── test_undock_reverse.py # Reverse undocking & 25cm station delivery test
│           ├── launch/                  # Launch Files
│           │   ├── unity_bridge.launch  # Starts TCP endpoint on port 10000
│           │   ├── move_to_point.launch # Runs move_to_point_node with custom parameters
│           │   ├── docking.launch       # Runs standalone docking_node
│           │   ├── test_dock_reverse.launch # Runs pallet staging & reverse docking test
│           │   ├── test_undock_reverse.launch # Runs reverse undocking & delivery mission
│           │   └── milestone1.launch    # Runs Milestone 1 autonomous mission
│           └── tests/                   # Automated Unit Test Suite
│               ├── test_move_to_point.py# Unit tests for waypoint navigation math & state transitions
│               └── test_docking.py      # Unit tests for docking kinematics, offsets & clearances
│
└── docs/                                # Documentation & Guides
    ├── SCRIPTS_AND_NODES.md             # Complete reference for every script, node, launch file & API
    ├── WAREHOUSE_METRICS_AND_CLEARANCE.md # Pallet, rack, double-stack headroom, station & AMR lift metrics
    ├── ARCHITECTURE.md                  # Unified system architecture & blueprint
    ├── TROUBLESHOOTING.md               # Turnkey setup & troubleshooting guide
    ├── lessons/                         # All 13 sequential lab tutorials (01-13)
    └── Reports/                         # Weekly PDF progress reports for Moodle
```

---

## 4. Useful Development Commands & Automated Testing

### Running the Test Suite
All unit tests can be executed directly inside the Docker container:

```bash
# Make sure ROS environment is sourced:
source /opt/ros/noetic/setup.bash

# Run ALL unit tests in the package:
python3 -m unittest discover -s /catkin_ws/src/amr_navigation/tests -p "test_*.py"

# Or run individual test suites:
# 1. Point-to-Point controller math, angle normalization & transitions:
python3 /catkin_ws/src/amr_navigation/tests/test_move_to_point.py

# 2. Docking kinematics, lever arm offsets, wheel stiction limits & station dropoff clearances:
python3 /catkin_ws/src/amr_navigation/tests/test_docking.py
```

#### What the tests verify:
- **`test_move_to_point.py`**: Validates angle normalization ($[-\pi, \pi]$), degree-to-radian conversions, in-place rotation vs. forward drive thresholds, state progression (`ALIGN_TO_GOAL` $\to$ `DRIVE_TO_GOAL` $\to$ `ALIGN_FINAL_YAW` $\to$ `ARRIVED`), and anti-stiction parameters.
- **`test_docking.py`**: Validates `compute_dock_target_pose()` calculations with lever arm offsets, velocity ramp profiles with guaranteed minimum creep ($\ge 0.07\text{ m/s}$ to avoid Unity wheel stiction stalls), and vertical dropoff clearances on the $25\text{ cm}$ staging station ($40\text{ mm}$ clearance at $0.28\text{ m}$ transit, free float at $0.21\text{ m}$ deposit).

### ROS Diagnostics & Introspection
```bash
# Check running ROS topics:
rostopic list

# Echo live odometry data (drive_center frame):
rostopic echo /odom

# Echo motor velocity commands:
rostopic echo /cmd_vel

# Echo lift commands:
rostopic echo /lift_cmd

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

---

## 6. Documentation

| Document | Description |
|----------|-------------|
| [`docs/SCRIPTS_AND_NODES.md`](docs/SCRIPTS_AND_NODES.md) | Complete reference for every script, library module, ROS node, and launch file — how they work, their parameters, and how they chain together |
| [`docs/WAREHOUSE_METRICS_AND_CLEARANCE.md`](docs/WAREHOUSE_METRICS_AND_CLEARANCE.md) | Physical dimensions of the warehouse, pallets, rack, staging station; lift window table; AMR kinematics |