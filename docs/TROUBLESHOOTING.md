# Project Setup Guide and Technical Report

**Project:** Collaborative Pallet/Tote Stacker AMR (UC6)

**Course:** 61CSE326 — Robot Obstacle-Avoidance Planning Software, WS2026

**Institution:** Vietnamese-German University, Computer Science & Engineering

**Supervisor:** Dr.-Ing. Quang Huan Dong

**Documentation review:** 19 September 2026

## 1. Purpose and setup scope

This guide explains what the development environment requires, why each component is needed, how to configure it, and how to check that it works. The intended reader is a teammate setting up the project on a Windows computer.

The project uses a differential-drive autonomous mobile robot (AMR) with a vertical lift to transport totes from storage racks to human-operated staging locations. The main research task is static and dynamic obstacle avoidance. Fine-grained shelf manipulation and physical high-payload lifting are outside the initial simulation scope.

The current repository follows **Windows 11 + native Unity + Dockerized ROS 1 Noetic**. Earlier ROS 2 Humble, Nav2, Distrobox, and micro-ROS instructions in `report_week1.tex` and `PROJECT_KICKOFF.docx` describe a previous plan. They should not be mixed into this setup.

### 1.1 What is available now

| Component | Evidence in the repository | Meaning for setup |
|---|---|---|
| ROS environment | Dockerfile, Compose file, entrypoint | Build configuration is present; it still needs to be built on each machine. |
| Unity–ROS endpoint | Endpoint source and `unity_bridge.launch` | A bidirectional communication test can be configured. |
| Unity test script | `unity_scripts/RosTestBridge.cs` | Publishes a heartbeat and receives velocity messages. |
| Robot description | `amr_description/urdf/stacker_amr.urdf` | A starting model is available for inspection/import. |
| Navigation launch | `amr_navigation/launch/move_base.launch` | Selects DWA, but does not load a complete navigation configuration. |
| Unity warehouse project | No complete Unity project found in the inspected repository | Obtain the team's project separately or create a project for the bridge test. |
| Costmaps, sensor simulation, localization, mission execution | Not supplied as a complete working integration | Follow the integration checklist in Section 5 after the basic setup passes. |

The context documents report an earlier successful loopback test. This guide was checked against source files; Docker, Unity, and RViz were not run during this documentation review.

## 2. Requirements

### 2.1 Workstation and host software

| Requirement | Purpose | Readiness check |
|---|---|---|
| Windows 11 with hardware virtualization and WSL 2 | Runs the Linux backend used by Docker Desktop | `wsl --status` reports WSL availability. |
| Docker Desktop using Linux containers and the WSL 2 engine | Runs the ROS environment | `docker version` shows both client and server. |
| Git | Retrieves and tracks project source | `git --version` succeeds. |
| Unity Hub and the team's agreed Unity Editor version | Runs the warehouse simulator | The project opens without package/compiler errors. |
| VS Code or another editor | Edits C#, ROS files, and documentation | The project folder can be opened. |
| VcXsrv, if using the Windows X11 display route | Displays RViz from the Linux container | An X server is running and accepts the intended container connection. |
| Internet access during initial setup | Downloads images, packages, and Unity dependencies | Docker build and Unity package installation can complete. |
| Available RAM, disk space, and GPU capacity | Supports Unity and Docker running together | Check current vendor requirements and test the actual scene; the repository defines no measured hardware minimum. |

Use the exact Unity version recorded in the shared project's `ProjectSettings/ProjectVersion.txt` when available. The source documents mention both Unity 6 and Unity 2022.3 LTS; they do not establish a single verified version for all teammates.

For current Windows and WSL prerequisites, consult [Docker's Windows installation guide](https://docs.docker.com/desktop/setup/install/windows-install/) and [WSL backend documentation](https://docs.docker.com/desktop/features/wsl/).

### 2.2 Software inside Docker

The Dockerfile uses `osrf/ros:noetic-desktop-full`, based on Ubuntu 20.04. It installs navigation packages including `move_base`, DWA, TEB, map server, AMCL, robot state publishing, TF utilities, and rosserial. It also includes Python and build utilities.

These packages belong inside the container. A separate native ROS installation on Windows is not required for this workflow. Installing a package makes its tools available; it does not configure the robot or prove navigation works.

### 2.3 Unity packages and project inputs

| Input | Required for |
|---|---|
| ROS-TCP-Connector configured for ROS 1 | The initial Unity–ROS communication test |
| `RosTestBridge.cs` attached to an active GameObject | Publishing heartbeat messages and receiving test commands |
| URDF Importer | Importing the stacker model for the later robot simulation |
| Warehouse scene, robot controllers, LiDAR and odometry publishers | Full navigation integration after the bridge test |

### 2.4 Version and support note

ROS Noetic reached end of life on 31 May 2025. This guide retains the repository's selected course stack; containerization does not restore upstream support. Record the working image ID, endpoint revision, Unity version, and package versions when the team establishes a reproducible baseline. See the [official ROS Noetic end-of-life announcement](https://www.ros.org/blog/noetic-eol/).

## 3. Background knowledge

### 3.1 Responsibilities of each component

| Component | What it does | What teammates should understand |
|---|---|---|
| Unity | Simulates the physical environment, robot motion, and sensors | C# components, scenes, physics, and message publishing/subscription |
| WSL 2 | Provides the Linux environment behind the selected Docker backend | It supports the container runtime; it is not the robot simulator. |
| Docker image | Defines the ROS installation and dependencies | Rebuild when the Dockerfile or installed dependencies change. |
| Docker container | Runs an instance of that environment | Starting the container does not automatically start the ROS bridge. |
| Bind mount | Makes `catkin_ws` on Windows visible as `/catkin_ws` in Docker | Source edits are shared, but running nodes may need restarting or rebuilding. |
| Catkin | Builds ROS 1 packages and creates a workspace environment | Run `catkin_make`, then source `devel/setup.bash`. |
| ROS-TCP bridge | Exchanges ROS messages between Unity and the ROS graph | Unity connects to the published host port; the endpoint runs inside Docker. |
| `move_base` | Combines global planning, local control, and costmaps | Needs valid sensor data, transforms, localization, and configuration. |
| RViz | Visualizes ROS data | It displays scans and planning output; it does not simulate robot physics. |

The supplied project notes attribute the ROS 1 choice to supervisor feedback and the team's course scope. This is a project decision, not a claim that ROS 1 guarantees better performance or simpler operation for every application.

### 3.2 System layout

```text
Windows workstation
|
+-- Unity simulation
|     ROS-TCP-Connector
|          |
|          | TCP to 127.0.0.1:10000
|          v
+-- Docker Desktop / WSL 2
|     ROS container: ros1_amr_core
|       +-- ROS-TCP endpoint (listens on 0.0.0.0:10000)
|       +-- ROS master and project nodes
|       +-- move_base / costmaps (later integration)
|       +-- RViz -- display connection --> Windows X server
|
+-- Windows editor
      catkin_ws/ <-- bind mount --> /catkin_ws in the container
```

Port `10000` carries the Unity bridge connection. The Compose file also publishes ROS master port `11311`; publishing that port alone is not a complete multi-machine ROS networking configuration. The current loopback ROS settings are intended for nodes in this container.

### 3.3 Topics, messages, and coordinate frames

A **node** is a ROS process. A **topic** is a named message stream. A **publisher** sends messages and a **subscriber** receives them. A **service** is a request/response interface rather than a continuous stream.

| Interface | Message type | Intended direction | Current role |
|---|---|---|---|
| `/unity_heartbeat` | `std_msgs/String` | Unity → ROS | Implemented by the bridge test script |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → Unity | Test script receives it; a proper wheel controller remains an integration task |
| `/scan` | `sensor_msgs/LaserScan` | Unity → ROS | Planned LiDAR input to costmaps |
| `/odom` | `nav_msgs/Odometry` | Unity → ROS | Planned robot pose/velocity estimate |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | Assigned transform publishers → ROS consumers | Required frame relationships |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | RViz or a goal publisher → navigation | Later navigation target input |
| `/lift_cmd` | Not finalized | Mission logic → lift controller | Choose a topic such as `std_msgs/Bool`, or a service such as `std_srvs/SetBool`; these are different interfaces. |

A typical frame chain is `map → odom → base_footprint → base_link → laser_link`. The final names must match the robot description, sensor messages, and navigation parameters. Assign one publisher to each transform. Publishing `/odom` does not automatically publish its corresponding TF transform; `robot_state_publisher` computes robot-link transforms from the robot model and joint states.

The global planner finds a route to the destination. The local planner computes short-term motion commands. Costmaps represent occupied space and clearance margins. DWA is selected in the supplied launch file; TEB is installed for later evaluation.

## 4. Setup steps

**Terminal convention:** Run `powershell` blocks on Windows. Run `bash` blocks inside the ROS container. Keep the bridge running in its own terminal during tests.

### Step 1 — Prepare Windows

1. Enable/install WSL 2 if it is not already available, following the linked Docker prerequisites.
2. Install Docker Desktop and enable **Use the WSL 2 based engine**. Use Linux containers.
3. Install Git, Unity Hub, the agreed Unity Editor, and an editor.
4. Start Docker Desktop and wait for its engine to become ready.

**Windows PowerShell:**

```powershell
wsl --status
docker version
docker compose version
git --version
```

**Expected result:** WSL is available, Docker reports a running server, and Git and Compose return versions. Resolve failures here before building the project.

### Step 2 — Locate the repository and inspect its inputs

This checkout is located at the following path. Teammates should substitute their own repository path.

```powershell
Set-Location D:\Project\Robot\CollaborativeAMRStockerization
Get-ChildItem docker
Get-ChildItem catkin_ws\src
```

Relevant structure:

```text
CollaborativeAMRStockerization/
  docker/
    Dockerfile
    docker-compose.yml
    entrypoint.sh
  catkin_ws/src/
    ROS-TCP-Endpoint/
    amr_description/
    amr_navigation/
  unity_scripts/
    RosTestBridge.cs
  docs/
```

The endpoint source is already present in this checkout. Do not clone another copy into the workspace. If a teammate's checkout lacks it, obtain the same endpoint revision as the team's working baseline. Do not assume that a historical branch name or the latest upstream revision matches this project.

**Expected result:** Docker inputs and the three ROS source directories are present.

### Step 3 — Build and start the container

**Windows PowerShell:**

```powershell
Set-Location D:\Project\Robot\CollaborativeAMRStockerization\docker
docker compose config
docker compose build
docker compose up -d
docker compose ps
```

**Expected result:** Service `ros1_stack`, with container name `ros1_amr_core`, is running. Initial build duration depends on downloads and machine resources.

The Compose file contains WSLg socket mounts and defaults `DISPLAY` to `:0`. Those entries do not establish that GUI forwarding works on every Windows installation. The bridge test can be completed before configuring RViz in Step 7.

### Step 4 — Build the Catkin workspace

**Windows PowerShell:**

```powershell
docker exec -it ros1_amr_core bash
```

**Inside the container:**

```bash
cd /catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source /catkin_ws/devel/setup.bash
rospack find amr_navigation
rospack find ros_tcp_endpoint
```

**Expected result:** Catkin finishes successfully and both packages resolve to paths inside `/catkin_ws/src`.

Repeat the two `source` commands in each new container terminal that needs project packages. The image sources base ROS in interactive Bash, but a new `docker exec` shell does not rerun the container entrypoint to source the built workspace.

### Step 5 — Start the ROS bridge

**Inside the prepared container terminal:**

```bash
roslaunch amr_navigation unity_bridge.launch
```

**Expected result:** The endpoint starts listening on port `10000`. `roslaunch` starts a ROS master if needed. Leave this terminal open; do not start another copy of the bridge.

### Step 6 — Configure Unity and test both directions

1. Open the team's Unity project. If it is unavailable, create a 3D project for the communication test and record the Editor version used.
2. In **Window → Package Manager → + → Add package from git URL**, add the connector if it is missing:

   ```text
   https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
   ```

3. Wait for package import and resolve compilation errors before continuing. Record the installed revision with the team's baseline.
4. Open **Robotics → ROS Settings** and select ROS 1, address `127.0.0.1`, port `10000`.
5. Copy `unity_scripts/RosTestBridge.cs` into the Unity project's `Assets/Scripts` folder if it is not already included.
6. Create an empty GameObject named `ROS_Manager` and attach `RosTestBridge`.
7. Press **Play** and inspect the Unity Console for the bridge readiness log.

Open a **second Windows PowerShell terminal**:

```powershell
docker exec -it ros1_amr_core bash
```

**Inside this second container terminal:**

```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
rosnode list
rostopic echo /unity_heartbeat
```

**Expected result:** A heartbeat string arrives approximately once per second. Press `Ctrl+C` to stop the echo command, then test the reverse direction:

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

**Expected result:** Unity logs that it received `/cmd_vel`, with zero linear and angular values. This checks message delivery without needing robot motion.

**Test limitation:** `RosTestBridge` translates its attached GameObject when a command callback arrives. It is not a differential-drive wheel controller, does not apply angular velocity, and does not provide odometry or collision-avoidance behavior. Passing this test establishes communication only.

### Step 7 — Configure RViz display

For the Windows VcXsrv route described in the project troubleshooting history:

1. Install and start VcXsrv/XLaunch on Windows.
2. Select **Multiple windows** and **Start no client**.
3. Configure X-server access and Windows Firewall to permit the intended Docker connection. The historical setup used **Disable access control**; if reproducing that local workaround, restrict inbound access rather than exposing the display server broadly.
4. In a prepared container terminal, select the Windows display server:

```bash
export DISPLAY=host.docker.internal:0.0
rviz
```

**Expected result:** An RViz window opens on Windows. An empty display is normal before sensor and navigation nodes exist.

The export applies to that shell; repeat it in new RViz terminals. WSLg is an alternative only after confirming its socket mounts and environment work with the local Docker installation. Do not assume the existing Compose comments prove it is configured correctly.

### Step 8 — Record the setup result

| Check | Pass condition |
|---|---|
| Container | `docker compose ps` shows the service running. |
| Workspace | `catkin_make` succeeds and project packages resolve. |
| Bridge | Endpoint starts without port conflicts. |
| Unity → ROS | `/unity_heartbeat` messages arrive. |
| ROS → Unity | Unity logs the test `/cmd_vel` message. |
| Visualization | RViz opens through the chosen display route. |

Record the date, machine, software versions, endpoint revision, and any remaining failures. These results are the environment baseline; autonomous navigation needs the next stage.

## 5. Next-stage integration requirements

Before treating `roslaunch amr_navigation move_base.launch` as a navigation demo, complete the following work:

1. Import and validate the robot model, wheel geometry, collisions, and lift joint in Unity.
2. Implement a differential-drive controller that consumes `/cmd_vel` and handles command timeout behavior.
3. Publish `/scan` and `/odom` with consistent units, timestamps, and frame IDs.
4. Establish the TF chain and choose a consistent time source. If ROS simulation time is enabled, supply `/clock`.
5. Supply a known map and localization strategy, or explicitly configure a suitable initial odometry-based test. SLAM is outside the documented initial scope.
6. Add and load costmap and planner configuration: robot footprint, obstacle sources, inflation, frame names, velocity limits, and goal tolerances.
7. Define and implement the lift interface and mission sequence.
8. Test A-to-B motion, static obstacles, and then dynamic obstacles while recording collisions, clearance, and completion time.

The current `move_base.launch` selects DWA and remaps topics. It does not load the costmap YAML files mentioned in the planning documents. A running `move_base` process alone is not evidence that this integration is complete.

## 6. Notes and troubleshooting

### 6.1 Common problems

| Symptom | Likely cause | Check or corrective action |
|---|---|---|
| Docker client cannot reach the server | Desktop engine is stopped or wrong container mode is selected | Start Docker Desktop, select Linux containers, and repeat `docker version`. |
| `catkin_make: command not found` | Base ROS environment is not sourced | Run `source /opt/ros/noetic/setup.bash`. |
| ROS cannot find `amr_navigation` | Workspace not built or overlay not sourced | Build `/catkin_ws`, then source `/catkin_ws/devel/setup.bash` in this terminal. |
| `/usr/bin/env: 'python\r': No such file or directory` | CRLF line endings in a Linux script | Run `dos2unix` on the specific affected file and retain LF line endings. The image already includes `dos2unix` and `python-is-python3`. |
| Endpoint script is not executable | Script permissions do not permit execution | Inspect the reported file and apply `chmod +x` to that script if needed. |
| Address already in use | Another endpoint owns port 10000 | Stop the earlier launch with `Ctrl+C`; use `rosnode list` to identify the process. Avoid killing every Python process. |
| Unity does not connect | Endpoint stopped, wrong settings, compilation errors, or inactive test component | Confirm ROS 1 and `127.0.0.1:10000`, check endpoint output, and verify `RosTestBridge` is enabled in the running scene. |
| Heartbeat works but the robot does not drive | The test script is not a wheel controller | Confirm the receive log, then implement the robot controller described in Section 5. |
| RViz cannot connect to display | X server unavailable or incorrect `DISPLAY`/access settings | Start the selected display server and check `echo "$DISPLAY"` in the same shell used for RViz. |
| Windows blocks a Unity component | A host security policy or file-trust issue | Inspect the specific Windows notification, verify the installer source, and repair/reinstall from Unity Hub as appropriate. Do not assume all security blocks support the same override. |
| Referenced endpoint branch does not exist | Instructions refer to a different upstream revision | Use the endpoint already supplied with this checkout or the team's recorded revision. |
| No scan, map, or path in RViz | Publishers/configuration are absent, or frames do not match | Check topic availability and TF before changing visualization settings. |

### 6.2 Daily workflow

**Start the environment — Windows PowerShell:**

```powershell
Set-Location D:\Project\Robot\CollaborativeAMRStockerization\docker
docker compose up -d
docker exec -it ros1_amr_core bash
```

**Start the bridge — container Bash:**

```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
roslaunch amr_navigation unity_bridge.launch
```

Then start the Unity scene and open RViz in a separate prepared terminal if needed.

| Change | Follow-up |
|---|---|
| Python node or YAML parameter file | Restart the affected process or launch file to load the change. |
| C++ source, package build configuration, or message definitions | Rebuild with `catkin_make`, source the workspace, and restart affected nodes. |
| Unity C# script | Let Unity recompile and check the Console. |
| Dockerfile or system dependencies | Rebuild the image and recreate the container. |

To finish, stop Unity Play mode, press `Ctrl+C` in active ROS launch terminals, and exit the container shells. From the Windows `docker` directory, run:

```powershell
docker compose down
```

Source files and build outputs in the bind-mounted workspace remain on Windows. Changes made only to the container's own filesystem may be lost when it is recreated; put reproducible dependency changes in the Dockerfile.

### 6.3 Future hardware work

STM32/ESP32, physical LiDAR, and rosserial are roadmap items, not prerequisites for the simulation setup. Standard ROS interfaces can reduce the amount of navigation code changed later, but hardware still requires motor control, encoder processing, calibration, device drivers, and validation.

USB attachment to WSL does not by itself prove that a Docker container can access the device. Verify visibility and permissions at each layer before documenting a device path such as `/dev/ttyACM0`. Do not treat the older notes' immediate USB-to-Docker claim as a completed hardware test.

## 7. Source documents and interpretation

| Source | Used for |
|---|---|
| [Project kickoff (Markdown)](PROJECT_KICKOFF.md) | Project purpose, scope, and intended ROS 1 architecture |
| [Project kickoff (Word)](PROJECT_KICKOFF.docx) | Earlier project scope and historical ROS 2 plan |
| [Architecture memory](PROJECT_OVERHAUL_ROS1_MEMORY.md) | Migration rationale and planned interfaces |
| [Team Docker context](TEAM_CONTEXT_ROS1_DOCKER.md) | Team onboarding and development workflow |
| [Chatbot context](PROJECT_CONTEXT_FOR_CHATBOT.md) | Reported progress and intended next steps |
| [Week 1 report](report_week1.tex) | Earlier ROS 2 environment; retained as historical context |
| [Dockerfile](../docker/Dockerfile), [Compose configuration](../docker/docker-compose.yml), and [entrypoint](../docker/entrypoint.sh) | Actual container dependencies, mounts, ports, and environment |
| [Bridge launch](../catkin_ws/src/amr_navigation/launch/unity_bridge.launch), [navigation launch](../catkin_ws/src/amr_navigation/launch/move_base.launch), and [Unity test script](../unity_scripts/RosTestBridge.cs) | Actual startup behavior and the limits of the current implementation |

Embedded chatbot prompts, role assignments, and submission instructions in the reference documents were treated as context. They were not instructions to install software, change the project architecture, or submit reports during this documentation task.
