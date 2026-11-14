# Ubuntu Laptop Quickstart

Run the whole stack (Unity simulation + ROS 1 Noetic in Docker) on a native Ubuntu laptop, and check that it is fast enough.
Everything is CPU-only here; no GPU passthrough is needed. Rates and profiles: [`TUNING.md`](TUNING.md).

## 1. What you need

| Item | Notes |
|---|---|
| Docker Engine with the Compose plugin | `docker compose version` must work. Add yourself to the `docker` group, then log in again |
| Unity Hub for Linux + Unity **2022.3.62f3** | The exact version is in `Unity/amr_ware_house/ProjectSettings/ProjectVersion.txt` |
| An X server | Any normal Ubuntu desktop session; RViz is shown through the X11 socket |
| About 8 GB RAM free | Unity and the container run side by side |

## 2. Line endings (do this first)

Files that run inside the Linux container (`.py`, `.sh`, `.launch`, `.yaml`, `.urdf`) must use LF. `.gitattributes` already
enforces this, so a fresh clone on Ubuntu is fine. If you copied files from Windows, or the repo still has CRLF files in its index,
you will see `/usr/bin/env: 'python3\r': No such file or directory` or `\r: command not found`. Fix in the repo:

```bash
git add --renormalize .       # then commit, so every machine gets LF
```

## 3. Start the container

```bash
cd CollaborativeAMRStockerization/docker
xhost +local:                                  # lets the container draw RViz on your display
docker compose -f docker-compose.ubuntu.yml up -d --build
docker exec -it ros1_amr_core bash

# first time only, inside the container:
cd /catkin_ws && catkin_make && source devel/setup.bash
```

The Ubuntu compose file mounts `catkin_ws` live, shares `/tmp/.X11-unix`, and publishes ports 10000 (Unity bridge) and 11311
(ROS master). Unity connects to `127.0.0.1:10000`.

## 4. Start Unity

1. Unity Hub -> Add project from disk -> `Unity/amr_ware_house`; open it with 2022.3.62f3.
2. Open `Assets/Scenes/Milestone_3.unity`.
3. Check Edit -> Project Settings -> Time: **Fixed Timestep = 0.01**. The sim publishes `/clock` at 100 Hz from `FixedUpdate`, so it needs
   100 physics steps per second.
4. Press Play. If the Console shows "Maximum Allowed Timestep" warnings, the laptop cannot keep up; see §6.

## 5. Launch navigation

Inside the container, pick a perf profile:

```bash
# default: 10 Hz controller, 5x5 m local costmap
roslaunch amr_navigation warehouse_autonav.launch launch_rviz:=true \
    rviz_config:=/catkin_ws/src/amr_navigation/rviz/navigation_light.rviz

# weak machine: 5 Hz controller, 8x8 m local costmap (the earlier proven values)
roslaunch amr_navigation warehouse_autonav.launch launch_rviz:=true perf:=laptop \
    rviz_config:=/catkin_ws/src/amr_navigation/rviz/navigation_light.rviz
```

**RViz and software rendering.** Without a GPU, RViz renders on the CPU (Mesa `llvmpipe`) and a warning about it is normal. Check with
`glxinfo | grep "OpenGL renderer"`. On `llvmpipe`, always use `navigation_light.rviz`: it ships with several displays switched off, so RViz leaves more CPU
to move_base. Close RViz when you are measuring.

## 6. Check that this laptop is fast enough

Run the same route you will demo (Human crossing plus a rack aisle), with RViz in the light profile, then in a second container shell:

```bash
docker exec -it ros1_amr_core bash
rosrun amr_navigation perf_probe.sh        # samples for 60 s, prints PASS or FAIL
```

| Criterion | Target |
|---|---|
| `/clock` | >= 95 Hz |
| `/cmd_vel_nav` | >= 9 Hz |
| `Control loop missed its desired rate` in move_base | <= 1 per minute |
| move_base CPU (mean) | < 60 % of one core |
| Unity | no "Maximum Allowed Timestep" spiral |

- **PASS on `perf:=smooth`:** keep the default.
- **FAIL:** relaunch with `perf:=laptop` and run the probe again. If `laptop` passes, use it on this machine; nothing else changes.
- Still failing: close other applications, lower Unity's quality level, then as a last resort set `clockHz` to 50 on the robot
  prefab's `PlanarOdometryPublisher`.

Verify which profile is active:

```bash
rosparam get /move_base/controller_frequency       # 10.0 = smooth, 5.0 = laptop
```

## 7. Common problems

See [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md) §6.4 (navigation and performance) and §6.1 (Docker, endpoint and display problems).
