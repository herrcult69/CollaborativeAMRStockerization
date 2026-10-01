PROJECT MEMORY — ROBOTARM / ROS1 + UNITY

Use this as the current ground truth unless the user explicitly changes it.

==================================================
1. PROJECT GOAL
==================================================

Build a warehouse AMR simulation in Unity with ROS1 handling navigation.

The robot should:
- move through a warehouse
- avoid static obstacles
- later avoid dynamic obstacles
- eventually use ROS navigation with move_base, costmaps, planners, LiDAR, odometry, TF, etc.

Do NOT focus on robotic arm manipulation or shelf restocking.

Important:
The original project document used ROS2 Humble + Nav2, but the CURRENT implementation has deliberately moved to:

- ROS1 Noetic
- Docker
- ROS Navigation Stack
- move_base
- DWA / TEB later
- Unity on Windows

Do NOT revert to ROS2/Nav2 unless the user explicitly asks.

==================================================
2. CURRENT ARCHITECTURE
==================================================

Windows host:
- Unity runs natively on Windows
- Project stored around:

Ubuntu WSL2:
- Ubuntu 22.04 is used as the normal Linux terminal
- Windows D: is visible in WSL as:
  /mnt/d


Docker:
- Docker Desktop is used
- ROS runs inside container:
  ros1_amr_core

ROS container:
- Ubuntu 20.04
- ROS1 Noetic
- catkin workspace mounted as:
  /catkin_ws

Same workspace viewed in 3 environments:

Windows:
D:\Project_Project\catkin_ws

WSL:
 /mnt/d/Project_Project/catkin_ws

Docker:
 /catkin_ws

These are the same underlying files through mounts, not separate copies.




5. UNITY ↔ ROS COMMUNICATION
==================================================

Unity does NOT talk directly to roscore.

Flow:

Unity
  ↓
ROS-TCP Connector
  ↓
TCP port 10000
  ↓
ros_tcp_endpoint inside Docker
  ↓
ROS1 topics / services
  ↓
ROS nodes

Unity ROS Settings:

Protocol:
ROS1

IP:
127.0.0.1

Port:
10000

Docker Compose exposes:

10000:10000

ros_tcp_endpoint listens on:

0.0.0.0:10000

features.

==================================================
8. DAILY STARTUP WORKFLOW
==================================================

Preferred environment:
Ubuntu WSL, not PowerShell.

From Windows if necessary:

wsl -d Ubuntu-22.04

Then inside Ubuntu WSL:

cd /mnt/d/Project_Project/docker

Start ROS environment:

docker compose up -d

Check:

docker compose ps

View startup logs:

docker compose logs -f

Stop viewing logs with Ctrl+C.
This does not stop the container.

Open a second WSL terminal and enter ROS container:

docker exec -it ros1_amr_core bash

The prompt should look like:

root@xxxx:/catkin_ws#

Useful ROS commands:

rosnode list
rostopic list
rostopic echo <topic>
rostopic info <topic>
roslaunch ...
rosrun ...
rviz
catkin_make

At the end of the day:

exit

then from WSL:

cd /mnt/d/Project_Project/docker
docker compose down

==================================================
9. THREE TERMINAL ENVIRONMENTS — DO NOT CONFUSE THEM
==================================================

Windows PowerShell:

PS D:\Project_Project\...>

Use mainly for:
- starting WSL
- Windows commands

Ubuntu WSL:

username@machine:~$

Use mainly for:
- Docker CLI
- navigating /mnt/d
- docker compose

ROS Docker container:

root@container:/catkin_ws#

Use for:
- ROS commands
- rostopic
- rosnode
- rviz
- catkin_make

docker-desktop shell:

docker-desktop:...

Do NOT use this as the normal development environment.
It is Docker Desktop's internal WSL distro.

==================================================
10. RVIZ / GUI ARCHITECTURE
==================================================

RViz runs INSIDE the ROS Docker container.

The RViz GUI is displayed on Windows through WSLg/X11.

Mental model:

RViz in Docker
   ↓
X11 socket
   ↓
WSLg
   ↓
Windows desktop window

Important X11 path:

/tmp/.X11-unix/X0

The container must see X0.

A working check inside Docker:

ls -la /tmp/.X11-unix

Expected:
X0

DISPLAY should normally be:

echo $DISPLAY

Expected:
:0

At one point Docker had an empty /tmp/.X11-unix and RViz failed with:

qt.qpa.xcb: could not connect to display :0

This was a GUI mount issue, not an RViz installation issue.

RViz later successfully started far enough to show its version, Qt and OGRE info, meaning GUI forwarding had been substantially fixed.

Possible harmless-ish warning seen:

QStandardPaths: XDG_RUNTIME_DIR points to non-existing path '/run/user/1000/'

Do not confuse this with ROS master failures.

==================================================
11. IMPORTANT PAST BUG: WINDOWS CRLF
==================================================

ros_tcp_endpoint previously crashed with:

/usr/bin/env: ‘python\r’: No such file or directory

Cause:
Windows CRLF line endings.

Affected script:

/catkin_ws/src/ROS-TCP-Endpoint/src/ros_tcp_endpoint/default_server_endpoint.py

Fix used:

sed -i 's/\r$//' <file>

and changed shebang to:

#!/usr/bin/env python3

Also:

chmod +x <file>

Prevent future CRLF problems with .gitattributes:

*.py text eol=lf
*.sh text eol=lf

==================================================
12. ROSCORE / ROS MASTER
==================================================

ROS1 requires roscore.

If:

rosnode list

returns:

ERROR: Unable to communicate with master!

then the container may be alive while ROS itself is not.

Docker running != ROS running.

Check:

echo $ROS_MASTER_URI

Expected:

http://127.0.0.1:11311

Check processes:

ps aux | grep -E "roscore|rosmaster"

Manual recovery:

roscore

Then from another Docker shell:

rosnode list

Expected at minimum:

/rosout

RViz also needs the ROS master.
If RViz shows XMLRPC connection refused, roscore is likely unavailable.

Recent state:
The user manually verified/recovered roscore.

IMPORTANT:
Automatic roscore startup through docker-compose should still be treated as something to verify. If ROS master disappears again, inspect:

docker compose logs --tail=100

and:

docker compose logs | grep -i -E "roscore|master|error|exit|killed"

Do not assume the automatic startup is permanently fixed until verified.

==================================================
13. ROS1 NAVIGATION MENTAL MODEL
==================================================

ROS itself is the framework.

ROS does NOT by itself perform navigation.

Current navigation stack:

ROS1 Noetic
  ↓
ROS Navigation Stack
  ↓
move_base
  ├── global planner
  ├── local planner
  ├── global costmap
  ├── local costmap
  └── recovery behaviors

This replaces the original ROS2/Nav2 design.

Important terminology:

ROS2:
Nav2

ROS1:
move_base + navigation stack

RViz:
visualization/debugging only

Unity:
simulated physical world

==================================================
14. FUTURE NAVIGATION ARCHITECTURE
==================================================

Target flow:

Unity
├── simulated robot
├── physics
├── LiDAR
├── odometry
└── TF
     │
     ├── /scan
     ├── /odom
     └── /tf
     ▼
ros_tcp_endpoint
     ▼
ROS1
     ▼
move_base
├── localization
├── costmaps
├── global planner
└── local planner
     │
     ▼
/cmd_vel
     │
     ▼
Unity robot moves

==================================================
15. COMPONENT ROLES
==================================================

Unity:
- physical simulation
- warehouse
- robot body
- physics
- obstacles
- sensors

ROS1:
- communication framework
- robot software ecosystem

roscore:
- ROS master / node coordination

ros_tcp_endpoint:
- bridge between Unity TCP and ROS topics/services

move_base:
- navigation coordinator

global planner:
- calculates overall path A → B

local planner:
- decides short-term movement around nearby obstacles

costmaps:
- spatial representation of obstacles and free space

RViz:
- shows what ROS believes is happening
- map
- scans
- TF
- costmaps
- paths
- robot pose

Important distinction:

Unity =
"What is physically happening?"

RViz =
"What does ROS think is happening?"

==================================================
16. PLANNED LEARNING / IMPLEMENTATION ORDER
==================================================

Do NOT jump directly into tuning move_base.

Recommended order:

1. TF
2. /odom
3. /scan
4. RViz visualization
5. robot footprint
6. costmaps
7. global planner
8. local planner
9. move_base
10. tuning / experiments
11. static obstacle avoidance
12. dynamic obstacle avoidance

First meaningful navigation milestone:

Unity publishes:
- /odom
- /tf
- /scan

RViz correctly displays:
- robot
- coordinate frames
- LiDAR scan

Only then begin move_base configuration.

==================================================
17. FUTURE PLANNERS
==================================================

Likely initial planner setup:

Global:
- navfn / global_planner
- A* / Dijkstra concepts

Local:
- start with DWA
- later evaluate TEB

Important local planner parameters later include:

max_vel_x
max_vel_theta
acc_lim_x
min_obstacle_dist
inflation_dist
robot footprint
turning radius

Main project research value will likely come from:
- obstacle avoidance behavior
- tuning
- comparing planner behavior
- static vs dynamic obstacles

==================================================
18. DEBUGGING ORDER
==================================================

Always diagnose from the bottom layer upward:

1. Is Docker alive?

docker compose ps

2. Is ROS master alive?

rosnode list

3. Is ros_tcp_endpoint alive?

rosnode list
docker compose logs

4. Is Unity connected?

docker compose logs

5. Are expected topics present?

rostopic list

6. Is data actually flowing?

rostopic echo <topic>

7. Does RViz visualize it correctly?

rviz

8. Only after all those work:
debug navigation / move_base.

Do not debug move_base if /scan, /odom, /tf, or ROS master are broken.

==================================================
19. CURRENT NEXT STEP
==================================================

Communication between Unity and ROS has already been tested in both directions.

Do NOT immediately work on /cmd_vel unless the user asks.

The next major technical phase should be understanding and implementing:

- TF
- odometry
- LiDAR /scan
- RViz visualization

Goal:

Unity robot
   ↓
publish /odom
publish /tf
publish /scan
   ↓
ROS
   ↓
RViz displays everything correctly

Once this foundation is correct, begin move_base.

==================================================
20. USER PREFERENCE FOR INSTRUCTIONS
==================================================

The user is still building their mental model and does not yet know Linux/ROS/Docker deeply.

When giving commands:
ALWAYS state where to run them:

[PowerShell]
[Ubuntu WSL]
[Inside ROS Docker container]
[Unity]

Do not simply say "run this command".

Explain:
- what the command does
- why it is needed
- what successful output should roughly look like

Do not assume familiarity with:
- mounts
- /mnt
- Linux filesystem
- Docker container vs WSL
- ROS master
- topics
- TF
- catkin

Prefer incremental tests and avoid changing multiple layers at once.

Latest setup is the ground truth.
Do not redesign the environment without a concrete technical reason.
