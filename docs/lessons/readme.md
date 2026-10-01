Yes. The cleanest way to understand this setup is to stop thinking of it as “a bunch of Linux paths” and instead think of it as **four layers sitting on top of your Windows machine**.

Here is the complete mental model I would use from now on.

```text
                         YOUR PHYSICAL COMPUTER
                               WINDOWS
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  Windows filesystem                                                 │
│                                                                     │
│  D:\Project_Project                                                 │
│  ├── catkin_ws        ← your ROS source code                        │
│  ├── docker                                                         │
│  │   ├── Dockerfile                                                 │
│  │   ├── docker-compose.yml                                         │
│  │   └── entrypoint.sh                                              │
│  └── unity_sim         ← Unity project                              │
│                                                                     │
│  ┌──────────────────┐                  ┌──────────────────────────┐  │
│  │      UNITY       │                  │       Ubuntu WSL2        │  │
│  │                  │                  │                          │  │
│  │ warehouse        │                  │ Linux environment        │  │
│  │ robot physics    │                  │                          │  │
│  │ sensors          │                  │ Windows D: appears as:   │  │
│  │                  │                  │ /mnt/d/                  │  │
│  └────────┬─────────┘                  │                          │  │
│           │                            │ /mnt/d/Project_Project   │  │
│           │ TCP port 10000             │                          │  │
│           │                            │ WSLg                     │  │
│           │                            │ └── graphical display    │  │
│           │                            └──────────┬───────────────┘  │
│           │                                       │                  │
│           │                             Docker Desktop               │
│           │                                       │                  │
│           │                              ┌────────▼─────────┐        │
│           └─────────────────────────────►│ ROS1 CONTAINER   │        │
│                                          │                  │        │
│                                          │ Ubuntu 20.04     │        │
│                                          │ ROS1 Noetic      │        │
│                                          │ roscore          │        │
│                                          │ ros_tcp_endpoint │        │
│                                          │ move_base        │        │
│                                          │ RViz             │        │
│                                          │                  │        │
│                                          │ /catkin_ws       │        │
│                                          └──────────────────┘        │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

The most important idea is:

> **Windows owns the actual project files. WSL gives you a Linux view of Windows. Docker gives ROS its own isolated Linux environment. Unity stays on Windows.**

Now I'll unpack every piece.

---

## 1. Windows is the real computer underneath everything

Your actual project currently lives here:

```text
D:\Project_Project
```

This is a Windows path.

Windows organizes storage using drive letters:

```text
C:\
D:\
E:\
```

So your files might physically look like:

```text
D:\Project_Project
│
├── catkin_ws
│   └── src
│
├── docker
│   ├── Dockerfile
│   ├── docker-compose.yml
│   └── entrypoint.sh
│
└── unity_sim
```

When you're using PowerShell and see:

```text
PS D:\Project_Project\docker>
```

you are operating directly in the **Windows environment**.

---

# 2. Linux does not use `C:` or `D:`

Linux has a fundamentally different filesystem model.

Instead of:

```text
C:\
D:\
```

Linux has one single filesystem tree starting from:

```text
/
```

This `/` is called the **root directory**.

It is roughly the Linux equivalent of saying:

> Start from the absolute top of the filesystem.

For example:

```text
/
├── bin
├── etc
├── home
├── opt
├── root
├── tmp
├── usr
└── mnt
```

Therefore:

```text
/catkin_ws
```

means:

```text
/
└── catkin_ws
```

It does **not** mean a disk.

It's simply a directory directly underneath Linux's root `/`.

---

# 3. What does `/mnt` mean?

This is one of the concepts that caused a lot of confusion earlier.

`mnt` is short for:

> **mount**

A mount is basically:

> Make another filesystem/storage location appear at this directory.

Linux doesn't normally use drive letters.

Instead, another disk is attached somewhere inside the existing filesystem tree.

For example, WSL makes your Windows `D:` drive accessible as:

```text
/mnt/d
```

So these two refer to the same Windows files:

```text
Windows:

D:\Project_Project
```

and:

```text
Ubuntu WSL:

/mnt/d/Project_Project
```

Conceptually:

```text
Windows                        Ubuntu WSL

D:\                           /mnt/d/
│                                │
└── Project_Project              └── Project_Project
    │                                │
    ├── catkin_ws                   ├── catkin_ws
    ├── docker                      ├── docker
    └── unity_sim                   └── unity_sim
```

There are **not two copies**.

WSL is simply viewing the Windows drive through a Linux path.

That is why:

```bash
cd /mnt/d/Project_Project
```

takes you to:

```text
D:\Project_Project
```

---

# 4. What exactly is WSL?

WSL means:

> **Windows Subsystem for Linux**

It lets you run a real Linux environment while Windows is still your main operating system.

So instead of dual booting:

```text
Restart computer
→ choose Windows OR Ubuntu
```

WSL allows:

```text
Windows
└── Ubuntu Linux running alongside Windows
```

Therefore you can have:

```text
Windows PowerShell
PS D:\Project_Project>
```

and at the same time:

```text
Ubuntu terminal
user@computer:~$
```

The second terminal follows Linux rules.

---

# 5. Why did `docker-desktop` appear as WSL?

This was another source of confusion.

Docker Desktop internally uses WSL2.

So before installing Ubuntu, this:

```powershell
wsl -l -v
```

showed:

```text
docker-desktop
```

But that Linux environment belongs to **Docker itself**.

Think of it as:

```text
docker-desktop
=
Docker's engine room
```

It is not intended to be:

```text
your Linux development machine
```

After installing Ubuntu, the desirable model becomes:

```text
WSL2
├── Ubuntu-22.04       ← YOU work here
│
└── docker-desktop     ← Docker works here internally
```

You usually shouldn't manually work inside `docker-desktop`.

---

# 6. So why do we want Ubuntu WSL?

Because some of our tools are naturally Linux-oriented:

```text
ROS
bash
catkin
Linux filesystem commands
X11 / WSLg
Docker tooling
```

Instead of forcing Windows PowerShell to behave like Linux, Ubuntu WSL gives us a normal Linux terminal.

You can still edit the exact same files using VS Code on Windows.

So our development arrangement becomes:

```text
VS Code
    ↓
edit files on D:\

Ubuntu WSL
    ↓
Linux commands

Docker
    ↓
ROS runtime

Unity
    ↓
simulation
```

---

# 7. Now Docker enters the picture

WSL and Docker are **not the same thing**.

This distinction is extremely important.

```text
WSL
=
Linux environment integrated into Windows


Docker
=
system for creating isolated application environments
called containers
```

We use Docker because ROS1 Noetic officially expects Ubuntu 20.04.

But we don't want to install Ubuntu 20.04 as our entire operating system.

So Docker gives ROS its own little environment:

```text
Docker container
│
├── Ubuntu 20.04
├── ROS1 Noetic
├── move_base
├── RViz
├── ros_tcp_endpoint
└── our ROS packages
```

You can think of it as a lightweight isolated computer dedicated to ROS.

---

# 8. Docker image versus Docker container

These two words are related but different.

A **Docker image** is the template.

A **container** is a running instance of that template.

Like:

```text
Blueprint              Actual house
   │                        │
   ▼                        ▼
Docker image        Docker container
```

Our Dockerfile creates the image.

Then Docker Compose creates a running container from it.

```text
Dockerfile
    │
    ▼
docker build
    │
    ▼
ROS image
    │
    ▼
docker compose up
    │
    ▼
ros1_amr_core container
```

---

# 9. What does the Dockerfile do?

The Dockerfile answers:

> **What software should exist inside our ROS environment?**

For example:

```dockerfile
FROM osrf/ros:noetic-desktop-full
```

means:

> Start from an Ubuntu environment that already contains ROS1 Noetic desktop.

Then we install things such as:

```text
navigation
TEB
rosserial
other dependencies
```

And:

```dockerfile
WORKDIR /catkin_ws
```

means:

> When someone enters the container, make `/catkin_ws` the default directory.

It does **not** connect your Windows folder.

It does **not** create your ROS workspace.

It just says:

```text
default working directory = /catkin_ws
```

Approximately like automatically doing:

```bash
cd /catkin_ws
```

---

# 10. Then what does `docker-compose.yml` do?

Docker Compose answers a different question:

> **When the container runs, how should it be connected to the outside world?**

This includes:

```text
ports
folders
environment variables
startup command
container name
```

This line is particularly important:

```yaml
- ../catkin_ws:/catkin_ws
```

Remember the format:

```text
HOST : CONTAINER
```

Therefore:

```text
../catkin_ws : /catkin_ws
       ↑             ↑
your files       Docker path
```

Assume Compose is here:

```text
D:\Project_Project\docker\docker-compose.yml
```

Then:

```text
..
```

means:

> go up one folder

So:

```text
../catkin_ws
```

becomes:

```text
D:\Project_Project\catkin_ws
```

Docker then makes that folder visible inside the container as:

```text
/catkin_ws
```

This operation is called a **bind mount**.

---

# 11. What exactly is a mount?

A mount is easiest to understand as a window.

Imagine your actual files are:

```text
D:\Project_Project\catkin_ws
```

The container cannot normally see Windows files.

So we tell Docker:

> When something inside Docker looks at `/catkin_ws`, show it the files from `D:\Project_Project\catkin_ws`.

So:

```text
REAL FILES

D:\Project_Project\catkin_ws
        │
        │ bind mount
        ▼

DOCKER VIEW

/catkin_ws
```

If Windows contains:

```text
D:\Project_Project\catkin_ws\src\robot_control
```

Docker sees:

```text
/catkin_ws/src/robot_control
```

Same file.

Not copied.

---

# 12. So now there are three different-looking paths for one folder

This is probably the single most useful thing to memorize.

```text
                    SAME FILES

Windows:
D:\Project_Project\catkin_ws
                 │
                 │ WSL mounts Windows D:
                 ▼
Ubuntu WSL:
/mnt/d/Project_Project/catkin_ws
                 │
                 │ Docker bind mount
                 ▼
ROS container:
/catkin_ws
```

So when you see:

```text
D:\Project_Project\catkin_ws
```

or:

```text
/mnt/d/Project_Project/catkin_ws
```

or:

```text
/catkin_ws
```

you may actually be looking at the **same workspace from three environments**.

---

# 13. What is `catkin_ws` actually for?

`catkin_ws` means:

> Catkin Workspace

Catkin is ROS1's build system.

Your workspace roughly looks like:

```text
catkin_ws
│
├── src
│   ├── ROS-TCP-Endpoint
│   ├── amr_navigation
│   └── your other ROS packages
│
├── build
│
└── devel
```

You mainly write source code under:

```text
src/
```

Then:

```bash
catkin_make
```

reads `src/` and generates:

```text
build/
devel/
```

Think:

```text
src
 ↓
catkin_make
 ↓
build + devel
```

`build/` contains compilation machinery.

`devel/` contains the prepared runtime environment.

---

# 14. Why do we source `devel/setup.bash`?

After building:

```bash
catkin_make
```

we get:

```text
/catkin_ws/devel/setup.bash
```

ROS needs that file to learn:

> These custom packages belong to my environment.

So:

```bash
source /catkin_ws/devel/setup.bash
```

means approximately:

> Add our project's ROS packages to the current shell environment.

Before:

```text
ROS knows:
roscore
rviz
move_base
...
```

After sourcing:

```text
ROS knows:
roscore
rviz
move_base
+
our own packages
+
ros_tcp_endpoint
```

---

# 15. What does `entrypoint.sh` do?

Every time the container starts, Docker executes our entrypoint.

Our entrypoint essentially performs:

```bash
source /opt/ros/noetic/setup.bash
```

meaning:

> Enable ROS Noetic.

Then:

```bash
source /catkin_ws/devel/setup.bash
```

meaning:

> Enable our project's ROS workspace too.

So mentally:

```text
Container starts
      ↓
entrypoint.sh
      ↓
load ROS Noetic
      ↓
load our workspace
      ↓
run requested command
```

---

# 16. Where is ROS actually running?

Inside the container.

```text
Docker container: ros1_amr_core

Ubuntu 20.04
│
├── ROS1 Noetic
├── roscore
├── ros_tcp_endpoint
├── navigation
├── move_base
├── TEB/DWA
└── RViz
```

Windows itself does not need ROS installed.

Ubuntu WSL itself also does not need ROS installed.

That is intentional.

```text
Windows       → Unity
Ubuntu WSL    → convenient Linux interface
Docker        → ROS1
```

---

# 17. Where does Unity run?

Unity runs natively on Windows.

```text
Windows
└── Unity
```

Unity handles the simulated physical environment:

```text
warehouse
robot body
wheels
physics
LiDAR
people
carts
obstacles
```

You can think:

```text
Unity = simulated reality
```

---

# 18. How does Unity communicate with ROS?

Through a TCP connection.

Unity contains:

```text
ROS-TCP Connector
```

Docker contains:

```text
ros_tcp_endpoint
```

They connect using:

```text
127.0.0.1:10000
```

So:

```text
Unity
ROS-TCP Connector
       │
       │ TCP port 10000
       ▼
Docker
ros_tcp_endpoint
       │
       ▼
ROS topics/services
```

For example:

```text
Unity → ROS

/scan
/odom
/tf
sensor information
```

And:

```text
ROS → Unity

/cmd_vel
navigation commands
service calls
```

---

# 19. What does `127.0.0.1` mean?

It means:

> this computer

It is also called:

```text
localhost
```

So Unity connecting to:

```text
127.0.0.1:10000
```

means:

> Connect to port 10000 on this machine.

Docker Compose has:

```yaml
ports:
  - "10000:10000"
```

which means:

```text
Windows port 10000
       │
       ▼
Docker port 10000
```

Therefore:

```text
Unity
127.0.0.1:10000
       │
       ▼
Docker
ros_tcp_endpoint:10000
```

---

# 20. What is `roscore`?

`roscore` is the coordination infrastructure for ROS1.

It helps ROS nodes find each other.

For example:

```text
LiDAR node
navigation node
RViz
ros_tcp_endpoint
```

all register into the same ROS system.

Simplified:

```text
                 roscore
                    │
          ┌─────────┼─────────┐
          │         │         │
     endpoint   move_base    RViz
```

But `roscore` is **not the robot brain** itself.

It's more like ROS's switchboard/directory.

---

# 21. What is `move_base`?

This is much closer to what we'd call the navigation brain.

```text
move_base
├── global planner
├── local planner
├── global costmap
└── local costmap
```

It reads things such as:

```text
/map
/scan
/odom
/tf
navigation goal
```

and determines:

```text
/cmd_vel
```

which tells the robot:

```text
move forward this fast
turn this fast
```

---

# 22. What is RViz then?

RViz is a **ROS visualization/debugging application**.

It runs with ROS inside Docker.

It can display:

```text
laser scans
map
robot pose
TF frames
costmaps
planned route
local trajectory
```

But RViz does not simulate physics.

That is Unity's job.

A useful distinction is:

```text
UNITY

"What is happening in the simulated physical world?"


RVIZ

"What does ROS think is happening?"
```

---

# 23. Why does RViz need WSLg?

Because RViz is a graphical Linux application.

But it is running inside:

```text
Docker Linux container
```

while your monitor belongs to:

```text
Windows
```

Something must transfer the Linux graphical window to Windows.

That is where:

```text
WSLg
```

comes in.

WSLg means roughly:

> WSL GUI integration.

It allows Linux graphical applications to appear as normal Windows desktop windows.

Conceptually:

```text
RViz
inside Docker
     │
     ▼
Linux graphical socket
     │
     ▼
WSLg
     │
     ▼
Windows display
     │
     ▼
RViz window on your monitor
```

---

# 24. And what is `/tmp/.X11-unix/X0`?

Linux graphical applications historically use a protocol called:

```text
X11
```

The X server provides a communication socket.

For display `:0`, that socket commonly appears as:

```text
/tmp/.X11-unix/X0
```

So:

```bash
DISPLAY=:0
```

basically says:

> Send my graphical window to display number 0.

And:

```text
/tmp/.X11-unix/X0
```

is the actual communication endpoint.

That's why earlier this was important:

```bash
ls /tmp/.X11-unix
```

If you saw:

```text
X0
```

there was a graphical server to communicate with.

If the directory was empty:

```text
.
..
```

RViz had nowhere to send its window.

---

# 25. Then what is `/mnt/wslg`?

This is another mount.

Remember:

```text
/mnt = mounted external/special filesystem locations
```

WSL creates:

```text
/mnt/wslg
```

to expose WSLg-related resources.

So unlike:

```text
/mnt/d
```

which represents your Windows `D:` drive,

```text
/mnt/wslg
```

represents resources used by WSL's graphical subsystem.

These are two different mounts:

```text
/mnt
├── d       ← Windows D: drive
└── wslg    ← WSL graphical subsystem
```

---

# 26. Why do we mount WSLg again into Docker?

Because Docker is yet another isolated Linux environment.

Ubuntu can see:

```text
/mnt/wslg
```

but the Docker container doesn't automatically see Ubuntu's files.

So Compose needs to expose those resources to the container.

Conceptually:

```text
WSL / WSLg
/mnt/wslg/.X11-unix/X0
             │
             │ mount
             ▼
Docker
/tmp/.X11-unix/X0
             │
             ▼
RViz
```

That is exactly why we were debugging the missing `X0`.

---

# 27. The complete project data flow

Now we can combine everything:

```text
                          WINDOWS
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│                      Unity                                  │
│                simulated warehouse                          │
│                        │                                    │
│            sensor data │ /scan /odom /tf                    │
│                        │                                    │
│                        ▼                                    │
│                  TCP port 10000                             │
│                        │                                    │
│                        ▼                                    │
│              ┌────────────────────┐                         │
│              │ Docker ROS1       │                         │
│              │                    │                         │
│              │ ros_tcp_endpoint   │                         │
│              │       │            │                         │
│              │       ▼            │                         │
│              │ ROS topics         │                         │
│              │       │            │                         │
│              │       ▼            │                         │
│              │ move_base          │                         │
│              │ planners/costmaps  │                         │
│              │       │            │                         │
│              │       ▼            │                         │
│              │ /cmd_vel           │                         │
│              └───────┬────────────┘                         │
│                      │                                      │
│                      └────────────────────► Unity            │
│                                         moves robot         │
│                                                             │
│              RViz inside Docker                             │
│                      │                                      │
│                      ▼                                      │
│                     WSLg                                    │
│                      │                                      │
│                      ▼                                      │
│              RViz window on Windows                         │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

# 28. And the complete filesystem flow

Separate this mentally from the robot data flow.

```text
FILES

Windows
D:\Project_Project
│
├── catkin_ws
├── docker
└── unity_sim
       │
       │ WSL view of Windows disk
       ▼
Ubuntu WSL
/mnt/d/Project_Project
│
├── catkin_ws
├── docker
└── unity_sim
       │
       │ Docker bind mount
       ▼
Docker container
/catkin_ws
```

This flow is about:

```text
files
source code
directories
```

It has nothing to do with ROS messages.

---

# 29. Keep these two concepts separate

This will make almost everything easier.

```text
FILESYSTEM CONNECTION

Windows D:
    ↓
WSL /mnt/d
    ↓
Docker /catkin_ws
```

versus:

```text
ROBOT COMMUNICATION

Unity
    ↓ ↑
TCP :10000
    ↓ ↑
ros_tcp_endpoint
    ↓ ↑
ROS topics
    ↓ ↑
navigation
```

They are completely separate systems.

One moves **files**.

The other moves **robot data**.

---

# 30. Which terminal am I currently in?

This is also worth learning to recognize immediately.

| Prompt looks like         | You are in                   | Typical commands                             |
| ------------------------- | ---------------------------- | -------------------------------------------- |
| `PS D:\Project_Project>`  | Windows PowerShell           | `wsl`, Windows commands                      |
| `user@PC:~$`              | Ubuntu WSL                   | `cd /mnt/d/...`, Docker CLI                  |
| `root@abc123:/catkin_ws#` | ROS Docker container         | `rosnode`, `rostopic`, `rviz`, `catkin_make` |
| `docker-desktop:...#`     | Docker's internal WSL system | Normally don't work here                     |

That table alone will probably prevent a lot of future confusion.

---

# 31. The three Docker files, finally

Now those three lines from earlier should have a much clearer meaning:

```text
Dockerfile:

WORKDIR /catkin_ws
```

means:

> Inside our ROS container, use `/catkin_ws` as the default current directory.

---

```text
docker-compose.yml:

../catkin_ws:/catkin_ws
```

means:

> Connect our real project workspace to that container directory.

---

```text
entrypoint.sh:

source /catkin_ws/devel/setup.bash
```

means:

> Tell ROS about packages that were compiled from that workspace.

Together:

```text
REAL WORKSPACE
D:\Project_Project\catkin_ws
          │
          │ mount
          ▼
DOCKER
/catkin_ws
          │
          │ WORKDIR
          ▼
we operate here
          │
          │ catkin_make
          ▼
/catkin_ws/devel/setup.bash
          │
          │ source
          ▼
ROS knows our packages
```

---

# The final mental model

If you remember only this, you'll have enough context to understand most of what we do next:

```text
┌────────────────────────────────────────────┐
│ WINDOWS                                    │
│                                            │
│ D:\Project_Project                         │
│     = real project files                   │
│                                            │
│ Unity                                      │
│     = simulated physical world             │
│                                            │
│ Ubuntu WSL                                 │
│     = Linux terminal/environment           │
│     = sees D: as /mnt/d                    │
│     = provides WSLg for Linux GUIs         │
│                                            │
│ Docker                                     │
│     = isolated ROS environment             │
│                                            │
│     /catkin_ws                             │
│       = mounted view of real catkin_ws     │
│                                            │
│     ROS1                                   │
│       = communication framework            │
│                                            │
│     move_base                              │
│       = navigation                         │
│                                            │
│     RViz                                   │
│       = visualize what ROS sees            │
│                                            │
└────────────────────────────────────────────┘
```

And the robot itself conceptually works like this:

```text
          sensor observations
       ┌────────────────────────►
       │
┌──────┴─────┐       TCP       ┌───────────────┐
│   UNITY    │ ◄──────────────►│     ROS1      │
│            │                 │               │
│ simulated  │                 │ navigation    │
│ world      │                 │ decision      │
│ physics    │                 │ making        │
└──────▲─────┘                 └───────┬───────┘
       │                               │
       │           /cmd_vel            │
       └───────────────────────────────┘

                                │
                                │ ROS data
                                ▼
                           ┌──────────┐
                           │   RViz   │
                           │          │
                           │ debugger │
                           └──────────┘
```

So from this point onward, whenever I give you a command, I'll distinguish explicitly between **PowerShell**, **Ubuntu WSL**, and **inside the ROS Docker container**. That should remove one of the biggest sources of ambiguity we've had so far.
