# XStack migration verification — 2026-10-04

Implementation spans the Unity `amr_ware_house` and ROS
`CollaborativeAMRStockerization` repositories. Both merges are left uncommitted.
The ROS merge introduced during implementation retains its mission/docking
additions; the legacy description package and lesson controller are also retained.
Existing comments in `go_to_point.py` were preserved.

## Completed checks

| Check | Result |
|---|---|
| Unity 2022.3.62f3 batch compilation | Passed |
| Milestone_1, WarehouseTraining, SampleScene asset/script references | Passed Unity editor validation |
| One odometry/clock publisher and one scan publisher per scene | Passed |
| Wheel spacing, explicit XStack joint references, 32/8 solver, 0.01 s timestep | Passed |
| Catkin workspace build | Passed for all three packages |
| Frame acceptance/rejection, missing/stale/future odometry, goal math | 12 tests passed |
| ROS profile selection, costmaps, mapping, localization, TF/URDF consistency | 7 tests passed |
| Incoming move-to-point and docking tests | 7 + 5 tests passed |
| Live Unity -> ROS /clock, /odom, /scan and TF | Passed |
| Live laser ray samples compared with Unity raycasts | Passed (within 0.03 m) |
| Actual gmapping capture and map save | Passed |
| AMCL initialization from captured map pose | Passed (0.000924 m pose continuity error) |
| Three DWA navigation goals | All succeeded within 0.20 m / 0.20 rad |

The Unity footprint measurement used 10 active unloaded robot colliders and
transformed their local geometry into the axle frame. The saved rectangle is
x `[-0.54, 0.37]`, y `[-0.23, 0.23]` m, with 0.02 m padding. The dormant pallet
is excluded. The measured nominal settled chassis height was 0.05 m.

## Physics smoke tests

These tests simulate the actual saved scenes with the controller's wheel and
friction configuration, without a ROS connection. Commands run for 3 s after
2 s settling; stopping is checked after another 2 s.

| Robot | Command | Forward travel | ROS yaw change |
|---|---|---:|---:|
| XStack | +0.10 m/s | +0.30 m | 0.00 rad |
| XStack | -0.10 m/s | -0.29 m | 0.00 rad |
| XStack | +0.20 rad/s | 0.00 m | +0.48 rad |
| XStack | -0.20 rad/s | 0.00 m | -0.49 rad |
| XStack | +0.03 m/s | +0.09 m | 0.00 rad |
| Legacy | +0.10 m/s | +0.30 m | 0.00 rad |
| Legacy | -0.10 m/s | -0.30 m | 0.00 rad |
| Legacy | +0.20 rad/s | 0.00 m | +0.53 rad |
| Legacy | -0.20 rad/s | 0.00 m | -0.53 rad |
| Legacy | +0.03 m/s | +0.09 m | 0.00 rad |

All cases stopped below 0.03 m/s and remained within 0.05 degrees of upright.
The new robot under-rotates relative to the ideal command, but achieved both
directions and subsequently passed the closed-loop DWA goal checks. This is
measured behavior, not a claim of perfect actuator tracking.

## Live mapping and DWA

The actual ROS TF laser offset was `(0.33, 0, 0.13)` relative to `drive_center`.
At the starting pose, side scan rays agreed with Unity at approximately 5.90 m;
the center ray correctly reported no hit within 10 m. Odometry and TF use the
same ground-projected axle reference.

`milestone_1_01` was generated from two full turns separated by an 8 m forward
aisle traverse. It is a measured 800 x 800 occupancy grid at 0.05 m resolution.
Unobserved/occluded space remains unknown. It is not a claim of complete
warehouse exploration. AMCL ran after stopping gmapping; DWA ran with the
mapping command publisher stopped.

| Goal in map (x, y, yaw) | Position error | Heading error | Action result |
|---|---:|---:|---|
| (8.989, -0.184, 0) | 0.184 m | 0.033 rad | SUCCEEDED |
| (9.989, 0.316, 0) | 0.193 m | 0.181 rad | SUCCEEDED |
| (7.989, -0.184, pi) | 0.190 m | 0.078 rad | SUCCEEDED |

The third goal includes turning and returning along the aisle. These tests
demonstrate functional DWA integration, not general obstacle-avoidance safety.
Transient startup TF extrapolation and repeated gmapping timestamp warnings
occurred; they did not prevent mapping, AMCL, or successful navigation.

## Remaining validation limits

- RViz visual inspection and a dedicated contact/collision audit were not performed.
- Dynamic-obstacle avoidance, loaded-pallet motion and docking integration were
  not exercised. The incoming mission code is preserved, not certified by these
  navigation tests.
- Legacy regression includes editor configuration, physics and profile tests;
  a fresh legacy end-to-end AMCL/DWA run was not performed.
- Extend mapping before using goals in unobserved warehouse aisles.
- The optional `check_urdf` executable was unavailable in the container; URDF
  parentage, connectivity and sensor-mount consistency passed the Python tests.

Local raw evidence is in `D:\Project\Robot\.diagnostics\xstack`: collider
measurement, scene validation, physics CSV, live sensor/mapping report and DWA
goal report. See [startup and repeatable checks](XSTACK_MIGRATION.md).
