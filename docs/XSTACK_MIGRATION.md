# XStack migration and startup

XStack is the default navigation robot. Use `Milestone_1` in the adjacent Unity
project with `robot_profile:=xstack`. Use `WarehouseTraining` or `SampleScene`
with `robot_profile:=legacy`. Run one Unity scene and one robot at a time.

## Frame and geometry contract

| Profile | Odometry child / navigation base | Pose tracked in Unity | Chassis mount (ROS xyz) | Laser mount from chassis |
|---|---|---|---|---|
| xstack | `drive_center` | moving `drive_centre` | `(0.10, 0, 0.05)` | `(0.23, 0, 0.08)` |
| legacy | `base_footprint` | moving `base_link` | `(0, 0, 0.05)` | `(0.25, 0, 0.225)` |

`drive_center` is the **ground projection** of the axle midpoint, not the
axle-height link in the original Unity import URDF. The helper's height does
not enter planar odometry. `base_link` is 10 cm ahead of the new reference.
The Unity articulation hierarchy is unchanged. The ROS adaptation is
`amr_description/urdf/xstack_amr.urdf`; do not reimport it over the Unity prefab.

Navigation TF is `map -> odom -> drive_center -> base_link -> laser_link`.
Unity owns `odom -> drive_center`, `/odom`, and `/clock`. `laser_tf.launch`
owns the fixed mounts. Gmapping OR AMCL owns `map -> odom`, never both.
`display.launch` is standalone robot-description inspection using
robot_state_publisher and synthetic GUI joint positions; stop the static
mount launch before using it. It is not a source of measured lift/wheel state.

The unloaded XStack collider envelope, measured in Unity, is x `[-0.54, 0.37]`,
y `[-0.23, 0.23]` metres plus 0.02 m padding. Its dormant demonstration pallet
is excluded. Loaded operation requires a new envelope. The ROS profile and
Unity footprint preview use identical values. RobotBody colliders remain
physical but are excluded from scan rays. Scan: 181 rays, 180 degrees, 10 Hz,
0.05–10 m. Odometry: 20 Hz. Heading hold and rear-caster removal are disabled.

The new mission/docking code is retained alongside the lesson controller.
Do not run mission, docking, `move_to_point`, `go_to_point`, keyboard driving,
and `move_base` concurrently: they can all command the same robot.

## Start the simulator and ROS

**[PowerShell]** Start the existing container and enter it:

```powershell
docker start ros1_amr_core
docker exec -it ros1_amr_core bash
```

**[Inside ROS Docker container]** In each new terminal, source the environment:

```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
```

After code changes, build with `cd /catkin_ws && catkin_make`. Expect a
successful build of `amr_description`, `amr_navigation`, and the TCP endpoint.

**[Inside ROS Docker container, terminal 1]** Start the bridge; roslaunch starts
the ROS master if needed:

```bash
roslaunch amr_navigation unity_bridge.launch
```

**[Inside ROS Docker container, terminal 2]** Start sensor mounts:

```bash
roslaunch amr_navigation laser_tf.launch robot_profile:=xstack
```

**[Unity]** Open `Assets/Scenes/Milestone_1.unity`, leave the demonstration
payload inactive and dynamic-obstacle Animator components disabled, and enter
Play Mode. The connector uses ROS1 at `127.0.0.1:10000`.

**[Inside ROS Docker container, terminal 3]** Check data before mapping:

```bash
rostopic hz /odom
rostopic hz /scan
rosrun tf tf_echo drive_center laser_link
```

Run these checks individually, stopping each with Ctrl+C. Expect approximately
20 Hz odometry, 10 Hz scans, and laser translation `(0.33, 0, 0.13)` with zero
relative rotation. Verify scan/wall alignment and the footprint in RViz.

## Mapping, localization and navigation

**[Inside ROS Docker container, terminal 3]** With AMCL stopped:

```bash
roslaunch amr_navigation warehouse_mapping.launch robot_profile:=xstack
```

**[Unity]** Drive slowly through clear aisles with WASD, sweeping the sensor
view around corners. Keep animated obstacles disabled while building the map.

**[Inside ROS Docker container, terminal 4]** Save the actual map when coverage
is sufficient. Use a new filename for subsequent mapping runs:

```bash
rosrun map_server map_saver -f /catkin_ws/src/amr_navigation/maps/milestone_1_01
```

Stop gmapping with Ctrl+C before starting AMCL. XStack now defaults to the
measured `milestone_1_01` map; legacy defaults to `warehouse_training_01`.
The command below makes the selection explicit. Never use the legacy map with
Milestone_1. The new map covers the observed starting aisle and its visible
surroundings, not every occluded warehouse area; extend it before setting goals
in unexplored aisles.

```bash
roslaunch amr_navigation warehouse_localization.launch robot_profile:=xstack map_file:=/catkin_ws/src/amr_navigation/maps/milestone_1_01.yaml
```

**[RViz]** Set Fixed Frame to `map`, add Map, LaserScan, TF and costmap displays.
Set the initial estimate using **2D Pose Estimate**. Map coordinates are not
Unity world coordinates. Confirm that scans align with the saved map.

**[Inside ROS Docker container, separate terminal]** Inspect costmaps without
commanding the robot:

```bash
roslaunch amr_navigation costmap_preview.launch robot_profile:=xstack
```

Stop the preview, release keyboard controls, and stop all other controllers
before enabling navigation:

```bash
roslaunch amr_navigation warehouse_navigation.launch robot_profile:=xstack
```

**[RViz]** Send a short **2D Nav Goal** in known free space first, then aisle
goals. Goals refer to the axle midpoint. Acceptance is 0.20 m position and
0.20 rad heading, without collisions or persistent TF errors. Only after this
baseline passes, re-enable the dynamic-obstacle Animator instances and check
scan marking/clearing and avoidance.

For regression, stop these nodes and Unity Play Mode, switch to
`WarehouseTraining`, use `robot_profile:=legacy` consistently, and explicitly
select `warehouse_training_01.yaml` for localization. Restart ROS nodes after
restarting Unity because its simulation clock resets.

## Repeatable checks

**[Inside ROS Docker container]** These tests start no robot nodes:

```bash
python3 /catkin_ws/src/amr_navigation/scripts/test_go_to_point.py -v
python3 /catkin_ws/src/amr_navigation/tests/test_robot_profiles.py -v
python3 -m unittest discover -s /catkin_ws/src/amr_navigation/tests -p 'test_move_to_point.py' -v
python3 -m unittest discover -s /catkin_ws/src/amr_navigation/tests -p 'test_docking.py' -v
```

**[PowerShell, Unity editor closed]** Compile and validate the three saved
scenes without saving changes:

```powershell
& 'D:\Project\2022.3.62f3\Editor\Unity.exe' -batchmode -nographics -quit -projectPath 'D:\Project\Robot\amr_ware_house' -executeMethod XStackMigration.Validate -logFile 'D:\Project\Robot\.diagnostics\xstack-validation.log'
```

Use `XStackMigration.ValidateMotion` instead for offline physics checks. Reports
go to `D:\Project\Robot\.diagnostics\xstack`. `ConfigureAndValidate` is the
explicit asset migration command: it rewrites the XStack prefab, scene Animator
overrides, and ROS footprint profile; do not use it as a read-only check.
`XStackRuntimeCheck.Run` is an opt-in batch Play Mode harness, bounded to ten
minutes; it runs the real publishers and writes raycast samples. It does not
command motion. Creating `.diagnostics/xstack/stop-runtime.txt` ends that harness.
