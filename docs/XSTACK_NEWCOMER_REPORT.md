# XStack migration: newcomer handover

Prepared on 2026-10-04. Read this first for the explanation, then use the
[startup guide](XSTACK_MIGRATION.md) for commands and the
[verification report](XSTACK_VERIFICATION.md) for measured results.

## What was accomplished

The project now supports your friend's XStack robot as the default navigation
robot, while retaining the previous robot for comparison. I resolved the merge
conflicts, connected the new robot's sensors and coordinate frames to ROS,
measured its collision footprint, created a map from actual simulated laser
data, and tested navigation.

Both repositories have their migration changes staged. **Staged means prepared
for a Git commit; it does not mean committed.** The merge commits were deliberately
left for review. No push was performed as part of this migration.

The existing architecture remains:

```text
Unity: robot, warehouse, physics and sensors
       | laser scans, pose, simulation time
       v
ROS-TCP bridge: transports messages between Unity and ROS
       v
ROS in Docker: localization, maps and navigation
       | velocity commands
       v
Unity controller: turns commands into wheel movement
```

RViz is the inspection window for what ROS believes. It does not simulate the
robot's physics or drive the robot by itself.

## The important new concepts

**A coordinate frame is a named position and orientation.** It lets software
say where something is relative to something else. TF is the collection of
relationships between these frames.

The new robot's wheel axle is 10 cm behind the chassis reference called
`base_link`. Navigation now follows the axle midpoint projected onto the floor,
called `drive_center`. The Unity helper object is spelled `drive_centre`; its
configured ROS frame name is `drive_center`. This spelling difference is
intentional and does not require renaming Unity objects.

```text
map -> odom -> drive_center -> base_link -> laser_link
```

| Name | Meaning for a newcomer |
|---|---|
| `map` | Coordinates in the saved warehouse map |
| `odom` | A local reference for tracking movement; the current Unity publisher starts it at the robot's initial pose |
| `drive_center` | Ground-projected midpoint between XStack's drive wheels |
| `base_link` | The chassis reference, 10 cm ahead and nominally 5 cm above `drive_center` |
| `laser_link` | The laser sensor reference |

This is a change to how ROS describes the robot. I did not reparent Unity's
articulation bodies or replace the physics model. The ROS robot description
has been adapted separately; do not import it over the existing Unity robot.

**Odometry** is the robot's movement estimate. Here it comes from Unity's actual
pose, not simulated wheel encoders. **Localization**, handled by AMCL, estimates
where the robot is in a saved map. **Gmapping** builds a map from sensor data.

**A footprint** is the robot's outline viewed from above. Navigation uses it
when checking whether the robot fits through free space. XStack's unloaded
outline extends 0.37 m forward, 0.54 m backward, and 0.23 m to either side of
`drive_center`, with another 0.02 m of padding. A carried pallet may extend
beyond this outline; loaded navigation has not been validated.

**DWA** is the local planner: it evaluates short possible motions and chooses
a velocity command. The algorithm is unchanged. The new reference makes its
pose, velocity and footprint agree with the new axle position. This does not
promise perfect motion or universal obstacle avoidance, but the tested DWA
goals worked with the new setup.

## Choose one matching setup

A robot profile is a named bundle of ROS settings. It does not change the
Unity scene for you.

| Selection | New setup | Previous setup |
|---|---|---|
| Unity scene | `Milestone_1` | `WarehouseTraining` |
| ROS argument | `robot_profile:=xstack` | `robot_profile:=legacy` |
| Navigation base | `drive_center` | `base_footprint` |
| Saved map | `milestone_1_01.yaml` | `warehouse_training_01.yaml` |
| Wheel spacing | 0.40 m | 0.44 m |

`SampleScene` is also preserved as a legacy sensor/testing scene. Use
`WarehouseTraining` with its matching map for legacy localization; the presence
of a scene does not mean another scene's map matches it.

The new map is a real gmapping output. It was captured using a complete turn,
an 8 m aisle traverse, and another complete turn. It covers observed areas,
not every part of the warehouse. Unknown regions are not proven free space.

## What to expect when you run it

Opening Unity alone will not start autonomous navigation. You must start ROS,
the bridge, sensor mounts, localization, and navigation, then send a goal.
The temporary test processes were stopped after testing, and the ROS container
was returned to its previous stopped state.

For a first session, use this order:

1. **[Windows]** Start Docker Desktop and wait until its engine is running.
2. **[PowerShell]** Run `docker start ros1_amr_core`, then
   `docker exec -it ros1_amr_core bash`. This enters the existing ROS container.
3. **[Inside ROS Docker container]** Source `/opt/ros/noetic/setup.bash` and
   `/catkin_ws/devel/setup.bash`, then start `unity_bridge.launch`.
4. **[A second ROS container terminal]** Source the same environment and start
   `laser_tf.launch robot_profile:=xstack`. This publishes the sensor mounts.
5. **[Unity]** Open `Milestone_1` and enter Play Mode. Keep the demonstration
   payload inactive. The connector should connect to `127.0.0.1:10000` using ROS1.
6. **[ROS]** Check that `/odom` and `/scan` are arriving. Expected rates are
   approximately 20 Hz and 10 Hz. The laser has 181 rays across 180 degrees,
   with a maximum range of 10 m.
7. **[ROS]** Start `warehouse_localization.launch robot_profile:=xstack`.
   It selects the new map by default.
8. **[RViz]** Set the fixed frame to `map`, display the map and scan, and use
   **2D Pose Estimate** to initialize the robot near its actual mapped position.
   Confirm that scan points agree with walls before driving autonomously.
9. **[ROS]** Start `warehouse_navigation.launch robot_profile:=xstack`.
   **[RViz]** Send a short **2D Nav Goal** in known free space.

Use the [startup guide](XSTACK_MIGRATION.md) for the complete copyable launch
commands. Each long-running launch stays in its own terminal. For each new
container shell, source the ROS and catkin setup files again.

You should see the robot navigate its axle-center reference toward the goal,
then stop within the configured tolerance. The current tolerance is 0.20 m
position and 0.20 rad heading (about 11.5 degrees), so success does **not** mean
exactly touching the goal marker. This is not yet precision pallet docking.

## Key things to remember

- **Match scene, profile and map.** Choosing `xstack` in ROS does not replace a
  legacy robot that is still open in Unity.
- **Use one motion controller at a time.** Keyboard input, DWA, the point
  controllers and mission/docking code can compete over robot movement.
- **Use one map-to-odometry publisher.** Stop gmapping before starting AMCL.
  Mapping and localization are separate operating modes.
- **Avoid duplicate sensor transforms.** The normal navigation startup uses
  `laser_tf.launch`. Do not also start `display.launch`, which uses
  robot_state_publisher for standalone model inspection.
- **Restart the navigation/localization session after restarting Unity Play
  Mode.** Unity's simulation clock and local odometry origin restart.
- **Map coordinates are not Unity world coordinates.** Use RViz/map coordinates
  for navigation goals. Goals locate the axle midpoint, not the fork tips.
- **Keep the baseline unloaded.** Heading hold and automatic rear-caster removal
  are disabled. Do not enable them just because the options exist.
- **A passed test is evidence for the tested case.** Dynamic obstacles, loaded
  operation and mission-level docking remain separate validation tasks.

## What was verified, and what is still open

Unity compiled successfully. All three retained scenes passed editor checks.
ROS built successfully, and the automated suite passed **31 tests**. Both robots
passed basic forward, reverse, turning, low-speed and stopping checks in Unity
physics.

Live checks verified the clock, odometry, laser samples and TF mounts. A map was
saved from the live data. AMCL initialized successfully. All three DWA goals
succeeded, with position errors of 0.184–0.193 m and heading errors of
0.033–0.181 rad.

Still open: a human RViz visual review, a dedicated collision/contact audit,
dynamic-obstacle testing, loaded-pallet and docking integration tests, broader
warehouse mapping, and a fresh end-to-end legacy localization/navigation run.
See the [verification report](XSTACK_VERIFICATION.md) for exact evidence and
limitations.

## What is `.diagnostics`, and do we need it?

The exact folder name is **`.diagnostics`**, located at
`D:\Project\Robot\.diagnostics`. It is a local development/test workspace, not
a ROS package or a Unity runtime requirement. The leading dot is a naming
convention for auxiliary folders; it does not make it part of the robot.

| Contents | Purpose | Needed for normal operation? |
|---|---|---|
| `xstack/` reports and JSON/CSV snapshots | Raw evidence from geometry, physics, sensor and goal checks | No |
| `xstack-*.log` and other logs | Compiler/editor/test troubleshooting history | No |
| Migration and probe Python scripts | One-off migration work and live test orchestration | No |
| `UnityPhysicsLab/` | Earlier isolated physics experiments, including a separate Unity project's generated files | No |
| Backups such as `StackerController.before.cs` | Comparison and investigation material | No |

The real source code, scenes, robot profiles and saved map live in the two
repositories. Deleting `.diagnostics` would not undo the migration or remove
`milestone_1_01` from the ROS package.

**Recommendation: keep it through review, then archive or clean it when no
diagnostic run is active.** Save `xstack/` if you want the raw evidence behind
the report. The older physics lab can contain substantial generated Unity
caches; it is not another project you must open for everyday navigation.

The tracked editor utilities `XStackMigration.cs` and `XStackRuntimeCheck.cs`
are different from the output folder: they live under `Assets/Editor` and
should remain in Git so checks can be repeated. They recreate the output
directory when invoked. Previously recorded measurements and one-off scripts
are not automatically recovered if deleted.

These utilities do not require a migration rerun before normal Play Mode.
`Validate` checks saved configuration; `ValidateMotion` exercises physics;
`ConfigureAndValidate` **changes assets and the footprint configuration**.
Do not casually rerun the one-off migration scripts as a startup step.

The outer `Robot` folder is not either repository's Git root. Its
`.diagnostics` folder is outside both repositories and is not included by
staging files inside them. Share the tracked reports and source changes with
teammates; they do not need the entire local diagnostics workspace.
