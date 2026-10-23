# Pallet Detection, Named Locations & Precision Docking

How Milestone 2 picks a pallet by name, finds it with the laser, and backs its forks into the pocket.
It covers the mechanism first, then [how to run it](#6-running-a-mission).

Code: `amr_navigation/src/amr_navigation/{pallet_detect,docking,locations}.py`, `scripts/milestone2_mission.py`,
Unity exporter `amr_ware_house/Assets/Editor/RosLocationExport.cs`.

---

## 1. Big picture

```
 named location ──► move_base ──► snapshot ──► laser detection ──► turn ──► reverse docking ──► lift
 (YAML, map)        (map, ~20cm)  map→odom     (odom, ~mm)         180°     (odom, ≤ 8 mm gate)
```

| Stage | Frame | Accuracy | Job |
|---|---|---|---|
| move_base | map (AMCL) | 10–20 cm | get near the rack |
| snapshot | map→odom, frozen once | AMCL error, a few cm / ~0.5° | no AMCL jumps from here on |
| detection | odom + front laser | a few mm / < 0.5° | measure the real pallet, removing the AMCL error |
| dock_reverse | odom, 20 Hz | ≤ 8 mm, gate + retries | put the forks exactly in the pocket |

Each layer only has to be good enough for the next one to take over.

The laser is on the **front** and the forks are on the **rear**. So the robot faces the rack to measure the
pallet, then turns 180° to back in.

---

## 2. Named locations

Missions take **names**, not coordinates: `package:=package_5 station:=station_2`.

### 2.1 Where the names come from
`config/locations_milestone_3.yaml` is generated in Unity by **Tools → Export ROS Locations**:

- It finds every `StandardPallet` (using the centre of its `TopDeck`, since the prefab pivot sits on a corner)
  and every `Workbench`.
- It converts them to the ROS map frame with the **same math as `PlanarOdometryPublisher`**, relative to
  `robotBase` (drive_centre) at the spawn. So the YAML, `/odom` at start and a map built from that spawn all agree.
- The dock heading `yaw` is the robot heading while reversing in. It points from the object out to the aisle:
  270° for the north-facing rows, 90° for the south-facing rows, 180° for the stations.
- Names are sorted north to south, then west to east. `height` is the base height:
  0.029 (floor), 0.292, 0.709, 0.972 m.

```yaml
packages:
  package_5: {x: 4.003, y: 5.122, yaw: 270.0, height: 0.029, unity: "Warehouse/North rack/Set1/StandardPallet (9)"}
stations:
  station_2: {x: 13.440, y: 0.000, yaw: 180.0, height: 0.100, unity: "StoreFront/Workbench (1)"}
```

> Export with Play **stopped**. In Play mode the robot has moved away from the spawn, and the exporter refuses to run.
> Re-export whenever objects or the spawn move. Do not hand-edit the file.
> On a real robot, write the same YAML from measured pallet centres; the code doesn't change.

### 2.2 From object centre to drive_center stop
Every entry is a **real object centre**. The docking code stops drive_center a fixed distance in front of it,
along the dock heading:

```
stop = centre + dock_offset · (cos yaw, sin yaw)          # compute_dock_target_pose
```

| Param | Default | Meaning |
|---|---|---|
| `pallet_dock_offset` | 0.342 m | pallet centre → drive_center; forks about 85–90% into the pocket |
| `station_dock_offset` | 0.54 m | Workbench centre → drive_center; pallet centred over the deck |
| `load_face_offset` | 0.15 m | pallet centre → load box front face (used by detection) |

Example, `package_5`: `5.122 − 0.342 = 4.780`. drive_center stops at (4.003, 4.780).

```
  y
5.122  ● pallet centre (YAML)
4.972  ═ load box face            (centre − 0.15)
4.780  ⊕ drive_center dock stop   (centre − 0.342)
3.580  ⊕ stage                    (stop − rack_standoff 1.2)
2.980  ⊕ pre-stage, move_base goal (stage − approach 0.6)
```

A pallet that isn't on the floor (`height` > 0.10 m) is rejected while `pick_level` is `bottom`.

---

## 3. Getting to the rack

1. **move_base → pre-stage** (map frame, 20 cm / 0.2 rad tolerance).
2. **Snapshot.** Wait 1 s for AMCL to settle, read `odom←map` **once** (dx, dy, dth) and freeze it. From here
   on everything runs on smooth `/odom`. Map targets are converted once:
   `x_o = dx + cos(dth)·x − sin(dth)·y`, `y_o = dy + sin(dth)·x + cos(dth)·y`, `yaw_o = yaw + dth`.
   AMCL's error is now baked into these targets, and detection removes it.
3. **MoveToPoint → stage, facing the rack** (heading = dock yaw + 180°, 5 cm / 2°). The laser is then about
   1.07 m from the box face, and the 0.35 m face covers about 37 rays at 0.5°.

---

## 4. Laser pallet detection (`pallet_detect.py`)

The front laser (about 0.13 m high) passes over the pallet blocks (0–0.06 m) and the deck (0.06–0.08 m) and hits
the **load box** (0.35 m wide, 0.30 m deep, centred on the pallet). Its front face is a straight segment of known
width, which gives the pallet centre and heading.

For each scan:

1. **Rays → points in odom.** TF gives the laser pose in odom, and each range `r` at angle `a` becomes a point.
   This doesn't use AMCL.
2. **Region of interest.** Keep the points within ±30 cm sideways and ±20 cm in depth of the *expected* face
   (snapshot pallet + `load_face_offset`). AMCL only has to land the face inside this box.
3. **Median-depth seed.** Keep the points within 3 cm of the median depth. The face holds most of the points, so
   this drops the rack behind and the **side walls of the neighbouring boxes**. Rays through the 7 cm gaps hit
   those walls, and they would pull a plain least-squares fit about 2 cm behind the face.
4. **Line fit, twice.** Fit a line by PCA, drop points more than 1.5 cm off it, and refit.
5. **Split into boxes.** Neighbouring loads (every 0.42 m) are on the same line. Sort the points along it and cut
   at gaps > 3 cm. Keep the segment nearest the expected face.
6. **Checks.** At least 10 points; rms ≤ 1 cm; width 0.35 ± 0.05 m; ≤ 10 cm and ≤ 5° from the expected pose.
7. **Face → pallet.** Normal `n` = perpendicular to the line, pointing at the robot.
   `pallet = face midpoint − 0.15·n`, `dock yaw = direction of n`.

The mission runs this on **5 scans**. It needs at least 3 successes that agree within 5 mm and 1°; otherwise it
**aborts**.

```
>>> PALLET DETECTED (odom): (4.004, 5.002, 270.00 deg) | expected (3.985, 5.012, 270.24) |
    lateral +18.7 mm, depth +10.1 mm, yaw -0.24 deg | 5 scans
```
Here AMCL's estimate was off by 18.7 mm sideways, more than twice the 8 mm fork clearance. The laser
corrected it.

**The targets are then rebuilt from the detection:** `dock = detected centre + 0.342 along the detected yaw`,
`stage = dock − 1.2 m`. The robot turns in place to the detected yaw (8 cm position tolerance; any lateral offset
left over is fixed during the reverse).

---

## 5. Reverse docking (`dock_reverse`)

The lift goes to entry height, then the robot backs along the **pallet axis**: a fixed line in odom through the
dock stop at the detected yaw. Detection and the robot pose are both in odom, so every control step measures the
robot **against the pallet's axis**, whatever the turn left behind.

### 5.1 Line-following law (`reverse_line_command`, every 50 ms)
```
e_lat   = sideways offset from the axis        e_yaw = heading error
yaw_des = atan(2.5 · e_lat), clamped ±10°      ← lean slightly toward the axis
w       = 1.6 · (yaw_des − e_yaw), clamped ±0.18 rad/s
v       = −max(0.07, 0.7 · remaining), capped at dock_speed
```
When reversing, the sideways error changes at `v · sin(e_yaw)`. The robot leans toward the axis, slides onto it,
and as `e_lat → 0` the lean goes to 0, so it ends straight and centred, much like parallel parking. The error
shrinks with a time constant of about `1/(2.5·|v|)` ≈ 5 s. The 1.2 m runway is about 3 time constants, which
removes about 95% of it. The tight turn-rate clamp stops the forks swinging into the pallet blocks.

### 5.2 Entry gate (rack)
With 0.50 m left to go (about 13 cm before the tips touch the pocket):
`drive_center lateral = e_lat`, `tip lateral = e_lat − 0.54·sin(e_yaw)`. Both must be ≤ **8 mm**. Otherwise the
robot pulls forward to the start and retries (2 retries), then aborts.

### 5.3 Finish and stall guard
- Done when the remaining distance ≤ 0, or the distance ≤ 4 cm; then the final lateral offset is checked (≤ 8 mm, rack).
- **Stall guard:** reversing without 5 mm of progress for 1.5 s (sim time) means something is in the way.
  - **Station** (`station_contact_tol` 0.10 m): a stall with ≤ 0.10 m left counts as docked
    (`PALLET DOCK COMPLETE (contact, 0.062m short)`). The mission lowers the pallet and undocks; the pallet is
    still fully on the 0.8 m deck.
  - **Rack**, or any other stall: stop, log `PALLET DOCK FAILED (stalled)`, and abort. The robot never pushes forever.

Then: lift to pick height, `undock` straight forward to the stage with the heading locked, transit height,
move_base to the station.

---

## 6. Running a mission

### 6.1 One-time: export the locations
1. [Unity] Open the scene (`Milestone_3`). **Play must be stopped.**
2. [Unity] **Tools → Export ROS Locations**, then save to
   `catkin_ws/src/amr_navigation/config/locations_milestone_3.yaml`.
3. [Unity] Console: `[ROS Export] 128 packages, 3 stations -> ...`

### 6.2 Each run
1. [Unity] Laser **Ray Count 361** (0.5°; 181 rays limits detection to about 8 mm), then **Play**.
2. [Inside ROS Docker container]
   ```bash
   roslaunch amr_navigation warehouse_autonav.launch launch_rviz:=true
   ```
3. [Inside ROS Docker container] in a second terminal:
   ```bash
   roslaunch amr_navigation milestone2.launch                                       # package_5 → station_2
   roslaunch amr_navigation milestone2.launch package:=package_21 station:=station_1
   roslaunch amr_navigation milestone2.launch package:=package_101                  # south rack, yaw 90°
   ```
   Pick names from the YAML. Only floor pallets (`height: 0.029`) work with the default `pick_level:=bottom`.

> After every Unity Stop/Play, restart `warehouse_autonav.launch`. `/clock` restarts, and move_base would otherwise
> reject goals (state 8, RECALLED).

Other launches that take names (they run on `/odom` only, so start them from the spawn):
```bash
roslaunch amr_navigation milestone1.launch package:=package_5 station:=station_2
roslaunch amr_navigation test_dock_reverse.launch package:=package_5
roslaunch amr_navigation test_undock_reverse.launch station:=station_2
```

### 6.3 Useful arguments / params

| Name | Default | Notes |
|---|---|---|
| `package`, `station` | `package_5`, `station_2` | entries in `locations_file` |
| `locations_file` | `config/locations_milestone_3.yaml` | export per scene |
| `detect_pallet` | `true` | `false` docks on the snapshot alone (no laser check) |
| `pallet_dock_offset` / `station_dock_offset` | 0.342 / 0.54 m | insertion depth |
| `~load_face_offset`, `~load_width` | 0.15 / 0.35 m | load box geometry (script params) |
| `~station_contact_tol` | 0.10 m | stall this close to the stop counts as docked |
| `rack_standoff` / `standoff` | 1.2 / 0.8 m | reverse runway at rack / station |

### 6.4 Reading the log

| Line | Meaning |
|---|---|
| `MAP->ODOM SNAPSHOT: Rack \| dx dy dth` | frozen AMCL correction |
| `[Detect] scan rejected: <reason>` | one scan failed (only a problem if fewer than 3 of 5 pass) |
| `PALLET DETECTED (odom): ... lateral, depth, yaw` | how far AMCL was from the real pallet |
| `PALLET DETECTION FAILED` | mission aborts: no or wrong load, or the face is too far from expected |
| `Entry check failed ... (attempt n/3)` | forks not lined up; pulling out to retry |
| `PALLET DOCK COMPLETE: Rack Insertion ... Lateral` | final fork alignment (≤ 8 mm) |
| `PALLET DOCK COMPLETE (contact, …short)` | touched the station early; continuing |
| `PALLET DOCK FAILED (stalled)` | blocked; mission aborted |

Common detection rejects: `only N points ...` (load missing, or the laser isn't facing it; check Ray Count),
`face width ...` (wrong object or merged boxes), `detected pallet X m from expected` (AMCL far off, or the wrong package name).

---

## 7. Limits and next steps
- **Blind reverse.** After detection the robot can't see the pallet (the laser is on the front). For about 1.5 m
  it trusts odom. Unity odom is perfect; on a real robot a 1° heading drift during the 180° turn gives about
  21 mm at 1.2 m. Next: inject odom noise in Unity to measure it, then an **IMU** (heading) and/or a **rear
  sensor** to re-detect the pallet while reversing.
- **Station has no detection.** It relies on the AMCL snapshot (a few cm). The same face fit could measure the bench front.
- **Map near the station:** `milestone_3_02` draws the bench front about 16 cm closer than Unity does (x ≈ 13). The
  cause is unresolved; it doesn't affect the dock offsets, which were measured on ground-truth odom.
- **Floor level only:** upper pallets are above the laser plane, and the bottom lift profile can't reach them.
- **Cargo-dependent detection (Empty Bays):** The current scan plane ($Z \approx 0.22\text{ m}$) tracks the blue cargo box face. If a bay is empty, detection fails. See [RACK_LEG_DOCKING_PROPOSAL.md](RACK_LEG_DOCKING_PROPOSAL.md) for the technical analysis and migration plan to Rack Leg Pair Detection ($Z \approx 0.08\text{ m}$).
