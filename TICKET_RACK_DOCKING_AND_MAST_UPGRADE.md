# [TICKET] Rack Docking Architecture & AMR Mast Extension for Tier-4 Pallets

> [!IMPORTANT]
> **DIRECTIVE FOR THE CODING AGENT PROCESSING THIS TICKET:**  
> When you (the agent) receive this ticket, you **MUST immediately ask your user** to make decisions on the following two prerequisites before executing any code changes:
> 1. **Warehouse Remapping (`Assets/Scenes/NewMap_noObs.unity`):**  
>    *Ask the user:* "The scene `Assets/Scenes/NewMap_noObs.unity` removes the static clutter obstacles. Do you want to run `warehouse_mapping.launch` now to record and generate a clean new `.yaml` / `.pgm` costmap without static obstacles?"
> 2. **LiDAR Ray Count Configuration (361 vs 181 rays):**  
>    *Ask the user:* "Do you want to run the 2D LiDAR with **361 rays** (0.5° resolution, higher density for the 8 mm precision entry gate) or stay at **181 rays** (1.0° resolution)? If 361 is selected, verify that `PlanarLaserScanPublisher` in Unity Inspector has `Ray Count = 361`."

**Status:** Open / Proposed  
**Priority:** High (Blocks Top-Shelf Picking & Empty-Bay Docking)  
**Assigned To:** Engineering Team  
**Authoritative Documentation References:**  
- 📖 [`docs/WAREHOUSE_METRICS_AND_CLEARANCE.md`](docs/WAREHOUSE_METRICS_AND_CLEARANCE.md) — Comprehensive metric clearances, 4-tier heights, and lift kinematics.
- 📖 [`docs/RACK_LEG_DOCKING_PROPOSAL.md`](docs/RACK_LEG_DOCKING_PROPOSAL.md) — Mathematical formulation and algorithm design for Rack Leg Pair detection.
- 📖 [`docs/pallet_detection.md`](docs/pallet_detection.md) — Milestone 2 mission architecture, snapshot math, and 20 Hz line-following controller.

---

## 1. Summary of Changes & Problem Statement

Following the recent map and rack updates in `amr_ware_house`:
1. Two new rack prefabs were introduced:
   - **`Rack-DetectReady1`**: features small indicator lips/shields on the front beam.
   - **`Rack-DetectReady2`**: features 5 structural floor legs forming 4 distinct bay openings.
2. Pallet elevations now span **4 vertical tiers** ($0.11\text{ m}$, $0.37\text{ m}$, $0.78\text{ m}$, and $1.05\text{ m}$).
3. **Critical Physical Blocker:** The AMR robot's mast is currently $0.85\text{ m}$ tall and software-limited to $0.75\text{ m}$ lift stroke. It **physically cannot reach the top shelf pallets at $1.05\text{ m}$** (it falls short by $\approx 30\text{ cm}$).
4. **Detection Limitation:** Our current Milestone 2 laser detection (`detect_load_face`) tracks the front plastic face of the blue cargo box. If a bay is empty (or when docking to deposit a pallet), the laser shoots straight into the back wall and the mission aborts.

---

## 2. Part A: Rack Prefab Evaluation & Detection Architecture

We evaluated both new rack prefabs for laser docking perception:

### Option 1: `Rack-DetectReady1` (Front Lips / Shields) — NOT RECOMMENDED
* **The "Behind-the-Lip" Trap (Split-Depth Artifact):**  
  In this prefab, pallets sit directly behind the front shield ($3-5\text{ cm}$ gap). Laser rays that graze past the lip edges strike the pallet face slightly deeper:
  $$d_{\text{lip}} = 1.10\text{ m} \quad \text{vs} \quad d_{\text{pallet}} = 1.14\text{ m}$$
  Because 2D LiDAR has beam divergence and angular resolution steps, this produces a **staircase/dual-plane point cloud**. A standard PCA line fit across these points tilts diagonally and exceeds the straightness limit ($\text{RMS} > 3\text{ cm}$ vs threshold $1\text{ cm}$), causing scans to be falsely rejected.
* **Mechanical Snag Risk:** Any front lip tall enough to reliably bounce 2D laser beams risks catching the bottom of the fork carriage during ground-level insertion ($Z \approx 20\text{ mm}$).

### Option 2: `Rack-DetectReady2` (Bottom Legs / Pillar Pair) — RECOMMENDED
* **Clean Laser Signature:**  
  The 5 bottom legs are spaced across the $1.80\text{ m}$ rack at **$0.420\text{ m}$ pitch**.  
  Each bay is flanked by two $0.08\text{ m}$ wide steel legs, leaving a **$0.340\text{ m}$ completely unobstructed corridor** for the forks.
* **Invariant to Bay Contents:**  
  The laser detects the **two vertical steel pillars** of the bay. The signature is 100% identical whether the bay is full, holds an empty pallet, or is completely empty. This allows micro-perception docking for both **pick** and **deposit** missions.

---

## 3. Part B: AMR Mast & Lift Height Extension (Physical Upgrade)

### The Math:
From `Rack-DetectReady1` and `Rack-DetectReady2`, the 4 storage tier elevations are:
- **Tier 1 (Lower Bay, Bottom Pallet):** $Y_{\text{base}} \approx 0.110\text{ m}$ $\rightarrow$ Pick stroke: $q = +0.090\text{ m}$
- **Tier 2 (Lower Bay, Stacked Pallet):** $Y_{\text{base}} \approx 0.373\text{ m}$ $\rightarrow$ Pick stroke: $q = +0.353\text{ m}$
- **Tier 3 (Upper Shelf, Lower Pallet):** $Y_{\text{base}} \approx 0.781\text{ m}$ $\rightarrow$ Pick stroke: $q = +0.761\text{ m}$ (exceeds current $0.75\text{ m}$ limit!)
- **Tier 4 (Upper Shelf, Top Pallet):** $Y_{\text{base}} \approx 1.044\text{ m}$ $\rightarrow$ Pick stroke: $q = \mathbf{+1.024\text{ m}}$ (**FAIL: Current max is $+0.750\text{ m}$**)

### Applied Tuned Values:
- **`xstack_amr.urdf`**: `mast_link` box height set to **`0.90m`** (`origin xyz="0 0 0.45"`), `lift_joint` upper limit set to **`0.80m`**.
- **`StackerController.cs`**: `maxLiftHeight = 0.80f;`
- **`xstack_amr.prefab`**: visual/collision scale Y set to **`0.90m`** (`pos Y = 0.45m`), ArticulationBody upperLimit set to **`0.80m`**.

*Result:* Tiers 1, 2, and 3 ($q_{\text{pick}} \le 0.761\text{ m}$) are **100% supported** without making the single-stage mast awkwardly tall. Tier 4 ($1.05\text{ m}$) is recognized as an optional future reach target (which would require a 2-stage telescopic mast).

---

## 4. Part C: Dynamic Location Export Workflow (`RosLocationExport.cs`)

Because pallet elevations shift with different rack models, coordinates must not be hardcoded in mission scripts.

The newly created `Assets/Editor/RosLocationExport.cs` solves this cleanly:
1. When scene modifications are complete, select **Tools → Export ROS Locations** in the Unity Editor toolbar (with Play mode **stopped**).
2. The script queries every `StandardPallet` in the scene, computes its `/odom` coordinates and exact base height $Y$, and exports them to:
   `catkin_ws/src/amr_navigation/config/locations_milestone_3.yaml`
3. Missions can then pick any pallet by semantic name:
   ```bash
   roslaunch amr_navigation milestone2.launch package:=package_42 station:=station_1
   ```
4. The mission script automatically selects the appropriate lift profile (`bottom`, `stack`, `shelf`, `top_shelf`) based on the exported `height`.

---

## 5. Implementation Action Items & Checklist

- [x] **Action 0: Interactive User Alignment (Agent Prompting)**
  - [x] Prompt user: Remap warehouse from `Assets/Scenes/NewMap_noObs.unity` (without static obstacles).
  - [x] Prompt user: Confirm 361 LiDAR rays (0.5° resolution) vs 181 rays in Unity Inspector.
  - [x] Remap warehouse from `NewMapnoObs.unity` → `maps/new_map_noobs_01.yaml/.pgm` (now the xstack default map).
- [x] **Action 1: Unity Robot Mast & Joint Extension**
  - [x] Update `Assets/StackerController.cs` line 21: `maxLiftHeight = 1.15f;`.
  - [x] Update `xstack_amr.prefab` mast mesh and ArticulationBody limits.
  - [x] Verify manual lift test (pressing `R` in Unity editor reaches above the top shelf deck).
- [x] **Action 2: URDF Synchronization**
  - [x] Update `xstack_amr.urdf` mast box height ($1.25\text{ m}$) and prismatic limit ($1.15\text{ m}$).
- [x] **Action 3: Adopt `Rack-DetectReady2` as Primary Rack Prefab**
  - [x] Standardize scene layout on `Rack-DetectReady2` (with the 5 floor legs).
  - [x] Re-run **Tools → Export ROS Locations** to generate the updated `locations_milestone_3.yaml`.
- [x] **Action 4: Implement Rack Leg Pair Detection Algorithm**
  - [x] Add `detect_rack_legs()` in `pallet_detect.py` to cluster the two leg posts at $0.42\text{ m}$ span.
  - [x] Add automated unit tests in `test_pallet_detect.py` verifying leg detection on empty and loaded bays.

---

## 6. Sprint Results (2026-10-26)

**Delivered beyond the original checklist:**
- **Lift heights from the export:** entry = pallet `height` − 0.040 m, pick = entry + 0.050 m (fixed table and `pick_level` removed). The old 20 mm rise only lifted pallets that dropped at Play start; `Rack-DetectReady2` pallets are now seated on their supports so exported heights match physics (0.112 / 0.392 / 0.782 / 1.062 m).
- **Pick units + inventory:** picking a pallet takes everything stacked on it (same bay, < 0.30 m apart); picked slots go to `/warehouse_inventory/empty`, empty slots are refused, `reset_inventory:=true` after restarting Play. Box presence remains the warehouse's responsibility.
- **Station detection:** `detect_station()` measures the workbench face before the reverse, so station docks also correct AMCL error (`detect_station:=false` restores coordinate-only docking). The robot then drives onto the measured line, measures again, and reverses with a 1.2 m runway and the rack's 8 mm entry check, so loads land square (0.1 mm / 0.02°).
- **Rack fixes:** runners re-centred on their bays, upper shelf aligned, 3 `Rack-DetectReady1` replaced; new map `new_map_noobs_01` is the xstack default.

**End-to-end runs in `NewMapnoObs` (Unity + ROS):**

| Package | Tier | Carried | AMCL error corrected | Result |
|---|---|---|---|---|
| package_1 | 1 | 2 pallets | 8.5 mm (rack) | delivered to station_1 |
| package_4 | 2 | – (bay already empty) | 18 mm (rack) | empty-bay detection confirmed |
| package_3 | 4 | 1 pallet | 43 mm (rack) | delivered to station_2 |
| package_2 | 3 | 1 pallet (top gone) | 17 mm (rack) | delivered to station_3 |
| package_33 | 1 | 2 pallets | 35 mm (rack) | delivered to station_1 |
| package_5 | 1 | 2 pallets | 32 mm (rack), 73 mm (station) | delivered to station_2, station detected within 1 mm |
| package_9 | 1 | 2 pallets | 47 mm (rack), 88 mm (station) | delivered to station_1 (after station limit fix) |
| package_13 | 1 | 2 pallets | 4 mm (rack), 115 mm (station) | delivered to station_3 (after station limit fix) |
| package_5 | 1 | 2 pallets | 75 mm (station) | station_2 with alignment: 0.1 mm / 0.02° (was 5.9 mm / 1.27°) |

**Known limits / follow-ups:**
- AMCL is 7-12 cm off in the station room (vs 1-5 cm at the racks). `detect_station` allows 0.20 m (racks 0.10 m, which first rejected station_1/3); worth checking AMCL tuning or map quality near the workbenches.
- Stacked pallets are 0.280 m apart vs the 0.30 m stack threshold; consider 0.335 m (midpoint to the 0.390 m shelf gap).
- After restarting Unity Play, restart localization + navigation (sim clock resets) and use `reset_inventory:=true`.
- `milestone1_mission.py` / `test_dock_reverse.py` still use the old `pick_level` table.
