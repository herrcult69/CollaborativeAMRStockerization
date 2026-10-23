# Autonomous Mobile Robot (AMR) Warehouse Metrics, Kinematics & Clearance Documentation

This document provides the authoritative physical metrics, vertical clearances, and kinematic transformations for the **XStack Autonomous Pallet Stacker AMR**, the **Warehouse Racks (`Rack-DetectReady1` and `Rack-DetectReady2`)**, and the **Standard Pallet / Cargo Units** in the `amr_ware_house` simulation and ROS 1 navigation stack.

---

## 1. Executive Summary & Core Dimensions

| Entity | Parameter | Metric Value | Notes |
| :--- | :--- | :--- | :--- |
| **Standard Pallet** | Base Footprint ($W \times D$) | $0.400\text{ m} \times 0.350\text{ m}$ | Standard European-style mini pallet |
| | Empty Pallet Height (Deck Top) | $0.080\text{ m}$ | $80\text{ mm}$ total pallet structure |
| | Fork Entry Pocket Cavity Width | $0.310\text{ m}$ | Pocket width between outer blocks |
| | Fork Entry Pocket Cavity Height | $0.060\text{ m}$ | $60\text{ mm}$ fork insertion vertical clearance |
| | Top Deck Slat Thickness | $0.020\text{ m}$ | $20\text{ mm}$ load platform board |
| **Cargo Box (ToteBox)** | Box Dimensions ($W \times H \times D$) | $0.350\text{ m} \times 0.200\text{ m} \times 0.300\text{ m}$ | Standard Blue Tote Box |
| | Single Loaded Unit Height | $0.280\text{ m}$ | $80\text{ mm}$ pallet + $200\text{ mm}$ box |
| | Double Stacked Unit Height | $0.560\text{ m}$ | Two pallet+box tiers stacked directly |
| **Rack-DetectReady1** | Structure Type | **Front Lip / Shielded Lip** | Features front indicator shields |
| | Front Lip Dimensions ($W \times H \times D$) | $0.070\text{ m} \times 0.050\text{ m} \times 0.016\text{ m}$ | Tabs on front runner beam |
| **Rack-DetectReady2** | Structure Type | **Bottom Legs / Pillar Supported** | Features 5 structural legs on floor |
| | Leg Footprint ($W \times D \times H$) | $0.080\text{ m} \times 0.400\text{ m} \times 0.040\text{ m}$ | 5 vertical steel posts |
| | Bay Pitch (Leg Center-to-Center) | **$0.420\text{ m}$** | 4 bays across $1.80\text{ m}$ rack span |
| | Clear Fork Opening Between Legs | **$0.340\text{ m}$** | Completely unobstructed corridor |
| **Vertical Storage Tiers** | **Tier 1 (Lower Bay - Bottom Pallet)** | **$Y \approx 0.110 - 0.121\text{ m}$** | Elevated above floor by bottom plank/legs |
| | **Tier 2 (Lower Bay - Stacked Pallet)** | **$Y \approx 0.373 - 0.384\text{ m}$** | Upper stack in bottom bay |
| | **Tier 3 (Upper Bay - Bottom Pallet)** | **$Y \approx 0.781 - 0.798\text{ m}$** | Lower stack on middle shelf beam |
| | **Tier 4 (Upper Bay - Top Shelf Pallet)** | **$Y \approx 1.044 - 1.061\text{ m}$** | **Highest storage level** |
| **XStack AMR (Current)** | Rigid Mast Height | **$0.900\text{ m}$** | Visual/structural mast |
| | Prismatic Lift Stroke Limit ($q$) | **$-0.020\text{ m}$ to $+0.800\text{ m}$** | Reaches up to Tier 3 ($q_{\text{pick}} \approx 0.76\text{ m}$) |
| | Fork Tines ($L \times W \times H$) | $0.375\text{ m} \times 0.050\text{ m} \times 0.020\text{ m}$ | Steel tines |
| | Fork Center-to-Center Spacing | $0.240\text{ m}$ | Centers fit inside $0.31\text{ m}$ cavity |
| | Tracking Reference Frame | **`drive_center`** | Axle center ($X = -0.10\text{ m}$ from chassis) |

---

## 2. Pallet & Cargo Geometry (`StandardPallet.prefab`)

From `Assets/Prefabs/StandardPallet.prefab`:

```
                       +-----------------------------+  ▲
                       |                             |  | ToteBox_Pepsi
                       |        BLUE CARGO BOX       |  | Height: 0.20m (200mm)
                       |                             |  |
                 ▲     +=============================+  ▼
 TopDeck: 0.02m  |     |=============================|  ▲ Pallet Deck Top: 0.08m
                 ▼     +---+                     +---+  | Pocket Cavity: 0.06m (60mm)
Outer Blocks: 0.06m    |   | <--- 0.31m Cavity-> |   |  |
=======================+===+=====================+===+==▼ Pallet Base (Y_base)
```

1. **Outer Support Blocks**:
   - Left Block: $X = 0.034\text{ m}$, Scale $0.03\text{ m} \times 0.06\text{ m} \times 0.35\text{ m}$.
   - Right Block: $X = 0.374\text{ m}$, Scale $0.03\text{ m} \times 0.06\text{ m} \times 0.35\text{ m}$.
   - Pocket Cavity: width $= 0.374 - (0.034 + 0.030) = \mathbf{0.310\text{ m}}$, height $= \mathbf{0.060\text{ m}}$ ($60\text{ mm}$).
2. **Top Deck**:
   - Center $Y = 0.070\text{ m}$, thickness $= 0.020\text{ m}$.
   - Underside (pocket ceiling): $Y_{\text{base}} + 0.060\text{ m}$.
   - Top surface (cargo platform): $Y_{\text{base}} + 0.080\text{ m}$.
3. **ToteBox Cargo**:
   - Height $= 0.200\text{ m}$. Spans from $Y_{\text{base}} + 0.080\text{ m}$ to $Y_{\text{base}} + 0.280\text{ m}$.

---

## 3. Warehouse Rack Prefabs: `Rack-DetectReady1` vs. `Rack-DetectReady2`

The warehouse features two specialized detection-ready rack prefabs:

### 3.1 `Rack-DetectReady1` (Front Lip / Shield Type)
- **Concept:** Uses small physical vertical tabs (`Shield`, `Shield (1)`, etc.) mounted on the front edge of `Addup 1` ($X_{\text{scale}} = 1.8\text{ m}, Y_{\text{scale}} = 0.1\text{ m}, Z_{\text{scale}} = 0.4\text{ m}$).
- **Shield Geometry:** $0.07\text{ m}$ wide, $0.05\text{ m}$ tall, $0.016\text{ m}$ thick.
- **Critical Risk (The "Behind-the-Lip" Split-Depth Trap):**
  When pallets sit inside the bay, their wooden faces rest just $3-5\text{ cm}$ behind the shield. Laser beams grazing the shield edges hit the pallet behind, creating a dual-depth staircase return ($d = 1.10\text{ m}$ on shield vs $d = 1.14\text{ m}$ on pallet). This causes single-line PCA regression straightness errors ($\text{RMS} > 3\text{ cm}$), causing `detect_load_face()` to fail the `max_rms = 0.01` threshold.

### 3.2 `Rack-DetectReady2` (Bottom Legs / Pillar Pair Type) — RECOMMENDED
- **Concept:** Features a bottom plank (`BottomPlank`, $1.80\text{ m} \times 0.05\text{ m} \times 0.40\text{ m}$) supported by **5 structural floor legs** (`BottomLeg` 1 through 5).
- **Leg Coordinates (Local relative to Rack center):**
  - Leg 1: $X \approx -0.84\text{ m}$
  - Leg 2: $X \approx -0.43\text{ m}$
  - Leg 3: $X \approx -0.01\text{ m}$
  - Leg 4: $X \approx +0.41\text{ m}$
  - Leg 5: $X \approx +0.83\text{ m}$
- **Bay Characteristics:**
  - 4 bays per rack unit.
  - Center-to-center bay spacing: $\mathbf{0.420\text{ m}}$.
  - Clear entry corridor width between legs: $\mathbf{0.340\text{ m}}$ (leaves $50\text{ mm}$ clearance on each side of the AMR's $0.24\text{ m}$ fork carriage).
- **Laser Signature:** Two sharp pillar clusters at $d \approx 1.10\text{ m}$ flanking a deep open void. 100% immune to empty/loaded bay states.

---

## 4. Vertical Storage Tiers & Required Lift Kinematics

With `Rack-DetectReady1` and `Rack-DetectReady2` placed in `Milestone_2.unity` at $Y \approx 0.78 - 0.79\text{ m}$, the 4 storage tiers have the following absolute elevations:

```
========================================================================================
TIER 4 (Top Shelf - Upper Stack) : Pallet Base Y = 1.044m - 1.061m  ◄── [UNREACHABLE NOW!]
----------------------------------------------------------------------------------------
TIER 3 (Top Shelf - Lower Stack) : Pallet Base Y = 0.781m - 0.798m  ◄── [At stroke fringe]
========================================================================================
[MIDDLE SHELF BEAM (Y ≈ 0.71m - 0.76m)]
========================================================================================
TIER 2 (Bottom Bay - Upper Stack): Pallet Base Y = 0.373m - 0.384m  ◄── [Fully supported]
----------------------------------------------------------------------------------------
TIER 1 (Bottom Bay - Lower Stack): Pallet Base Y = 0.110m - 0.121m  ◄── [Fully supported]
========================================================================================
[FLOOR LEVEL (Y = 0.000m)]
```

### Detailed Height & Prismatic Lift Table:

Fork tines have thickness $20\text{ mm}$. The tine top surface relative to prismatic coordinate $q$ is:
$$Z_{\text{fork\_top}}(q) = 0.070\text{ m} + q$$
To enter a pallet pocket centered at cavity mid-height ($Y_{\text{base}} + 0.030\text{ m}$), the required prismatic command is:
$$q_{\text{entry}} = Y_{\text{base}} + 0.030\text{ m} - 0.070\text{ m} = Y_{\text{base}} - 0.040\text{ m}$$
To pick the pallet ($+20\text{ mm}$ lift):
$$q_{\text{pick}} = q_{\text{entry}} + 0.020\text{ m} = Y_{\text{base}} - 0.020\text{ m}$$

| Tier | Pallet Base $Y$ | Pocket Cavity Span | Entry Command $q_{\text{entry}}$ | Pick Command $q_{\text{pick}}$ | Current Capability ($q \le 0.80$) |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **Tier 1 (Bottom Bay - Lower)** | $0.110\text{ m}$ | $0.110 \to 0.170\text{ m}$ | $\mathbf{+0.070\text{ m}}$ | $\mathbf{+0.090\text{ m}}$ | ✅ Supported |
| **Tier 2 (Bottom Bay - Upper)** | $0.373\text{ m}$ | $0.373 \to 0.433\text{ m}$ | $\mathbf{+0.333\text{ m}}$ | $\mathbf{+0.353\text{ m}}$ | ✅ Supported |
| **Tier 3 (Top Shelf - Lower)** | $0.781\text{ m}$ | $0.781 \to 0.841\text{ m}$ | $\mathbf{+0.741\text{ m}}$ | $\mathbf{+0.761\text{ m}}$ | ✅ Supported ($0.761\text{ m} \le 0.800\text{ m}$) |
| **Tier 4 (Top Shelf - Upper)** | $1.044\text{ m}$ | $1.044 \to 1.104\text{ m}$ | $\mathbf{+1.004\text{ m}}$ | $\mathbf{+1.024\text{ m}}$ | ❌ Requires 2-stage telescopic mast |

---

## 5. Required AMR Mast & Lift Extension

As shown above, the robot **physically cannot service Tier 4** because the single-stage mast is only $0.85\text{ m}$ tall and the lift software clamps at $0.75\text{ m}$.

### Required Modifications:
1. **`xstack_amr.urdf`**:
   - Increase `mast_link` visual and collision height:
     ```xml
     <!-- Change box height from 0.85m to 1.25m, visual origin z = 0.625m -->
     <geometry><box size="0.06 0.32 1.25"/></geometry>
     ```
   - Extend `lift_joint` prismatic upper limit:
     ```xml
     <!-- Change upper from 0.75 to 1.15 -->
     <limit lower="-0.02" upper="1.15" effort="1500.0" velocity="0.12"/>
     ```
2. **`StackerController.cs`**:
   - Update `maxLiftHeight`:
     ```csharp
     public float maxLiftHeight = 1.15f; // extended to reach Tier 4 (1.05m pallets)
     ```
3. **Unity Prefab `xstack_amr.prefab`**:
   - In `mast_link`, adjust scale/box height to match $1.25\text{ m}$.
   - In `fork_carriage_link` `ArticulationBody`, update linear limit upper to $1.15\text{ m}$.

---

## 6. Dynamic Location Export Workflow (`RosLocationExport.cs`)

Because pallet elevations change depending on the rack prefab, hardcoding $(x, y, z)$ coordinates is prohibited.

### The Automated Pipeline:
1. In Unity, open the target scene (e.g., `Milestone_2.unity`). Ensure Play is **STOPPED**.
2. Click **Tools → Export ROS Locations**.
3. `RosLocationExport.cs` iterates through all `StandardPallet` prefabs (anchoring to `TopDeck`) and `Workbench` instances:
   - Computes $(x, y)$ in the `/odom` coordinate frame.
   - Computes reverse docking approach `yaw` pointing toward the aisle.
   - Records the exact physical base height `height`.
4. Saves directly to `catkin_ws/src/amr_navigation/config/locations_milestone_3.yaml`.
5. Missions load packages **by semantic name** (`package_1`, `package_42`, `station_1`). The mission script inspects `height` to automatically select the correct lift profile (`bottom`, `stack`, `shelf`, `top_shelf`).

---

## 7. Staging Station (Workbench) Metrics

- **Station Deck Surface Elevation**: $Z_{\text{station}} = \mathbf{0.250\text{ m}}$ ($25\text{ cm}$)
- **Aisle Transit Prismatic Height**: $q = \mathbf{+0.280\text{ m}}$ (Pallet bottom at $0.290\text{ m}$, providing $40\text{ mm}$ clearance over the $0.25\text{ m}$ deck).
- **Deposit Touchdown Command**: $q = \mathbf{+0.210\text{ m}}$ (Forks sink to $Z = 0.280\text{ m}$, fully offloading pallet weight onto the station deck).
