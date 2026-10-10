# Autonomous Mobile Robot (AMR) Warehouse Metrics, Kinematics & Clearance Documentation

This document provides the authoritative physical metrics, vertical clearances, and kinematic transformations for the **XStack Autonomous Pallet Stacker AMR**, the **Warehouse Racks**, and the **Standard Pallet / Cargo Units** in the `amr_ware_house` simulation and ROS 1 navigation stack.

---

## 1. Executive Summary & Core Dimensions

| Entity | Parameter | Metric Value | Imperial / Notes |
| :--- | :--- | :--- | :--- |
| **Standard Pallet** | Base Footprint (Width $\times$ Depth) | $0.400\text{ m} \times 0.350\text{ m}$ | $400\text{ mm} \times 350\text{ mm}$ |
| | Empty Pallet Height (Deck Top) | $0.080\text{ m}$ | $80\text{ mm}$ total height |
| | Fork Entry Pocket Opening Width | $0.310\text{ m}$ | $310\text{ mm}$ between outer blocks |
| | Fork Entry Pocket Cavity Height | $0.060\text{ m}$ | **$60\text{ mm}$ ground clearance** |
| | Pallet Top Deck Slat Thickness | $0.020\text{ m}$ | $20\text{ mm}$ |
| **Cargo Box (ToteBox)** | Box Dimensions ($W \times H \times D$) | $0.350\text{ m} \times 0.200\text{ m} \times 0.300\text{ m}$ | Standard Pepsi Tote Box |
| | Single Loaded Pallet Total Height | $0.280\text{ m}$ | $280\text{ mm}$ ($80\text{ mm}$ pallet + $200\text{ mm}$ box) |
| **Double Stacked Pallet** | Total Stack Height | $0.560\text{ m}$ | **$560\text{ mm}$ from floor to top box** |
| | Upper Pallet Pocket Cavity | $0.280\text{ m} - 0.340\text{ m}$ | $60\text{ mm}$ cavity height |
| **Warehouse Rack** | Leg Height | $0.650\text{ m}$ | Upright clearance |
| | Bay Width Between Uprights | $1.750\text{ m}$ | Fits up to 4 pallet stacks across |
| | Middle Shelf Beam Thickness | $0.050\text{ m}$ | $50\text{ mm}$ structural steel beam |
| | Underside of Middle Shelf Beam | **$0.592\text{ m}$** | **$592\text{ mm}$ overhead ceiling** |
| | Top Surface of Middle Shelf Beam | $0.642\text{ m} \approx 0.650\text{ m}$ | Shelf Level 2 floor |
| **XStack AMR** | Prismatic Lift Stroke Limit | **$-0.020\text{ m}$ to $+0.750\text{ m}$** | Range: $770\text{ mm}$ vertical stroke |
| | Fork Tines ($L \times W \times H$) | $0.375\text{ m} \times 0.050\text{ m} \times 0.020\text{ m}$ | $20\text{ mm}$ thick steel tines |
| | Fork Tine Center-to-Center Spacing | $0.240\text{ m}$ | $240\text{ mm}$ tine pitch |
| | Odometry Tracking Reference Frame | **`drive_center`** | **Drive wheel axle fulcrum** |
| | Fork Center Lever Arm from `drive_center` | **$0.350\text{ m}$** | Rearward along robot $-X$ axis |

---

## 2. Pallet & Cargo Geometry Breakdown (`StandardPallet.prefab`)

From `Assets/Prefabs/StandardPallet.prefab`:

```
                       +-----------------------------+  ▲
                       |                             |  | ToteBox_Pepsi
                       |        PEPSI TOTE BOX       |  | Height: 0.20m (200mm)
                       |                             |  |
                 ▲     +=============================+  ▼
 TopDeck: 0.02m  |     |=============================|  ▲ Pallet Deck Top: 0.08m
                 ▼     +---+                     +---+  | Pocket Cavity: 0.06m (60mm)
Outer Blocks: 0.06m    |   | <--- 0.31m Cavity-> |   |  |
=======================+===+=====================+===+==▼ Floor (0.00m)
```

1. **Outer Support Blocks**:
   - Left Block: $X = 0.034\text{ m}$, Scale $0.03\text{ m} \times 0.06\text{ m} \times 0.35\text{ m}$.
   - Right Block: $X = 0.374\text{ m}$, Scale $0.03\text{ m} \times 0.06\text{ m} \times 0.35\text{ m}$.
   - Vertical span: $Y \in [0.000\text{ m}, 0.060\text{ m}]$.
2. **Top Deck**:
   - Center $Y = 0.070\text{ m}$, thickness $= 0.020\text{ m}$.
   - Underside (pocket ceiling): $Y = 0.060\text{ m}$ ($60\text{ mm}$).
   - Top surface (cargo platform): $Y = 0.080\text{ m}$ ($80\text{ mm}$).
3. **ToteBox_Pepsi Cargo**:
   - Center $Y = 0.180\text{ m}$, height $= 0.200\text{ m}$.
   - Sits on deck: spans $Y \in [0.080\text{ m}, 0.280\text{ m}]$.

---

## 3. Double-Stacked Pallet & Rack Clearance Analysis

In warehouse configurations `Set1.prefab` and `Set2.prefab`, pallets are double-stacked under a 2-tier rack:

```
==========================================================  ▲ Overhead Shelf Beam
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\  | Bottom: 0.592m
----------------------------------------------------------  ▼
                 ▲  [CRITICAL HEADROOM: 32mm (3.2cm)!]
                 ▼
+--------------------------------------------------------+  ▲
|                                                        |  | Top Tote Box
|                 UPPER PEPSI TOTE BOX                   |  | (0.36m -> 0.56m)
+========================================================+  ▼
|================== UPPER PALLET DECK ===================|  ▲ 0.34m -> 0.36m
+---+                                                +---+  | Upper Cavity: 0.28m -> 0.34m
|   | <------------ Upper Pallet Pocket ------------>|   |  ▼
+--------------------------------------------------------+  ▲
|                                                        |  | Bottom Tote Box
|                LOWER PEPSI TOTE BOX                    |  | (0.08m -> 0.28m)
+========================================================+  ▼
|================== LOWER PALLET DECK ===================|  ▲ 0.06m -> 0.08m
+---+                                                +---+  | Lower Cavity: 0.00m -> 0.06m
|   | <------------ Lower Pallet Pocket ------------>|   |  ▼
==========================================================  Floor (0.00m)
```

### Critical Headroom Calculation:
- Top of Double Stacked Cargo: **$0.560\text{ m}$** ($560\text{ mm}$)
- Underside of Overhead Rack Beam: **$0.592\text{ m}$** ($592\text{ mm}$)
- **Net Vertical Clearance**:
  $$\Delta Z_{\text{headroom}} = 0.592\text{ m} - 0.560\text{ m} = \mathbf{0.032\text{ m}} = \mathbf{32\text{ mm}}\; (3.2\text{ cm})$$

> [!CAUTION]
> **STRICT LIFT LIMIT UNDER RACKS**:
> When lifting a double-stacked pallet from the bottom pocket under a rack shelf, the lift stroke **MUST NOT EXCEED $+0.020\text{ m}$ ($20\text{ mm}$)**.
> Lifting by $\ge 32\text{ mm}$ will smash the top Pepsi cargo box into the overhead structural beam!

---

## 4. AMR Fork Carriage & Prismatic Lift Kinematics

From `XStackDesign.urdf` and `StackerController.cs`:

### Kinematic Tree from Ground to Fork Tines:
1. `base_footprint`: $Z = 0.000\text{ m}$ (ground surface)
2. `base_link`: $Z = 0.050\text{ m}$ (chassis ground clearance)
3. `mast_link`: $Z = 0.050\text{ m}$ (rigid upright single-stage mast, height $0.85\text{ m}$)
4. `lift_joint`: Prismatic joint along $+Z$. Origin: $Z = 0.050 + 0.010 = 0.060\text{ m}$.
   - Prismatic position variable: $q \in [-0.020\text{ m}, +0.750\text{ m}]$
5. `left_fork_link` / `right_fork_link`: Tine thickness $= 0.020\text{ m}$ ($20\text{ mm}$).
   - **Fork Tine Underside**: $Z_{\text{fork\_bottom}}(q) = 0.050 + q\text{ m}$
   - **Fork Tine Top Surface**: $Z_{\text{fork\_top}}(q) = 0.070 + q\text{ m}$

### Lift Operating Modes (Standardized Option 1: Uniform 20mm Lift Across All Levels):

| Maneuver / Level | Prismatic Position $q$ | Fork Tines ($Z_{\text{bot}} \to Z_{\text{top}}$) | Pallet Bottom / State | Clearance / Margin |
| :--- | :---: | :---: | :---: | :--- |
| **Ground Insertion (Level 1 / `bottom`)** | **$q = -0.020\text{ m}$** | $30\text{ mm} \to 50\text{ mm}$ | Cavity: $0 \to 60\text{ mm}$ | **Enters cleanly**: $30\text{ mm}$ floor margin, $10\text{ mm}$ ceiling margin |
| **Ground Pick (Level 1 / `bottom`)** | **$q = +0.010\text{ m}$** | $60\text{ mm} \to 80\text{ mm}$ | Pallet lifts **$+20\text{ mm}$ off floor** | **$12\text{ mm}$ safe headroom** below $0.592\text{ m}$ under-rack beam |
| **Upper Stack Entry (Level 2 / `top`)** | **$q = +0.240\text{ m}$** | $290\text{ mm} \to 310\text{ mm}$ | Cavity: $280 \to 340\text{ mm}$ | **Enters cleanly**: centers tines in upper pocket |
| **Upper Stack Pick (Level 2 / `top`)** | **$q = +0.290\text{ m}$** | $340\text{ mm} \to 360\text{ mm}$ | Pallet lifts **$+20\text{ mm}$ off lower box** | **$12\text{ mm}$ safe headroom** below $0.592\text{ m}$ under-rack beam |
| **Tier-2 Shelf Entry (Level 3 / `shelf`)** | **$q = +0.640\text{ m}$** | $690\text{ mm} \to 710\text{ mm}$ | Cavity: $650 \to 710\text{ mm}$ | **Enters cleanly**: enters cavity onto shelf |
| **Tier-2 Shelf Pick (Level 3 / `shelf`)** | **$q = +0.685\text{ m}$** | $735\text{ mm} \to 755\text{ mm}$ | Pallet lifts **$+20\text{ mm}$ off shelf beam** | Safe clearance below upper rack roof |
| **Corridor Transit (Over Station Clearance)** | **$q = +0.280\text{ m}$** | $330\text{ mm} \to 350\text{ mm}$ | Pallet bottom at $0.290\text{ m}$ | **$40\text{ mm}$ clearance** over $0.25\text{ m}$ staging station |
| **Staging Station Deposit** | **$q = +0.210\text{ m}$** | $260\text{ mm} \to 280\text{ mm}$ | Pallet rests on $0.24\text{ m}$ deck | Forks sink to $0.21\text{ m}$, **floating free in pocket** |

---

## 5. Odometry Coordinate Transformation: `drive_center`

### The Shift from `base_link` to `drive_center`
In differential-drive robotics, spinning in-place rotates the robot about the midpoint of its drive axle (the Instantaneous Center of Rotation / fulcrum).
- `base_link` is located at $X = 0.000\text{ m}$ (front chassis origin).
- The drive wheel axle is located at $X = -0.100\text{ m}$.
- When `/odom` previously tracked `base_link`, rotating in-place caused `base_link` to swing along an arc of radius $0.10\text{ m}$, generating artificial translational drift during Phase 1 and Phase 3 yaw alignment!
- **Fix Applied**: `PlanarOdometryPublisher.cs` now tracks **`drive_centre`** (`drive_center`) at $X = -0.100\text{ m}$. Spinning in place produces **zero translational drift** ($\Delta x = 0, \Delta y = 0$).

### Updated Kinematic Lever Arms (Reference: `drive_center`):

```
       +---------------------------------------------------------+
       | [Front Bumper / LiDAR] (+0.33m)                         |
       |                      base_link (X = +0.10m from drive)  |
       |                                                         |
===O===+==================== drive_center =======================+===O=== (X = 0.00m)
(Left Wheel)                [Track Width = 0.40m]               (Right Wheel)
       |                                                         |
       |                      Mast Link (X = -0.09m)             |
       |                      Fork Carriage (X = -0.12m)         |
       |                      Fork Base (X = -0.16m)             |
       |                                                         |
       |                      FORK REFERENCE (X = -0.22m) <====== [Dock Offset d_fork]
       |                      Fork Tine Center (X = -0.35m)      |
       |                      Fork Tips (X = -0.535m)            |
       +---------------------------------------------------------+
```

| Landmark | Position rel. to `base_link` | Position rel. to `drive_center` | Role in Navigation |
| :--- | :--- | :--- | :--- |
| **Front Bumper / Lidar** | $+0.230\text{ m}$ | **$+0.330\text{ m}$** | Forward safety perimeter |
| **`base_link`** | $0.000\text{ m}$ | **$+0.100\text{ m}$** | Forward geometric chassis link |
| **`drive_center`** | $-0.100\text{ m}$ | **$0.000\text{ m}$** | **Tracked `/odom` coordinate frame** |
| **Mast Link** | $-0.190\text{ m}$ | **$-0.090\text{ m}$** | Structural lift upright |
| **Fork Carriage** | $-0.220\text{ m}$ | **$-0.120\text{ m}$** | Prismatic lift mount |
| **Fork Base** | $-0.260\text{ m}$ | **$-0.160\text{ m}$** | Fork tine root |
| **Fork Reference ($d_{\text{fork}}$)** | $-0.320\text{ m}$ | **$-0.220\text{ m}$** | **Deep Tine Penetration ($85\text{--}90\%$)** |
| **Geometric Center of Tines** | $-0.450\text{ m}$ | **$-0.350\text{ m}$** | Shallow Midpoint Penetration ($50\%$) |
| **Fork Tips** | $-0.635\text{ m}$ | **$-0.535\text{ m}$** | Rearmost tip boundary |

---

## 6. Universal Reverse Docking Target Math

When commanding the robot to reverse into a pallet located at $(x_p, y_p)$ with docking orientation $\theta_{\text{dock}}$:

$$\begin{aligned}
x_{\text{drive\_center}} &= x_p + d_{\text{fork}} \cdot \cos(\theta_{\text{dock}}) \\
y_{\text{drive\_center}} &= y_p + d_{\text{fork}} \cdot \sin(\theta_{\text{dock}})
\end{aligned}$$

where $d_{\text{fork}} = \mathbf{0.350\text{ m}}$ (distance from `drive_center` to fork pocket center).

### Benchmark Pick Station Example:
- Pallet Cavity: $(4.00, 5.00)$
- Reverse Docking Yaw: $270.0^\circ$ ($\cos 270^\circ = 0, \; \sin 270^\circ = -1.0$)
- Offset Calculation:
  $$\begin{aligned}
  x_{\text{drive}} &= 4.00 + 0.35 \cdot (0) = \mathbf{4.00\text{ m}} \\
  y_{\text{drive}} &= 5.00 + 0.35 \cdot (-1.0) = \mathbf{4.65\text{ m}}
  \end{aligned}$$

When `drive_center` stops at $(4.00, 4.65)$, the fork tines are centered at $(4.00, 5.00)$ inside the pallet, with the chassis safely parked at $y = 4.55\text{ m}$ outside the pallet face.

---

## 7. Staging Station (25cm Stocker) & Reverse Undocking Metrics

From `Workbench.prefab` and `RobotRoute test.unity`:

```
              +-----------------------------+
              |                             |  ToteBox_Pepsi Cargo
              +=============================+
              |=============================|  Pallet Deck
              +---+                     +---+
              |   |  <- Pallet Pocket ->|   |
==============+===+=====================+===+==============  ▲ Pallet Bottom: 0.250m
|                                                         |  |
|             STAGING STATION / WORKBENCH                 |  | Station Height: 0.250m (25cm)
|                                                         |  |
===========================================================  ▼ Floor (0.000m)
```

### Staging Station Geometry & Coordinate Placement:
- **Surface Elevation**: $Z_{\text{station}} = \mathbf{0.250\text{ m}}$ ($25\text{ cm}$)
- **Dock Target Cavity**: $(X, Y) = (\mathbf{13.00\text{ m}}, \mathbf{0.00\text{ m}})$
- **Docking Heading**: $\theta = \mathbf{180.0^\circ}$ (Facing West, rear forks pointing East into dock)
- **Drive Center Stop Pose**:
  $$x_{\text{drive}} = x_{\text{dock}} + \text{fork\_offset} \cdot \cos(180^\circ) = 13.00 - 0.22 = \mathbf{12.78\text{ m}}$$
- **Pre-Dock Staging Standoff Pose**:
  $$x_{\text{stage}} = x_{\text{drive}} + 0.8 \cdot \cos(180^\circ) = 12.78 - 0.80 = \mathbf{11.98\text{ m}} \approx \mathbf{12.00\text{ m}}$$
  *(Stopping at $X = 12.00\text{ m}$ leaves a generous $1.0\text{ m}$ frontal clearance from the dock, completely preventing collision during highway arrival and in-place yaw rotation).*

### Lift Clearances During Drop-Off / Undocking:

$$\begin{aligned}
Z_{\text{pallet\_bottom}}(q) &= 0.010\text{ m} + q \\
Z_{\text{fork\_bottom}}(q) &= 0.050\text{ m} + q \\
Z_{\text{fork\_top}}(q) &= 0.070\text{ m} + q
\end{aligned}$$

| State | Prismatic Stroke $q$ | Pallet Bottom $Z$ | Fork Top $Z$ | Clearance / Action |
| :--- | :--- | :--- | :--- | :--- |
| **Aisle Transit & Approach** | $\mathbf{+0.280\text{ m}}$ | $0.290\text{ m}$ | $0.350\text{ m}$ | **Clears $0.25\text{ m}$ station deck by $40\text{ mm}$ ($4\text{ cm}$)**. Glides safely over table. |
| **Station Touchdown** | $+0.240\text{ m}$ | $0.250\text{ m}$ | $0.310\text{ m}$ | Pallet outer blocks make contact with $0.25\text{ m}$ station deck. |
| **Fork Disengagement** | $\mathbf{+0.210\text{ m}}$ | $0.250\text{ m}$ (supported) | $0.280\text{ m}$ | **Forks sink into cavity**: $10\text{ mm}$ gap above station, $30\text{ mm}$ gap below pocket ceiling. 100% weight offloaded. |
| **Undock Extraction** | $+0.210\text{ m}$ | $0.250\text{ m}$ | $0.280\text{ m}$ | AMR pulls forward ($12.78\text{ m} \rightarrow 12.00\text{ m}$), cleanly sliding forks out of pallet. |

---

## 8. Verification & Implementation Reference

- Kinematics Helper: [`compute_dock_target_pose()`](file:///p:/AntiGravity/FinalYear/Project_Project/CollaborativeAMRStockerization/catkin_ws/src/amr_navigation/src/amr_navigation/docking.py#L17-L34) in `amr_navigation.docking`.
- Test Reverse Docking (Pickup): [`test_dock_reverse.py`](file:///p:/AntiGravity/FinalYear/Project_Project/CollaborativeAMRStockerization/catkin_ws/src/amr_navigation/scripts/test_dock_reverse.py).
- Test Reverse Undocking (Delivery): [`test_undock_reverse.py`](file:///p:/AntiGravity/FinalYear/Project_Project/CollaborativeAMRStockerization/catkin_ws/src/amr_navigation/scripts/test_undock_reverse.py).
- Unity Ground Truth Publisher: [`PlanarOdometryPublisher.cs`](file:///P:/AntiGravity/FinalYear/Project_Project/Unity/amr_ware_house/Assets/PlanarOdometryPublisher.cs).
- Differential Drive Controller: [`StackerController.cs`](file:///P:/AntiGravity/FinalYear/Project_Project/Unity/amr_ware_house/Assets/StackerController.cs).
