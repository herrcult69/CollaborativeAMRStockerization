# Rack Bay & Leg Pair Docking: Technical Proposal & Migration Guide

This document records the architectural analysis, trade-off evaluation, and step-by-step implementation plan for transitioning from cargo-box face detection to **Rack Leg Pair (Pillar) Detection**.

---

## 1. Problem Statement & Motivation

### Current Implementation (`pallet_detect.py`)
In Milestone 2, the AMR mounts a 2D LiDAR on the front bumper at elevation $Z \approx 0.22\text{ m}$.
At this height, the horizontal laser fan slices directly through the **blue plastic cargo boxes** sitting on top of the pallets:
- The detection function (`detect_load_face`) fits a single $0.35\text{ m}$ continuous line across the front face of the cargo box.
- It filters out the $7\text{ cm}$ gap between adjacent boxes.

### The Limitation
1. **Empty Bay Failure:** If a rack bay is empty (or a worker removes the cargo box), laser rays pass directly through the open shelf into the back wall ($> 1\text{ m}$ deeper). The ROI filter finds zero points, and the mission immediately **aborts**.
2. **Cannot Deposit Into Empty Bays:** The robot cannot use micro-perception to align into an empty slot for cargo drop-off.
3. **Box Dependency:** The robot is tracking the *cargo*, not the physical infrastructure (pallet/shelf).

---

## 2. Evaluated Architectural Options

### Option A: Low-Profile Shelf Lip / Stop Plate
Attach a small physical marker lip or plate ($20-30\text{ cm}$ wide, $2-4\text{ cm}$ tall) to the front steel runner of each bay.

* **Pros:**
  - Can reuse existing single-line PCA code (`detect_load_face`) with a modified target width.
* **Cons & Critical Risks (The "Behind-the-Lip" Trap):**
  - **The Split-Depth / Staircase Artifact:** When a pallet is stored in the bay just $3-5\text{ cm}$ behind the lip, laser rays that glance past the lip edges or skim over the top hit the wooden pallet face at a slightly deeper plane ($d_{\text{lip}} = 1.10\text{ m}$ vs $d_{\text{pallet}} = 1.14\text{ m}$).
  - **PCA Line-Fit Ruin:** The regression attempts to fit a single line across points on two distinct depth planes. The straightness error jumps to $\text{RMS} > 3\text{ cm}$, causing `detect_load_face()` to fail the `max_rms = 0.01` threshold and reject the scan.
  - **Fork Snag Hazard:** The AMR forks must enter at ground level ($Z \approx 20\text{ mm}$). Any physical lip tall enough to reliably reflect 2D laser rays threatens to collide with the underside of the fork carriage during reverse insertion.
* **Verdict:** ❌ **REJECTED.**

---

### Option B: Rack Leg / Pillar Pair Detection ("Inverse Cavity") — RECOMMENDED
Lower the LiDAR slightly ($Z \approx 0.08\text{ m}$) so the scan plane intersects the **two vertical steel legs** framing each bay opening.

* **Laser Signature:**
  - **Left Leg:** A tight cluster of $5-8$ laser points at $(X \approx -0.21\text{ m}, Y \approx 1.10\text{ m})$.
  - **Right Leg:** A tight cluster of $5-8$ laser points at $(X \approx +0.21\text{ m}, Y \approx 1.10\text{ m})$.
  - **Bay Throat:** An open cavity in between where rays penetrate deeper.
* **Pros:**
  - **100% Invariant to Bay State:** Steel rack legs are permanent fixtures. The signature is identical whether the bay has a pallet, has an empty pallet, or is completely empty.
  - **100% Open Fork Corridor:** The entry between the two legs is completely unobstructed air.
  - **Zero Unity Model Changes:** The warehouse racks already possess physical vertical legs.
* **Verdict:** ✅ **ACCEPTED FOR FUTURE IMPLEMENTATION.**

---

### Option C: Rear / Fork Camera
Mount a camera facing backward between the forks to visually recognize pallet pockets via OpenCV (contour detection or fiducial AprilTags).

* **Verdict:** 💡 Recommended as a long-term research extension / future work for multi-tier upper racks.

---

## 3. Mathematical Formulation (Rack Leg Pair Detection)

```
                     [RACK BAY CAVITY]
                     (Rays pass deep)

        [Left Leg]                       [Right Leg]
        ••••••••                           ••••••••
            │                                 │
            ▼                                 ▼
      Centroid (P_L)                    Centroid (P_R)
            │                                 │
            └───────────────┬─────────────────┘
                            ▼
                    Bay Center (P_bay)
```

### 1. ROI Coordinate Windowing
Given expected bay center $P_{\text{exp}} = (x_e, y_e)$ and expected dock angle $\theta_e$:
- Unit dock vector: $\hat{u} = (\cos\theta_e, \sin\theta_e)$ (pointing outward from bay to aisle).
- Lateral vector: $\hat{v} = (-\sin\theta_e, \cos\theta_e)$.
- Filter laser points in `/odom` within lateral window $|(p - P_{\text{exp}}) \cdot \hat{v}| \le \frac{W_{\text{bay}}}{2} + 0.15\text{ m}$ and depth window $|(p - P_{\text{exp}}) \cdot \hat{u}| \le 0.20\text{ m}$.

### 2. Euclidean Clustering
Sort remaining ROI points laterally along $\hat{v}$.
Split points into clusters wherever the gap between consecutive points exceeds $\Delta s > 0.05\text{ m}$.
Filter clusters:
- Keep clusters with point count $N \ge 4$ and bounding width $\le 0.08\text{ m}$ (matching a rack post).
- Select the pair of clusters closest to the expected span $W_{\text{bay}} \approx 0.42\text{ m}$.

### 3. Metric Geometric Verification
Let the two cluster centroids in `/odom` be $P_L = (x_L, y_L)$ and $P_R = (x_R, y_R)$:
1. **Span Check:**
   $$W_{\text{meas}} = \sqrt{(x_R - x_L)^2 + (y_R - y_L)^2}$$
   $$\text{Condition: } |W_{\text{meas}} - W_{\text{bay}}| \le 0.03\text{ m}$$
2. **Symmetry Check:**
   Both $P_L$ and $P_R$ must be equidistant from the expected center line within $\pm 0.04\text{ m}$.

### 4. Docking Pose Extraction
1. **Bay Center:**
   $$P_{\text{bay}} = \frac{P_L + P_R}{2}$$
2. **Bay Facing Yaw:**
   $$\theta_{\text{bay}} = \operatorname{atan2}(y_R - y_L, \; x_R - x_L) - 90^\circ$$
3. **Drive Center Dock Target:**
   $$P_{\text{dock}} = P_{\text{bay}} + d_{\text{dock\_offset}} \cdot (\cos\theta_{\text{bay}}, \sin\theta_{\text{bay}})$$

---

## 4. What to Change From Current Implementation

### A. Unity & Robot URDF
1. **LiDAR Elevation:**
   - In Unity `laser_link` Transform, lower **Position Y** from $\approx 0.22\text{ m}$ down to $\approx 0.08\text{ m}$ (bumper level).
   - In `catkin_ws/src/amr_description/urdf/xstack_amr.urdf`, ensure `laser_joint` origin matches:
     ```xml
     <joint name="laser_joint" type="fixed">
       <origin xyz="0.23 0 0.08" rpy="0 0 0"/>
     </joint>
     ```

### B. Algorithm Module (`catkin_ws/src/amr_navigation/src/amr_navigation/`)
Create a new function in `pallet_detect.py` (or a dedicated `bay_detect.py`):
```python
def detect_rack_legs(points, exp_x, exp_y, exp_dock_yaw_deg,
                     bay_width=0.42, leg_width=0.05, span_tol=0.03):
    """
    Detects the pair of steel rack posts flanking a bay opening.
    Returns ((bay_x, bay_y, dock_yaw_deg), None) on success, or (None, reason) on error.
    """
```

### C. Mission Control (`milestone2_mission.py`)
Add a parameter to choose the detection mode:
```python
detect_mode = rospy.get_param("~detect_mode", "load_face")  # "load_face" or "rack_legs"

if detect_mode == "rack_legs":
    det = detect_rack_legs(pts, expected[0], expected[1], expected[2], bay_width=0.42)
else:
    det = detect_load_face(pts, expected[0], expected[1], expected[2], width=load_width)
```

---

## 5. Step-by-Step Implementation Checklist (When Ready)

- [ ] **Step 1: Unity Ray Verification**
  - Lower `laser_link` in Unity to $Y = 0.08\text{ m}$.
  - Stand the AMR in front of an empty rack bay.
  - In Scene view with Gizmos enabled, verify green raycasts hit the left and right orange leg posts and penetrate into the bay between them.
- [ ] **Step 2: Log Live LaserScan**
  - Save a bag or echo `/scan` to verify point cloud clustering around the two leg coordinates.
- [ ] **Step 3: Write Unit Tests**
  - Create `catkin_ws/src/amr_navigation/tests/test_rack_legs.py`.
  - Simulate synthetic scans with two square pillars at $\pm 0.21\text{ m}$ and test with noise ($3\text{ mm}$ gaussian).
  - Verify that rotated racks (e.g. $268^\circ, 273^\circ$) extract the correct dock yaw.
- [ ] **Step 4: Implement `detect_rack_legs()` in `pallet_detect.py`**
  - Implement lateral projection, gap splitting, cluster centroid calculation, and span validation.
- [ ] **Step 5: End-to-End Simulation Run**
  - Test picking from a loaded bay.
  - Test docking into an empty bay (simulating a deposit mission).
