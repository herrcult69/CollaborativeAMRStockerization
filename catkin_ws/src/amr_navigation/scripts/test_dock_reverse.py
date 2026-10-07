#!/usr/bin/env python3
"""
Test Pallet Staging & Reverse Docking Maneuver:
  1. Navigate to pre-dock staging pose in front of pallet pocket
  2. Lower lift mast to entry height (-0.02m) so forks enter 60mm cavity cleanly
  3. Reverse straight back into pallet cavity accounting for 0.22m fork_offset from drive_center
  4. Elevate lift mast to safe under-rack height (+0.015m) to clear floor without hitting overhead beam
  5. Pull straight forward (undock) to extract the pallet out into the aisle and raise mast to 0.15m
"""
import math
import rospy
from amr_navigation.move_to_point import MoveToPointController
from amr_navigation.docking import PalletDockingController, compute_dock_target_pose
from amr_navigation.lift import LiftController


def run_dock_test():
    rospy.init_node("test_dock_reverse")

    # Physical kinematics: distance from drive_center (tracked by /odom) to pallet reference
    # 0.35m = center of tines (~50% penetration halfway into pallet)
    # 0.22m = deep penetration (~85-90% of tine length into pallet pocket)
    fork_offset = float(rospy.get_param("~fork_offset", 0.22))  # meters

    # Target Pallet Pose (default: 4.0, 5.0; configurable via ROS params)
    pallet_x = float(rospy.get_param("~pallet_x", 4.0))
    pallet_y = float(rospy.get_param("~pallet_y", 5.0))
    dock_yaw = float(rospy.get_param("~dock_yaw", 270.0))  # degrees
    dock_speed = float(rospy.get_param("~dock_speed", 0.12))

    # Pickup level / height configuration (Standardized Option 1: Uniform 20mm Pallet Lift):
    # "bottom" / "1": entry = -0.020m, pick = +0.010m (lifts pallet 20mm from floor; 12mm headroom under beam)
    # "top" / "upper" / "2": entry = +0.240m, pick = +0.290m (lifts pallet 20mm from lower box; 12mm headroom under beam)
    # "shelf" / "3": entry = +0.640m, pick = +0.685m (centers forks in 60mm hole; lifts pallet 20mm off shelf beam)
    pick_level = str(rospy.get_param("~pick_level", "bottom")).strip().lower()
    if pick_level in ("top", "upper", "2"):
        default_entry = 0.240
        default_pick = 0.290
        open_aisle_transit = 0.290
    elif pick_level in ("shelf", "3"):
        default_entry = 0.640
        default_pick = 0.685
        open_aisle_transit = 0.280  # Lower down to stable carry height once clear of shelf
    else:
        default_entry = -0.020
        default_pick = 0.010
        open_aisle_transit = 0.150

    entry_lift_height = float(rospy.get_param("~entry_lift_height", default_entry))
    pick_lift_height = float(rospy.get_param("~pick_lift_height", default_pick))

    # Calculate required drive_center stopping pose so forks insert cleanly into pallet
    target_drive_x, target_drive_y = compute_dock_target_pose(
        pallet_x=pallet_x,
        pallet_y=pallet_y,
        dock_yaw_deg=dock_yaw,
        fork_offset=fork_offset
    )

    # Pre-dock staging waypoint: 1.2m standoff in front along heading (or custom param)
    raw_stage_x = rospy.get_param("~stage_x", None)
    raw_stage_y = rospy.get_param("~stage_y", None)
    if raw_stage_x is not None:
        stage_x = float(raw_stage_x)
    else:
        stage_x = target_drive_x + 1.2 * math.cos(math.radians(dock_yaw))
    if raw_stage_y is not None:
        stage_y = float(raw_stage_y)
    else:
        stage_y = target_drive_y + 1.2 * math.sin(math.radians(dock_yaw))

    nav = MoveToPointController(max_linear=0.35, max_angular=0.65)
    dock = PalletDockingController(default_dock_speed=dock_speed, default_tolerance=0.04)
    lift = LiftController(default_speed=0.06)

    rospy.loginfo("==================================================")
    rospy.loginfo("STARTING PALLET STAGING & REVERSE DOCKING TEST")
    rospy.loginfo("Target Pallet Cavity        : (%.2f, %.2f) | Dock Yaw: %.1f deg", pallet_x, pallet_y, dock_yaw)
    rospy.loginfo("Pickup Level Profile        : %s (Entry=%.3fm, Pick=%.3fm)", pick_level, entry_lift_height, pick_lift_height)
    rospy.loginfo("Pre-Dock Staging Pose       : (%.2f, %.2f)", stage_x, stage_y)
    rospy.loginfo("Fork Arm Lever Offset       : %.2fm (rearward from drive_center)", fork_offset)
    rospy.loginfo("Calculated drive_center Stop: (%.2f, %.2f)", target_drive_x, target_drive_y)
    rospy.loginfo("==================================================")

    # Step 1: Approach pre-dock staging point
    rospy.loginfo("\n>>> [STEP 1/5] Navigating to Staging Pose (%.2f, %.2f, %.1f deg)...", stage_x, stage_y, dock_yaw)
    ok = nav.navigate_to(gx=stage_x, gy=stage_y, goal_yaw=dock_yaw, pos_tolerance=0.05, label="Staging Pose")
    if not ok or rospy.is_shutdown():
        rospy.logwarn("Docking test aborted at Step 1.")
        return

    # Step 2: Position lift mast to pocket entry height
    rospy.loginfo("\n>>> [STEP 2/5] Positioning lift mast to entry height (%.3fm) [Level: %s]...",
                  entry_lift_height, pick_level)
    lift.set_height(entry_lift_height, speed=0.06)

    # Step 3: Precision straight-line reverse docking
    rospy.loginfo("\n>>> [STEP 3/5] Reversing straight into Pallet at (%.2f, %.2f)...", pallet_x, pallet_y)
    rospy.loginfo("Commanding drive_center to (%.2f, %.2f) [Speed: %.2fm/s]...", target_drive_x, target_drive_y, dock_speed)
    ok = dock.dock_reverse(
        target_x=target_drive_x,
        target_y=target_drive_y,
        dock_yaw=dock_yaw,
        speed=dock_speed,
        pos_tolerance=0.04,
        entry_lat_tol=0.008,  # ~10 mm fork-to-block clearance per side
        label="Pallet Insertion"
    )
    if not ok or rospy.is_shutdown():
        rospy.logwarn("Docking test aborted at Step 3.")
        return

    # Step 4: Under-rack extraction lift
    rospy.loginfo("\n>>> [STEP 4/5] Elevating mast to Under-Rack Pick Height (%.3fm)...", pick_lift_height)
    rospy.loginfo("  [Lifts pallet off support while preserving overhead clearance]")
    lift.set_height(pick_lift_height, speed=0.04)

    # Step 5: Pull straight forward (undock) to extract the pallet out into the aisle
    rospy.loginfo("\n>>> [STEP 5/5] Extracting pallet straight forward out of rack...")
    ok = dock.undock(
        target_x=stage_x,
        target_y=stage_y,
        dock_yaw=dock_yaw,
        speed=0.10,
        pos_tolerance=0.05,
        label="Rack Extraction"
    )
    if not ok or rospy.is_shutdown():
        rospy.logwarn("Docking test aborted at Step 5.")
        return

    # Position lift for open-aisle transit
    rospy.loginfo("Pallet clear of rack. Positioning to open-aisle transit height (%.3fm)...", open_aisle_transit)
    lift.set_height(open_aisle_transit, speed=0.06)

    rospy.loginfo("\n==================================================")
    rospy.loginfo("SUCCESS: PALLET DOCKING & RACK EXTRACTION COMPLETE!")
    rospy.loginfo("Pallet safely extracted into open aisle at height %.3fm.", open_aisle_transit)
    rospy.loginfo("==================================================")


if __name__ == "__main__":
    try:
        run_dock_test()
    except rospy.ROSInterruptException:
        pass

