#!/usr/bin/env python3
"""
Milestone 1 Mission Coordinator: Autonomous Pallet Replenishment Workflow.

Full End-to-End Pick-and-Place Mission Pipeline:
  1. Approach rack pre-dock staging pose at (pallet_x, ~3.98) facing 270 deg
  2. Lower lift mast to ground entry height (-0.02m)
  3. Precision straight-line reverse docking into pallet cavity at (pallet_x, pallet_y)
  4. Elevate lift mast to safe under-rack height (+0.015m, 17mm headroom below 0.592m beam)
  5. Pull straight forward out of rack (rack extraction / undock)
  6. Elevate lift mast to transit height (0.28m, 40mm clearance above 25cm staging station)
  7. Follow Manhattan orthogonal corridor: (4.0, 5.0) -> (4.0, 0.0) -> (12.10, 0.0), align 180 deg
  8. Precision reverse docking over the 25cm staging station to stop pose (12.90, 0.0)
  9. Lower mast to deposit height (0.21m) - pallet settles on station deck, forks float free
  10. Pull straight forward to staging standoff (12.10, 0.0), cleanly extracting forks
  11. Lower mast to idle/transit height (0.00m) - Mission Complete!
"""
import math
import rospy
from amr_navigation.move_to_point import MoveToPointController
from amr_navigation.docking import PalletDockingController, compute_dock_target_pose
from amr_navigation.lift import LiftController


def run_mission():
    rospy.init_node("milestone1_mission")

    # Workflow mode: set true if AMR already has the pallet loaded
    skip_pickup = bool(rospy.get_param("~skip_pickup", False))

    # Rack pickup parameters
    pallet_x = float(rospy.get_param("~pallet_x", 4.0))
    pallet_y = float(rospy.get_param("~pallet_y", 5.0))
    dock_yaw = float(rospy.get_param("~dock_yaw", 270.0))  # degrees
    fork_offset = float(rospy.get_param("~fork_offset", 0.22))  # deep tine penetration (85-90%)
    rack_standoff = float(rospy.get_param("~rack_standoff", 1.2))  # Standoff in front of rack dock pose

    # Transit & Highway Waypoints (Orthogonal routing strictly avoids diagonal blockage)
    corridor_x = float(rospy.get_param("~corridor_x", 4.0))
    corridor_y = float(rospy.get_param("~corridor_y", 0.0))

    # Staging Station / Stocker Target (Located at X=13.12, Y=0.0, 25cm tall)
    station_x = float(rospy.get_param("~station_x", 13.12))
    station_y = float(rospy.get_param("~station_y", 0.0))
    station_yaw = float(rospy.get_param("~station_yaw", 180.0))  # Facing West, rear pointing East into dock
    station_standoff = float(rospy.get_param("~station_standoff", rospy.get_param("~standoff", 0.8)))  # Standoff in front of drive_stop pose
    standoff = station_standoff  # Alias for backward compatibility

    # Kinematic Speeds
    dock_speed = float(rospy.get_param("~dock_speed", 0.12))
    transit_speed = float(rospy.get_param("~transit_speed", 0.45))
    lift_speed = float(rospy.get_param("~lift_speed", 0.08))

    # Pickup level / height configuration (Standardized Option 1: Uniform 20mm Pallet Lift):
    # "bottom" / "1": entry = -0.020m, pick = +0.010m (lifts pallet 20mm from floor; 12mm headroom under beam)
    # "top" / "upper" / "2": entry = +0.240m, pick = +0.290m (lifts pallet 20mm from lower box; 12mm headroom under beam)
    # "shelf" / "3": entry = +0.640m, pick = +0.685m (centers forks in 60mm hole; lifts pallet 20mm off shelf beam)
    pick_level = str(rospy.get_param("~pick_level", "bottom")).strip().lower()
    if pick_level in ("top", "upper", "2"):
        default_entry = 0.240
        default_pick = 0.290
        default_transit = 0.290
    elif pick_level in ("shelf", "3"):
        default_entry = 0.640
        default_pick = 0.685
        default_transit = 0.280  # Lower down to stable carry height once clear of shelf
    else:
        default_entry = -0.020
        default_pick = 0.010
        default_transit = 0.280

    ENTRY_LIFT_HEIGHT = float(rospy.get_param("~entry_lift_height", default_entry))
    PICK_LIFT_HEIGHT = float(rospy.get_param("~pick_lift_height", default_pick))

    # Lift Heights:
    # 1. Rack Entry: default -0.020m (or +0.240m for level 2 top pallet, +0.640m for level 3 shelf)
    # 2. Under-Rack Pick: default +0.010m (or +0.290m for level 2 top pallet, +0.685m for level 3 shelf)
    # 3. Transit Clearance: 0.28m-0.29m (pallet bottom at 0.010 + q -> 40-50mm clearance over 25cm station)
    # 4. Station Deposit: +0.21m (pallet touches down at 0.24m; forks lower to 0.21m floating in mid-cavity)
    raw_transit = rospy.get_param("~transit_lift_height", None)
    if raw_transit is None or (float(raw_transit) == 0.28 and pick_level in ("top", "upper", "2")):
        TRANSIT_LIFT_HEIGHT = default_transit
    else:
        TRANSIT_LIFT_HEIGHT = float(raw_transit)
    DEPOSIT_LIFT_HEIGHT = float(rospy.get_param("~deposit_lift_height", 0.21))

    # 1. Calculate Rack Pickup Poses:
    target_rack_x, target_rack_y = compute_dock_target_pose(
        pallet_x=pallet_x,
        pallet_y=pallet_y,
        dock_yaw_deg=dock_yaw,
        fork_offset=fork_offset
    )
    stage_rack_x = target_rack_x + rack_standoff * math.cos(math.radians(dock_yaw))
    stage_rack_y = target_rack_y + rack_standoff * math.sin(math.radians(dock_yaw))

    # 2. Calculate Staging Station Deposit Poses:
    # Stop pose: drive_center stops at station_x + fork_offset * cos(yaw) = 13.12 - 0.22 = 12.90m
    target_deposit_x, target_deposit_y = compute_dock_target_pose(
        pallet_x=station_x,
        pallet_y=station_y,
        dock_yaw_deg=station_yaw,
        fork_offset=fork_offset
    )

    # Pre-dock staging pose: standoff distance in front of stop pose along heading
    # stage_x = 12.90 + 0.8 * (-1.0) = 12.10m (~12.1m) -> ~1.0m clearance in front of dock
    stage_station_x = target_deposit_x + station_standoff * math.cos(math.radians(station_yaw))
    stage_station_y = target_deposit_y + station_standoff * math.sin(math.radians(station_yaw))

    retreat_x = stage_station_x  # Undock extracts back to pre-dock standoff

    # Initialize controllers (if skipping pickup, initial lift assumed ~0.15m)
    initial_lift = 0.15 if skip_pickup else 0.00
    nav = MoveToPointController(max_linear=transit_speed, max_angular=0.65)
    dock = PalletDockingController(default_dock_speed=dock_speed, default_tolerance=0.04)
    lift = LiftController(default_speed=lift_speed, initial_height=initial_lift)

    rospy.loginfo("==================================================")
    rospy.loginfo("STARTING MILESTONE 1 FULL AUTONOMOUS MISSION")
    rospy.loginfo("Mode                     : %s", "Skip Pickup (start with pallet)" if skip_pickup else "Full Pick-and-Deliver")
    rospy.loginfo("Target Rack Pallet       : (%.2f, %.2f) | Dock Yaw: %.1f deg", pallet_x, pallet_y, dock_yaw)
    rospy.loginfo("Pickup Level Profile     : %s (Entry=%.3fm, Pick=%.3fm)", pick_level, ENTRY_LIFT_HEIGHT, PICK_LIFT_HEIGHT)
    rospy.loginfo("Corridor Junction        : (%.2f, %.2f)", corridor_x, corridor_y)
    rospy.loginfo("Staging Station Target   : (%.2f, %.2f) [25cm Tall Dock]", station_x, station_y)
    rospy.loginfo("Station Stop Pose (Drive): (%.2f, %.2f)", target_deposit_x, target_deposit_y)
    rospy.loginfo("Pre-Dock Staging Pose    : (%.2f, %.2f) | Yaw: %.1f deg", stage_station_x, stage_station_y, station_yaw)
    rospy.loginfo("Lift Profiles            : Transit=%.3fm | Deposit=%.3fm", TRANSIT_LIFT_HEIGHT, DEPOSIT_LIFT_HEIGHT)
    rospy.loginfo("==================================================")

    # ---------------------------------------------------------
    # PHASE 1: Pallet Extraction from Rack (if not skipped)
    # ---------------------------------------------------------
    if not skip_pickup:
        rospy.loginfo("\n>>> [STEP 1/7] Navigating to Rack Staging Pose (%.2f, %.2f, %.1f deg)...",
                      stage_rack_x, stage_rack_y, dock_yaw)
        ok = nav.navigate_to(gx=stage_rack_x, gy=stage_rack_y, goal_yaw=dock_yaw, pos_tolerance=0.05, label="Rack Staging")
        if not ok or rospy.is_shutdown():
            rospy.logwarn("Mission aborted at Rack Staging.")
            return

        rospy.loginfo("\n>>> [STEP 2/7] Positioning lift mast to entry height (%.3fm) [Level: %s]...",
                      ENTRY_LIFT_HEIGHT, pick_level)
        lift.set_height(ENTRY_LIFT_HEIGHT, speed=0.06)

        rospy.loginfo("\n>>> [STEP 3/7] Reversing into Pallet at (%.2f, %.2f)...", pallet_x, pallet_y)
        ok = dock.dock_reverse(
            target_x=target_rack_x,
            target_y=target_rack_y,
            dock_yaw=dock_yaw,
            speed=dock_speed,
            pos_tolerance=0.04,
            entry_lat_tol=0.008,  # ~10 mm fork-to-block clearance per side
            label="Rack Insertion"
        )
        if not ok or rospy.is_shutdown():
            rospy.logwarn("Mission aborted during rack insertion.")
            return

        rospy.loginfo("\n>>> [STEP 4/7] Elevating mast to Under-Rack Pick Height (%.3fm)...", PICK_LIFT_HEIGHT)
        rospy.loginfo("  [Preserves safe headroom below 0.592m overhead beam]")
        lift.set_height(PICK_LIFT_HEIGHT, speed=0.04)

        rospy.loginfo("\n>>> [STEP 5/7] Extracting pallet straight forward out of rack...")
        ok = dock.undock(
            target_x=stage_rack_x,
            target_y=stage_rack_y,
            dock_yaw=dock_yaw,
            speed=0.10,
            pos_tolerance=0.05,
            label="Rack Extraction"
        )
        if not ok or rospy.is_shutdown():
            rospy.logwarn("Mission aborted during rack extraction.")
            return
    else:
        rospy.loginfo(">>> Skipping Rack Pickup (Pallet assumed already loaded on forks).")

    # ---------------------------------------------------------
    # PHASE 2: Elevate to Safe Transit Height (Clearing 25cm Station)
    # ---------------------------------------------------------
    rospy.loginfo("\n>>> [PHASE 2] Elevating lift mast to Transit Height (%.2fm)...", TRANSIT_LIFT_HEIGHT)
    rospy.loginfo("  [Pallet bottom sits at Z = 0.010m + %.2fm = %.2fm -> Clears 0.25m station by 40mm]",
                  TRANSIT_LIFT_HEIGHT, 0.010 + TRANSIT_LIFT_HEIGHT)
    lift.set_height(TRANSIT_LIFT_HEIGHT, speed=lift_speed)

    # ---------------------------------------------------------
    # PHASE 3: Orthogonal Corridor Highway Navigation
    # ---------------------------------------------------------
    rospy.loginfo("\n>>> [PHASE 3A] Navigating to Corridor Junction (%.2f, %.2f)...", corridor_x, corridor_y)
    rospy.loginfo("  [Following orthogonal Manhattan route to strictly avoid diagonal blockage]")
    ok = nav.navigate_to(gx=corridor_x, gy=corridor_y, goal_yaw=None, pos_tolerance=0.08, label="Corridor Junction")
    if not ok or rospy.is_shutdown():
        rospy.logwarn("Mission aborted at Corridor Junction.")
        return

    rospy.loginfo("\n>>> [PHASE 3B] Navigating along main highway to Pre-Dock Staging Pose (%.2f, %.2f)...",
                  stage_station_x, stage_station_y)
    rospy.loginfo("  [Standoff at %.2fm ensures ~1.0m frontal clearance from dock at (%.2f, %.2f)]",
                  stage_station_x, station_x, station_y)
    rospy.loginfo("  [Aligning to Yaw: %.1f deg (Facing West, forks pointing East towards stocker)]", station_yaw)
    ok = nav.navigate_to(gx=stage_station_x, gy=stage_station_y, goal_yaw=station_yaw, pos_tolerance=0.05, label="Stocker Staging")
    if not ok or rospy.is_shutdown():
        rospy.logwarn("Mission aborted at Stocker Staging.")
        return

    # Ensure lift is verified at transit height before entering station footprint
    lift.set_height(TRANSIT_LIFT_HEIGHT, speed=0.04)

    # ---------------------------------------------------------
    # PHASE 4: Precision Reverse Insertion Over 25cm Staging Station
    # ---------------------------------------------------------
    rospy.loginfo("\n>>> [PHASE 4] Reversing over 25cm Staging Station to drive_center Stop: (%.2f, %.2f)...",
                  target_deposit_x, target_deposit_y)
    rospy.loginfo("  [Pallet glides over 0.25m station with 40mm vertical clearance]")
    ok = dock.dock_reverse(
        target_x=target_deposit_x,
        target_y=target_deposit_y,
        dock_yaw=station_yaw,
        speed=dock_speed,
        pos_tolerance=0.04,
        label="Station Reverse Docking"
    )
    if not ok or rospy.is_shutdown():
        rospy.logwarn("Mission aborted during station reverse insertion.")
        return

    # ---------------------------------------------------------
    # PHASE 5: Station Touchdown & Fork Disengagement
    # ---------------------------------------------------------
    rospy.loginfo("\n>>> [PHASE 5] Lowering mast to Deposit Height (%.2fm)...", DEPOSIT_LIFT_HEIGHT)
    rospy.loginfo("  [Physics: At q=0.24m pallet settles onto station; at q=0.21m forks sink into pocket]")
    rospy.loginfo("  [Forks float freely: 10mm above station surface, 30mm below pallet cavity ceiling]")
    lift.set_height(DEPOSIT_LIFT_HEIGHT, speed=0.04)

    # Short pause to let PhysX contact settle
    rospy.sleep(0.5)

    # ---------------------------------------------------------
    # PHASE 6: Undocking Extraction (Drive Forward, Leave Pallet)
    # ---------------------------------------------------------
    rospy.loginfo("\n>>> [PHASE 6] Undocking: Driving forward to X=%.2f to withdraw forks...", retreat_x)
    rospy.loginfo("  [Pallet remains resting on 0.25m staging station deck]")
    ok = dock.undock(
        target_x=retreat_x,
        target_y=stage_station_y,
        dock_yaw=station_yaw,
        speed=0.10,
        pos_tolerance=0.05,
        label="Station Undock Extraction"
    )
    if not ok or rospy.is_shutdown():
        rospy.logwarn("Mission aborted during undock extraction.")
        return

    # ---------------------------------------------------------
    # PHASE 7: Mast Home / Mission Complete
    # ---------------------------------------------------------
    rospy.loginfo("\n>>> [PHASE 7] Lowering mast to idle/transit height (0.00m)...")
    lift.set_height(0.00, speed=lift_speed)

    rospy.loginfo("\n==================================================")
    rospy.loginfo("SUCCESS: MILESTONE 1 FULL MISSION COMPLETE!")
    rospy.loginfo("Pallet successfully picked from rack (%.2f, %.2f) and deposited on station (%.2f, %.2f).",
                  pallet_x, pallet_y, station_x, station_y)
    rospy.loginfo("AMR retreated to aisle at (%.2f, %.2f) with forks clear.", retreat_x, stage_station_y)
    rospy.loginfo("==================================================")


if __name__ == "__main__":
    try:
        run_mission()
    except rospy.ROSInterruptException:
        pass
