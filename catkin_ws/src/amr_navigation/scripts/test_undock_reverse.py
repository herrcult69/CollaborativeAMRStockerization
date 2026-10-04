#!/usr/bin/env python3
"""
Test Pallet Reverse Undocking & Delivery Mission:
  1. (Optional) Pick up double-stacked pallet from rack at (pallet_x, pallet_y, 270 deg)
  2. Elevate lift mast to safe transit height (0.28m) to clear the 25cm (0.25m) staging station
  3. Navigate to corridor junction (4.0, 0.0)
  4. Navigate down main corridor highway to stocker staging pose (~11.98m, 0.0), align to 180 deg
     [Staging pose accounts for fork_offset + standoff in front of station at (13.0, 0.0),
      strictly providing ~1.0m clearance to prevent frontal collision with the station]
  5. Precision reverse docking over the 25cm tall staging station (stop at 13.0 - fork_offset = 12.78m)
  6. Controlled lift lowering to deposit height (0.21m):
     - At 0.24m: Pallet touches down on the 0.25m station deck
     - At 0.21m: Forks lower inside pocket cavity (10mm clearance above deck, 30mm below ceiling)
       transferring 100% of pallet weight onto the station
  7. Undock extraction: Pull straight forward from 12.78m back to staging standoff (~11.98m) (locked yaw 180 deg)
     leaving the pallet resting securely on the staging station
  8. Lower mast to travel/idle height (0.00m) - Mission Complete!
"""
import math
import rospy
from amr_navigation.move_to_point import MoveToPointController
from amr_navigation.move_base_nav import MoveBaseNavigator
from amr_navigation.docking import PalletDockingController, compute_dock_target_pose
from amr_navigation.lift import LiftController


def run_undock_reverse_test():
    rospy.init_node("test_undock_reverse")

    # Workflow mode: set true if AMR already has the pallet loaded
    skip_pickup = bool(rospy.get_param("~skip_pickup", False))

    # Rack pickup parameters
    pallet_x = float(rospy.get_param("~pallet_x", 4.0))
    pallet_y = float(rospy.get_param("~pallet_y", 5.0))
    dock_yaw = float(rospy.get_param("~dock_yaw", 270.0))  # degrees
    fork_offset = float(rospy.get_param("~fork_offset", 0.22))  # deep tine penetration (85-90%)

    # Transit & Highway Waypoints (Orthogonal routing avoids diagonal blockage)
    corridor_x = float(rospy.get_param("~corridor_x", 4.0))
    corridor_y = float(rospy.get_param("~corridor_y", 0.0))

    # Staging Station / Stocker Target (Located at X=13.0, Y=0.0, 25cm tall)
    station_x = float(rospy.get_param("~station_x", 13.0))
    station_y = float(rospy.get_param("~station_y", 0.0))
    station_yaw = float(rospy.get_param("~station_yaw", 180.0))  # Facing West, rear pointing East into dock
    standoff = float(rospy.get_param("~standoff", 0.8))  # Standoff in front of drive_stop pose

    # Kinematic Speeds
    dock_speed = float(rospy.get_param("~dock_speed", 0.12))
    transit_speed = float(rospy.get_param("~transit_speed", 0.45))
    lift_speed = float(rospy.get_param("~lift_speed", 0.06))

    # Lift Heights:
    # 1. Rack Entry: -0.02m (forks 30mm off floor -> enters 60mm ground pocket)
    # 2. Under-Rack Pick: +0.015m (lifts pallet 15mm, preserves 17mm overhead beam headroom)
    # 3. Transit Clearance: +0.28m (pallet bottom at 0.010 + 0.28 = 0.29m -> 40mm clearance over 25cm station)
    # 4. Station Deposit: +0.21m (pallet touches down at 0.24m; forks lower to 0.21m floating in mid-cavity)
    TRANSIT_LIFT_HEIGHT = float(rospy.get_param("~transit_lift_height", 0.28))
    DEPOSIT_LIFT_HEIGHT = float(rospy.get_param("~deposit_lift_height", 0.21))

    # 1. Calculate Rack Pickup Poses:
    target_rack_x, target_rack_y = compute_dock_target_pose(
        pallet_x=pallet_x,
        pallet_y=pallet_y,
        dock_yaw_deg=dock_yaw,
        fork_offset=fork_offset
    )
    stage_rack_x = target_rack_x + 0.8 * math.cos(math.radians(dock_yaw))
    stage_rack_y = target_rack_y + 0.8 * math.sin(math.radians(dock_yaw))

    # 2. Calculate Staging Station Deposit Poses (taking into account the dock at 13.0, 0.0):
    # Stop pose: drive_center stops at station_x + fork_offset * cos(yaw) = 13.0 - 0.22 = 12.78m
    target_deposit_x, target_deposit_y = compute_dock_target_pose(
        pallet_x=station_x,
        pallet_y=station_y,
        dock_yaw_deg=station_yaw,
        fork_offset=fork_offset
    )

    # Pre-dock staging pose: standoff distance in front of stop pose along heading
    # stage_x = 12.78 + 0.8 * (-1.0) = 11.98m (~12.0m) -> 1.0m clearance in front of dock!
    stage_station_x = target_deposit_x + standoff * math.cos(math.radians(station_yaw))
    stage_station_y = target_deposit_y + standoff * math.sin(math.radians(station_yaw))

    # Explicit override support (using separate param name to avoid rosparam collision)
    if rospy.has_param("~custom_staging_x"):
        stage_station_x = float(rospy.get_param("~custom_staging_x"))
    if rospy.has_param("~custom_staging_y"):
        stage_station_y = float(rospy.get_param("~custom_staging_y"))

    retreat_x = stage_station_x  # Undock extracts back to pre-dock standoff

    # Initialize controllers (if skipping pickup, current lift height is assumed ~0.15m from previous extraction)
    initial_lift = 0.15 if skip_pickup else 0.00

    fine = MoveToPointController(max_linear=0.15, max_angular=0.65)   # slow, for last-cm alignment
    nav = MoveBaseNavigator(frame="odom", refine=fine)

    dock = PalletDockingController(default_dock_speed=dock_speed, default_tolerance=0.04)
    lift = LiftController(default_speed=lift_speed, initial_height=initial_lift)

    rospy.loginfo("==================================================")
    rospy.loginfo("STARTING REVERSE UNDOCKING & STAGING DELIVERY MISSION")
    rospy.loginfo("Mode                     : %s", "Skip Pickup (start with pallet)" if skip_pickup else "Full Pick-and-Deliver")
    rospy.loginfo("Corridor Junction        : (%.2f, %.2f)", corridor_x, corridor_y)
    rospy.loginfo("Staging Station Target   : (%.2f, %.2f) [25cm Tall Dock]", station_x, station_y)
    rospy.loginfo("Station Stop Pose (Drive): (%.2f, %.2f)", target_deposit_x, target_deposit_y)
    rospy.loginfo("Pre-Dock Staging Pose    : (%.2f, %.2f) | Yaw: %.1f deg", stage_station_x, stage_station_y, station_yaw)
    rospy.loginfo("Standoff Distance        : %.2fm (Clearance to station: ~%.2fm)", standoff, station_x - stage_station_x)
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

        rospy.loginfo("\n>>> [STEP 2/7] Lowering lift mast to entry height (-0.02m)...")
        lift.set_height(-0.02, speed=0.06)

        rospy.loginfo("\n>>> [STEP 3/7] Reversing into Pallet at (%.2f, %.2f)...", pallet_x, pallet_y)
        ok = dock.dock_reverse(
            target_x=target_rack_x,
            target_y=target_rack_y,
            dock_yaw=dock_yaw,
            speed=dock_speed,
            pos_tolerance=0.04,
            label="Rack Insertion"
        )
        if not ok or rospy.is_shutdown():
            rospy.logwarn("Mission aborted during rack insertion.")
            return

        rospy.loginfo("\n>>> [STEP 4/7] Elevating mast to +0.015m (Under-Rack Safe Headroom)...")
        lift.set_height(0.015, speed=0.04)

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
    lift.set_height(TRANSIT_LIFT_HEIGHT, speed=0.06)

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
    lift.set_height(0.00, speed=0.06)

    rospy.loginfo("\n==================================================")
    rospy.loginfo("SUCCESS: PALLET REVERSE UNDOCKING & STAGING DELIVERY COMPLETE!")
    rospy.loginfo("Pallet successfully deposited on 25cm staging station at (%.2f, %.2f).", station_x, station_y)
    rospy.loginfo("AMR retreated to aisle at (%.2f, %.2f) with forks clear.", retreat_x, stage_station_y)
    rospy.loginfo("==================================================")


if __name__ == "__main__":
    try:
        run_undock_reverse_test()
    except rospy.ROSInterruptException:
        pass
