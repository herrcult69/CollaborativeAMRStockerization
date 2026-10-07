#!/usr/bin/env python3
"""
Test Staging Station Reverse Docking & Undock Extraction:
  1. Navigate directly to Staging Station Pre-Dock Staging Pose (~11.98m, 0.0) facing 180 deg
  2. Elevate lift mast to safe transit height (0.28m) to clear the 25cm (0.25m) staging station
  3. Precision reverse docking over the 25cm tall staging station (stop at 13.0 - fork_offset = 12.78m)
  4. Lower mast to deposit height (0.21m):
     - At 0.24m: Pallet touches down on the 0.25m station deck
     - At 0.21m: Forks lower inside pocket cavity (10mm clearance above deck, 30mm below ceiling)
  5. Undock extraction: Pull straight forward from 12.78m back to staging standoff (~11.98m) (locked yaw 180 deg)
  6. Lower mast to idle/travel height (0.00m) - Mission Complete!
"""
import math
import rospy
from amr_navigation.move_to_point import MoveToPointController
from amr_navigation.docking import PalletDockingController, compute_dock_target_pose
from amr_navigation.lift import LiftController


def run_undock_reverse_test():
    rospy.init_node("test_undock_reverse")

    # Physical kinematics: distance from drive_center (tracked by /odom) to fork reference
    fork_offset = float(rospy.get_param("~fork_offset", 0.22))  # deep tine penetration (85-90%)

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
    # Transit Clearance: +0.28m (pallet bottom at 0.010 + 0.28 = 0.29m -> 40mm clearance over 25cm station)
    # Station Deposit: +0.21m (pallet touches down at 0.24m; forks lower to 0.21m floating in mid-cavity)
    TRANSIT_LIFT_HEIGHT = float(rospy.get_param("~transit_lift_height", 0.28))
    DEPOSIT_LIFT_HEIGHT = float(rospy.get_param("~deposit_lift_height", 0.21))

    # Calculate Staging Station Deposit Poses (dock target at 13.0, 0.0):
    # Stop pose: drive_center stops at station_x + fork_offset * cos(yaw) = 13.0 - 0.22 = 12.78m
    target_deposit_x, target_deposit_y = compute_dock_target_pose(
        pallet_x=station_x,
        pallet_y=station_y,
        dock_yaw_deg=station_yaw,
        fork_offset=fork_offset
    )

    # Pre-dock staging pose: standoff distance in front of stop pose along heading
    # stage_x = 12.78 + 0.8 * (-1.0) = 11.98m (~12.0m) -> 1.0m clearance in front of dock
    stage_station_x = target_deposit_x + standoff * math.cos(math.radians(station_yaw))
    stage_station_y = target_deposit_y + standoff * math.sin(math.radians(station_yaw))

    # Explicit override support (using separate param name to avoid rosparam collision)
    if rospy.has_param("~custom_staging_x"):
        stage_station_x = float(rospy.get_param("~custom_staging_x"))
    if rospy.has_param("~custom_staging_y"):
        stage_station_y = float(rospy.get_param("~custom_staging_y"))

    retreat_x = stage_station_x  # Undock extracts back to pre-dock standoff

    # Initialize controllers
    nav = MoveToPointController(max_linear=transit_speed, max_angular=0.65)
    dock = PalletDockingController(default_dock_speed=dock_speed, default_tolerance=0.04)
    lift = LiftController(default_speed=lift_speed, initial_height=0.00)

    rospy.loginfo("==================================================")
    rospy.loginfo("STARTING STAGING STATION REVERSE DOCK & UNDOCK TEST")
    rospy.loginfo("Staging Station Target   : (%.2f, %.2f) [25cm Tall Dock]", station_x, station_y)
    rospy.loginfo("Station Stop Pose (Drive): (%.2f, %.2f)", target_deposit_x, target_deposit_y)
    rospy.loginfo("Pre-Dock Staging Pose    : (%.2f, %.2f) | Yaw: %.1f deg", stage_station_x, stage_station_y, station_yaw)
    rospy.loginfo("Standoff Distance        : %.2fm (Clearance to station: ~%.2fm)", standoff, station_x - stage_station_x)
    rospy.loginfo("Lift Profiles            : Transit=%.3fm | Deposit=%.3fm", TRANSIT_LIFT_HEIGHT, DEPOSIT_LIFT_HEIGHT)
    rospy.loginfo("==================================================")

    # ---------------------------------------------------------
    # STEP 1: Navigate to Station Staging Standoff Pose
    # ---------------------------------------------------------
    rospy.loginfo("\n>>> [STEP 1/6] Navigating to Pre-Dock Staging Pose (%.2f, %.2f, %.1f deg)...",
                  stage_station_x, stage_station_y, station_yaw)
    ok = nav.navigate_to(gx=stage_station_x, gy=stage_station_y, goal_yaw=station_yaw, pos_tolerance=0.05, label="Station Staging")
    if not ok or rospy.is_shutdown():
        rospy.logwarn("Mission aborted at Station Staging.")
        return

    # ---------------------------------------------------------
    # STEP 2: Elevate to Safe Transit Height (Clearing 25cm Station)
    # ---------------------------------------------------------
    rospy.loginfo("\n>>> [STEP 2/6] Elevating lift mast to Transit Height (%.2fm)...", TRANSIT_LIFT_HEIGHT)
    rospy.loginfo("  [Forks/pallet bottom clear 0.25m station by 40mm during approach]")
    lift.set_height(TRANSIT_LIFT_HEIGHT, speed=lift_speed)

    # ---------------------------------------------------------
    # STEP 3: Precision Reverse Insertion Over 25cm Staging Station
    # ---------------------------------------------------------
    rospy.loginfo("\n>>> [STEP 3/6] Reversing over 25cm Staging Station to drive_center Stop: (%.2f, %.2f)...",
                  target_deposit_x, target_deposit_y)
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
    # STEP 4: Station Touchdown / Deposit Height
    # ---------------------------------------------------------
    rospy.loginfo("\n>>> [STEP 4/6] Lowering mast to Deposit Height (%.2fm)...", DEPOSIT_LIFT_HEIGHT)
    rospy.loginfo("  [At q=0.24m pallet settles onto station; at q=0.21m forks float freely in cavity]")
    lift.set_height(DEPOSIT_LIFT_HEIGHT, speed=0.04)

    # Short pause to settle
    rospy.sleep(0.5)

    # ---------------------------------------------------------
    # STEP 5: Undocking Extraction (Drive Forward, Clear Forks)
    # ---------------------------------------------------------
    rospy.loginfo("\n>>> [STEP 5/6] Undocking: Driving forward to X=%.2f to withdraw forks...", retreat_x)
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
    # STEP 6: Mast Home / Mission Complete
    # ---------------------------------------------------------
    rospy.loginfo("\n>>> [STEP 6/6] Lowering mast to idle/transit height (0.00m)...")
    lift.set_height(0.00, speed=lift_speed)

    rospy.loginfo("\n==================================================")
    rospy.loginfo("SUCCESS: STATION REVERSE DOCK & UNDOCK COMPLETE!")
    rospy.loginfo("AMR completed reverse docking and extraction at station (%.2f, %.2f).", station_x, station_y)
    rospy.loginfo("AMR retreated to aisle at (%.2f, %.2f) with forks clear.", retreat_x, stage_station_y)
    rospy.loginfo("==================================================")


if __name__ == "__main__":
    try:
        run_undock_reverse_test()
    except rospy.ROSInterruptException:
        pass
