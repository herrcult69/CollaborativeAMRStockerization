#!/usr/bin/env python3
"""
Milestone 1 Mission Coordinator: Autonomous Pallet Stacker Workflow.

Orchestrates multi-waypoint navigation and pallet lifting:
  1. Move to (4.0, 4.0), align to 270 deg (facing pick station), lift up to 0.35m
  2. Move to (4.0, 0.0) transit waypoint out of pick bay
  3. Move to (13.0, 0.0), align to 180 deg (facing storage bay), set lift to 0.25m
  4. Move back to (12.0, 0.0) retreat transit waypoint
"""
import rospy
from amr_navigation.docking import ThreePhaseDockingController
from amr_navigation.lift import LiftController


def run_mission():
    rospy.init_node("milestone1_mission")

    # Gentle warehouse speeds (max_linear = 0.35 m/s, max_angular = 0.65 rad/s) 
    controller = ThreePhaseDockingController(max_linear=0.75, max_angular=0.65) # Changed to 0.75 for faster
    # Gentle hydraulic lift speed (0.06 m/s = 6 cm/s) to prevent physics shock
    lift = LiftController(default_speed=0.1) # Change to 0.1 for faster

    rospy.loginfo("==================================================")
    rospy.loginfo("STARTING MILESTONE 1 PALLET STACKER MISSION")
    rospy.loginfo("Profile: (4,4, 270deg, lift 0.35) -> (4,0) -> (13,0, 180deg, lift 0.25) -> (12,0)")
    rospy.loginfo("Speed Limits: Linear %.2fm/s | Angular %.2frad/s | Lift %.2fm/s",
                  controller.max_linear, controller.max_angular, lift.default_speed)
    rospy.loginfo("==================================================")

    # Step 1: Move to (4.0, 4.0), turn 270 deg, lift to 0.35 m
    rospy.loginfo("\n>>> [STEP 1/4] Navigating to Pick Station (4.0, 4.0) with yaw 270 deg...")
    ok = controller.navigate_to(
        gx=4.0, gy=4.0, goal_yaw=270.0,
        pos_tolerance=0.05, label="WP1 (Pick Station)"
    )
    if not ok or rospy.is_shutdown():
        rospy.logwarn("Mission aborted at Step 1 (Navigation).")
        return

    rospy.loginfo(">>> [STEP 1/4] Elevating mast to 0.35m...")
    lift.set_height(0.35, speed=0.06)

    # Step 2: Move to (4.0, 0.0) transit waypoint
    rospy.loginfo("\n>>> [STEP 2/4] Moving to corridor transit junction (4.0, 0.0)...")
    ok = controller.navigate_to(
        gx=4.0, gy=0.0, goal_yaw=None,
        pos_tolerance=0.08, label="WP2 (Corridor Junction)"
    )
    if not ok or rospy.is_shutdown():
        rospy.logwarn("Mission aborted at Step 2.")
        return

    # Step 3: Move to (12.5, 0.0), align to 180 deg, drop lift to 0.25 m
    rospy.loginfo("\n>>> [STEP 3/4] Navigating to Storage Bay (12.5, 0.0) with yaw 180 deg...")
    ok = controller.navigate_to(
        gx=12.8, gy=0.0, goal_yaw=180.0,
        pos_tolerance=0.05, label="WP3 (Storage Bay)"
    )
    if not ok or rospy.is_shutdown():
        rospy.logwarn("Mission aborted at Step 3 (Navigation).")
        return

    rospy.loginfo(">>> [STEP 3/4] Lowering mast to 0.25m...")
    lift.set_height(0.20, speed=0.1)

    # Step 4: Move back out to (12.0, 0.0)
    rospy.loginfo("\n>>> [STEP 4/4] Retreating back out to (12.0, 0.0)...")
    ok = controller.navigate_to(
        gx=12.0, gy=0.0, goal_yaw=None,
        pos_tolerance=0.08, label="WP4 (Retreat Aisle)"
    )
    if not ok or rospy.is_shutdown():
        rospy.logwarn("Mission aborted at Step 4.")
        return

    controller.stop()
    rospy.loginfo("\n==================================================")
    rospy.loginfo("SUCCESS: MILESTONE 1 FULL MISSION COMPLETE!")
    rospy.loginfo("All 4 navigation & lift steps successfully completed.")
    rospy.loginfo("==================================================")


if __name__ == "__main__":
    try:
        run_mission()
    except rospy.ROSInterruptException:
        pass
