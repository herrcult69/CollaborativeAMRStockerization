#!/usr/bin/env python3
"""
Milestone 2 Mission: Milestone 1 pick-and-deliver with move_base driving the long legs.

Control alternates, one /cmd_vel publisher at a time:
  move_base (NavFn + DWA, ~20 cm)  -> pre-stage, nose toward the stage
  MoveToPoint (5 cm / 4 deg)       -> stage, turned to the dock heading
  Docking / Lift                   -> pick or deposit, exactly as Milestone 1

Requires unity_bridge, laser_tf, warehouse_localization (AMCL) and warehouse_navigation (move_base).
Precision legs use /odom coordinates; valid while the robot spawns at the map origin (map ~= odom).
"""
import math
import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from amr_navigation.move_to_point import MoveToPointController
from amr_navigation.docking import PalletDockingController, compute_dock_target_pose, compute_prestage_pose
from amr_navigation.lift import LiftController


def move_base_to(client, nav, x, y, yaw_deg, timeout, label):
    """Send one map-frame goal and block until move_base finishes. Stops the robot on any failure."""
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2.0)
    goal.target_pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2.0)

    rospy.loginfo(">>> MOVE_BASE GOAL: %s (%.2f, %.2f, %.1f deg)", label, x, y, yaw_deg)
    client.send_goal(goal)
    finished = client.wait_for_result(rospy.Duration(timeout))
    state = client.get_state()
    if finished and state == GoalStatus.SUCCEEDED:
        rospy.loginfo(">>> MOVE_BASE REACHED: %s", label)
        return True

    client.cancel_goal()
    nav.stop()
    rospy.logerr(">>> MOVE_BASE FAILED: %s (%s, state=%d)", label,
                 "finished" if finished else "timeout %.0fs" % timeout, state)
    return False


def run_mission():
    rospy.init_node("milestone2_mission")

    skip_pickup = bool(rospy.get_param("~skip_pickup", False))

    pallet_x = float(rospy.get_param("~pallet_x", 4.0))
    pallet_y = float(rospy.get_param("~pallet_y", 5.0))
    dock_yaw = float(rospy.get_param("~dock_yaw", 270.0))
    fork_offset = float(rospy.get_param("~fork_offset", 0.22))

    station_x = float(rospy.get_param("~station_x", 13.12))
    station_y = float(rospy.get_param("~station_y", 0.0))
    station_yaw = float(rospy.get_param("~station_yaw", 180.0))
    standoff = float(rospy.get_param("~standoff", 0.8))
    rack_standoff = float(rospy.get_param("~rack_standoff", 1.2))

    # Distance from stage to the move_base goal; MoveToPoint drives this last stretch straight
    approach = float(rospy.get_param("~approach", 0.6))
    move_base_timeout = float(rospy.get_param("~move_base_timeout", 180.0))

    dock_speed = float(rospy.get_param("~dock_speed", 0.12))
    transit_speed = float(rospy.get_param("~transit_speed", 0.45))
    lift_speed = float(rospy.get_param("~lift_speed", 0.08))

    # Same lift profiles as Milestone 1
    pick_level = str(rospy.get_param("~pick_level", "bottom")).strip().lower()
    if pick_level in ("top", "upper", "2"):
        default_entry, default_pick, default_transit = 0.240, 0.290, 0.290
    elif pick_level in ("shelf", "3"):
        default_entry, default_pick, default_transit = 0.640, 0.685, 0.280
    else:
        default_entry, default_pick, default_transit = -0.020, 0.010, 0.280

    ENTRY_LIFT_HEIGHT = float(rospy.get_param("~entry_lift_height", default_entry))
    PICK_LIFT_HEIGHT = float(rospy.get_param("~pick_lift_height", default_pick))
    raw_transit = rospy.get_param("~transit_lift_height", None)
    if raw_transit is None or (float(raw_transit) == 0.28 and pick_level in ("top", "upper", "2")):
        TRANSIT_LIFT_HEIGHT = default_transit
    else:
        TRANSIT_LIFT_HEIGHT = float(raw_transit)
    DEPOSIT_LIFT_HEIGHT = float(rospy.get_param("~deposit_lift_height", 0.21))

    # Rack poses: dock target -> stage (rack_standoff) -> pre-stage
    target_rack_x, target_rack_y = compute_dock_target_pose(pallet_x, pallet_y, dock_yaw, fork_offset)
    stage_rack_x = target_rack_x + rack_standoff * math.cos(math.radians(dock_yaw))
    stage_rack_y = target_rack_y + rack_standoff * math.sin(math.radians(dock_yaw))
    pre_rack = compute_prestage_pose(stage_rack_x, stage_rack_y, dock_yaw, approach)

    # Station poses
    target_deposit_x, target_deposit_y = compute_dock_target_pose(station_x, station_y, station_yaw, fork_offset)
    stage_station_x = target_deposit_x + standoff * math.cos(math.radians(station_yaw))
    stage_station_y = target_deposit_y + standoff * math.sin(math.radians(station_yaw))
    pre_station = compute_prestage_pose(stage_station_x, stage_station_y, station_yaw, approach)

    initial_lift = 0.15 if skip_pickup else 0.00
    nav = MoveToPointController(max_linear=transit_speed, max_angular=0.65)
    dock = PalletDockingController(default_dock_speed=dock_speed, default_tolerance=0.04)
    lift = LiftController(default_speed=lift_speed, initial_height=initial_lift)

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    if not client.wait_for_server(rospy.Duration(30.0)):
        rospy.logerr("move_base not running. Start warehouse_localization + warehouse_navigation first.")
        return

    rospy.loginfo("==================================================")
    rospy.loginfo("STARTING MILESTONE 2 MISSION (move_base + precision docking)")
    rospy.loginfo("Mode              : %s", "Skip Pickup" if skip_pickup else "Full Pick-and-Deliver")
    rospy.loginfo("Rack   pre-stage  : (%.2f, %.2f, %.1f) -> stage (%.2f, %.2f, %.1f) -> dock (%.2f, %.2f)",
                  pre_rack[0], pre_rack[1], pre_rack[2], stage_rack_x, stage_rack_y, dock_yaw,
                  target_rack_x, target_rack_y)
    rospy.loginfo("Station pre-stage : (%.2f, %.2f, %.1f) -> stage (%.2f, %.2f, %.1f) -> dock (%.2f, %.2f)",
                  pre_station[0], pre_station[1], pre_station[2], stage_station_x, stage_station_y, station_yaw,
                  target_deposit_x, target_deposit_y)
    rospy.loginfo("Lift              : Entry=%.3f Pick=%.3f Transit=%.3f Deposit=%.3f [%s]",
                  ENTRY_LIFT_HEIGHT, PICK_LIFT_HEIGHT, TRANSIT_LIFT_HEIGHT, DEPOSIT_LIFT_HEIGHT, pick_level)
    rospy.loginfo("==================================================")

    # ---------------------------------------------------------
    # LEG 1-3: move_base to rack, precision stage, pick
    # ---------------------------------------------------------
    if not skip_pickup:
        rospy.loginfo("\n>>> [LEG 1] move_base -> Rack Pre-Stage")
        if not move_base_to(client, nav, pre_rack[0], pre_rack[1], pre_rack[2], move_base_timeout, "Rack Pre-Stage"):
            return

        rospy.loginfo("\n>>> [LEG 2] MoveToPoint -> Rack Staging")
        ok = nav.navigate_to(gx=stage_rack_x, gy=stage_rack_y, goal_yaw=dock_yaw, pos_tolerance=0.05, label="Rack Staging")
        if not ok or rospy.is_shutdown():
            rospy.logwarn("Mission aborted at Rack Staging.")
            return

        rospy.loginfo("\n>>> [LEG 3] Pick: entry height %.3fm, reverse, lift, extract", ENTRY_LIFT_HEIGHT)
        lift.set_height(ENTRY_LIFT_HEIGHT, speed=0.06)
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

        lift.set_height(PICK_LIFT_HEIGHT, speed=0.04)
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

    rospy.loginfo("\n>>> Transit height %.2fm", TRANSIT_LIFT_HEIGHT)
    lift.set_height(TRANSIT_LIFT_HEIGHT, speed=lift_speed)

    # ---------------------------------------------------------
    # LEG 4-5: move_base to station, precision stage
    # ---------------------------------------------------------
    rospy.loginfo("\n>>> [LEG 4] move_base -> Station Pre-Stage")
    if not move_base_to(client, nav, pre_station[0], pre_station[1], pre_station[2], move_base_timeout, "Station Pre-Stage"):
        return

    rospy.loginfo("\n>>> [LEG 5] MoveToPoint -> Stocker Staging")
    ok = nav.navigate_to(gx=stage_station_x, gy=stage_station_y, goal_yaw=station_yaw, pos_tolerance=0.05, label="Stocker Staging")
    if not ok or rospy.is_shutdown():
        rospy.logwarn("Mission aborted at Stocker Staging.")
        return

    # ---------------------------------------------------------
    # LEG 6: deposit, exactly as Milestone 1
    # ---------------------------------------------------------
    rospy.loginfo("\n>>> [LEG 6] Deposit: reverse onto station, lower to %.2fm, extract", DEPOSIT_LIFT_HEIGHT)
    lift.set_height(TRANSIT_LIFT_HEIGHT, speed=0.04)
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

    lift.set_height(DEPOSIT_LIFT_HEIGHT, speed=0.04)
    rospy.sleep(0.5)  # let PhysX contact settle

    ok = dock.undock(
        target_x=stage_station_x,
        target_y=stage_station_y,
        dock_yaw=station_yaw,
        speed=0.10,
        pos_tolerance=0.05,
        label="Station Undock Extraction"
    )
    if not ok or rospy.is_shutdown():
        rospy.logwarn("Mission aborted during undock extraction.")
        return

    lift.set_height(0.00, speed=lift_speed)

    rospy.loginfo("\n==================================================")
    rospy.loginfo("SUCCESS: MILESTONE 2 MISSION COMPLETE!")
    rospy.loginfo("Pallet picked from rack (%.2f, %.2f) and deposited on station (%.2f, %.2f).",
                  pallet_x, pallet_y, station_x, station_y)
    rospy.loginfo("==================================================")


if __name__ == "__main__":
    try:
        run_mission()
    except rospy.ROSInterruptException:
        pass
