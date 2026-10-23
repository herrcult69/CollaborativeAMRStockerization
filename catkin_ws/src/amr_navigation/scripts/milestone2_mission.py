#!/usr/bin/env python3
"""
Milestone 2 Mission: Milestone 1 pick-and-deliver with move_base driving the long legs.

Control alternates, one /cmd_vel publisher at a time:
  move_base (NavFn + DWA, ~20 cm)  -> pre-stage, nose toward the stage
  MoveToPoint (5 cm / 4 deg)       -> stage, turned to the dock heading
  Docking / Lift                   -> pick or deposit, exactly as Milestone 1

Requires unity_bridge, laser_tf, warehouse_localization (AMCL) and warehouse_navigation (move_base).
The package and station are named entries of config/locations_*.yaml (object centres in map).
At each move_base -> precision handoff the map->odom transform is read
once from AMCL and frozen; the precision controllers then run on smooth /odom against converted targets.
At the rack the robot stops facing the pallet and measures its load face with the laser; docking then
follows the detected pallet, not the map target (abort if detection fails or disagrees by > 10 cm / 5 deg).
"""
import math
import actionlib
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from amr_navigation.move_to_point import MoveToPointController
from amr_navigation.docking import (PalletDockingController, compute_dock_target_pose, compute_prestage_pose,
                                    map_pose_to_odom)
from amr_navigation.lift import LiftController
from amr_navigation.locations import FLOOR_MAX_HEIGHT, find_location
from amr_navigation.pallet_detect import detect_load_face, scan_to_points

TF_ERRORS = (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException)


def quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def snapshot_map_to_odom(tf_buffer, nav, settle, label):
    """Read odom<-map once after the robot has stopped. Returns (dx, dy, dth) or None (robot stopped)."""
    rospy.sleep(settle)
    try:
        t = tf_buffer.lookup_transform("odom", "map", rospy.Time(0), rospy.Duration(2.0))
    except TF_ERRORS as e:
        nav.stop()
        rospy.logerr(">>> MAP->ODOM SNAPSHOT FAILED: %s (%s). Is AMCL running?", label, e)
        return None
    dth = quat_yaw(t.transform.rotation)
    dx, dy = t.transform.translation.x, t.transform.translation.y
    rospy.loginfo(">>> MAP->ODOM SNAPSHOT: %s | dx=%.3f dy=%.3f dth=%.2f deg", label, dx, dy, math.degrees(dth))
    return dx, dy, dth


def detect_pallet(tf_buffer, expected, width, face_offset, scans=5):
    """
    Measure the pallet from its load face in odom over several scans (robot standing still, facing it).
    Returns (x, y, dock_yaw_deg) in odom, or None after logging why.
    """
    rospy.sleep(0.5)
    found = []
    for _ in range(scans):
        try:
            scan = rospy.wait_for_message("/scan", LaserScan, timeout=2.0)
            t = tf_buffer.lookup_transform("odom", scan.header.frame_id, rospy.Time(0), rospy.Duration(0.5))
        except (rospy.ROSException,) + TF_ERRORS as e:
            rospy.logwarn("[Detect] scan/TF unavailable: %s", e)
            continue
        pts = scan_to_points(scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max,
                             t.transform.translation.x, t.transform.translation.y, quat_yaw(t.transform.rotation))
        pose, why = detect_load_face(pts, expected[0], expected[1], expected[2], width=width, face_offset=face_offset)
        if pose is None:
            rospy.logwarn("[Detect] scan rejected: %s", why)
        else:
            found.append(pose)
    if len(found) < 3:
        rospy.logerr(">>> PALLET DETECTION FAILED: %d/%d scans found the load face.", len(found), scans)
        return None
    n = float(len(found))
    mx, my = sum(p[0] for p in found) / n, sum(p[1] for p in found) / n
    myaw = sum(p[2] for p in found) / n
    spread = max(math.hypot(p[0] - mx, p[1] - my) for p in found)
    spread_yaw = max(abs(p[2] - myaw) for p in found)
    if spread > 0.005 or spread_yaw > 1.0:  # 3 mm range noise alone gives ~0.7 deg per scan
        rospy.logerr(">>> PALLET DETECTION FAILED: scans disagree (%.1f mm, %.2f deg).", spread * 1000.0, spread_yaw)
        return None
    ux, uy = math.cos(math.radians(expected[2])), math.sin(math.radians(expected[2]))
    lat = -(mx - expected[0]) * uy + (my - expected[1]) * ux
    depth = (mx - expected[0]) * ux + (my - expected[1]) * uy
    rospy.loginfo(">>> PALLET DETECTED (odom): (%.3f, %.3f, %.2f deg) | expected (%.3f, %.3f, %.2f) | "
                  "lateral %+.1f mm, depth %+.1f mm, yaw %+.2f deg | %d scans",
                  mx, my, myaw, expected[0], expected[1], expected[2],
                  lat * 1000.0, depth * 1000.0, myaw - expected[2], len(found))
    return mx, my, myaw


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

    # Named object centres from config/locations_*.yaml (loaded into ~packages / ~stations by the launch file)
    package = str(rospy.get_param("~package", "package_5"))
    station = str(rospy.get_param("~station", "station_2"))
    pallet_x, pallet_y, dock_yaw, pallet_h = find_location(rospy.get_param("~packages", {}), package)
    station_x, station_y, station_yaw, _ = find_location(rospy.get_param("~stations", {}), station)
    # drive_center stops this far in front of the object centre when docked (see compute_dock_target_pose)
    pallet_dock_offset = float(rospy.get_param("~pallet_dock_offset", 0.342))
    station_dock_offset = float(rospy.get_param("~station_dock_offset", 0.54))
    # Touching the station this close to the stop counts as docked (stall guard in dock_reverse)
    station_contact_tol = float(rospy.get_param("~station_contact_tol", 0.10))

    standoff = float(rospy.get_param("~standoff", 0.8))
    rack_standoff = float(rospy.get_param("~rack_standoff", 1.2))

    # Distance from stage to the move_base goal; MoveToPoint drives this last stretch straight
    approach = float(rospy.get_param("~approach", 0.6))
    move_base_timeout = float(rospy.get_param("~move_base_timeout", 180.0))

    dock_speed = float(rospy.get_param("~dock_speed", 0.12))
    transit_speed = float(rospy.get_param("~transit_speed", 0.45))
    lift_speed = float(rospy.get_param("~lift_speed", 0.08))

    # Dynamic lift profiles matching measured rack heights (supports auto detection from pallet_h)
    pick_level = str(rospy.get_param("~pick_level", "auto")).strip().lower()
    if pick_level in ("top_shelf", "shelf_top", "4"):
        default_entry, default_pick, default_transit = 1.005, 1.025, 0.280
    elif pick_level in ("shelf", "3"):
        default_entry, default_pick, default_transit = (0.740, 0.765, 0.280) if pallet_h > 0.70 else (0.640, 0.685, 0.280)
    elif pick_level in ("top", "upper", "stack", "2"):
        default_entry, default_pick, default_transit = (0.335, 0.355, 0.280) if pallet_h > 0.32 else (0.240, 0.290, 0.290)
    elif pick_level in ("bottom", "floor", "1"):
        default_entry, default_pick, default_transit = (0.070, 0.090, 0.280) if pallet_h > 0.08 else (-0.020, 0.010, 0.280)
    else:  # "auto" or unrecognized: select dynamically from YAML pallet_h!
        if pallet_h > 0.90:
            default_entry, default_pick, default_transit = 1.005, 1.025, 0.280
            pick_level = "top_shelf (auto)"
        elif pallet_h > 0.60:
            default_entry, default_pick, default_transit = (0.740, 0.765, 0.280) if pallet_h > 0.70 else (0.640, 0.685, 0.280)
            pick_level = "shelf (auto)"
        elif pallet_h > 0.20:
            default_entry, default_pick, default_transit = (0.335, 0.355, 0.280) if pallet_h > 0.32 else (0.240, 0.290, 0.290)
            pick_level = "stack (auto)"
        else:
            default_entry, default_pick, default_transit = (0.070, 0.090, 0.280) if pallet_h > 0.08 else (-0.020, 0.010, 0.280)
            pick_level = "bottom (auto)"

    ENTRY_LIFT_HEIGHT = float(rospy.get_param("~entry_lift_height", default_entry))
    PICK_LIFT_HEIGHT = float(rospy.get_param("~pick_lift_height", default_pick))
    raw_transit = rospy.get_param("~transit_lift_height", None)
    if raw_transit is None or (float(raw_transit) == 0.28 and "top" in pick_level):
        TRANSIT_LIFT_HEIGHT = default_transit
    else:
        TRANSIT_LIFT_HEIGHT = float(raw_transit)
    DEPOSIT_LIFT_HEIGHT = float(rospy.get_param("~deposit_lift_height", 0.21))

    # Safety check: bottom lift profile without auto-detect only fits ground pallets
    valid_upper_levels = ("top", "upper", "stack", "2", "shelf", "3", "top_shelf", "shelf_top", "4")
    if not skip_pickup and not any(k in pick_level for k in valid_upper_levels) and not pick_level.endswith("(auto)") and pallet_h > FLOOR_MAX_HEIGHT:
        rospy.logerr("%s sits on an upper rack level (%.2f m); specify pick_level or use pick_level:=auto.",
                     package, pallet_h)
        return

    # Rack poses: dock target -> stage (rack_standoff) -> pre-stage
    target_rack_x, target_rack_y = compute_dock_target_pose(pallet_x, pallet_y, dock_yaw, pallet_dock_offset)
    stage_rack_x = target_rack_x + rack_standoff * math.cos(math.radians(dock_yaw))
    stage_rack_y = target_rack_y + rack_standoff * math.sin(math.radians(dock_yaw))
    pre_rack = compute_prestage_pose(stage_rack_x, stage_rack_y, dock_yaw, approach)

    # Station poses
    target_deposit_x, target_deposit_y = compute_dock_target_pose(station_x, station_y, station_yaw, station_dock_offset)
    stage_station_x = target_deposit_x + standoff * math.cos(math.radians(station_yaw))
    stage_station_y = target_deposit_y + standoff * math.sin(math.radians(station_yaw))
    pre_station = compute_prestage_pose(stage_station_x, stage_station_y, station_yaw, approach)

    initial_lift = 0.15 if skip_pickup else 0.00
    nav = MoveToPointController(max_linear=transit_speed, max_angular=0.65)
    dock = PalletDockingController(default_dock_speed=dock_speed, default_tolerance=0.04)
    lift = LiftController(default_speed=lift_speed, initial_height=initial_lift)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)  # noqa: F841 (keeps the buffer filled)
    snapshot_settle = float(rospy.get_param("~snapshot_settle", 1.0))
    use_detection = bool(rospy.get_param("~detect_pallet", True))
    load_width = float(rospy.get_param("~load_width", 0.35))              # load box front face width (m)
    # Load box front face, measured from the pallet centre toward the aisle (box 0.30 m deep, centred)
    load_face_offset = float(rospy.get_param("~load_face_offset", 0.15))

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    if not client.wait_for_server(rospy.Duration(30.0)):
        rospy.logerr("move_base not running. Start warehouse_localization + warehouse_navigation first.")
        return

    rospy.loginfo("==================================================")
    rospy.loginfo("STARTING MILESTONE 2 MISSION (move_base + precision docking)")
    rospy.loginfo("Mode              : %s", "Skip Pickup" if skip_pickup else "Full Pick-and-Deliver")
    rospy.loginfo("Package / station : %s (%.3f, %.3f, %.1f) / %s (%.3f, %.3f, %.1f)",
                  package, pallet_x, pallet_y, dock_yaw, station, station_x, station_y, station_yaw)
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

        T = snapshot_map_to_odom(tf_buffer, nav, snapshot_settle, "Rack")
        if T is None:
            return
        stage_o = map_pose_to_odom(stage_rack_x, stage_rack_y, dock_yaw, *T)
        dock_o = map_pose_to_odom(target_rack_x, target_rack_y, dock_yaw, *T)

        rospy.loginfo("\n>>> [LEG 2] MoveToPoint -> Rack Staging")
        if use_detection:
            # Stop square to the rack (dock heading + 180, e.g. +90 deg for a 270 deg dock) so the laser
            # looks straight at the load, then turn for the reverse
            ok = nav.navigate_to(gx=stage_o[0], gy=stage_o[1], goal_yaw=(stage_o[2] + 180.0) % 360.0,
                                 pos_tolerance=0.05, yaw_tolerance=2.0, label="Rack Staging (facing pallet)")
            if not ok or rospy.is_shutdown():
                rospy.logwarn("Mission aborted at Rack Staging.")
                return
            pallet_o = map_pose_to_odom(pallet_x, pallet_y, dock_yaw, *T)
            det = detect_pallet(tf_buffer, pallet_o, load_width, load_face_offset)
            if det is None:
                nav.stop()
                rospy.logerr("Mission aborted: pallet not confirmed by the laser.")
                return
            line_yaw = math.radians(det[2])
            dock_o = compute_dock_target_pose(det[0], det[1], det[2], pallet_dock_offset) + (det[2],)
            turn_at = stage_o
            stage_o = (dock_o[0] + rack_standoff * math.cos(line_yaw),
                       dock_o[1] + rack_standoff * math.sin(line_yaw), det[2])
            # Turn in place where the robot stands; dock_reverse then line-follows onto the detected axis
            ok = nav.navigate_to(gx=turn_at[0], gy=turn_at[1], goal_yaw=det[2], pos_tolerance=0.08,
                                 label="Rack Staging (turn)")
        else:
            ok = nav.navigate_to(gx=stage_o[0], gy=stage_o[1], goal_yaw=stage_o[2], pos_tolerance=0.05,
                                 label="Rack Staging")
        if not ok or rospy.is_shutdown():
            rospy.logwarn("Mission aborted at Rack Staging.")
            return

        rospy.loginfo("\n>>> [LEG 3] Pick: entry height %.3fm, reverse, lift, extract", ENTRY_LIFT_HEIGHT)
        lift.set_height(ENTRY_LIFT_HEIGHT, speed=0.06)
        ok = dock.dock_reverse(
            target_x=dock_o[0],
            target_y=dock_o[1],
            dock_yaw=dock_o[2],
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
            target_x=stage_o[0],
            target_y=stage_o[1],
            dock_yaw=stage_o[2],
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

    T = snapshot_map_to_odom(tf_buffer, nav, snapshot_settle, "Station")
    if T is None:
        return
    stage_o = map_pose_to_odom(stage_station_x, stage_station_y, station_yaw, *T)
    dock_o = map_pose_to_odom(target_deposit_x, target_deposit_y, station_yaw, *T)

    rospy.loginfo("\n>>> [LEG 5] MoveToPoint -> Stocker Staging")
    ok = nav.navigate_to(gx=stage_o[0], gy=stage_o[1], goal_yaw=stage_o[2], pos_tolerance=0.05, label="Stocker Staging")
    if not ok or rospy.is_shutdown():
        rospy.logwarn("Mission aborted at Stocker Staging.")
        return

    # ---------------------------------------------------------
    # LEG 6: deposit, exactly as Milestone 1
    # ---------------------------------------------------------
    rospy.loginfo("\n>>> [LEG 6] Deposit: reverse onto station, lower to %.2fm, extract", DEPOSIT_LIFT_HEIGHT)
    lift.set_height(TRANSIT_LIFT_HEIGHT, speed=0.04)
    ok = dock.dock_reverse(
        target_x=dock_o[0],
        target_y=dock_o[1],
        dock_yaw=dock_o[2],
        speed=dock_speed,
        pos_tolerance=0.04,
        contact_tol=station_contact_tol,
        label="Station Reverse Docking"
    )
    if not ok or rospy.is_shutdown():
        rospy.logwarn("Mission aborted during station reverse insertion.")
        return

    lift.set_height(DEPOSIT_LIFT_HEIGHT, speed=0.04)
    rospy.sleep(0.5)  # let PhysX contact settle

    ok = dock.undock(
        target_x=stage_o[0],
        target_y=stage_o[1],
        dock_yaw=stage_o[2],
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
    rospy.loginfo("%s picked from the rack and deposited on %s.", package, station)
    rospy.loginfo("==================================================")


if __name__ == "__main__":
    try:
        run_mission()
    except rospy.ROSInterruptException:
        pass
