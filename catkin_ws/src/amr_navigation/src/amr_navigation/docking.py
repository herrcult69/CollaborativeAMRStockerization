"""
Dedicated AMR Pallet Docking & Insertion Module.

Performs constrained straight-line docking maneuvers (reverse insertion and forward extraction)
into pallet pockets with active heading stabilization and ZERO in-place spinning.
"""
import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from amr_navigation.move_to_point import normalize_angle, deg_to_rad


def compute_dock_target_pose(pallet_x, pallet_y, dock_yaw_deg, fork_offset=0.22):
    """
    Computes the required drive_center coordinate so that the fork tines reach (pallet_x, pallet_y).
    Forks are mounted on the robot's rear (-X axis).
    /odom tracks the differential-drive fulcrum (drive_center, X = -0.10m from base_link).

    fork_offset controls tine penetration depth:
      - 0.35m → fork tine midpoint over cavity (~50% insertion, old default)
      - 0.22m → fork heel near cavity entry (~85-90% insertion, current default)

    Formula:
        target_x = pallet_x + fork_offset * cos(yaw)
        target_y = pallet_y + fork_offset * sin(yaw)

    :param pallet_x: Target pallet pocket / cavity center X (meters)
    :param pallet_y: Target pallet pocket / cavity center Y (meters)
    :param dock_yaw_deg: Robot orientation during reverse docking in degrees (e.g. 270.0)
    :param fork_offset: Distance from drive_center to fork heel reference point in meters (default: 0.22m)
    :return: (target_drive_x, target_drive_y)
    """
    yaw_rad = deg_to_rad(dock_yaw_deg)
    tx = float(pallet_x) + float(fork_offset) * math.cos(yaw_rad)
    ty = float(pallet_y) + float(fork_offset) * math.sin(yaw_rad)
    return tx, ty


# Backward compatibility alias
compute_dock_base_pose = compute_dock_target_pose


def compute_prestage_pose(stage_x, stage_y, dock_yaw_deg, approach=0.6):
    """
    move_base goal handed over to MoveToPoint: `approach` m further along the dock heading,
    nose turned back toward the stage, so MoveToPoint only drives straight then turns once.
    :return: (x, y, yaw_deg)
    """
    yaw_rad = deg_to_rad(dock_yaw_deg)
    return (float(stage_x) + approach * math.cos(yaw_rad),
            float(stage_y) + approach * math.sin(yaw_rad),
            (float(dock_yaw_deg) + 180.0) % 360.0)


def reverse_line_command(x, y, yaw, tx, ty, line_yaw, v_max, k_lat=2.5, max_tilt=math.radians(10.0)):
    """
    Control law for reversing onto the pallet axis: the line through (tx, ty) along line_yaw.

    Reversing gives d(e_lat)/dt = v * sin(e_yaw) with v < 0, so a small tilt of
    atan(k_lat * e_lat) toward the offset side brings drive_center back onto the axis;
    the tilt itself vanishes as e_lat -> 0, leaving the robot straight.

    :return: (v, w, along, e_lat, e_yaw); along > 0 while the target is still behind the robot,
             e_lat > 0 when drive_center is left of the axis (seen facing line_yaw).
    """
    ux, uy = math.cos(line_yaw), math.sin(line_yaw)
    rx, ry = x - tx, y - ty
    along = rx * ux + ry * uy
    e_lat = -rx * uy + ry * ux
    e_yaw = normalize_angle(yaw - line_yaw)
    yaw_des = max(-max_tilt, min(max_tilt, math.atan(k_lat * e_lat)))
    # Minimum creep of 0.07 m/s breaks Unity wheel stiction cleanly
    v = -min(v_max, max(0.07, 0.70 * abs(along)))
    # Tight clamp so the robot cannot swing the forks sideways
    w = max(-0.18, min(0.18, 1.6 * (yaw_des - e_yaw)))
    return v, w, along, e_lat, e_yaw


class PalletDockingController:
    """
    Precision Pallet Docking Controller.
    Designed for fork insertion into pallets where chassis rotation must be strictly prevented.
    """

    def __init__(self, default_dock_speed=0.08, default_tolerance=0.03):
        self.default_dock_speed = float(rospy.get_param("~dock_speed", default_dock_speed))
        self.default_tolerance = float(rospy.get_param("~dock_pos_tolerance", default_tolerance))
        self.receipt_timeout = float(rospy.get_param("~receipt_timeout", 1.5))
        self.clock_skew_tolerance = float(rospy.get_param("~clock_skew_tolerance", 1.0))

        self.lock = threading.Lock()
        self.sample = None

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.sub = rospy.Subscriber("/odom", Odometry, self.on_odom, queue_size=1)
        rospy.on_shutdown(self.stop)

    def on_odom(self, msg):
        if msg.header.frame_id != "odom" or msg.child_frame_id not in ("base_footprint", "base_link", "drive_center"):
            return

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        values = [p.x, p.y, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in values):
            return

        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1e-6:
            return

        x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

        with self.lock:
            self.sample = (p.x, p.y, yaw, time.monotonic(), msg.header.stamp.to_sec())

    def stop(self):
        cmd = Twist()
        self.pub.publish(cmd)

    def dock_reverse(self, target_x, target_y, dock_yaw=None, speed=None, pos_tolerance=None, label="Pallet Insertion", fork_offset=0.0,
                     entry_lat_tol=None, entry_along=0.50, fork_tip=0.54, retries=2):
        """
        Reverse into the pallet pocket towards (target_x, target_y), steering drive_center onto
        the pallet axis (dock_yaw through the target) and forbidding in-place spinning.

        :param target_x: Goal X (drive_center) inside pallet cavity (meters)
        :param target_y: Goal Y (drive_center) inside pallet cavity (meters)
        :param dock_yaw: Pallet axis orientation in degrees (e.g. 270.0). If None, uses initial heading.
        :param speed: Max reverse speed in m/s (default: 0.08 m/s = 8 cm/s)
        :param pos_tolerance: Insertion depth tolerance in meters (default: 0.03 m = 3 cm)
        :param label: Logging descriptor
        :param fork_offset: Lever arm offset in meters from drive_center to fork center along rear -X axis (default: 0.0)
        :param entry_lat_tol: If set, lateral limit (m) for drive_center and fork tips when the tips reach the
                              pocket, and for the final pose; a miss pulls forward and retries. None disables.
        :param entry_along: Remaining travel (m) at which the fork tips reach the pocket face.
                            Milestone_1 pallet: 0.54 tip + 0.175 half depth - 0.22 fork_offset ~= 0.50.
        :param fork_tip: Distance (m) from drive_center back to the fork tips.
        :param retries: Pull-out-and-retry attempts after a failed entry check.
        """
        tx = float(target_x)
        ty = float(target_y)
        if fork_offset > 0.0 and dock_yaw is not None:
            raw_tx, raw_ty = tx, ty
            tx, ty = compute_dock_target_pose(raw_tx, raw_ty, dock_yaw, fork_offset)
            rospy.loginfo("[Pallet Docking] Applied Fork Offset: %.2fm -> drive_center target adjusted from (%.2f, %.2f) to (%.2f, %.2f)",
                          fork_offset, raw_tx, raw_ty, tx, ty)

        v_max = float(speed) if speed is not None else self.default_dock_speed
        tol = float(pos_tolerance) if pos_tolerance is not None else self.default_tolerance
        locked_yaw_rad = deg_to_rad(dock_yaw) if dock_yaw is not None else None

        rate = rospy.Rate(20)
        rospy.loginfo("==================================================")
        rospy.loginfo(">>> STARTING PALLET REVERSE DOCKING: %s", label)
        rospy.loginfo("Target drive_center: (%.2f, %.2f) | Locked Yaw: %s | Max Creep Speed: -%.2fm/s",
                      tx, ty, f"{dock_yaw:.1f} deg" if dock_yaw is not None else "Current", v_max)
        rospy.loginfo("==================================================")

        # Wait for odom
        while not rospy.is_shutdown():
            with self.lock:
                sample = self.sample
            if sample is not None:
                break
            self.stop()
            rate.sleep()

        if sample is None:
            return False

        # If no explicit dock yaw was provided, lock to current orientation
        if locked_yaw_rad is None:
            locked_yaw_rad = sample[2]

        start_x, start_y = sample[0], sample[1]
        entry_checked = False
        attempts = 0

        while not rospy.is_shutdown():
            with self.lock:
                sample = self.sample

            if sample is None:
                self.stop()
                rate.sleep()
                continue

            x, y, yaw, received, stamp = sample
            receipt_gap = time.monotonic() - received
            now_sec = rospy.Time.now().to_sec()
            age = now_sec - stamp

            if receipt_gap > self.receipt_timeout or abs(age) > self.clock_skew_tolerance:
                self.stop()
                rospy.logerr("[Docking] Odometry timeout. Stopped.")
                return False

            dist_to_dock = math.hypot(tx - x, ty - y)
            v_cmd, w_cmd, along, e_lat, e_yaw = reverse_line_command(x, y, yaw, tx, ty, locked_yaw_rad, v_max)

            # Arrived within tolerance, or reached the target depth along the pallet axis
            if dist_to_dock <= tol or along <= 0.0:
                for _ in range(6):
                    self.stop()
                    time.sleep(0.04)
                ok = entry_lat_tol is None or abs(e_lat) <= entry_lat_tol
                log = rospy.loginfo if ok else rospy.logerr
                log(">>> PALLET DOCK %s: %s at (%.3f, %.3f), Yaw: %.2f deg (Dist: %.3fm | Lateral: %+.1fmm | YawErr: %+.2f deg)",
                    "COMPLETE" if ok else "FAILED (lateral offset)", label, x, y, math.degrees(yaw),
                    dist_to_dock, e_lat * 1000.0, math.degrees(e_yaw))
                return ok

            # Entry gate: fork tips are about to reach the pallet pocket
            if entry_lat_tol is not None and not entry_checked and along <= entry_along:
                tip_lat = e_lat - fork_tip * math.sin(e_yaw)
                # Both fork ends inside the tolerance => the whole tine is (yaw is implied)
                if max(abs(e_lat), abs(tip_lat)) <= entry_lat_tol:
                    entry_checked = True
                    rospy.loginfo("[%s] Entry check passed: Lateral %+.1fmm | Tips %+.1fmm | YawErr %+.2f deg",
                                  label, e_lat * 1000.0, tip_lat * 1000.0, math.degrees(e_yaw))
                else:
                    self.stop()
                    rospy.logwarn("[%s] Entry check failed: Lateral %+.1fmm | Tips %+.1fmm | YawErr %+.2f deg (attempt %d/%d)",
                                  label, e_lat * 1000.0, tip_lat * 1000.0, math.degrees(e_yaw), attempts + 1, retries + 1)
                    if attempts >= retries:
                        return False
                    attempts += 1
                    # Pull forward to where this approach started, then reverse again
                    if not self.undock(start_x, start_y, dock_yaw=math.degrees(locked_yaw_rad), label=label + " Retry Pull-out"):
                        return False
                    continue

            cmd = Twist()
            cmd.linear.x = v_cmd
            cmd.angular.z = w_cmd
            self.pub.publish(cmd)

            rospy.loginfo_throttle(1, "[%s] Remaining: %.3fm | Lateral: %+.1fmm | YawErr: %+.1f deg | V: %.2fm/s",
                                   label, along, e_lat * 1000.0, math.degrees(e_yaw), v_cmd)

            rate.sleep()

        return False

    def undock(self, target_x, target_y, dock_yaw=None, speed=0.10, pos_tolerance=0.04, label="Pallet Extraction"):
        """
        Pull straight forward out of the pallet cavity to clear forks.
        Locks heading and forbids turning until forks are completely clear.
        """
        tx = float(target_x)
        ty = float(target_y)
        v_max = float(speed)
        tol = float(pos_tolerance)
        locked_yaw_rad = deg_to_rad(dock_yaw) if dock_yaw is not None else None

        rate = rospy.Rate(20)
        rospy.loginfo(">>> EXTRACTING FORKS (UNDOCK): %s to (%.2f, %.2f)", label, tx, ty)

        while not rospy.is_shutdown():
            with self.lock:
                sample = self.sample

            if sample is None:
                self.stop()
                rate.sleep()
                continue

            x, y, yaw, received, stamp = sample
            if locked_yaw_rad is None:
                locked_yaw_rad = yaw

            dx = tx - x
            dy = ty - y
            dist = math.hypot(dx, dy)
            yaw_err = normalize_angle(locked_yaw_rad - yaw)
            d_long = dx * math.cos(yaw) + dy * math.sin(yaw)

            if dist <= tol or d_long <= 0.0:
                for _ in range(6):
                    self.stop()
                    time.sleep(0.04)
                rospy.loginfo(">>> UNDOCK COMPLETE: %s clear of pallet.", label)
                return True

            # Controlled forward velocity (minimum 0.07 m/s breaks Unity wheel stiction cleanly)
            v_cmd = min(v_max, max(0.07, 0.70 * d_long))
            w_cmd = max(-0.18, min(0.18, 1.6 * yaw_err))

            cmd = Twist()
            cmd.linear.x = v_cmd
            cmd.angular.z = w_cmd
            self.pub.publish(cmd)

            rospy.loginfo_throttle(1, "[%s] Clearing Dist: %.3fm | V: %.2fm/s", label, dist, v_cmd)
            rate.sleep()

        return False
