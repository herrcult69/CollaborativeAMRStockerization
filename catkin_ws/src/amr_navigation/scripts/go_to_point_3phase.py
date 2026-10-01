#!/usr/bin/env python3
"""
Optimized 3-Phase AMR Docking / Point-to-Point Controller.

Phases:
  1. ALIGN_TO_GOAL: Rotate in place to face target position (gx, gy).
  2. DRIVE_TO_GOAL: Drive forward with smooth deceleration and heading correction.
  3. ALIGN_FINAL_YAW: Rotate in place to match required docking orientation (goal_yaw).
  4. ARRIVED: Lock zero velocity, report errors, and trigger pallet lift mechanism.
"""
import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def deg_to_rad(deg):
    """Convert degrees to normalized radians in [-pi, pi]."""
    return normalize_angle(math.radians(float(deg)))


class ThreePhaseDockingController:
    STATE_ALIGN_TO_GOAL = "1_ALIGN_TO_GOAL"
    STATE_DRIVE_TO_GOAL = "2_DRIVE_TO_GOAL"
    STATE_ALIGN_FINAL_YAW = "3_ALIGN_FINAL_YAW"
    STATE_ARRIVED = "4_ARRIVED"

    def __init__(self):
        # Target coordinates and orientation (Exposed in DEGREES for human clarity)
        self.gx = float(rospy.get_param("~goal_x", 2.0))
        self.gy = float(rospy.get_param("~goal_y", 0.0))

        # Accept goal_yaw in DEGREES (e.g. 0.0, 90.0, -90.0, 180.0)
        # (Also supports ~goal_yaw_rad if an advanced node passes radians directly)
        if rospy.has_param("~goal_yaw_rad"):
            self.goal_yaw = normalize_angle(float(rospy.get_param("~goal_yaw_rad")))
        else:
            yaw_deg = float(rospy.get_param("~goal_yaw", rospy.get_param("~goal_yaw_deg", 0.0)))
            self.goal_yaw = deg_to_rad(yaw_deg)

        # Tolerances
        self.pos_tolerance = float(rospy.get_param("~pos_tolerance", 0.05))     # 5 cm precision
        yaw_tol_deg = float(rospy.get_param("~yaw_tolerance", rospy.get_param("~yaw_tolerance_deg", 3.0)))
        self.yaw_tolerance = math.radians(yaw_tol_deg)  # Default: 3.0 degrees tight docking window

        # Automated pallet lift mechanism (Milestone 1)
        self.auto_lift = bool(rospy.get_param("~auto_lift", True))
        self.lift_height = float(rospy.get_param("~lift_height", 0.25))         # 0.25 m elevation

        # Internal kinematic and physics defaults
        self.heading_align_threshold = math.radians(12.0)  # ~12 deg to start driving
        self.max_linear = float(rospy.get_param("~max_linear", 0.75))
        self.min_linear = float(rospy.get_param("~min_linear", 0.06))
        self.max_angular = float(rospy.get_param("~max_angular", 0.8))
        self.min_angular = float(rospy.get_param("~min_angular", 0.35))  # Breaks 45kg chassis stiction
        self.wheel_offset_x = float(rospy.get_param("~wheel_offset_x", 0.0))
        self.receipt_timeout = float(rospy.get_param("~receipt_timeout", 1.5))
        self.clock_skew_tolerance = float(rospy.get_param("~clock_skew_tolerance", 1.0))

        self.state = self.STATE_ALIGN_TO_GOAL
        self.lock = threading.Lock()
        self.sample = None

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.lift_pub = rospy.Publisher("/lift_cmd", Float32, queue_size=1, latch=True)
        self.sub = rospy.Subscriber("/odom", Odometry, self.on_odom, queue_size=1)
        rospy.on_shutdown(self.stop)

    def on_odom(self, msg):
        if msg.header.frame_id != "odom" or msg.child_frame_id != "base_footprint":
            rospy.logerr_throttle(2, "Expected odom -> base_footprint; ignoring message")
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

        # Optional: Transform base_link position to physical drive axle center
        if abs(self.wheel_offset_x) > 1e-4:
            axle_x = p.x + self.wheel_offset_x * math.cos(yaw)
            axle_y = p.y + self.wheel_offset_x * math.sin(yaw)
        else:
            axle_x, axle_y = p.x, p.y

        with self.lock:
            self.sample = (axle_x, axle_y, yaw, time.monotonic(), msg.header.stamp.to_sec())

    def stop(self):
        cmd = Twist()
        self.pub.publish(cmd)

    def compute_step(self, x, y, yaw):
        dx = self.gx - x
        dy = self.gy - y
        dist_to_goal = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        heading_error = normalize_angle(angle_to_goal - yaw)
        final_yaw_error = normalize_angle(self.goal_yaw - yaw)

        linear = 0.0
        angular = 0.0

        # State Machine Transitions
        if self.state == self.STATE_ALIGN_TO_GOAL:
            # If already at target position, skip driving and align final yaw directly
            if dist_to_goal <= self.pos_tolerance:
                self.state = self.STATE_ALIGN_FINAL_YAW
                rospy.loginfo("[3-Phase] Already within position tolerance (dist=%.3fm <= tol=%.3fm). Transitioning -> %s",
                              dist_to_goal, self.pos_tolerance, self.STATE_ALIGN_FINAL_YAW)
            elif abs(heading_error) < self.heading_align_threshold:
                self.state = self.STATE_DRIVE_TO_GOAL
                rospy.loginfo("[3-Phase] Heading aligned (err=%.2f deg). Transitioning -> %s",
                              math.degrees(heading_error), self.STATE_DRIVE_TO_GOAL)
            else:
                # Proportional in-place rotation
                sign = 1.0 if heading_error > 0 else -1.0
                angular = max(self.min_angular, min(self.max_angular, 1.8 * abs(heading_error))) * sign

        if self.state == self.STATE_DRIVE_TO_GOAL:
            # Phase 2: Drive towards goal with simultaneous heading correction and deceleration ramp
            if dist_to_goal <= self.pos_tolerance:
                self.state = self.STATE_ALIGN_FINAL_YAW
                rospy.loginfo("[3-Phase] Arrived within position tolerance (dist=%.3fm). Transitioning -> %s",
                              dist_to_goal, self.STATE_ALIGN_FINAL_YAW)
            elif abs(heading_error) > 0.65 and dist_to_goal > 2.0 * self.pos_tolerance:
                self.state = self.STATE_ALIGN_TO_GOAL
                rospy.logwarn("[3-Phase] Large heading deviation (err=%.2f deg). Re-aligning -> %s",
                              math.degrees(heading_error), self.STATE_ALIGN_TO_GOAL)
            else:
                # Smooth deceleration ramp near target
                target_v = min(self.max_linear, max(self.min_linear, 0.55 * dist_to_goal))
                # Reduce forward speed if angle error is noticeable
                linear = target_v * max(0.0, math.cos(heading_error))
                # Active steering correction while driving
                angular = max(-self.max_angular, min(self.max_angular, 2.0 * heading_error))

        if self.state == self.STATE_ALIGN_FINAL_YAW:
            # Phase 3: Final terminal docking orientation alignment
            if abs(final_yaw_error) <= self.yaw_tolerance:
                self.state = self.STATE_ARRIVED
                rospy.loginfo("[3-Phase] Target docking orientation reached (yaw_err=%.2f deg)! -> %s",
                              math.degrees(final_yaw_error), self.STATE_ARRIVED)
            else:
                sign = 1.0 if final_yaw_error > 0 else -1.0
                angular = max(self.min_angular, min(self.max_angular, 1.5 * abs(final_yaw_error))) * sign

        active_err = final_yaw_error if self.state == self.STATE_ALIGN_FINAL_YAW else heading_error
        return linear, angular, dist_to_goal, active_err, final_yaw_error

    def run(self):
        rospy.loginfo("==================================================")
        rospy.loginfo("3-Phase AMR Docking Controller Started")
        rospy.loginfo("Goal: (%.2f, %.2f) | Goal Yaw: %.2f deg (%.3f rad)",
                      self.gx, self.gy, math.degrees(self.goal_yaw), self.goal_yaw)
        rospy.loginfo("Tolerances: Pos: %.3fm (%.1f cm) | Yaw: %.2f deg",
                      self.pos_tolerance, self.pos_tolerance * 100.0,
                      math.degrees(self.yaw_tolerance))
        rospy.loginfo("Auto Lift: %s | Height: %.2fm", self.auto_lift, self.lift_height)
        rospy.loginfo("==================================================")

        rate = rospy.Rate(20)  # 20 Hz control loop

        while not rospy.is_shutdown():
            with self.lock:
                sample = self.sample

            if sample is None:
                self.stop()
                rospy.logwarn_throttle(2, "Waiting for /odom stream...")
                rate.sleep()
                continue

            x, y, yaw, received, stamp = sample
            receipt_gap = time.monotonic() - received
            now_sec = rospy.Time.now().to_sec()
            age = now_sec - stamp

            # Robust timeout check (accommodates 1.5s gap for Unity frame drops)
            if receipt_gap > self.receipt_timeout or abs(age) > self.clock_skew_tolerance:
                self.stop()
                rospy.logerr("Odometry timeout: receipt_gap=%.3fs, stamp_age=%.3fs. Stopped.", receipt_gap, age)
                return

            linear, angular, dist, active_err, final_yaw_err = self.compute_step(x, y, yaw)

            cmd = Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            self.pub.publish(cmd)

            rospy.loginfo_throttle(1, "[%s] Dist: %.2fm | Linear: %.2fm/s | Angular: %.2frad/s | Err: %.1f deg",
                                   self.state, dist, linear, angular, math.degrees(active_err))

            if self.state == self.STATE_ARRIVED:
                # Send active zero velocity multiple times to ensure bridge delivery
                for _ in range(10):
                    self.stop()
                    time.sleep(0.04)

                # Automated lift trigger (Milestone 1)
                if self.auto_lift:
                    lift_msg = Float32(data=self.lift_height)
                    self.lift_pub.publish(lift_msg)
                    rospy.loginfo("[3-Phase] Pallet lift triggered: published %.2fm to /lift_cmd", self.lift_height)

                rospy.loginfo("==================================================")
                rospy.loginfo("SUCCESS: AMR DOCKED AT GOAL!")
                rospy.loginfo("Final Error: Dist=%.3fm (limit %.3fm) | Yaw=%.2f deg (limit %.2f deg)",
                              dist, self.pos_tolerance, math.degrees(final_yaw_err), math.degrees(self.yaw_tolerance))
                rospy.loginfo("==================================================")
                return

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("go_to_point_3phase")
    controller = ThreePhaseDockingController()
    try:
        controller.run()
    finally:
        controller.stop()
