"""
3-Phase AMR Precision Docking & Waypoint Navigation Module.

Phases:
  1. ALIGN_TO_GOAL: Rotate in place to face target position (gx, gy).
  2. DRIVE_TO_GOAL: Drive forward with smooth deceleration and heading correction.
  3. ALIGN_FINAL_YAW: Rotate in place to match required docking orientation (goal_yaw).
  4. ARRIVED: Lock zero velocity and report completion.
"""
import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def deg_to_rad(deg):
    """Convert degrees to normalized radians in [-pi, pi]."""
    if deg is None:
        return None
    return normalize_angle(math.radians(float(deg)))


class ThreePhaseDockingController:
    STATE_ALIGN_TO_GOAL = "1_ALIGN_TO_GOAL"
    STATE_DRIVE_TO_GOAL = "2_DRIVE_TO_GOAL"
    STATE_ALIGN_FINAL_YAW = "3_ALIGN_FINAL_YAW"
    STATE_ARRIVED = "4_ARRIVED"

    def __init__(self, max_linear=None, max_angular=None):
        # Target coordinates and orientation
        self.gx = float(rospy.get_param("~goal_x", 2.0))
        self.gy = float(rospy.get_param("~goal_y", 0.0))

        if rospy.has_param("~goal_yaw_rad"):
            self.goal_yaw = normalize_angle(float(rospy.get_param("~goal_yaw_rad")))
        else:
            yaw_deg = float(rospy.get_param("~goal_yaw", rospy.get_param("~goal_yaw_deg", 0.0)))
            self.goal_yaw = deg_to_rad(yaw_deg)

        # Tolerances
        self.pos_tolerance = float(rospy.get_param("~pos_tolerance", 0.05))     # 5 cm precision
        yaw_tol_deg = float(rospy.get_param("~yaw_tolerance", rospy.get_param("~yaw_tolerance_deg", 4.0)))
        self.yaw_tolerance = math.radians(yaw_tol_deg)  # 4.0 degrees default (clean fork docking)

        # Kinematic and speed limits (Gentle warehouse speeds, tuned for Unity PhysX friction)
        self.heading_align_threshold = math.radians(12.0)  # ~12 deg to start driving
        default_max_linear = max_linear if max_linear is not None else 0.35
        default_max_angular = max_angular if max_angular is not None else 0.65
        self.max_linear = float(rospy.get_param("~max_linear", default_max_linear))
        self.min_linear = float(rospy.get_param("~min_linear", 0.06))
        self.max_angular = float(rospy.get_param("~max_angular", default_max_angular))
        self.min_angular = float(rospy.get_param("~min_angular", 0.42))  # Overcomes Unity tire scrub stiction
        self.wheel_offset_x = float(rospy.get_param("~wheel_offset_x", 0.0))
        self.receipt_timeout = float(rospy.get_param("~receipt_timeout", 1.5))
        self.clock_skew_tolerance = float(rospy.get_param("~clock_skew_tolerance", 1.0))

        self.state = self.STATE_ALIGN_TO_GOAL
        self.lock = threading.Lock()
        self.sample = None

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.sub = rospy.Subscriber("/odom", Odometry, self.on_odom, queue_size=1)
        rospy.on_shutdown(self.stop)

    def set_goal(self, gx, gy, goal_yaw=None, pos_tolerance=None):
        """Configure target for navigation."""
        self.gx = float(gx)
        self.gy = float(gy)
        self.goal_yaw = deg_to_rad(goal_yaw) if goal_yaw is not None else None
        if pos_tolerance is not None:
            self.pos_tolerance = float(pos_tolerance)
        self.state = self.STATE_ALIGN_TO_GOAL

    def on_odom(self, msg):
        if msg.header.frame_id != "odom" or msg.child_frame_id not in ("base_footprint", "base_link", "drive_center"):
            rospy.logerr_throttle(2, "Expected odom -> base_footprint/base_link; ignoring message")
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
        has_final_yaw = self.goal_yaw is not None
        final_yaw_error = normalize_angle(self.goal_yaw - yaw) if has_final_yaw else 0.0

        linear = 0.0
        angular = 0.0

        # Phase 1: Turn towards goal point
        if self.state == self.STATE_ALIGN_TO_GOAL:
            if dist_to_goal <= self.pos_tolerance:
                if has_final_yaw:
                    self.state = self.STATE_ALIGN_FINAL_YAW
                else:
                    self.state = self.STATE_ARRIVED
            elif abs(heading_error) < self.heading_align_threshold:
                self.state = self.STATE_DRIVE_TO_GOAL
            else:
                sign = 1.0 if heading_error > 0 else -1.0
                angular = max(self.min_angular, min(self.max_angular, 1.6 * abs(heading_error))) * sign

        # Phase 2: Drive towards goal
        if self.state == self.STATE_DRIVE_TO_GOAL:
            if dist_to_goal <= self.pos_tolerance:
                if has_final_yaw:
                    self.state = self.STATE_ALIGN_FINAL_YAW
                else:
                    self.state = self.STATE_ARRIVED
            elif abs(heading_error) > 0.65 and dist_to_goal > 2.0 * self.pos_tolerance:
                self.state = self.STATE_ALIGN_TO_GOAL
            else:
                # Gentle deceleration ramp
                target_v = min(self.max_linear, max(self.min_linear, 0.45 * dist_to_goal))
                linear = target_v * max(0.0, math.cos(heading_error))
                angular = max(-self.max_angular, min(self.max_angular, 1.8 * heading_error))

        # Phase 3: Align final orientation
        if self.state == self.STATE_ALIGN_FINAL_YAW:
            # Closed-loop recovery: if turning on the spot causes position drift beyond tolerance
            if dist_to_goal > 1.6 * self.pos_tolerance:
                if abs(heading_error) > 0.65:
                    self.state = self.STATE_ALIGN_TO_GOAL
                else:
                    self.state = self.STATE_DRIVE_TO_GOAL
            elif abs(final_yaw_error) <= self.yaw_tolerance:
                self.state = self.STATE_ARRIVED
            else:
                sign = 1.0 if final_yaw_error > 0 else -1.0
                angular = max(self.min_angular, min(self.max_angular, 1.8 * abs(final_yaw_error))) * sign

        active_err = final_yaw_error if self.state == self.STATE_ALIGN_FINAL_YAW else heading_error
        return linear, angular, dist_to_goal, active_err, final_yaw_error

    def navigate_to(self, gx, gy, goal_yaw=None, pos_tolerance=None, label="Target"):
        """Blocking call to execute 3-phase navigation to a target."""
        self.set_goal(gx, gy, goal_yaw, pos_tolerance)
        rate = rospy.Rate(20)

        yaw_str = f"{goal_yaw:.1f} deg" if goal_yaw is not None else "Transit"
        rospy.loginfo(">>> Navigating to %s: (%.2f, %.2f) | Yaw: %s | Max Speed: %.2fm/s",
                      label, self.gx, self.gy, yaw_str, self.max_linear)

        last_yaw = None
        stall_counter = 0

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

            if receipt_gap > self.receipt_timeout or abs(age) > self.clock_skew_tolerance:
                self.stop()
                rospy.logerr("Odometry timeout (gap=%.3fs, age=%.3fs). Stopped.", receipt_gap, age)
                return False

            linear, angular, dist, active_err, final_yaw_err = self.compute_step(x, y, yaw)

            # Anti-stiction dither: detect if robot is physically stuck against floor friction
            if abs(angular) > 1e-4 and abs(linear) < 1e-4:
                if last_yaw is not None and abs(normalize_angle(yaw - last_yaw)) < 0.003:
                    stall_counter += 1
                    # If stuck for > 0.6 seconds (12 cycles at 20Hz), apply a breakaway kick
                    if stall_counter > 12:
                        kick_speed = min(self.max_angular, abs(angular) + 0.18)
                        angular = math.copysign(kick_speed, angular)
                        if stall_counter % 20 == 0:
                            rospy.loginfo_throttle(1, "[Docking] Stiction detected at yaw %.1f deg - applying breakaway kick (%.2frad/s)",
                                                   math.degrees(yaw), angular)
                else:
                    stall_counter = 0
            else:
                stall_counter = 0

            last_yaw = yaw

            cmd = Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            self.pub.publish(cmd)

            rospy.loginfo_throttle(1, "[%s | %s] Dist: %.2fm | Linear: %.2fm/s | Ang: %.2frad/s | Err: %.1f deg",
                                   label, self.state, dist, linear, angular, math.degrees(active_err))

            if self.state == self.STATE_ARRIVED:
                for _ in range(6):
                    self.stop()
                    time.sleep(0.04)

                rospy.loginfo(">>> REACHED %s at (%.3f, %.3f), Yaw: %.2f deg (Dist err: %.3fm)",
                              label, x, y, math.degrees(yaw), dist)
                return True

            rate.sleep()

        return False

    def run(self):
        """Run single-goal node as configured from ROS parameters."""
        goal_yaw_deg = math.degrees(self.goal_yaw) if self.goal_yaw is not None else None
        success = self.navigate_to(self.gx, self.gy, goal_yaw=goal_yaw_deg,
                                   pos_tolerance=self.pos_tolerance, label="Goal")
        if success:
            rospy.loginfo("==================================================")
            rospy.loginfo("SUCCESS: AMR DOCKED AT GOAL!")
            rospy.loginfo("==================================================")
