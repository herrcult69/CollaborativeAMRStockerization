#!/usr/bin/env python3
"""Lesson controller: one position goal in odom, on a clear flat floor."""
import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def command_for_goal(x, y, yaw, gx, gy, tolerance, max_linear, max_angular):
    distance = math.hypot(gx - x, gy - y)
    error = math.atan2(gy - y, gx - x) - yaw
    error = math.atan2(math.sin(error), math.cos(error))
    if distance <= tolerance:
        return 0.0, 0.0, distance
    angular = max(-max_angular, min(max_angular, 1.5 * error))
    # Turn first when facing away; slow down near the target.
    linear = min(max_linear, 0.6 * distance) if abs(error) < 0.35 else 0.0
    return linear, angular, distance


class GoToPoint:
    def __init__(self):
        self.gx = float(rospy.get_param("~goal_x", 2.0))
        self.gy = float(rospy.get_param("~goal_y", 0.0))
        self.tolerance = float(rospy.get_param("~tolerance", 0.2))
        self.max_linear = float(rospy.get_param("~max_linear", 0.2))
        self.max_angular = float(rospy.get_param("~max_angular", 0.5))
        values = [self.gx, self.gy, self.tolerance, self.max_linear, self.max_angular]
        if not all(math.isfinite(v) for v in values) or min(values[2:]) <= 0:
            raise ValueError("Goal must be finite; tolerance and speed limits must be positive")
        self.lock = threading.Lock()
        self.sample = None
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
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
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 1e-6:
            return
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        with self.lock:
            self.sample = (p.x, p.y, yaw, time.monotonic(), msg.header.stamp.to_sec())

    def stop(self):
        self.pub.publish(Twist())

    def run(self):
        rospy.loginfo("Goal in odom: (%.2f, %.2f), tolerance %.2f m", self.gx, self.gy, self.tolerance)
        # Wall-clock loop continues to stop commands even if ROS simulation time pauses.
        while not rospy.is_shutdown():
            with self.lock:
                sample = self.sample
            if sample is None:
                self.stop()
                rospy.logwarn_throttle(2, "Waiting for /odom")
            else:
                x, y, yaw, received, stamp = sample
                age = rospy.Time.now().to_sec() - stamp
                if time.monotonic() - received > 0.5 or age > 0.5 or age < -0.5:
                    self.stop()
                    rospy.logerr("Odometry timing invalid: receipt gap=%.3fs, ROS time minus stamp=%.3fs. Stopped; restart after fixing clocks.", time.monotonic() - received, age)
                    return
                linear, angular, distance = command_for_goal(
                    x, y, yaw, self.gx, self.gy, self.tolerance, self.max_linear, self.max_angular)
                cmd = Twist()
                cmd.linear.x, cmd.angular.z = linear, angular
                self.pub.publish(cmd)
                rospy.loginfo_throttle(1, "Distance %.2f m | linear %.2f m/s | angular %.2f rad/s", distance, linear, angular)
                if distance <= self.tolerance:
                    # Repeat stop briefly before exiting to allow bridge delivery.
                    for _ in range(10):
                        self.stop()
                        time.sleep(0.05)
                    rospy.loginfo("ARRIVED: within %.2f m of goal. Stop commands sent.", self.tolerance)
                    return
            time.sleep(0.05)


if __name__ == "__main__":
    rospy.init_node("go_to_point")
    controller = GoToPoint()
    try:
        controller.run()
    finally:
        controller.stop()
