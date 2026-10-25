#!/usr/bin/env python3
"""Scales move_base forward speed by distance to the nearest obstacle ahead.
/cmd_vel_nav (move_base) -> /cmd_vel. Publishes only while move_base is
actively commanding, so docking scripts that write /cmd_vel are unaffected."""
import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class VelocityGate:
    def __init__(self):
        self.laser_x = rospy.get_param("~laser_x", 0.33)      # laser ahead of drive_center
        self.nose_x = rospy.get_param("~nose_x", 0.37)        # footprint front edge
        self.half_w = rospy.get_param("~half_width", 0.23) + rospy.get_param("~side_margin", 0.07)
        self.d_stop = rospy.get_param("~d_stop", 0.3)        # full stop at this gap from the nose
        self.horizon = rospy.get_param("~horizon_s", 2.5)     # slow-zone = d_stop + v * horizon
        self.release = rospy.get_param("~release_rate", 0.8)  # max scale increase per second
        self.scan_timeout = rospy.get_param("~scan_timeout", 0.5)
        self.cmd_timeout = rospy.get_param("~cmd_timeout", 0.3)
        self.rate = 20.0

        self.gap, self.scan_t = float("inf"), None
        self.cmd, self.cmd_t = None, 0.0
        self.scale = 1.0

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber("/cmd_vel_nav", Twist, self.on_cmd, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / self.rate), self.on_timer)

    def on_scan(self, msg):
        gap, a = float("inf"), msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:   # skips 0, nan, inf, max-range
                px = self.laser_x + r * math.cos(a)
                py = r * math.sin(a)
                if px >= 0.0 and abs(py) <= self.half_w:
                    gap = min(gap, px - self.nose_x)
            a += msg.angle_increment
        self.gap, self.scan_t = gap, rospy.Time.now().to_sec()

    def on_cmd(self, msg):
        self.cmd, self.cmd_t = msg, rospy.Time.now().to_sec()

    def on_timer(self, _):
        now = rospy.Time.now().to_sec()
        if self.cmd is None or now - self.cmd_t > self.cmd_timeout:
            return                                   # move_base idle: stay silent
        out = Twist()
        out.angular.z = self.cmd.angular.z
        v = self.cmd.linear.x
        if v > 0.0:
            if self.scan_t is None or now - self.scan_t > self.scan_timeout:
                target = 0.0                         # blind: do not drive forward
            else:
                d_slow = self.d_stop + max(v, 0.05) * self.horizon
                target = max(0.0, min(1.0, (self.gap - self.d_stop) / (d_slow - self.d_stop)))
            if target < self.scale:
                self.scale = target                  # brake immediately
            else:
                self.scale = min(target, self.scale + self.release / self.rate)
            out.linear.x = v * self.scale
            if self.scale < 0.99:
                rospy.loginfo_throttle(1.0, "[gate] gap %.2fm -> scale %.2f", self.gap, self.scale)
        else:
            out.linear.x = v
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("velocity_gate")
    VelocityGate()
    rospy.spin()