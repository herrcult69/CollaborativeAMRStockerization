#!/usr/bin/env python3
"""Gates move_base velocity (Option B: Industrial Safety Shield + Closing-Rate Yield).
Protective stop at close range, proportional deceleration in warning zone, and early HOLD for oncoming traffic.
Subscribes: /cmd_vel_nav (from move_base), /scan (LaserScan)
Publishes: /cmd_vel (to robot driver), /velocity_gate/state (diagnostic state)
"""
import math
import threading

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from amr_navigation.gate_logic import DEFAULTS, GateLogic


class VelocityGate:
    def __init__(self):
        self.laser_x = rospy.get_param("~laser_x", DEFAULTS["laser_x"])
        params = {k: rospy.get_param("~" + k, v) for k, v in DEFAULTS.items()}
        params["half_w"] = rospy.get_param("~half_width", 0.23) + rospy.get_param("~side_margin", 0.10)
        self.logic = GateLogic(**params)

        self.cmd_timeout = rospy.get_param("~cmd_timeout", 0.3)
        self.rate = 20.0
        self.cmd, self.cmd_t = None, 0.0
        self.last_state = None
        self.moving_out = False
        self.lock = threading.Lock()

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.state_pub = rospy.Publisher("/velocity_gate/state", String, queue_size=1, latch=True)
        rospy.Subscriber("/scan", LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber("/cmd_vel_nav", Twist, self.on_cmd, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / self.rate), self.on_timer)

    def on_scan(self, msg):
        now = rospy.Time.now().to_sec()
        pts, a = [], msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                pts.append((self.laser_x + r * math.cos(a), r * math.sin(a)))
            a += msg.angle_increment
        with self.lock:
            self.logic.update_scan(pts, now)

    def on_cmd(self, msg):
        self.cmd, self.cmd_t = msg, rospy.Time.now().to_sec()

    def publish(self, out):
        self.pub.publish(out)
        self.moving_out = out.linear.x != 0.0 or out.angular.z != 0.0

    def on_timer(self, _):
        try:
            self.tick()
        except Exception as e:
            rospy.logerr_throttle(1.0, "[gate] error, publishing zero: %s", e)
            self.publish(Twist())

    def tick(self):
        now = rospy.Time.now().to_sec()
        if self.cmd is None or now - self.cmd_t > self.cmd_timeout:
            if self.moving_out:
                self.publish(Twist())
            return

        out = Twist()
        with self.lock:
            out.linear.x, out.angular.z = self.logic.step(self.cmd.linear.x, self.cmd.angular.z, now)

        if self.logic.state != self.last_state:
            why = " [%s]" % self.logic.reason if self.logic.state == "hold" else ""
            rospy.loginfo("[gate] %s -> %s%s (gap %.2fm, closing_rate %.2fm/s)",
                          self.last_state, self.logic.state, why, self.logic.gap, self.logic.closing_rate)
            self.state_pub.publish(String(self.logic.state))
            self.last_state = self.logic.state
        elif self.logic.scale < 0.99 and out.linear.x > 0:
            rospy.loginfo_throttle(1.0, "[gate] gap %.2fm -> scale %.2f (closing_rate %.2fm/s)",
                                   self.logic.gap, self.logic.scale, self.logic.closing_rate)

        self.publish(out)


if __name__ == "__main__":
    rospy.init_node("velocity_gate")
    VelocityGate()
    rospy.spin()
