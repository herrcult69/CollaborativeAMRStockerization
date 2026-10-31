#!/usr/bin/env python3
"""Gates move_base velocity: slows/stops for obstacles ahead and holds fully still while yielding to a head-on
moving obstacle; for an obstacle crossing from the side it predicts continue / pass / stop and picks the safe one.
/cmd_vel_nav (move_base) -> /cmd_vel. Publishes only while move_base is
actively commanding (plus one zero Twist when it goes idle), so docking scripts that write /cmd_vel are unaffected.
Decision logic lives in amr_navigation.gate_logic; state is published on /velocity_gate/state."""
import math
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from amr_navigation.gate_logic import DEFAULTS, FreeSpaceDetector, GateLogic


class VelocityGate:
    def __init__(self):
        self.laser_x = rospy.get_param("~laser_x", DEFAULTS["laser_x"])      # laser ahead of drive_center
        params = {k: rospy.get_param("~" + k, v) for k, v in DEFAULTS.items()}
        params["half_w"] = rospy.get_param("~half_width", 0.23) + rospy.get_param("~side_margin", 0.07)
        self.logic = GateLogic(**params)
        # Moving-obstacle detection compares scans in the odom frame; without TF it falls back to distance only.
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.detector = FreeSpaceDetector(ref_age=rospy.get_param("~moving_ref_age", 0.4),
                                          tol=rospy.get_param("~moving_tol", 0.12))
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.cmd_timeout = rospy.get_param("~cmd_timeout", 0.3)
        self.rate = 20.0
        self.cmd, self.cmd_t = None, 0.0
        self.last_state = None
        self.last_action = None
        self.moving_out = False     # last published Twist was non-zero
        self.lock = threading.Lock()   # scan callback and timer run in different threads; GateLogic is not thread-safe

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.state_pub = rospy.Publisher("/velocity_gate/state", String, queue_size=1, latch=True)
        rospy.Subscriber("/scan", LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber("/cmd_vel_nav", Twist, self.on_cmd, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / self.rate), self.on_timer)

    def laser_pose(self, msg):
        try:
            tr = self.tf_buffer.lookup_transform(self.odom_frame, msg.header.frame_id, msg.header.stamp,
                                                 rospy.Duration(0.05)).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, "[gate] no %s pose, moving-obstacle yield off: %s", self.odom_frame, e)
            return None
        q = tr.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return tr.translation.x, tr.translation.y, yaw

    def on_scan(self, msg):
        now = rospy.Time.now().to_sec()
        pose = self.laser_pose(msg)
        flags, base = None, None
        if pose is not None:
            flags = self.detector.update(msg.ranges, msg.angle_min, msg.angle_increment,
                                         msg.range_min, msg.range_max, pose, now)
            x, y, yaw = pose
            base = (x - self.laser_x * math.cos(yaw), y - self.laser_x * math.sin(yaw), yaw)
        pts, moving, a = [], [], msg.angle_min
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max:   # skips 0, nan, inf, max-range
                pts.append((self.laser_x + r * math.cos(a), r * math.sin(a)))
                moving.append(flags is not None and flags[i])
            a += msg.angle_increment
        with self.lock:
            self.logic.update_scan(pts, now, moving if flags is not None else None, base)

    def on_cmd(self, msg):
        self.cmd, self.cmd_t = msg, rospy.Time.now().to_sec()

    def publish(self, out):
        self.pub.publish(out)
        self.moving_out = out.linear.x != 0.0 or out.angular.z != 0.0

    def on_timer(self, _):
        # rospy's Timer thread dies on an uncaught exception and /cmd_vel would freeze on its last value.
        try:
            self.tick()
        except Exception as e:
            rospy.logerr_throttle(1.0, "[gate] error, publishing zero: %s", e)
            self.publish(Twist())

    def tick(self):
        now = rospy.Time.now().to_sec()
        if self.cmd is None or now - self.cmd_t > self.cmd_timeout:
            if self.moving_out:                      # move_base went idle: make sure the robot is not left moving
                self.publish(Twist())
            return                                   # then stay silent for the docking scripts
        out = Twist()
        with self.lock:
            out.linear.x, out.angular.z = self.logic.step(self.cmd.linear.x, self.cmd.angular.z, now)
        if self.logic.state != self.last_state:
            why = " [%s]" % self.logic.reason if self.logic.state == "hold" else ""
            rospy.loginfo("[gate] %s -> %s%s (gap %.2fm, head-on %.2fm, tracks %d)", self.last_state,
                          self.logic.state, why, self.logic.gap, self.logic.dyn_gap, len(self.logic.tracks))
            self.state_pub.publish(String(self.logic.state))
            self.last_state = self.logic.state
        if self.logic.action != self.last_action:
            if self.logic.action:
                rospy.loginfo("[gate] side obstacle: %s at %.2f m/s (TEB asked %.2f)%s", self.logic.action,
                              out.linear.x, self.cmd.linear.x,
                              " - stopping would leave us in its path" if self.logic.action == "escape" else "")
            else:
                why = self.logic.pass_note or ("TEB stopped" if self.cmd.linear.x <= 0.0 else
                                               "holding" if self.logic.state == "hold" else "no longer needed")
                rospy.loginfo("[gate] side obstacle: pass done (%s)", why)
            self.last_action = self.logic.action
        elif self.logic.scale < 0.99 and out.linear.x > 0 and not self.logic.action:
            rospy.loginfo_throttle(1.0, "[gate] gap %.2fm -> scale %.2f", self.logic.gap, self.logic.scale)
        self.publish(out)


if __name__ == "__main__":
    rospy.init_node("velocity_gate")
    VelocityGate()
    rospy.spin()
