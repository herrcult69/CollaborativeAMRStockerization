#!/usr/bin/env python3
"""Gates move_base velocity: Dual-Shield Linear Heading Guard Architecture.
Protective stop at close range, proportional linear deceleration in warning zone,
and linear heading lock (restricting TEB swerving into racks).
Subscribes: /cmd_vel_nav (from move_base), /scan (LaserScan)
Publishes: /cmd_vel (to robot driver), /velocity_gate/state (diagnostic state),
/velocity_gate/blocker (map-frame cloud closing a given-up passage in the global costmap)
"""
import math
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray

from amr_navigation.gate_logic import DEFAULTS, GateLogic, blocker_points, path_heading_error


class VelocityGate:
    def __init__(self):
        self.laser_x = rospy.get_param("~laser_x", DEFAULTS["laser_x"])
        self.base_frame = rospy.get_param("~base_frame", "drive_center")
        params = {k: rospy.get_param("~" + k, v) for k, v in DEFAULTS.items()}
        params["half_w"] = rospy.get_param("~half_width", 0.23) + rospy.get_param("~side_margin", 0.10)
        self.logic = GateLogic(**params)

        self.cmd_timeout = rospy.get_param("~cmd_timeout", 0.5)
        self.rate = 20.0
        self.cmd, self.cmd_t = None, 0.0
        self.last_state = None
        self.moving_out = False
        self.lock = threading.Lock()
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.blocker_cloud = None   # stored map-frame points
        self.blocker_until = 0.0
        self.plan_xy, self.plan_t = [], 0.0   # latest global plan (map frame) for the GIVEUP rotation shim
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.state_pub = rospy.Publisher("/velocity_gate/state", String, queue_size=1, latch=True)
        self.shield_pub = rospy.Publisher("/velocity_gate/shields", MarkerArray, queue_size=1)
        self.blocker_pub = rospy.Publisher("/velocity_gate/blocker", PointCloud2, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber(rospy.get_param("~plan_topic", "/move_base/NavfnROS/plan"), Path, self.on_plan, queue_size=1)
        rospy.Subscriber("/cmd_vel_nav", Twist, self.on_cmd, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / self.rate), self.on_timer)
        rospy.Timer(rospy.Duration(0.1), self.publish_shields)
        rospy.Timer(rospy.Duration(1.0), self.on_blocker_timer)

    def on_scan(self, msg):
        now = rospy.Time.now().to_sec()
        pts, a = [], msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                pts.append((self.laser_x + r * math.cos(a), r * math.sin(a)))
            a += msg.angle_increment
        with self.lock:
            self.logic.update_scan(pts, now)

    def on_plan(self, msg):
        self.plan_xy = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        self.plan_t = rospy.Time.now().to_sec()

    def heading_error(self, now):
        """Bearing error to the new global path, or None if the plan is stale/missing or tf fails."""
        if not self.plan_xy or now - self.plan_t > self.logic.p["shim_plan_age"]:
            return None
        try:
            tf = self.tf_buf.lookup_transform(self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.05))
        except Exception:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return path_heading_error((t.x, t.y, yaw), self.plan_xy, self.logic.p["shim_lookahead"])

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
        herr = self.heading_error(now) if self.logic.state == "giveup" else None
        with self.lock:
            out.linear.x, out.angular.z = self.logic.step(self.cmd.linear.x, self.cmd.angular.z, now, herr)

        if self.logic.state != self.last_state:
            why = " [%s]" % self.logic.reason if self.logic.state == "hold" else ""
            info = " | %s" % self.logic.trigger_info if self.logic.trigger_info else ""
            rospy.loginfo("[gate] %s -> %s%s%s (fwd_gap: %.2fm)",
                          self.last_state, self.logic.state, why, info, self.logic.gap)
            self.state_pub.publish(String(self.logic.state))
            if self.logic.state == "giveup":
                self.set_blocker(now)
            self.last_state = self.logic.state
        else:
            # Throttled 1.0s live telemetry so console shows real-time state and velocities
            flank_str = "(%.2fm, %.2fm)" % self.logic.flank_obs if self.logic.flank_obs else "clear"
            rospy.loginfo_throttle(1.0, "[gate] gap: %.2fm | state: %s | scale: %.2f | v_out: %.2fm/s, w_out: %.2frad/s | flank: %s",
                                   self.logic.gap, self.logic.state, self.logic.scale, out.linear.x, out.angular.z, flank_str)

        self.publish(out)

    def set_blocker(self, now):
        """On GIVEUP: close the blocked passage in the global costmap so NavFn re-routes."""
        try:
            tf = self.tf_buf.lookup_transform(self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.2))
        except Exception as e:
            rospy.logwarn("[gate] blocker skipped, tf failed: %s", e)
            return
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        pts = [(t.x + c * x - s * y, t.y + s * x + c * y, 0.1)
               for x, y in blocker_points(self.logic.gap, self.logic.p)]
        self.blocker_cloud = pts
        self.blocker_until = now + self.logic.p["blocker_ttl"]
        mid = pts[len(pts) // 2]
        rospy.loginfo("[gate] blocker set at (%.2f, %.2f) for %.0f s", mid[0], mid[1], self.logic.p["blocker_ttl"])

    def on_blocker_timer(self, _):
        if self.blocker_cloud is None:
            return
        now = rospy.Time.now().to_sec()
        if now < self.blocker_until:
            hdr = Header(stamp=rospy.Time.now(), frame_id=self.map_frame)
            self.blocker_pub.publish(point_cloud2.create_cloud_xyz32(hdr, self.blocker_cloud))
            return
        if self.logic.state == "hold":
            return  # clear only while not in HOLD (costmap reset is a short blind moment)
        self.blocker_cloud = None
        try:
            rospy.wait_for_service("/move_base/clear_costmaps", timeout=0.5)
            rospy.ServiceProxy("/move_base/clear_costmaps", Empty)()
            rospy.loginfo("[gate] blocker expired, costmaps cleared")
        except Exception as e:
            rospy.logwarn("[gate] clear_costmaps failed: %s", e)

    def publish_shields(self, _):
        if self.shield_pub.get_num_connections() == 0:
            return  # Zero overhead when RViz is closed

        p = self.logic.p
        nose_x = p["nose_x"]
        d_inner = p.get("d_inner", p.get("d_stop", 0.35))
        d_outer = p.get("d_outer", p.get("d_slow", 1.20))
        half_w = p["half_w"]
        flank_x_min = p.get("flank_x_min", -0.20)
        flank_w = p.get("flank_w", 0.35)

        with self.lock:
            state = self.logic.state
            scale = self.logic.scale
            gap = self.logic.gap
            reason = self.logic.reason
            flank_breach = self.logic.flank_breach
            crossing_obs = self.logic.crossing_obs

        stamp = rospy.Time.now()

        # 1. Inner Protective Shield (Stop Zone)
        m_inner = Marker()
        m_inner.header.frame_id = self.base_frame
        m_inner.header.stamp = stamp
        m_inner.ns = "shields"
        m_inner.id = 1
        m_inner.type = Marker.CUBE
        m_inner.action = Marker.ADD
        m_inner.pose.position.x = nose_x + d_inner / 2.0
        m_inner.pose.position.y = 0.0
        m_inner.pose.position.z = 0.10
        m_inner.pose.orientation.w = 1.0
        m_inner.scale.x = d_inner
        m_inner.scale.y = 2.0 * half_w
        m_inner.scale.z = 0.20
        if gap <= d_inner or (state == "hold" and reason == "blocked"):
            m_inner.color.r, m_inner.color.g, m_inner.color.b, m_inner.color.a = 1.0, 0.0, 0.0, 0.70
        elif state == "hold" and reason == "crossing":
            m_inner.color.r, m_inner.color.g, m_inner.color.b, m_inner.color.a = 1.0, 0.0, 1.0, 0.60
        else:
            m_inner.color.r, m_inner.color.g, m_inner.color.b, m_inner.color.a = 0.9, 0.15, 0.15, 0.20

        # 2. Outer Warning Shield (Deceleration Corridor)
        m_outer = Marker()
        m_outer.header.frame_id = self.base_frame
        m_outer.header.stamp = stamp
        m_outer.ns = "shields"
        m_outer.id = 2
        m_outer.type = Marker.CUBE
        m_outer.action = Marker.ADD
        len_outer = max(0.05, d_outer - d_inner)
        m_outer.pose.position.x = nose_x + d_inner + len_outer / 2.0
        m_outer.pose.position.y = 0.0
        m_outer.pose.position.z = 0.07
        m_outer.pose.orientation.w = 1.0
        m_outer.scale.x = len_outer
        m_outer.scale.y = 2.0 * half_w
        m_outer.scale.z = 0.14
        if state == "free" and scale < 0.99:
            m_outer.color.r, m_outer.color.g, m_outer.color.b, m_outer.color.a = 1.0, 0.60, 0.0, 0.60
        elif state == "hold":
            m_outer.color.r, m_outer.color.g, m_outer.color.b, m_outer.color.a = 0.75, 0.50, 0.0, 0.15
        else:
            m_outer.color.r, m_outer.color.g, m_outer.color.b, m_outer.color.a = 0.95, 0.85, 0.10, 0.18

        # 3. Flank Left Shield
        len_flank = nose_x - flank_x_min
        mid_x = (nose_x + flank_x_min) / 2.0
        w_flank = flank_w - 0.23
        mid_y = (flank_w + 0.23) / 2.0

        m_flank_l = Marker()
        m_flank_l.header.frame_id = self.base_frame
        m_flank_l.header.stamp = stamp
        m_flank_l.ns = "shields"
        m_flank_l.id = 3
        m_flank_l.type = Marker.CUBE
        m_flank_l.action = Marker.ADD
        m_flank_l.pose.position.x = mid_x
        m_flank_l.pose.position.y = mid_y
        m_flank_l.pose.position.z = 0.10
        m_flank_l.pose.orientation.w = 1.0
        m_flank_l.scale.x = len_flank
        m_flank_l.scale.y = w_flank
        m_flank_l.scale.z = 0.20

        # 4. Flank Right Shield
        m_flank_r = Marker()
        m_flank_r.header.frame_id = self.base_frame
        m_flank_r.header.stamp = stamp
        m_flank_r.ns = "shields"
        m_flank_r.id = 4
        m_flank_r.type = Marker.CUBE
        m_flank_r.action = Marker.ADD
        m_flank_r.pose.position.x = mid_x
        m_flank_r.pose.position.y = -mid_y
        m_flank_r.pose.position.z = 0.10
        m_flank_r.pose.orientation.w = 1.0
        m_flank_r.scale.x = len_flank
        m_flank_r.scale.y = w_flank
        m_flank_r.scale.z = 0.20

        if flank_breach:
            for m in (m_flank_l, m_flank_r):
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.8, 0.80
        else:
            for m in (m_flank_l, m_flank_r):
                m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.5, 0.9, 0.16

        # 5. Floating Status Billboard
        m_text = Marker()
        m_text.header.frame_id = self.base_frame
        m_text.header.stamp = stamp
        m_text.ns = "shields"
        m_text.id = 0
        m_text.type = Marker.TEXT_VIEW_FACING
        m_text.action = Marker.ADD
        m_text.pose.position.x = 0.10
        m_text.pose.position.y = 0.0
        m_text.pose.position.z = 0.70
        m_text.pose.orientation.w = 1.0
        m_text.scale.z = 0.14

        gap_str = "%.2fm" % gap if gap < 99.0 else "clear"
        if state == "hold":
            if reason == "crossing":
                m_text.text = "SHIELD: YIELD (crossing) | gap: %s" % gap_str
            else:
                m_text.text = "SHIELD: HOLD [%s] | gap: %s" % (reason or "blocked", gap_str)
            m_text.color.r, m_text.color.g, m_text.color.b, m_text.color.a = 1.0, 0.2, 0.2, 1.0
        elif scale < 0.99 and state == "free":
            m_text.text = "SHIELD: SLOW (%.0f%%) | gap: %s" % (scale * 100, gap_str)
            m_text.color.r, m_text.color.g, m_text.color.b, m_text.color.a = 1.0, 0.7, 0.0, 1.0
        elif state == "giveup":
            m_text.text = "SHIELD: GIVEUP / REROUTE"
            m_text.color.r, m_text.color.g, m_text.color.b, m_text.color.a = 0.9, 0.4, 1.0, 1.0
        else:
            m_text.text = "SHIELD: CLEAR | gap: %s" % gap_str
            m_text.color.r, m_text.color.g, m_text.color.b, m_text.color.a = 0.2, 1.0, 0.3, 0.90

        # 6. Suspected crossing object
        m_cross = Marker()
        m_cross.header.frame_id = self.base_frame
        m_cross.header.stamp = stamp
        m_cross.ns = "shields"
        m_cross.id = 6
        if crossing_obs is not None and state == "hold" and reason == "crossing":
            m_cross.type = Marker.SPHERE
            m_cross.action = Marker.ADD
            m_cross.pose.position.x = crossing_obs[0]
            m_cross.pose.position.y = crossing_obs[1]
            m_cross.pose.position.z = 0.15
            m_cross.pose.orientation.w = 1.0
            m_cross.scale.x = m_cross.scale.y = m_cross.scale.z = 0.20
            m_cross.color.r, m_cross.color.g, m_cross.color.b, m_cross.color.a = 1.0, 0.0, 1.0, 0.90
        else:
            m_cross.action = Marker.DELETE

        arr = MarkerArray()
        arr.markers = [m_inner, m_outer, m_flank_l, m_flank_r, m_text, m_cross]
        self.shield_pub.publish(arr)


if __name__ == "__main__":
    rospy.init_node("velocity_gate")
    VelocityGate()
    rospy.spin()
