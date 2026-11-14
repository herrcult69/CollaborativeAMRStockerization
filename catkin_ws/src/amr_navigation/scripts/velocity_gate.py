#!/usr/bin/env python3
"""Gates move_base velocity: Dual-Shield architecture with spin-first rotation.
Protective stop in a narrow inner rectangle, proportional deceleration in a half-ellipse outer shield,
crossing-obstacle yield, and in-place rotation toward the global plan when facing away from it.
Subscribes: /cmd_vel_nav (from move_base), /scan (LaserScan), /odom (measured v/w), Navfn plan (spin-first heading)
Publishes: /cmd_vel (to robot driver), /velocity_gate/state (diagnostic state),
/velocity_gate/blocker (map-frame cloud closing a given-up passage in the global costmap)
"""
import math
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray

from amr_navigation.gate_logic import DEFAULTS, GateLogic, blocker_points, path_heading_error, path_remaining


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
        self.last_spin = False
        self.moving_out = False
        self.lock = threading.Lock()
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.blocker_cloud = None   # stored map-frame points
        self.blocker_until = 0.0
        self.plan_xy, self.plan_t = [], 0.0   # latest global plan (map frame) for spin-first and the GIVEUP shim
        self.odom_vw, self.odom_t = (0.0, 0.0), 0.0   # measured twist for crossing ego-motion compensation
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        # TEB band monitor: forces TEB to rebuild its warm-started band through dynamic_reconfigure (see gate_logic.band_update)
        self.teb_ns = rospy.get_param("~teb_ns", "/move_base/TebLocalPlannerROS")
        self.teb_client, self.teb_base, self.teb_try_t = None, 1.0, -1e9
        self.reinit_on = False
        self.last_reinit_n = 0

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.state_pub = rospy.Publisher("/velocity_gate/state", String, queue_size=1, latch=True)
        self.shield_pub = rospy.Publisher("/velocity_gate/shields", MarkerArray, queue_size=1)
        self.blocker_pub = rospy.Publisher("/velocity_gate/blocker", PointCloud2, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber(rospy.get_param("~odom_topic", "/odom"), Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber(rospy.get_param("~plan_topic", "/move_base/NavfnROS/plan"), Path, self.on_plan, queue_size=1)
        rospy.Subscriber(rospy.get_param("~local_plan_topic", self.teb_ns + "/local_plan"), Path, self.on_local_plan, queue_size=1)
        rospy.Subscriber("/cmd_vel_nav", Twist, self.on_cmd, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / self.rate), self.on_timer)
        rospy.Timer(rospy.Duration(0.1), self.on_reinit_timer)
        rospy.on_shutdown(self.on_shutdown)
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
            fresh = now - self.odom_t <= self.logic.p["odom_timeout"]
            self.logic.update_scan(pts, now, vw=self.odom_vw if fresh else None)

    def on_odom(self, msg):
        self.odom_vw = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        self.odom_t = rospy.Time.now().to_sec()

    def on_plan(self, msg):
        self.plan_xy = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        self.plan_t = rospy.Time.now().to_sec()

    def _base_pose(self, frame):
        """Pose (x, y, yaw) of the base frame expressed in `frame`, or None if tf fails."""
        try:
            tf = self.tf_buf.lookup_transform(frame, self.base_frame, rospy.Time(0), rospy.Duration(0.05))
        except Exception:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    @staticmethod
    def _to_base(pose, pts):
        """Express (x, y) points given in a frame in the base frame, where `pose` is the base pose in that frame."""
        x0, y0, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        return [(c * (x - x0) + s * (y - y0), -s * (x - x0) + c * (y - y0)) for x, y in pts]

    def on_local_plan(self, msg):
        """TEB local plan -> band monitor. Both plans are converted to the robot frame first."""
        plan = self.plan_xy
        now = rospy.Time.now().to_sec()
        if not plan or len(msg.poses) < 3 or now - self.plan_t > self.logic.p["shim_plan_age"]:
            return
        p_local, p_map = self._base_pose(msg.header.frame_id), self._base_pose(self.map_frame)
        if p_local is None or p_map is None:
            return
        local = self._to_base(p_local, [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses])
        glob = self._to_base(p_map, plan)
        i = min(range(len(glob)), key=lambda k: glob[k][0] ** 2 + glob[k][1] ** 2)
        with self.lock:
            self.logic.band_update(local, glob[i:i + 120], now)

    def on_reinit_timer(self, _):
        """Apply logic.reinit_active to TEB (force_reinit_new_goal_dist 0.0 = rebuild the band every cycle)."""
        want = self.logic.reinit_active(rospy.Time.now().to_sec())
        if want == self.reinit_on:
            return
        if self.teb_client is None:
            now = rospy.Time.now().to_sec()
            if now - self.teb_try_t < 2.0:
                return
            self.teb_try_t = now
            try:
                import dynamic_reconfigure.client as dyn_client
                self.teb_client = dyn_client.Client(self.teb_ns, timeout=0.5)
                self.teb_base = self.teb_client.get_configuration(timeout=1.0)["force_reinit_new_goal_dist"]
            except Exception as e:
                self.teb_client = None
                rospy.logwarn_throttle(10.0, "[gate] TEB reinit unavailable (%s): %s", self.teb_ns, e)
                return
        self._set_reinit(want)

    def _set_reinit(self, on):
        try:
            self.teb_client.update_configuration({"force_reinit_new_goal_dist": 0.0 if on else self.teb_base})
            self.reinit_on = on
            if not on:
                rospy.loginfo("[gate] teb reinit end")
        except Exception as e:
            self.teb_client = None
            rospy.logwarn("[gate] TEB reinit reconfigure failed: %s", e)

    def on_shutdown(self):
        if self.reinit_on and self.teb_client is not None:
            self._set_reinit(False)

    def heading_error(self, now, spin_guard=False):
        """Bearing error to the new global path, or None if the plan is stale/missing or tf fails.

        spin_guard: also return None when less than spin_min_remaining of plan is left, so spin-first never fights
        TEB's goal-yaw rotation near the goal (bearing to a point at the robot is meaningless).
        """
        if not self.plan_xy or now - self.plan_t > self.logic.p["shim_plan_age"]:
            return None
        pose = self._base_pose(self.map_frame)
        if pose is None:
            return None
        if spin_guard and path_remaining(pose, self.plan_xy) < self.logic.p["spin_min_remaining"]:
            return None
        return path_heading_error(pose, self.plan_xy, self.logic.p["shim_lookahead"])

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
        if self.logic.state == "giveup":
            herr = self.heading_error(now)
        elif self.logic.p["spin_enable"]:
            herr = self.heading_error(now, spin_guard=True)
        else:
            herr = None
        with self.lock:
            out.linear.x, out.angular.z = self.logic.step(self.cmd.linear.x, self.cmd.angular.z, now, herr)

        if self.logic.state != self.last_state:
            why = " [%s]" % self.logic.reason if self.logic.state == "hold" else ""
            if self.logic.spinning:
                why += " (spin)"
            info = " | %s" % self.logic.trigger_info if self.logic.trigger_info else ""
            rospy.loginfo("[gate] %s -> %s%s%s (fwd_gap: %.2fm)",
                          self.last_state, self.logic.state, why, info, self.logic.gap)
            self.state_pub.publish(String(self.logic.state))
            if self.logic.state == "giveup":
                self.set_blocker(now)
            self.last_state = self.logic.state
        else:
            # Throttled 1.0s live telemetry so console shows real-time state and velocities
            rospy.loginfo_throttle(1.0, "[gate] gap: %.2fm | outer: %.2fm | state: %s%s | scale: %.2f | v_out: %.2fm/s, w_out: %.2frad/s",
                                   self.logic.gap, self.logic.gap_outer, self.logic.state,
                                   " (spin)" if self.logic.spinning else "", self.logic.scale, out.linear.x, out.angular.z)

        if self.logic.reinit_count != self.last_reinit_n:
            self.last_reinit_n = self.logic.reinit_count
            rospy.loginfo("[gate] teb reinit (%s)", self.logic.reinit_reason)

        if self.logic.spinning != self.last_spin:
            rospy.loginfo("[gate] spin %s (heading error %s)", "start" if self.logic.spinning else "stop",
                          "n/a" if herr is None else "%.0f deg" % math.degrees(herr))
            self.last_spin = self.logic.spinning

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

    def _marker(self, mid, mtype, stamp, z=0.0):
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = stamp
        m.ns = "shields"
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        return m

    @staticmethod
    def _rgba(m, r, g, b, a):
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a

    def publish_shields(self, _):
        """Draw the shields exactly as GateLogic defines them (same params, drive_center frame).

        inner  : rectangle, x in [nose, nose + max(d_inner, d_hold_static)], |y| <= inner_half_w   (HOLD zone)
        outer  : half-ellipse centred on the nose, forward semi-axis d_outer, lateral semi-axis outer_half_w
                 (slow-down zone; speed scale is 1.0 on the rim and scale_min at the inner depth)
        flank  : only drawn when flank_enable (two boxes beside the chassis)
        cross  : thin outline of the crossing watch zone, x in [nose, nose + cross_x_max], |y| <= cross_zone_w
        """
        if self.shield_pub.get_num_connections() == 0:
            return  # Zero overhead when RViz is closed

        p = self.logic.p
        nose_x = p["nose_x"]
        d_stop = max(p["d_inner"], p["d_hold_static"])   # depth at which the gate actually HOLDs
        d_outer, outer_w = p["d_outer"], p["outer_half_w"]
        inner_w = p["inner_half_w"]

        with self.lock:
            state = self.logic.state
            scale = self.logic.scale
            gap = self.logic.gap
            gap_outer = self.logic.gap_outer
            reason = self.logic.reason
            flank_breach = self.logic.flank_breach
            crossing_obs = self.logic.crossing_obs
            spinning = self.logic.spinning

        stamp = rospy.Time.now()

        # 1. Inner Protective Shield: HOLD rectangle
        m_inner = self._marker(1, Marker.CUBE, stamp, z=0.10)
        m_inner.pose.position.x = nose_x + d_stop / 2.0
        m_inner.scale.x, m_inner.scale.y, m_inner.scale.z = d_stop, 2.0 * inner_w, 0.20
        if gap <= d_stop or (state == "hold" and reason == "blocked"):
            self._rgba(m_inner, 1.0, 0.0, 0.0, 0.70)
        elif state == "hold" and reason == "crossing":
            self._rgba(m_inner, 1.0, 0.0, 1.0, 0.60)
        else:
            self._rgba(m_inner, 0.9, 0.15, 0.15, 0.25)

        # 2. Outer Warning Shield: half-ellipse (filled fan + outline), points in the same form as update_scan
        rim = []
        n_seg = 36
        for i in range(n_seg + 1):
            th = -math.pi / 2.0 + math.pi * i / n_seg
            rim.append((nose_x + d_outer * math.cos(th), outer_w * math.sin(th)))
        if state == "free" and scale < 0.99:
            col = (1.0, 0.60, 0.0)
            fill_a, line_a = 0.30, 0.95
        elif state == "hold":
            col = (0.75, 0.50, 0.0)
            fill_a, line_a = 0.10, 0.60
        else:
            col = (0.95, 0.85, 0.10)
            fill_a, line_a = 0.10, 0.70

        m_outer = self._marker(2, Marker.TRIANGLE_LIST, stamp, z=0.05)
        m_outer.scale.x = m_outer.scale.y = m_outer.scale.z = 1.0
        for a, b in zip(rim[:-1], rim[1:]):
            for q in ((nose_x, 0.0), a, b):
                m_outer.points.append(Point(q[0], q[1], 0.0))
        self._rgba(m_outer, col[0], col[1], col[2], fill_a)

        m_rim = self._marker(8, Marker.LINE_STRIP, stamp, z=0.06)
        m_rim.scale.x = 0.02
        m_rim.points = [Point(x, y, 0.0) for x, y in rim + [rim[0]]]   # closed along the nose line
        self._rgba(m_rim, col[0], col[1], col[2], line_a)

        # 3/4. Flank shield boxes: only when the shield is enabled (otherwise remove any stale markers)
        flank = []
        for mid, sign in ((3, 1.0), (4, -1.0)):
            m = self._marker(mid, Marker.CUBE, stamp, z=0.10)
            if p["flank_enable"]:
                w_flank = p["flank_w"] - 0.23
                m.pose.position.x = (nose_x + p["flank_x_min"]) / 2.0
                m.pose.position.y = sign * (p["flank_w"] + 0.23) / 2.0
                m.scale.x, m.scale.y, m.scale.z = nose_x - p["flank_x_min"], w_flank, 0.20
                if flank_breach:
                    self._rgba(m, 1.0, 0.0, 0.8, 0.80)
                else:
                    self._rgba(m, 0.1, 0.5, 0.9, 0.16)
            else:
                m.action = Marker.DELETE
            flank.append(m)

        # 5. Floating Status Billboard
        m_text = self._marker(0, Marker.TEXT_VIEW_FACING, stamp, z=0.70)
        m_text.pose.position.x = 0.10
        m_text.scale.z = 0.14

        gap_str = "%.2fm" % gap if gap < 99.0 else "clear"
        out_str = "%.2fm" % gap_outer if gap_outer < d_outer else "clear"
        if state == "hold":
            if reason == "crossing":
                m_text.text = "SHIELD: YIELD (crossing) | gap: %s" % gap_str
            else:
                m_text.text = "SHIELD: HOLD [%s] | gap: %s" % (reason or "blocked", gap_str)
                if spinning:
                    m_text.text += " | SPIN"
            self._rgba(m_text, 1.0, 0.2, 0.2, 1.0)
        elif spinning:
            m_text.text = "SHIELD: SPIN toward plan"
            self._rgba(m_text, 0.3, 0.8, 1.0, 1.0)
        elif scale < 0.99 and state == "free":
            m_text.text = "SHIELD: SLOW (%.0f%%) | outer: %s" % (scale * 100, out_str)
            self._rgba(m_text, 1.0, 0.7, 0.0, 1.0)
        elif state == "giveup":
            m_text.text = "SHIELD: GIVEUP / REROUTE"
            self._rgba(m_text, 0.9, 0.4, 1.0, 1.0)
        else:
            m_text.text = "SHIELD: CLEAR"
            self._rgba(m_text, 0.2, 1.0, 0.3, 0.90)

        # 6. Suspected crossing object
        m_cross = self._marker(6, Marker.SPHERE, stamp, z=0.15)
        if crossing_obs is not None and state == "hold" and reason == "crossing":
            m_cross.pose.position.x = crossing_obs[0]
            m_cross.pose.position.y = crossing_obs[1]
            m_cross.scale.x = m_cross.scale.y = m_cross.scale.z = 0.20
            self._rgba(m_cross, 1.0, 0.0, 1.0, 0.90)
        else:
            m_cross.action = Marker.DELETE

        # 7. Crossing watch zone outline
        zx0, zx1, zy = nose_x, nose_x + p["cross_x_max"], p["cross_zone_w"]
        m_zone = self._marker(7, Marker.LINE_STRIP, stamp, z=0.02)
        m_zone.scale.x = 0.01
        m_zone.points = [Point(x, y, 0.0) for x, y in ((zx0, -zy), (zx1, -zy), (zx1, zy), (zx0, zy), (zx0, -zy))]
        self._rgba(m_zone, 0.3, 0.7, 1.0, 0.35)

        arr = MarkerArray()
        arr.markers = [m_inner, m_outer, m_rim, flank[0], flank[1], m_text, m_cross, m_zone]
        self.shield_pub.publish(arr)


if __name__ == "__main__":
    rospy.init_node("velocity_gate")
    VelocityGate()
    rospy.spin()
