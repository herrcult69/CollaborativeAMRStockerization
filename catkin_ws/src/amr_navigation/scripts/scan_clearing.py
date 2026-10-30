#!/usr/bin/env python3
"""Publishes /scan_clearing: a clearing-only 360 degree scan that removes the trails dynamic obstacles leave in
the costmaps (densified front rays + map-free blind arc behind the lidar).
/scan + /map + TF map->laser -> /scan_clearing. Logic lives in amr_navigation.scan_clearing; the costmaps consume
it as the laser_clearing source in config/costmap_common.yaml (marking: false)."""
import math

import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

from amr_navigation.scan_clearing import clearing_scan


class ScanClearing:
    def __init__(self):
        self.factor = rospy.get_param("~densify", 4)
        self.blind_range = rospy.get_param("~blind_range", 4.0)
        self.map_margin = rospy.get_param("~map_margin", 0.25)   # stay this far from mapped racks/walls
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.grid = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pub = rospy.Publisher("/scan_clearing", LaserScan, queue_size=1)
        rospy.Subscriber("/map", OccupancyGrid, self.on_map, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.on_scan, queue_size=1)

    def on_map(self, msg):
        info = msg.info
        if abs(info.origin.orientation.z) > 1e-6:
            rospy.logwarn("[scan_clearing] rotated map origin not supported; blind arc clearing off")
            return
        self.grid = (info.resolution, info.origin.position.x, info.origin.position.y,
                     info.width, info.height, msg.data)

    def laser_pose(self, msg):
        try:
            tr = self.tf_buffer.lookup_transform(self.map_frame, msg.header.frame_id, msg.header.stamp,
                                                 rospy.Duration(0.05)).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(10.0, "[scan_clearing] no %s pose, blind arc not cleared: %s", self.map_frame, e)
            return None
        q = tr.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return tr.translation.x, tr.translation.y, yaw

    def on_scan(self, msg):
        pose = self.laser_pose(msg) if self.grid is not None else None
        ranges, a0, inc = clearing_scan(msg.ranges, msg.angle_min, msg.angle_increment, self.factor,
                                        pose, self.grid, self.blind_range, self.map_margin)
        out = LaserScan()
        out.header = msg.header
        out.angle_min, out.angle_increment = a0, inc
        out.angle_max = a0 + (len(ranges) - 1) * inc
        out.time_increment = 0.0      # rays are no longer in sweep order; project them all at the scan stamp
        out.scan_time = msg.scan_time
        out.range_min, out.range_max = msg.range_min, msg.range_max
        out.ranges = ranges.tolist()
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("scan_clearing")
    ScanClearing()
    rospy.spin()
