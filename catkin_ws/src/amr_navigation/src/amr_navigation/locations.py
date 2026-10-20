"""
Named warehouse locations (package_1, station_1, ...) exported from Unity by Tools > Export ROS Locations.

Launch files load config/locations_<scene>.yaml into the node's private namespace, giving ~packages and
~stations. Each entry is the object centre in the map frame plus its dock heading (the robot heading while
reversing in). Mission scripts turn the centre into a drive_center stop with compute_dock_target_pose and
pallet_dock_offset / station_dock_offset.
"""

# Floor pallets sit at 0.03 m in Milestone_3; the next rack level starts at 0.29 m
FLOOR_MAX_HEIGHT = 0.10


def find_location(table, name):
    """
    :param table: name -> {x, y, yaw, height, ...}, e.g. rospy.get_param("~packages", {})
    :param name: entry to look up, e.g. "package_5"
    :return: (x, y, yaw_deg, height): object centre in map (m), dock heading (deg), base height (m)
    """
    if not table:
        raise KeyError("no locations loaded for '%s': check locations_file in the launch file" % name)
    if name not in table:
        raise KeyError("location '%s' not in the locations file (%d entries)" % (name, len(table)))
    e = table[name]
    return float(e["x"]), float(e["y"]), float(e["yaw"]), float(e.get("height", 0.0))
