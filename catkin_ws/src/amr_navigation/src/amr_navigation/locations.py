"""
Named warehouse locations (package_1, station_1, ...) exported from Unity by Tools > Export ROS Locations.

Launch files load config/locations_<scene>.yaml into the node's private namespace, giving ~packages and
~stations. Each entry is the object centre in the map frame plus its dock heading (the robot heading while
reversing in). Mission scripts turn the centre into a drive_center stop with compute_dock_target_pose and
pallet_dock_offset / station_dock_offset.
"""

import math

# Floor/bottom pallets sit at 0.03 m (legacy) or 0.11-0.12 m on Rack-DetectReady; the next rack level starts at ~0.37 m
FLOOR_MAX_HEIGHT = 0.18
# A pallet rests on another when it is in the same bay and less than one stack step higher. A shelf deck
# between them makes the gap larger (Rack-DetectReady2: 0.263 m stacked, 0.408 m across the shelf).
STACK_MAX_STEP = 0.30
SAME_BAY = 0.02


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


def pick_unit(table, name, empty=()):
    """
    What the forks carry when picking `name`: the pallet plus every pallet still stacked on it, bottom first.
    Picking the top of a stack takes one pallet; picking the bottom takes the whole unit.
    :param empty: names already picked (their slots are empty, so nothing rests on them)
    """
    x, y, _, h = find_location(table, name)
    unit = [name]
    while True:
        above = [n for n in table if n not in empty and n not in unit
                 and math.hypot(float(table[n]["x"]) - x, float(table[n]["y"]) - y) <= SAME_BAY
                 and 0.0 < float(table[n].get("height", 0.0)) - h < STACK_MAX_STEP]
        if not above:
            return unit
        n = min(above, key=lambda k: float(table[k].get("height", 0.0)))
        unit.append(n)
        h = float(table[n].get("height", 0.0))
