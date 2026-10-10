#!/usr/bin/env python3
"""
AMR Pallet Docking Node.
ROS CLI node wrapper around amr_navigation.docking.PalletDockingController.

Parameters:
  ~dock_x: Goal X position inside pallet cavity (meters)
  ~dock_y: Goal Y position inside pallet cavity (meters)
  ~dock_yaw: Locked orientation in degrees (e.g. 270.0)
  ~dock_speed: Reverse speed limit in m/s (default: 0.08)
  ~dock_tolerance: Positional tolerance in meters (default: 0.03)
"""
import rospy
from amr_navigation.docking import PalletDockingController


def main():
    rospy.init_node("docking_node")
    dock_x = float(rospy.get_param("~dock_x", 4.0))
    dock_y = float(rospy.get_param("~dock_y", 5.0))
    dock_yaw = float(rospy.get_param("~dock_yaw", 270.0))
    speed = float(rospy.get_param("~dock_speed", 0.12))
    tol = float(rospy.get_param("~dock_tolerance", 0.04))
    fork_offset = float(rospy.get_param("~fork_offset", 0.22))

    controller = PalletDockingController(default_dock_speed=speed, default_tolerance=tol)
    try:
        controller.dock_reverse(
            dock_x, dock_y,
            dock_yaw=dock_yaw,
            speed=speed,
            pos_tolerance=tol,
            fork_offset=fork_offset
        )
    finally:
        controller.stop()


if __name__ == "__main__":
    main()
