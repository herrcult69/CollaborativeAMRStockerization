#!/usr/bin/env python3
"""
AMR Precision Docking & Waypoint Navigation Node.
ROS node wrapper around amr_navigation.docking.ThreePhaseDockingController.
"""
import rospy
from amr_navigation.docking import ThreePhaseDockingController


def main():
    rospy.init_node("docking_node")
    controller = ThreePhaseDockingController()
    try:
        controller.run()
    finally:
        controller.stop()


if __name__ == "__main__":
    main()
