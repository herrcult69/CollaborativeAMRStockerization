#!/usr/bin/env python3
"""
AMR Waypoint / Move-To-Point Node.
ROS CLI node wrapper around amr_navigation.move_to_point.MoveToPointController.
"""
import rospy
from amr_navigation.move_to_point import MoveToPointController


def main():
    rospy.init_node("move_to_point_node")
    controller = MoveToPointController()
    try:
        controller.run()
    finally:
        controller.stop()


if __name__ == "__main__":
    main()
