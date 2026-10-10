#!/usr/bin/env python3
"""
AMR Pallet Stacker Mast Elevation Command Node.
ROS CLI node wrapper around amr_navigation.lift.LiftController.

Usage:
  rosrun amr_navigation lift_node.py 0.35
  rosrun amr_navigation lift_node.py _height:=0.35 _lift_speed:=0.08
"""
import sys
import rospy
from amr_navigation.lift import LiftController


def main():
    if not rospy.core.is_initialized():
        rospy.init_node("lift_node", anonymous=True)

    target_height = 0.25
    if len(sys.argv) > 1 and not sys.argv[1].startswith("_"):
        try:
            target_height = float(sys.argv[1])
        except ValueError:
            rospy.logerr("Invalid height argument: %s", sys.argv[1])
            return
    else:
        target_height = float(rospy.get_param("~height", 0.25))

    speed = float(rospy.get_param("~lift_speed", rospy.get_param("~speed", 0.08)))
    lift = LiftController(default_speed=speed)
    rospy.sleep(0.2)
    lift.set_height(target_height)
    rospy.loginfo("[lift_node] Elevation to %.3fm completed.", target_height)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
