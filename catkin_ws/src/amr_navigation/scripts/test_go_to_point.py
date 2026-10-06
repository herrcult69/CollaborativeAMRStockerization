"""Offline checks; never publishes robot commands."""
import math
import unittest
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock, patch

# Nose/catkin imports this file without adding scripts/ to sys.path.
sys.path.insert(0, str(Path(__file__).resolve().parent))

# Explicit offline transport stand-ins: no ROS master and no real commands.
try:
    import rospy
except ModuleNotFoundError as exc:
    if exc.name != "rospy":
        raise
    rospy = types.ModuleType("rospy")
    for name in ("get_param", "Publisher", "Subscriber", "on_shutdown", "is_shutdown",
                 "loginfo", "loginfo_throttle", "logerr", "logerr_throttle", "logwarn_throttle"):
        setattr(rospy, name, MagicMock())
    rospy.Time = MagicMock()
    sys.modules["rospy"] = rospy
    class OfflineTwist:
        def __init__(self):
            self.linear = types.SimpleNamespace(x=0., y=0., z=0.)
            self.angular = types.SimpleNamespace(x=0., y=0., z=0.)
    for package, message, cls in (("geometry_msgs", "Twist", OfflineTwist),
                                  ("nav_msgs", "Odometry", object)):
        module = types.ModuleType(package + ".msg")
        setattr(module, message, cls)
        sys.modules[package] = types.ModuleType(package)
        sys.modules[package + ".msg"] = module

from go_to_point import command_for_goal, GoToPoint


class ControllerTests(unittest.TestCase):
    def command(self, x, y, yaw, gx=2, gy=0):
        return command_for_goal(x, y, yaw, gx, gy, 0.2, 0.2, 0.5)

    def test_stop_at_goal(self):
        self.assertEqual(self.command(2, 0, 1)[:2], (0, 0))

    def test_turn_before_driving(self):
        v, w, _ = self.command(0, 0, math.pi / 2)
        self.assertEqual(v, 0)
        self.assertLess(w, 0)

    def test_heading_wrap(self):
        v, w, _ = self.command(0, 0, math.pi - 0.01, -2, -0.02)
        self.assertGreater(v, 0)
        self.assertGreater(w, 0)

    def test_convergence_in_ideal_kinematic_simulation(self):
        for goal in [(2, 0), (2, 1), (-1, -1)]:
            x, y, yaw = 0., 0., 1.5
            for _ in range(4000):
                v, w, _ = self.command(x, y, yaw, *goal)
                self.assertLessEqual(abs(v), 0.2)
                self.assertLessEqual(abs(w), 0.5)
                x += v * math.cos(yaw) * 0.05
                y += v * math.sin(yaw) * 0.05
                yaw += w * 0.05
            self.assertLessEqual(math.hypot(goal[0] - x, goal[1] - y), 0.2)


class OdometryContractTests(unittest.TestCase):
    def setUp(self):
        self.transport = MagicMock()
        self.transport.get_param.side_effect = lambda name, default: default
        self.transport.Time.now.return_value.to_sec.return_value = 10.
        self.transport.is_shutdown.side_effect = [False, True]
        self.addCleanup(patch.stopall)
        patch("go_to_point.rospy", self.transport).start()
        patch("go_to_point.time.monotonic", return_value=10.).start()
        patch("go_to_point.time.sleep").start()
        self.controller = GoToPoint()

    def odom(self, child="drive_center", parent="odom"):
        return types.SimpleNamespace(
            header=types.SimpleNamespace(frame_id=parent, stamp=types.SimpleNamespace(to_sec=lambda: 10.)),
            child_frame_id=child,
            pose=types.SimpleNamespace(pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=0., y=0.),
                orientation=types.SimpleNamespace(x=0., y=0., z=0., w=1.))))

    def test_xstack_frame_accepted(self):
        self.controller.on_odom(self.odom())
        self.assertIsNotNone(self.controller.sample)

    def test_legacy_frame_accepted_when_configured(self):
        self.controller.base_frame = "base_footprint"
        self.controller.on_odom(self.odom("base_footprint"))
        self.assertIsNotNone(self.controller.sample)

    def test_mismatched_child_rejected(self):
        self.controller.on_odom(self.odom("base_footprint"))
        self.assertIsNone(self.controller.sample)

    def test_mismatched_parent_rejected(self):
        self.controller.on_odom(self.odom(parent="map"))
        self.assertIsNone(self.controller.sample)

    def assert_stopped(self):
        self.controller.run()
        commands = self.controller.pub.publish.call_args_list
        self.assertTrue(commands)
        for call in commands:
            msg = call.args[0]
            self.assertEqual((msg.linear.x, msg.angular.z), (0., 0.))

    def test_missing_odometry_stops(self):
        self.assert_stopped()

    def test_stale_receipt_stops(self):
        self.controller.sample = (0., 0., 0., 9., 10.)
        self.assert_stopped()

    def test_stale_simulation_stamp_stops(self):
        self.controller.sample = (0., 0., 0., 10., 9.)
        self.assert_stopped()

    def test_future_simulation_stamp_stops(self):
        self.controller.sample = (0., 0., 0., 10., 11.)
        self.assert_stopped()


if __name__ == "__main__":
    unittest.main()
