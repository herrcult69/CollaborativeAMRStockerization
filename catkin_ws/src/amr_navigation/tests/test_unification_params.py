#!/usr/bin/env python3
"""
Unit tests for unified parameters, LiftController defaults, and launch file contracts.
Verifies Phase 1 Parameter & Script Unification requirements.
"""
import inspect
import math
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

_pkg_src = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if os.path.isdir(_pkg_src) and _pkg_src not in sys.path:
    sys.path.insert(0, _pkg_src)

import roslaunch
import rospkg
from amr_navigation.lift import LiftController
from amr_navigation.docking import compute_dock_target_pose


class ParameterUnificationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        packages = rospkg.RosPack()
        cls.nav = Path(packages.get_path("amr_navigation"))

    def test_lift_controller_signature_default(self):
        sig = inspect.signature(LiftController.__init__)
        self.assertEqual(sig.parameters["default_speed"].default, 0.08)

    @patch("rospy.Publisher")
    @patch("rospy.get_param")
    def test_lift_controller_default_speed_init(self, mock_get_param, mock_pub):
        mock_get_param.side_effect = lambda key, default=None: default
        ctrl = LiftController()
        self.assertEqual(ctrl.default_speed, 0.08)

    @patch("rospy.Publisher")
    @patch("rospy.get_param")
    def test_lift_controller_custom_speed(self, mock_get_param, mock_pub):
        mock_get_param.side_effect = lambda key, default=None: default
        ctrl = LiftController(default_speed=0.05)
        self.assertEqual(ctrl.default_speed, 0.05)

    @patch("rospy.Publisher")
    @patch("rospy.get_param")
    def test_lift_controller_rosparam_override(self, mock_get_param, mock_pub):
        mock_get_param.side_effect = lambda key, default=None: 0.12 if "lift_speed" in key else default
        ctrl = LiftController(default_speed=0.08)
        self.assertEqual(ctrl.default_speed, 0.12)

    def test_station_docking_and_standoff_geometry_13_12(self):
        station_x = 13.12
        station_y = 0.0
        station_yaw = 180.0
        fork_offset = 0.22
        station_standoff = 0.8

        target_x, target_y = compute_dock_target_pose(
            pallet_x=station_x,
            pallet_y=station_y,
            dock_yaw_deg=station_yaw,
            fork_offset=fork_offset,
        )
        self.assertAlmostEqual(target_x, 12.90, places=2)
        self.assertAlmostEqual(target_y, 0.0, places=2)

        stage_x = target_x + station_standoff * math.cos(math.radians(station_yaw))
        stage_y = target_y + station_standoff * math.sin(math.radians(station_yaw))
        self.assertAlmostEqual(stage_x, 12.10, places=2)
        self.assertAlmostEqual(stage_y, 0.0, places=2)

    def test_rack_standoff_geometry(self):
        pallet_x = 4.0
        pallet_y = 5.0
        dock_yaw = 270.0
        fork_offset = 0.22
        rack_standoff = 1.2

        target_x, target_y = compute_dock_target_pose(
            pallet_x=pallet_x,
            pallet_y=pallet_y,
            dock_yaw_deg=dock_yaw,
            fork_offset=fork_offset,
        )
        self.assertAlmostEqual(target_x, 4.0, places=2)
        self.assertAlmostEqual(target_y, 4.78, places=2)

        stage_x = target_x + rack_standoff * math.cos(math.radians(dock_yaw))
        stage_y = target_y + rack_standoff * math.sin(math.radians(dock_yaw))
        self.assertAlmostEqual(stage_x, 4.0, places=2)
        self.assertAlmostEqual(stage_y, 3.58, places=2)

    def test_milestone1_launch_contract(self):
        config = roslaunch.ROSLaunchConfig()
        roslaunch.xmlloader.XmlLoader().load(
            str(self.nav / "launch" / "milestone1.launch"), config, argv=[], verbose=False
        )
        params = {k: p.value for k, p in config.params.items()}
        self.assertAlmostEqual(float(params["/milestone1_mission/station_x"]), 13.12, places=2)
        self.assertAlmostEqual(float(params["/milestone1_mission/lift_speed"]), 0.08, places=2)
        self.assertAlmostEqual(float(params["/milestone1_mission/standoff"]), 0.8, places=2)
        self.assertAlmostEqual(float(params["/milestone1_mission/rack_standoff"]), 1.2, places=2)
        self.assertAlmostEqual(float(params["/milestone1_mission/station_standoff"]), 0.8, places=2)

    def test_test_dock_reverse_launch_contract(self):
        config = roslaunch.ROSLaunchConfig()
        roslaunch.xmlloader.XmlLoader().load(
            str(self.nav / "launch" / "test_dock_reverse.launch"), config, argv=[], verbose=False
        )
        params = {k: p.value for k, p in config.params.items()}
        self.assertAlmostEqual(float(params["/test_dock_reverse/standoff"]), 1.2, places=2)
        self.assertAlmostEqual(float(params["/test_dock_reverse/transit_speed"]), 0.45, places=2)
        self.assertAlmostEqual(float(params["/test_dock_reverse/lift_speed"]), 0.08, places=2)

    def test_test_undock_reverse_launch_contract(self):
        config = roslaunch.ROSLaunchConfig()
        roslaunch.xmlloader.XmlLoader().load(
            str(self.nav / "launch" / "test_undock_reverse.launch"), config, argv=[], verbose=False
        )
        params = {k: p.value for k, p in config.params.items()}
        self.assertAlmostEqual(float(params["/test_undock_reverse/station_x"]), 13.12, places=2)
        self.assertAlmostEqual(float(params["/test_undock_reverse/lift_speed"]), 0.08, places=2)

    def test_milestone2_launch_contract(self):
        config = roslaunch.ROSLaunchConfig()
        roslaunch.xmlloader.XmlLoader().load(
            str(self.nav / "launch" / "milestone2.launch"), config, argv=[], verbose=False
        )
        params = {k: p.value for k, p in config.params.items()}
        self.assertAlmostEqual(float(params["/milestone2_mission/station_x"]), 13.12, places=2)
        self.assertAlmostEqual(float(params["/milestone2_mission/lift_speed"]), 0.08, places=2)
        self.assertAlmostEqual(float(params["/milestone2_mission/rack_standoff"]), 1.2, places=2)
        self.assertAlmostEqual(float(params["/milestone2_mission/standoff"]), 0.8, places=2)

    def test_docking_launch_contract(self):
        config = roslaunch.ROSLaunchConfig()
        roslaunch.xmlloader.XmlLoader().load(
            str(self.nav / "launch" / "docking.launch"), config, argv=[], verbose=False
        )
        params = {k: p.value for k, p in config.params.items()}
        self.assertAlmostEqual(float(params["/docking_node/dock_speed"]), 0.12, places=2)
        self.assertAlmostEqual(float(params["/docking_node/dock_tolerance"]), 0.04, places=2)


if __name__ == "__main__":
    unittest.main()
