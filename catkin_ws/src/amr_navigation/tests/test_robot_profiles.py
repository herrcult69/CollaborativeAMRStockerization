"""ROS launch/URDF contract tests. Loads configuration only; starts no nodes."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import roslaunch
import rospkg
import yaml


class RobotProfileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        packages = rospkg.RosPack()
        cls.nav = Path(packages.get_path("amr_navigation"))
        cls.description = Path(packages.get_path("amr_description"))

    def load(self, filename, profile=None, extra=()):
        config = roslaunch.ROSLaunchConfig()
        args = ([] if profile is None else ["robot_profile:=" + profile]) + list(extra)
        roslaunch.xmlloader.XmlLoader().load(str(self.nav / "launch" / filename), config, argv=args, verbose=False)
        return config

    def test_navigation_profiles_match_odom_reference(self):
        for profile, frame in [("legacy", "base_footprint"), ("xstack", "drive_center")]:
            for launch in ("warehouse_navigation.launch", "costmap_preview.launch", "move_base.launch"):
                with self.subTest(profile=profile, launch=launch):
                    config = self.load(launch, profile)
                    base_params = [p.value for k, p in config.params.items() if k.endswith("/robot_base_frame")]
                    self.assertEqual(base_params, [frame, frame])
                    footprints = [p.value for k, p in config.params.items() if k.endswith("/footprint")]
                    self.assertEqual(len(footprints), 2)
                    self.assertEqual(footprints[0], footprints[1])
                    self.assertEqual(len(footprints[0]), 4)

    def test_localization_mapping_and_controller_use_same_frame(self):
        for profile, frame in [("legacy", "base_footprint"), ("xstack", "drive_center")]:
            for filename, key, extra in [
                ("warehouse_mapping.launch", "/slam_gmapping/base_frame", ()),
                ("warehouse_localization.launch", "/amcl/base_frame_id", ("map_file:=/tmp/parse-only.yaml",)),
                ("go_to_point.launch", "/go_to_point/base_frame", ())]:
                with self.subTest(profile=profile, filename=filename):
                    self.assertEqual(self.load(filename, profile, extra).params[key].value, frame)

    def test_new_profile_is_default(self):
        config = self.load("warehouse_navigation.launch")
        self.assertEqual(config.params["/move_base/local_costmap/robot_base_frame"].value, "drive_center")

    def test_unknown_profile_rejected(self):
        with self.assertRaises(Exception):
            self.load("warehouse_navigation.launch", "typo")

    def test_localization_selects_measured_map_and_allows_override(self):
        for profile, filename in [("xstack", "milestone_1_01.yaml"), ("legacy", "warehouse_training_01.yaml")]:
            config = self.load("warehouse_localization.launch", profile)
            server = next(n for n in config.nodes if n.type == "map_server")
            self.assertIn(filename, server.args)
            self.assertTrue((self.nav / "maps" / filename).is_file())
            metadata = yaml.safe_load((self.nav / "maps" / filename).read_text())
            self.assertTrue((self.nav / "maps" / metadata['image']).is_file())
        config = self.load("warehouse_localization.launch", "xstack", ("map_file:=/tmp/explicit-map.yaml",))
        self.assertIn("/tmp/explicit-map.yaml", next(n for n in config.nodes if n.type == "map_server").args)

    def test_warehouse_autonav_launch_loads(self):
        config = self.load("warehouse_autonav.launch", "xstack")
        self.assertEqual(config.params["/move_base/local_costmap/robot_base_frame"].value, "drive_center")
        self.assertEqual(config.params["/amcl/base_frame_id"].value, "drive_center")
        self.assertEqual(config.params["/amcl/initial_pose_x"].value, 0.0)

        config_m = self.load("warehouse_autonav.launch", "xstack", extra=("launch_mission:=true",))
        self.assertEqual(config_m.params["/milestone2_mission/rack_standoff"].value, 1.2)
        self.assertEqual(config_m.params["/milestone2_mission/station_x"].value, 13.12)

    def test_standalone_description_profiles_resolve(self):
        for profile, name in [("xstack", "xstack_amr"), ("legacy", "reverse_stacker_amr")]:
            config = roslaunch.ROSLaunchConfig()
            roslaunch.xmlloader.XmlLoader().load(str(self.description / "launch/display.launch"), config,
                argv=["robot_profile:=" + profile], verbose=False)
            model = ET.fromstring(config.params['/robot_description'].value)
            self.assertEqual(model.attrib['name'], name)

    def test_mounts_match_description_and_have_one_parent(self):
        for profile, filename, root in [("legacy", "stacker_amr.urdf", "base_footprint"),
                                        ("xstack", "xstack_amr.urdf", "drive_center")]:
            with self.subTest(profile=profile):
                model = ET.parse(self.description / "urdf" / filename).getroot()
                links = {link.attrib["name"] for link in model.findall("link")}
                parents = {}
                joints = {}
                for joint in model.findall("joint"):
                    parent, child = joint.find("parent").attrib["link"], joint.find("child").attrib["link"]
                    self.assertNotIn(child, parents)
                    self.assertIn(parent, links)
                    self.assertIn(child, links)
                    parents[child] = parent
                    joints[(parent, child)] = joint
                self.assertEqual(links - set(parents), {root})
                for link in links:
                    seen = set()
                    while link in parents:
                        self.assertNotIn(link, seen)
                        seen.add(link)
                        link = parents[link]
                    self.assertEqual(link, root)
                for node in self.load("laser_tf.launch", profile).nodes:
                    args = node.args.split()
                    parent, child = args[-2:]
                    xyz = [float(v) for v in args[:3]]
                    origin = joints[(parent, child)].find("origin")
                    self.assertEqual(xyz, [float(v) for v in origin.attrib["xyz"].split()])


if __name__ == "__main__":
    unittest.main()
