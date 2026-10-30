#!/usr/bin/env python3
"""Pick units on the exported NewMapnoObs layout: a lower pallet takes its stack, an upper one goes alone."""
import os
import sys
import unittest

import yaml

_pkg_src = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if os.path.isdir(_pkg_src) and _pkg_src not in sys.path:
    sys.path.insert(0, _pkg_src)

from amr_navigation.locations import pick_unit

with open(os.path.join(os.path.dirname(__file__), "..", "config", "locations_milestone_3.yaml")) as f:
    PACKAGES = yaml.safe_load(f)["packages"]

# One bay of Rack-DetectReady2 (4): tier 1 / 2 stacked on the floor, tier 3 / 4 stacked on the shelf
T1, T2, T3, T4 = "package_1", "package_4", "package_2", "package_3"


class PickUnitTests(unittest.TestCase):
    def test_bay_layout_is_as_assumed(self):
        self.assertEqual([PACKAGES[n]["height"] for n in (T1, T2, T3, T4)], [0.112, 0.392, 0.782, 1.062])

    def test_lower_pallet_takes_its_stack(self):
        self.assertEqual(pick_unit(PACKAGES, T1), [T1, T2])
        self.assertEqual(pick_unit(PACKAGES, T3), [T3, T4])  # the shelf deck stops the stack at tier 2

    def test_upper_pallet_goes_alone(self):
        self.assertEqual(pick_unit(PACKAGES, T2), [T2])
        self.assertEqual(pick_unit(PACKAGES, T4), [T4])

    def test_lower_pallet_alone_once_its_top_is_gone(self):
        self.assertEqual(pick_unit(PACKAGES, T1, empty=[T2]), [T1])
        self.assertEqual(pick_unit(PACKAGES, T3, empty=[T4]), [T3])

    def test_every_lower_slot_in_the_export_carries_at_most_two(self):
        for name in PACKAGES:
            self.assertLessEqual(len(pick_unit(PACKAGES, name)), 2, name)


if __name__ == "__main__":
    unittest.main()
