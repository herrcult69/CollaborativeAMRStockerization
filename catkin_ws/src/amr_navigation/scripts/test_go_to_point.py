"""Offline checks; never publishes robot commands."""
import math
import unittest
from go_to_point import command_for_goal


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


if __name__ == "__main__":
    unittest.main()
