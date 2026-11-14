"""Focused checks for Sharad's online obstacle and replanning work."""

import unittest

from cropsafe_pathplanning.model import demo_field, demo_unknown_obstacle
from cropsafe_pathplanning.replanning import simulate_replanning
from cropsafe_pathplanning.search import a_star_search


class ReplanningTests(unittest.TestCase):
    def test_unknown_obstacle_is_not_in_initial_map(self):
        field = demo_field()
        hidden = demo_unknown_obstacle()
        self.assertNotIn(hidden, field.obstacles)
        self.assertIn(hidden, a_star_search(field).path)

    def test_robot_detects_blocked_route_and_reaches_goal_after_replanning(self):
        field = demo_field()
        hidden = demo_unknown_obstacle()
        result = simulate_replanning(field, {hidden})

        self.assertEqual(result.detected_obstacles, [hidden])
        self.assertEqual(result.replans, 1)
        self.assertTrue(result.reached_goal)
        self.assertEqual(result.final_path[0], field.start)
        self.assertEqual(result.final_path[-1], field.goal)
        self.assertNotIn(hidden, result.final_path)


if __name__ == "__main__":
    unittest.main()
