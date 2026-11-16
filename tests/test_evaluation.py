"""Integration checks for the shared comparison runner."""

import unittest

from cropsafe_pathplanning.evaluation import evaluate_algorithms
from cropsafe_pathplanning.field_scenario import make_field


class EvaluationIntegrationTests(unittest.TestCase):
    def test_all_submitted_algorithms_reach_goal_on_shared_scenario(self):
        field = make_field()
        summaries = evaluate_algorithms(field)

        self.assertEqual([item.algorithm for item in summaries], ["BFS", "DFS", "Greedy", "A*"])
        for item in summaries:
            with self.subTest(algorithm=item.algorithm):
                self.assertTrue(item.reached_goal)
                self.assertEqual(item.path[0], field.start)
                self.assertEqual(item.path[-1], field.goal)
                self.assertTrue(set(item.path).isdisjoint(field.blocked))
                self.assertGreater(item.expanded_nodes, 0)
                self.assertGreaterEqual(item.elapsed_seconds, 0.0)

    def test_astar_has_lowest_cost_under_shared_risk_model(self):
        summaries = evaluate_algorithms()
        costs = {item.algorithm: item.cost for item in summaries}

        self.assertEqual(min(costs, key=costs.get), "A*")


if __name__ == "__main__":
    unittest.main()
