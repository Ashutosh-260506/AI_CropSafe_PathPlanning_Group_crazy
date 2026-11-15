
import unittest

from cropsafe_pathplanning.model import Field
from cropsafe_pathplanning.search import a_star_search


class TestAStar(unittest.TestCase):

    def test_astar_reaches_goal(self):
        field = Field(
            height=5,
            width=7,
            start=(2, 0),
            goal=(2, 6)
        )

        result = a_star_search(field)

        self.assertTrue(result.reached_goal)
        self.assertEqual(result.path[0], field.start)
        self.assertEqual(result.path[-1], field.goal)

    def test_astar_avoids_blocked_cells(self):
        field = Field(
            height=5,
            width=7,
            start=(2, 0),
            goal=(2, 6),
            protected={(2, 3)},
            obstacles={(1, 2), (3, 2)}
        )

        result = a_star_search(field)

        self.assertTrue(result.reached_goal)
        self.assertTrue(set(result.path).isdisjoint(field.blocked))

    def test_safety_penalty_changes_route(self):
        low_risk_field = Field(
            height=5,
            width=7,
            start=(2, 0),
            goal=(2, 6),
            protected={(2, 3)},
            risk_penalty=0
        )

        high_risk_field = Field(
            height=5,
            width=7,
            start=(2, 0),
            goal=(2, 6),
            protected={(2, 3)},
            risk_penalty=3
        )

        low_result = a_star_search(low_risk_field)
        high_result = a_star_search(high_risk_field)

        self.assertTrue(low_result.reached_goal)
        self.assertTrue(high_result.reached_goal)

        self.assertEqual(low_result.path_length, 8)
        self.assertEqual(high_result.path_length, 10)

        self.assertEqual(low_result.cost, 8)
        self.assertEqual(high_result.cost, 10)

    def test_astar_reports_no_path(self):
        field = Field(
            height=3,
            width=7,
            start=(1, 0),
            goal=(1, 6),
            protected={(1, 3)},
            obstacles={(0, 3), (2, 3)}
        )

        result = a_star_search(field)

        self.assertFalse(result.reached_goal)
        self.assertEqual(result.path, [])

    def test_astar_path_cost(self):
        field = Field(
            height=5,
            width=7,
            start=(2, 0),
            goal=(2, 6),
            protected={(2, 3)},
            risk_penalty=3
        )

        result = a_star_search(field)

        expected_cost = sum(
            field.step_cost(cell)
            for cell in result.path[1:]
        )

        self.assertAlmostEqual(result.cost, expected_cost)


if __name__ == "__main__":
    unittest.main()
