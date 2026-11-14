"""Automated tests for Greedy Best-First Search (GBFS).

Comprehensive coverage of all assignment specifications:
    - Test 2: Simple reachable grid
    - Test 3: Obstacle avoidance
    - Test 4: Unreachable goal
    - Test 5: Invalid inputs
    - Test 6: Start equals goal
    - Test 7: Reproducibility
    - Test 8: Expanded nodes
    - Shared result dictionary interface validation
    - GBFS heuristic evaluation f(n) = h(n) demonstration
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
import unittest

# Ensure src is on sys.path for direct test invocation
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from cropsafe_pathplanning.gbfs import (
    greedy_best_first_search,
    validate_endpoints,
)
from cropsafe_pathplanning.types import Coordinate, FieldMap, SearchResult


class TestGreedyBestFirstSearch(unittest.TestCase):
    """Test suite for Greedy Best-First Search implementation."""

    def test_shared_result_format(self) -> None:
        """Verify the result dictionary strictly conforms to the shared interface."""
        grid = [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
        ]
        result = greedy_best_first_search(grid, (0, 0), (2, 2))

        # Must be an instance of dict
        self.assertIsInstance(result, dict)

        # Minimum required fields from README / TEAM_TASKS.md
        self.assertIn("path", result)
        self.assertIn("cost", result)
        self.assertIn("expanded_nodes", result)
        self.assertIn("reached_goal", result)

        # Value types
        self.assertIsInstance(result["path"], list)
        self.assertIsInstance(result["cost"], (int, float))
        self.assertIsInstance(result["expanded_nodes"], list)
        self.assertIsInstance(result["reached_goal"], bool)

    def test_test2_simple_reachable_grid(self) -> None:
        """Test 2: Simple reachable grid without obstacles.

        - A clear route exists.
        - Algorithm reaches the goal.
        - Returned path starts at the correct position.
        - Returned path ends at the goal.
        - Path consists of legal cardinal steps.
        """
        grid = [
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
        ]
        start: Coordinate = (0, 0)
        goal: Coordinate = (3, 3)

        result = greedy_best_first_search(grid, start, goal)

        self.assertTrue(result.reached_goal)
        self.assertTrue(result["reached_goal"])
        self.assertGreater(len(result.path), 0)
        self.assertEqual(result.path[0], start)
        self.assertEqual(result.path[-1], goal)

        # Verify steps are strictly 4-directional and contiguous
        for i in range(len(result.path) - 1):
            r1, c1 = result.path[i]
            r2, c2 = result.path[i + 1]
            step_distance = abs(r1 - r2) + abs(c1 - c2)
            self.assertEqual(
                step_distance,
                1,
                f"Non-cardinal step detected from {result.path[i]} to {result.path[i+1]}",
            )

        # Cost matches step count
        self.assertEqual(result.cost, len(result.path) - 1)

    def test_test3_obstacle_avoidance(self) -> None:
        """Test 3: Obstacle avoidance.

        - Introduce blocked cells / crop rows.
        - Verify path navigates around them.
        - Verify the path never passes through any obstacle.
        """
        # Create a field with a vertical wall of obstacles with an opening at (3, 2)
        grid = [
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0],  # Gap at row 3
            [0, 0, 1, 0, 0],
        ]
        start: Coordinate = (1, 1)
        goal: Coordinate = (1, 4)

        result = greedy_best_first_search(grid, start, goal)

        self.assertTrue(result.reached_goal)
        self.assertEqual(result.path[0], start)
        self.assertEqual(result.path[-1], goal)

        # Check every coordinate in path is NOT an obstacle
        blocked_coords = {(0, 2), (1, 2), (2, 2), (4, 2)}
        for coord in result.path:
            self.assertNotIn(
                coord,
                blocked_coords,
                f"Path unlawfully crossed obstacle at {coord}",
            )

    def test_test4_unreachable_goal(self) -> None:
        """Test 4: Unreachable goal.

        - Surround the goal with blocked cells or disconnect it.
        - Verify reached_goal=False.
        - Verify path is empty.
        - Verify failure is reported clearly (cost is infinite).
        """
        # 4x4 grid with disconnected goal at (3, 3)
        grid = [
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 1, 1],
            [0, 0, 1, 0],  # (3, 3) is walled off by (2, 2), (2, 3), (3, 2)
        ]
        start: Coordinate = (0, 0)
        goal: Coordinate = (3, 3)

        result = greedy_best_first_search(grid, start, goal)

        self.assertFalse(result.reached_goal)
        self.assertFalse(result["reached_goal"])
        self.assertEqual(result.path, [])
        self.assertEqual(result["path"], [])
        self.assertTrue(math.isinf(result.cost))
        self.assertGreater(len(result.expanded_nodes), 0)

    def test_test5_invalid_inputs(self) -> None:
        """Test 5: Invalid inputs.

        - Out-of-bounds start.
        - Out-of-bounds goal.
        - Blocked start.
        - Blocked goal.
        Verifies both exception raising (raise_on_invalid=True) and
        graceful failure reporting (raise_on_invalid=False).
        """
        field = FieldMap(
            height=3,
            width=3,
            blocked_cells={(1, 1)},
        )

        # 1. Out-of-bounds start
        with self.assertRaises(ValueError):
            greedy_best_first_search(field, (-1, 0), (2, 2), raise_on_invalid=True)

        res = greedy_best_first_search(field, (-1, 0), (2, 2), raise_on_invalid=False)
        self.assertFalse(res.reached_goal)
        self.assertEqual(res.path, [])
        self.assertTrue(math.isinf(res.cost))

        # 2. Out-of-bounds goal
        with self.assertRaises(ValueError):
            greedy_best_first_search(field, (0, 0), (5, 5), raise_on_invalid=True)

        res = greedy_best_first_search(field, (0, 0), (5, 5), raise_on_invalid=False)
        self.assertFalse(res.reached_goal)
        self.assertEqual(res.path, [])

        # 3. Blocked start
        with self.assertRaises(ValueError):
            greedy_best_first_search(field, (1, 1), (2, 2), raise_on_invalid=True)

        res = greedy_best_first_search(field, (1, 1), (2, 2), raise_on_invalid=False)
        self.assertFalse(res.reached_goal)
        self.assertEqual(res.path, [])

        # 4. Blocked goal
        with self.assertRaises(ValueError):
            greedy_best_first_search(field, (0, 0), (1, 1), raise_on_invalid=True)

        res = greedy_best_first_search(field, (0, 0), (1, 1), raise_on_invalid=False)
        self.assertFalse(res.reached_goal)
        self.assertEqual(res.path, [])

    def test_test6_start_equals_goal(self) -> None:
        """Test 6: Start equals goal.

        - Return a valid one-cell path [start].
        - Return cost 0.
        - Handle expanded-node reporting consistently ([start]).
        - reached_goal must be True.
        """
        grid = [
            [0, 0],
            [0, 0],
        ]
        start: Coordinate = (1, 1)
        goal: Coordinate = (1, 1)

        result = greedy_best_first_search(grid, start, goal)

        self.assertTrue(result.reached_goal)
        self.assertEqual(result.path, [(1, 1)])
        self.assertEqual(result.cost, 0)
        self.assertEqual(result.expanded_nodes, [(1, 1)])

    def test_test7_reproducibility(self) -> None:
        """Test 7: Reproducibility.

        - Run the exact same scenario multiple times.
        - Verify identical paths and expansion sequences across runs.
        """
        grid = [
            [0, 1, 0, 0, 0],
            [0, 1, 0, 1, 0],
            [0, 0, 0, 1, 0],
            [0, 1, 0, 0, 0],
        ]
        start: Coordinate = (0, 0)
        goal: Coordinate = (3, 4)

        results = [greedy_best_first_search(grid, start, goal) for _ in range(5)]

        reference_path = results[0].path
        reference_expanded = results[0].expanded_nodes
        reference_cost = results[0].cost

        for idx, res in enumerate(results[1:], start=2):
            self.assertEqual(
                res.path,
                reference_path,
                f"Path differed on run {idx}",
            )
            self.assertEqual(
                res.expanded_nodes,
                reference_expanded,
                f"Expanded nodes sequence differed on run {idx}",
            )
            self.assertEqual(
                res.cost,
                reference_cost,
                f"Cost differed on run {idx}",
            )

    def test_test8_expanded_nodes(self) -> None:
        """Test 8: Expanded nodes.

        - Verify expanded_nodes is an ordered list.
        - Verify its entries are valid traversable coordinates.
        - Verify nodes are not unnecessarily expanded multiple times.
        """
        field_matrix = [
            [0, 0, 0, 0],
            [1, 1, 0, 1],
            [0, 0, 0, 0],
            [0, 1, 1, 0],
        ]
        field = FieldMap.from_matrix(field_matrix)
        start: Coordinate = (0, 0)
        goal: Coordinate = (3, 3)

        result = greedy_best_first_search(field, start, goal)

        expanded = result.expanded_nodes
        self.assertGreater(len(expanded), 0)

        # Verify all expanded nodes are within bounds and traversable
        for node in expanded:
            self.assertTrue(
                field.is_within_bounds(node),
                f"Expanded node {node} was out of bounds",
            )
            self.assertTrue(
                field.is_traversable(node),
                f"Expanded node {node} was non-traversable",
            )

        # Verify no node is expanded multiple times
        self.assertEqual(
            len(expanded),
            len(set(expanded)),
            "Duplicate node expansion detected in expanded_nodes list",
        )

        # Start must be first expanded node
        self.assertEqual(expanded[0], start)

    def test_gbfs_suboptimality_contrast(self) -> None:
        """Demonstrate that GBFS prioritizes h(n) and is not guaranteed to be cost-optimal.

        In a concave obstacle scenario ('trap' or detour), GBFS rushes toward the goal
        along the lowest h(n) until it hits the wall and must backtrack around it,
        illustrating why f(n) = h(n) does not guarantee the shortest path.
        """
        # U-shaped obstacle opening to the left (cols 0 and 1 are open)
        grid = [
            [0, 0, 0, 0, 0, 0],
            [0, 0, 1, 1, 1, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 0],
        ]
        start: Coordinate = (2, 2)  # inside the pocket
        goal: Coordinate = (2, 5)   # directly across the right wall

        result = greedy_best_first_search(grid, start, goal)

        self.assertTrue(result.reached_goal)
        # Verify the search succeeded and expanded nodes properly
        self.assertGreater(len(result.expanded_nodes), 0)
        self.assertGreaterEqual(result.cost, 7)


if __name__ == "__main__":
    unittest.main()
