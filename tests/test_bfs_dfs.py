"""Tests for BFS and DFS implementations (Dushyant — Member 2).

Coverage
--------
Test 1  – Shared result-format contract (dict subclass, required keys, value types)
Test 2  – Simple reachable grid (open field, no obstacles)
Test 3  – Obstacle avoidance (vertical wall with one gap)
Test 4  – Unreachable goal (goal enclosed by obstacles)
Test 5  – Invalid inputs (out-of-bounds, blocked start/goal)
Test 6  – Start equals goal (zero-length path edge case)
Test 7  – Reproducibility (identical results across multiple runs)
Test 8  – Expanded-nodes contract (ordered list, in-bounds, not duplicated)
Test 9  – BFS optimality (shortest-step-count guarantee)
Test 10 – DFS non-optimality demonstration (may return a longer path than BFS)
Test 11 – No field mutation (FieldMap.blocked_cells unchanged after search)
Test 12 – Path traceability (every step in path is a legal single-cell move)
Test 13 – Custom cost function (optional cost_fn parameter)
Test 14 – Demo field scenario with the shared Field model (reachable)
Test 15 – Demo field scenario: fully enclosed unreachable goal
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import unittest

# Allow direct invocation without installing the package
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from cropsafe_pathplanning.bfs_dfs import breadth_first_search, depth_first_search
from cropsafe_pathplanning.types import Coordinate, FieldMap, SearchResult


# ---------------------------------------------------------------------------
# Shared helper grids
# ---------------------------------------------------------------------------

OPEN_4x4 = [
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
]

# Vertical wall with single gap at row 3
WALL_5x5 = [
    [0, 0, 1, 0, 0],
    [0, 0, 1, 0, 0],
    [0, 0, 1, 0, 0],
    [0, 0, 0, 0, 0],  # gap
    [0, 0, 1, 0, 0],
]

# Goal (3,3) is fully enclosed – unreachable
ENCLOSED_4x4 = [
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 1, 1],
    [0, 0, 1, 0],
]

# 5×5 open grid for optimality comparison
OPEN_5x5 = [
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
]


def _is_valid_cardinal_path(path: list[Coordinate]) -> bool:
    """Return True iff every consecutive pair in *path* is a legal 4-directional step."""
    for i in range(len(path) - 1):
        r1, c1 = path[i]
        r2, c2 = path[i + 1]
        if abs(r1 - r2) + abs(c1 - c2) != 1:
            return False
    return True


# ---------------------------------------------------------------------------
# BFS Tests
# ---------------------------------------------------------------------------

class TestBreadthFirstSearch(unittest.TestCase):

    # ------------------------------------------------------------------
    # Test 1 – Shared result-format contract
    # ------------------------------------------------------------------
    def test_01_result_format(self) -> None:
        """SearchResult is a dict subclass with required keys and correct types."""
        result = breadth_first_search(OPEN_4x4, (0, 0), (3, 3))

        self.assertIsInstance(result, dict, "SearchResult must inherit from dict")
        for key in ("path", "cost", "expanded_nodes", "reached_goal"):
            self.assertIn(key, result, f"Missing required key '{key}'")

        self.assertIsInstance(result["path"], list)
        self.assertIsInstance(result["cost"], (int, float))
        self.assertIsInstance(result["expanded_nodes"], list)
        self.assertIsInstance(result["reached_goal"], bool)

        # Property accessors must mirror dict values
        self.assertEqual(result.path, result["path"])
        self.assertEqual(result.cost, result["cost"])
        self.assertEqual(result.expanded_nodes, result["expanded_nodes"])
        self.assertEqual(result.reached_goal, result["reached_goal"])

    # ------------------------------------------------------------------
    # Test 2 – Simple reachable grid
    # ------------------------------------------------------------------
    def test_02_simple_reachable(self) -> None:
        """BFS finds a path on an obstacle-free grid."""
        start: Coordinate = (0, 0)
        goal: Coordinate = (3, 3)
        result = breadth_first_search(OPEN_4x4, start, goal)

        self.assertTrue(result.reached_goal)
        self.assertGreater(len(result.path), 0)
        self.assertEqual(result.path[0], start)
        self.assertEqual(result.path[-1], goal)
        self.assertTrue(_is_valid_cardinal_path(result.path))

    # ------------------------------------------------------------------
    # Test 3 – Obstacle avoidance
    # ------------------------------------------------------------------
    def test_03_obstacle_avoidance(self) -> None:
        """BFS navigates around a vertical wall."""
        start: Coordinate = (1, 1)
        goal: Coordinate = (1, 4)
        result = breadth_first_search(WALL_5x5, start, goal)

        self.assertTrue(result.reached_goal)
        self.assertEqual(result.path[0], start)
        self.assertEqual(result.path[-1], goal)

        field = FieldMap.from_matrix(WALL_5x5)
        for coord in result.path:
            self.assertFalse(
                field.is_blocked(coord),
                f"BFS path illegally crossed blocked cell {coord}",
            )
        self.assertTrue(_is_valid_cardinal_path(result.path))

    # ------------------------------------------------------------------
    # Test 4 – Unreachable goal
    # ------------------------------------------------------------------
    def test_04_unreachable_goal(self) -> None:
        """BFS returns failure when goal is enclosed by obstacles."""
        start: Coordinate = (0, 0)
        goal: Coordinate = (3, 3)
        result = breadth_first_search(ENCLOSED_4x4, start, goal)

        self.assertFalse(result.reached_goal)
        self.assertEqual(result.path, [])
        self.assertTrue(math.isinf(result.cost))
        # Expanded nodes should be non-empty (search was attempted)
        self.assertGreater(len(result.expanded_nodes), 0)

    # ------------------------------------------------------------------
    # Test 5 – Invalid inputs
    # ------------------------------------------------------------------
    def test_05_invalid_inputs(self) -> None:
        """BFS handles out-of-bounds and blocked endpoints gracefully."""
        field = FieldMap(height=4, width=4, blocked_cells={(2, 2)})

        # Out-of-bounds start (raise_on_invalid=True)
        with self.assertRaises((ValueError, TypeError)):
            breadth_first_search(field, (-1, 0), (3, 3), raise_on_invalid=True)

        # Out-of-bounds start (raise_on_invalid=False)
        res = breadth_first_search(field, (-1, 0), (3, 3), raise_on_invalid=False)
        self.assertFalse(res.reached_goal)
        self.assertEqual(res.path, [])

        # Out-of-bounds goal
        res = breadth_first_search(field, (0, 0), (10, 10), raise_on_invalid=False)
        self.assertFalse(res.reached_goal)

        # Blocked start
        with self.assertRaises((ValueError, TypeError)):
            breadth_first_search(field, (2, 2), (3, 3), raise_on_invalid=True)
        res = breadth_first_search(field, (2, 2), (3, 3), raise_on_invalid=False)
        self.assertFalse(res.reached_goal)

        # Blocked goal
        with self.assertRaises((ValueError, TypeError)):
            breadth_first_search(field, (0, 0), (2, 2), raise_on_invalid=True)
        res = breadth_first_search(field, (0, 0), (2, 2), raise_on_invalid=False)
        self.assertFalse(res.reached_goal)

    # ------------------------------------------------------------------
    # Test 6 – Start equals goal
    # ------------------------------------------------------------------
    def test_06_start_equals_goal(self) -> None:
        """BFS returns a single-cell path with cost 0 when start == goal."""
        pt: Coordinate = (2, 2)
        result = breadth_first_search(OPEN_4x4, pt, pt)

        self.assertTrue(result.reached_goal)
        self.assertEqual(result.path, [pt])
        self.assertEqual(result.cost, 0)

    # ------------------------------------------------------------------
    # Test 7 – Reproducibility
    # ------------------------------------------------------------------
    def test_07_reproducibility(self) -> None:
        """Multiple identical BFS calls must produce identical results."""
        start: Coordinate = (0, 0)
        goal: Coordinate = (3, 3)
        results = [breadth_first_search(OPEN_4x4, start, goal) for _ in range(5)]
        ref = results[0]
        for i, res in enumerate(results[1:], start=2):
            self.assertEqual(res.path, ref.path, f"Path differed on run {i}")
            self.assertEqual(res.cost, ref.cost, f"Cost differed on run {i}")
            self.assertEqual(
                res.expanded_nodes, ref.expanded_nodes,
                f"Expanded nodes differed on run {i}",
            )

    # ------------------------------------------------------------------
    # Test 8 – Expanded-nodes contract
    # ------------------------------------------------------------------
    def test_08_expanded_nodes_contract(self) -> None:
        """expanded_nodes is ordered, in-bounds, traversable, and duplicate-free."""
        field = FieldMap.from_matrix(WALL_5x5)
        result = breadth_first_search(field, (0, 0), (0, 4))

        expanded = result.expanded_nodes
        self.assertGreater(len(expanded), 0)

        # First expanded node must be start
        self.assertEqual(expanded[0], (0, 0))

        seen: set[Coordinate] = set()
        for node in expanded:
            self.assertTrue(field.is_within_bounds(node), f"{node} out of bounds")
            self.assertTrue(field.is_traversable(node), f"{node} is blocked")
            self.assertNotIn(node, seen, f"Duplicate expansion of {node}")
            seen.add(node)

    # ------------------------------------------------------------------
    # Test 9 – BFS optimality (shortest-step-count guarantee)
    # ------------------------------------------------------------------
    def test_09_bfs_shortest_path(self) -> None:
        """BFS path length equals the Manhattan distance on an open grid."""
        start: Coordinate = (0, 0)
        goal: Coordinate = (4, 4)
        result = breadth_first_search(OPEN_5x5, start, goal)

        self.assertTrue(result.reached_goal)
        expected_steps = abs(goal[0] - start[0]) + abs(goal[1] - start[1])  # = 8
        # Path list has len = steps + 1 (includes start)
        self.assertEqual(len(result.path) - 1, expected_steps)
        self.assertEqual(result.cost, expected_steps)

    # ------------------------------------------------------------------
    # Test 12 – Path traceability (cardinal steps only)
    # ------------------------------------------------------------------
    def test_12_path_traceability(self) -> None:
        """Every step in BFS path is a legal single-cell cardinal move."""
        result = breadth_first_search(WALL_5x5, (0, 0), (4, 4))
        self.assertTrue(result.reached_goal)
        self.assertTrue(
            _is_valid_cardinal_path(result.path),
            "BFS path contains a non-cardinal or multi-cell jump",
        )

    # ------------------------------------------------------------------
    # Test 13 – Custom cost function
    # ------------------------------------------------------------------
    def test_13_custom_cost_fn(self) -> None:
        """BFS accepts a custom cost_fn and uses it for cost calculation."""
        # Each step costs 2 instead of 1
        result = breadth_first_search(
            OPEN_4x4, (0, 0), (3, 3),
            cost_fn=lambda a, b: 2.0,
        )
        self.assertTrue(result.reached_goal)
        expected_steps = 6  # Manhattan distance on 4×4 from (0,0) to (3,3)
        self.assertAlmostEqual(result.cost, expected_steps * 2.0)

    # ------------------------------------------------------------------
    # Test 11 – No field mutation
    # ------------------------------------------------------------------
    def test_11_no_field_mutation(self) -> None:
        """BFS must not alter the FieldMap's blocked_cells."""
        field = FieldMap.from_matrix(WALL_5x5)
        original_blocked = set(field.blocked_cells)
        breadth_first_search(field, (0, 0), (0, 4))
        self.assertEqual(
            field.blocked_cells,
            original_blocked,
            "BFS mutated FieldMap.blocked_cells",
        )


# ---------------------------------------------------------------------------
# DFS Tests
# ---------------------------------------------------------------------------

class TestDepthFirstSearch(unittest.TestCase):

    # ------------------------------------------------------------------
    # Test 1 – Result format
    # ------------------------------------------------------------------
    def test_01_result_format(self) -> None:
        """DFS SearchResult conforms to the shared dict interface."""
        result = depth_first_search(OPEN_4x4, (0, 0), (3, 3))

        self.assertIsInstance(result, dict)
        for key in ("path", "cost", "expanded_nodes", "reached_goal"):
            self.assertIn(key, result)

        self.assertIsInstance(result["path"], list)
        self.assertIsInstance(result["cost"], (int, float))
        self.assertIsInstance(result["expanded_nodes"], list)
        self.assertIsInstance(result["reached_goal"], bool)

    # ------------------------------------------------------------------
    # Test 2 – Simple reachable grid
    # ------------------------------------------------------------------
    def test_02_simple_reachable(self) -> None:
        """DFS finds a path on an obstacle-free grid."""
        start: Coordinate = (0, 0)
        goal: Coordinate = (3, 3)
        result = depth_first_search(OPEN_4x4, start, goal)

        self.assertTrue(result.reached_goal)
        self.assertEqual(result.path[0], start)
        self.assertEqual(result.path[-1], goal)
        self.assertTrue(_is_valid_cardinal_path(result.path))

    # ------------------------------------------------------------------
    # Test 3 – Obstacle avoidance
    # ------------------------------------------------------------------
    def test_03_obstacle_avoidance(self) -> None:
        """DFS path never passes through a blocked cell."""
        start: Coordinate = (0, 0)
        goal: Coordinate = (4, 4)
        result = depth_first_search(WALL_5x5, start, goal)

        self.assertTrue(result.reached_goal)
        field = FieldMap.from_matrix(WALL_5x5)
        for coord in result.path:
            self.assertFalse(
                field.is_blocked(coord),
                f"DFS path illegally entered blocked cell {coord}",
            )
        self.assertTrue(_is_valid_cardinal_path(result.path))

    # ------------------------------------------------------------------
    # Test 4 – Unreachable goal
    # ------------------------------------------------------------------
    def test_04_unreachable_goal(self) -> None:
        """DFS returns failure when goal is enclosed by obstacles."""
        result = depth_first_search(ENCLOSED_4x4, (0, 0), (3, 3))

        self.assertFalse(result.reached_goal)
        self.assertEqual(result.path, [])
        self.assertTrue(math.isinf(result.cost))
        self.assertGreater(len(result.expanded_nodes), 0)

    # ------------------------------------------------------------------
    # Test 5 – Invalid inputs
    # ------------------------------------------------------------------
    def test_05_invalid_inputs(self) -> None:
        """DFS handles invalid endpoints identically to BFS."""
        field = FieldMap(height=4, width=4, blocked_cells={(2, 2)})

        with self.assertRaises((ValueError, TypeError)):
            depth_first_search(field, (-1, 0), (3, 3), raise_on_invalid=True)
        res = depth_first_search(field, (-1, 0), (3, 3), raise_on_invalid=False)
        self.assertFalse(res.reached_goal)

        res = depth_first_search(field, (0, 0), (10, 10), raise_on_invalid=False)
        self.assertFalse(res.reached_goal)

        with self.assertRaises((ValueError, TypeError)):
            depth_first_search(field, (2, 2), (3, 3), raise_on_invalid=True)
        with self.assertRaises((ValueError, TypeError)):
            depth_first_search(field, (0, 0), (2, 2), raise_on_invalid=True)

    # ------------------------------------------------------------------
    # Test 6 – Start equals goal
    # ------------------------------------------------------------------
    def test_06_start_equals_goal(self) -> None:
        """DFS returns a single-cell path with cost 0 when start == goal."""
        pt: Coordinate = (1, 1)
        result = depth_first_search(OPEN_4x4, pt, pt)

        self.assertTrue(result.reached_goal)
        self.assertEqual(result.path, [pt])
        self.assertEqual(result.cost, 0)

    # ------------------------------------------------------------------
    # Test 7 – Reproducibility
    # ------------------------------------------------------------------
    def test_07_reproducibility(self) -> None:
        """Multiple DFS runs on identical inputs produce identical results."""
        start: Coordinate = (0, 0)
        goal: Coordinate = (3, 3)
        results = [depth_first_search(OPEN_4x4, start, goal) for _ in range(5)]
        ref = results[0]
        for i, res in enumerate(results[1:], start=2):
            self.assertEqual(res.path, ref.path, f"DFS path differed on run {i}")
            self.assertEqual(res.cost, ref.cost, f"DFS cost differed on run {i}")

    # ------------------------------------------------------------------
    # Test 8 – Expanded-nodes contract
    # ------------------------------------------------------------------
    def test_08_expanded_nodes_contract(self) -> None:
        """DFS expanded_nodes is in-bounds, traversable, and duplicate-free."""
        field = FieldMap.from_matrix(OPEN_4x4)
        result = depth_first_search(field, (0, 0), (3, 3))

        self.assertEqual(result.expanded_nodes[0], (0, 0))
        seen: set[Coordinate] = set()
        for node in result.expanded_nodes:
            self.assertTrue(field.is_within_bounds(node))
            self.assertTrue(field.is_traversable(node))
            self.assertNotIn(node, seen, f"Duplicate DFS expansion of {node}")
            seen.add(node)

    # ------------------------------------------------------------------
    # Test 10 – DFS non-optimality demonstration
    # ------------------------------------------------------------------
    def test_10_dfs_non_optimality(self) -> None:
        """DFS path may be longer than BFS's shortest path.

        This demonstrates the key difference between the two algorithms:
        BFS guarantees the shortest step count; DFS does not.
        On an open 5×5 grid from (0,0) to (4,4) the shortest path is
        8 steps.  DFS may return a longer route.
        """
        start: Coordinate = (0, 0)
        goal: Coordinate = (4, 4)
        optimal_steps = 8  # Manhattan distance

        bfs_result = breadth_first_search(OPEN_5x5, start, goal)
        dfs_result = depth_first_search(OPEN_5x5, start, goal)

        self.assertTrue(bfs_result.reached_goal)
        self.assertTrue(dfs_result.reached_goal)

        # BFS must be optimal
        self.assertEqual(len(bfs_result.path) - 1, optimal_steps,
                         "BFS did not return the shortest path")

        # DFS path must be valid but is allowed to be longer
        self.assertTrue(_is_valid_cardinal_path(dfs_result.path))
        self.assertGreaterEqual(len(dfs_result.path) - 1, optimal_steps,
                                "DFS path was shorter than the Manhattan bound – impossible")

    # ------------------------------------------------------------------
    # Test 11 – No field mutation
    # ------------------------------------------------------------------
    def test_11_no_field_mutation(self) -> None:
        """DFS must not alter the FieldMap's blocked_cells."""
        field = FieldMap.from_matrix(WALL_5x5)
        original_blocked = set(field.blocked_cells)
        depth_first_search(field, (0, 0), (4, 4))
        self.assertEqual(
            field.blocked_cells,
            original_blocked,
            "DFS mutated FieldMap.blocked_cells",
        )

    # ------------------------------------------------------------------
    # Test 12 – Path traceability
    # ------------------------------------------------------------------
    def test_12_path_traceability(self) -> None:
        """Every consecutive pair in the DFS path is a legal 4-directional step."""
        result = depth_first_search(WALL_5x5, (0, 0), (4, 4))
        self.assertTrue(result.reached_goal)
        self.assertTrue(_is_valid_cardinal_path(result.path))


# ---------------------------------------------------------------------------
# Cross-algorithm integration Tests
# ---------------------------------------------------------------------------

class TestBFSDFSIntegration(unittest.TestCase):

    def test_14_demo_field_reachable(self) -> None:
        """BFS and DFS both find a path on the shared demo Field scenario.

        Uses the Field model from cropsafe_pathplanning.model so that the
        same scenario Hitesh's model defines is exercised here.
        """
        from cropsafe_pathplanning.model import demo_field

        demo = demo_field()
        # Build a FieldMap compatible with the shared bfs_dfs interface
        field_map = FieldMap(
            height=demo.height,
            width=demo.width,
            blocked_cells=set(demo.blocked),
        )
        start = demo.start
        goal = demo.goal

        bfs_res = breadth_first_search(field_map, start, goal)
        dfs_res = depth_first_search(field_map, start, goal)

        for label, res in [("BFS", bfs_res), ("DFS", dfs_res)]:
            self.assertTrue(res.reached_goal, f"{label} did not reach goal on demo field")
            self.assertEqual(res.path[0], start, f"{label} path does not start at start")
            self.assertEqual(res.path[-1], goal, f"{label} path does not end at goal")
            self.assertTrue(
                _is_valid_cardinal_path(res.path),
                f"{label} path contains a non-cardinal step",
            )
            # Verify no step enters a blocked cell
            for coord in res.path:
                self.assertFalse(
                    field_map.is_blocked(coord),
                    f"{label} path entered blocked cell {coord}",
                )

        # BFS must not return a longer-than-optimal step count
        self.assertLessEqual(
            len(bfs_res.path),
            len(dfs_res.path),
            "BFS returned more steps than DFS on demo field — unexpected",
        )

    def test_15_demo_field_unreachable(self) -> None:
        """BFS and DFS both report failure when goal is completely walled off.

        Builds a small field where goal is enclosed by blocked cells and
        verifies both algorithms detect the unreachable state correctly.
        """
        # 5×5 grid: goal at (4,4) is blocked on three sides, plus a full
        # row of blocked cells sealing it off
        field_map = FieldMap(
            height=5,
            width=5,
            blocked_cells={(3, 3), (3, 4), (4, 3)},  # walls around (4,4)
        )
        start: Coordinate = (0, 0)
        goal: Coordinate = (4, 4)

        for label, fn in [("BFS", breadth_first_search), ("DFS", depth_first_search)]:
            res = fn(field_map, start, goal)
            self.assertFalse(res.reached_goal, f"{label} wrongly reported goal reached")
            self.assertEqual(res.path, [], f"{label} returned non-empty path for unreachable goal")
            self.assertTrue(math.isinf(res.cost), f"{label} cost should be inf for unreachable goal")


if __name__ == "__main__":
    unittest.main(verbosity=2)

