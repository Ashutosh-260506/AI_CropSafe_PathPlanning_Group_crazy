"""Comprehensive Unit Tests for A* Search and Crop-Safe Cost Model.

Author: Ashutosh (Member 4)
Coverage:
- Shared Result Contract & dict-interface compatibility
- Dual-nature ExpandedNodeList (collection + integer count)
- Goal reaching on open and obstructed fields
- Obstacle and protected crop avoidance
- Crop-safe route cost model and safety penalty tradeoff
- Start equals goal edge case
- Invalid endpoints (out of bounds, blocked endpoints, type validation)
- Determinism and reproducibility
- Path continuity and 4-directional movement validity
- Field immutability (no mutation of blocked cells)
- Standalone cost functions (crop_safe_step_cost, make_crop_safe_cost_fn)
- Suboptimality contrast: A* cost-optimality vs Greedy Best-First Search
- Interoperability with both model.Field and types.FieldMap
"""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

# Ensure src is on sys.path for direct test invocation
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from cropsafe_pathplanning.astar import (
    ExpandedNodeList,
    SearchResult,
    a_star_search,
    crop_safe_step_cost,
    is_adjacent_to_protected,
    make_crop_safe_cost_fn,
)
from cropsafe_pathplanning.gbfs import greedy_best_first_search
from cropsafe_pathplanning.heuristic import manhattan_distance
from cropsafe_pathplanning.model import Field, demo_field
from cropsafe_pathplanning.types import Coordinate, FieldMap


def _is_valid_cardinal_path(path: list[Coordinate]) -> bool:
    """Return True iff every consecutive pair in path is a valid 4-directional step."""
    for i in range(len(path) - 1):
        r1, c1 = path[i]
        r2, c2 = path[i + 1]
        if abs(r1 - r2) + abs(c1 - c2) != 1:
            return False
    return True


class TestAStar(unittest.TestCase):
    """Test suite for A* search and the crop-safe cost model."""

    # -----------------------------------------------------------------------
    # Baseline tests (preserved from initial commit)
    # -----------------------------------------------------------------------

    def test_astar_reaches_goal(self) -> None:
        """A* successfully finds a path from start to goal on an open field."""
        field = Field(
            height=5,
            width=7,
            start=(2, 0),
            goal=(2, 6),
        )
        result = a_star_search(field, heuristic_fn=manhattan_distance)

        self.assertTrue(result.reached_goal)
        self.assertEqual(result.path[0], field.start)
        self.assertEqual(result.path[-1], field.goal)
        self.assertEqual(result.path_length, 6)

    def test_astar_avoids_blocked_cells(self) -> None:
        """A* strictly avoids obstacles and protected crop cells."""
        field = Field(
            height=5,
            width=7,
            start=(2, 0),
            goal=(2, 6),
            protected={(2, 3)},
            obstacles={(1, 2), (3, 2)},
        )
        result = a_star_search(field)

        self.assertTrue(result.reached_goal)
        self.assertTrue(set(result.path).isdisjoint(field.blocked))
        self.assertTrue(_is_valid_cardinal_path(result.path))

    def test_safety_penalty_changes_route(self) -> None:
        """Verify that increasing crop proximity penalty diverts A* to a safer route."""
        low_risk_field = Field(
            height=5,
            width=7,
            start=(2, 0),
            goal=(2, 6),
            protected={(2, 3)},
            risk_penalty=0.0,
        )

        high_risk_field = Field(
            height=5,
            width=7,
            start=(2, 0),
            goal=(2, 6),
            protected={(2, 3)},
            risk_penalty=3.0,
        )

        low_result = a_star_search(low_risk_field)
        high_result = a_star_search(high_risk_field)

        self.assertTrue(low_result.reached_goal)
        self.assertTrue(high_result.reached_goal)

        # Low risk path skirts near the protected crop (path length 8, cost 8)
        self.assertEqual(low_result.path_length, 8)
        self.assertEqual(low_result.cost, 8.0)

        # High risk path takes a wider berth around the crop (path length 10, cost 10)
        self.assertEqual(high_result.path_length, 10)
        self.assertEqual(high_result.cost, 10.0)

    def test_astar_reports_no_path(self) -> None:
        """A* reports failure with empty path and infinite cost when goal is sealed off."""
        field = Field(
            height=3,
            width=7,
            start=(1, 0),
            goal=(1, 6),
            protected={(1, 3)},
            obstacles={(0, 3), (2, 3)},
        )
        result = a_star_search(field)

        self.assertFalse(result.reached_goal)
        self.assertEqual(result.path, [])
        self.assertEqual(result.path_length, 0)
        self.assertTrue(math.isinf(result.cost))

    def test_astar_path_cost(self) -> None:
        """Reported path cost matches step cost summation."""
        field = Field(
            height=5,
            width=7,
            start=(2, 0),
            goal=(2, 6),
            protected={(2, 3)},
            risk_penalty=3.0,
        )
        result = a_star_search(field)

        expected_cost = sum(
            field.step_cost(cell)
            for cell in result.path[1:]
        )
        self.assertAlmostEqual(result.cost, expected_cost)

    # -----------------------------------------------------------------------
    # Shared Result Contract & ExpandedNodeList tests
    # -----------------------------------------------------------------------

    def test_search_result_shared_dict_contract(self) -> None:
        """SearchResult conforms to the project's shared dict schema and properties."""
        field = Field(height=4, width=4, start=(0, 0), goal=(3, 3))
        result = a_star_search(field)

        self.assertIsInstance(result, SearchResult)

        # Dict keys check
        for key in ("path", "cost", "expanded_nodes", "reached_goal"):
            self.assertIn(key, result)

        # Types check
        self.assertIsInstance(result["path"], list)
        self.assertIsInstance(result["cost"], (int, float))
        self.assertIsInstance(result["expanded_nodes"], list)
        self.assertIsInstance(result["reached_goal"], bool)

        # Properties check
        self.assertEqual(result.path, result["path"])
        self.assertEqual(result.cost, result["cost"])
        self.assertEqual(result.expanded_nodes, result["expanded_nodes"])
        self.assertEqual(result.reached_goal, result["reached_goal"])
        self.assertEqual(result.algorithm, "A*")
        self.assertGreaterEqual(result.elapsed_seconds, 0.0)

    def test_expanded_nodes_dual_nature(self) -> None:
        """expanded_nodes functions both as a list of coordinates and as an integer count."""
        field = Field(height=4, width=4, start=(0, 0), goal=(3, 3))
        result = a_star_search(field)

        expanded = result.expanded_nodes
        # Behaves as list and ExpandedNodeList
        self.assertIsInstance(expanded, list)
        self.assertIsInstance(expanded, ExpandedNodeList)
        self.assertGreater(len(expanded), 0)
        self.assertIsInstance(expanded[0], tuple)
        self.assertEqual(len(expanded[0]), 2)

        # Behaves as int for evaluation runners (expanded_total += result.expanded_nodes)
        total = 10
        total += expanded
        self.assertEqual(total, 10 + len(expanded))

        # Int conversion and string formatting
        self.assertEqual(int(expanded), len(expanded))
        formatted = f"{expanded:>6}"
        self.assertEqual(formatted, f"{len(expanded):>6}")

    # -----------------------------------------------------------------------
    # Edge Cases & Validation
    # -----------------------------------------------------------------------

    def test_start_equals_goal(self) -> None:
        """When start == goal, A* returns a 1-node path with cost 0 and expanded_nodes=[start]."""
        field_map = FieldMap(height=5, width=5)
        result = a_star_search(field_map, start=(2, 2), goal=(2, 2))

        self.assertTrue(result.reached_goal)
        self.assertEqual(result.path, [(2, 2)])
        self.assertEqual(result.cost, 0.0)
        self.assertEqual(result.path_length, 0)
        self.assertEqual(list(result.expanded_nodes), [(2, 2)])

    def test_invalid_endpoints_graceful_handling(self) -> None:
        """Invalid or out-of-bounds endpoints return failure when raise_on_invalid=False."""
        field_map = FieldMap(height=5, width=5, blocked_cells={(0, 0)})

        # Blocked start
        res_blocked_start = a_star_search(field_map, start=(0, 0), goal=(4, 4), raise_on_invalid=False)
        self.assertFalse(res_blocked_start.reached_goal)
        self.assertEqual(res_blocked_start.path, [])
        self.assertTrue(math.isinf(res_blocked_start.cost))

        # Out-of-bounds goal
        res_oob_goal = a_star_search(field_map, start=(1, 1), goal=(10, 10), raise_on_invalid=False)
        self.assertFalse(res_oob_goal.reached_goal)

    def test_invalid_endpoints_raise_exceptions(self) -> None:
        """Invalid endpoints raise ValueError or TypeError when raise_on_invalid=True."""
        field_map = FieldMap(height=5, width=5, blocked_cells={(1, 1)})

        with self.assertRaises(ValueError):
            a_star_search(field_map, start=(1, 1), goal=(4, 4), raise_on_invalid=True)

        with self.assertRaises(ValueError):
            a_star_search(field_map, start=(0, 0), goal=(8, 8), raise_on_invalid=True)

        with self.assertRaises(TypeError):
            a_star_search(field_map, start="invalid", goal=(4, 4), raise_on_invalid=True)  # type: ignore

    # -----------------------------------------------------------------------
    # Reproducibility & Field Immutability
    # -----------------------------------------------------------------------

    def test_reproducibility(self) -> None:
        """Multiple runs with identical inputs produce identical paths and node expansions."""
        field = demo_field()
        run1 = a_star_search(field)
        run2 = a_star_search(field)

        self.assertEqual(run1.path, run2.path)
        self.assertEqual(run1.cost, run2.cost)
        self.assertEqual(list(run1.expanded_nodes), list(run2.expanded_nodes))

    def test_no_field_mutation(self) -> None:
        """Search execution must not mutate blocked cells on FieldMap."""
        original_blocked = {(1, 1), (2, 2)}
        field_map = FieldMap(height=5, width=5, blocked_cells=original_blocked)

        a_star_search(field_map, start=(0, 0), goal=(4, 4))
        self.assertEqual(field_map.blocked_cells, original_blocked)

    def test_path_traceability_and_bounds(self) -> None:
        """Every path step must be within field bounds and a cardinal move from previous."""
        field = demo_field()
        result = a_star_search(field)

        self.assertTrue(result.reached_goal)
        self.assertTrue(_is_valid_cardinal_path(result.path))
        for coord in result.path:
            self.assertTrue(field.in_bounds(coord))
            self.assertNotIn(coord, field.blocked)

    # -----------------------------------------------------------------------
    # Crop-Safe Route Cost Model Standalone Tests
    # -----------------------------------------------------------------------

    def test_crop_safe_cost_functions_standalone(self) -> None:
        """Standalone cost function accurately assesses adjacency and charges risk surcharge."""
        protected = {(2, 2)}

        # Adjacent cells (up, down, left, right)
        adjacent_cells = [(1, 2), (3, 2), (2, 1), (2, 3)]
        for cell in adjacent_cells:
            self.assertTrue(is_adjacent_to_protected(cell, protected))
            cost = crop_safe_step_cost(cell, protected, risk_penalty=3.0, base_cost=1.0)
            self.assertEqual(cost, 4.0)

        # Diagonal or distant cells are not 4-directionally adjacent
        non_adjacent = [(1, 1), (3, 3), (0, 0), (2, 4)]
        for cell in non_adjacent:
            self.assertFalse(is_adjacent_to_protected(cell, protected))
            cost = crop_safe_step_cost(cell, protected, risk_penalty=3.0, base_cost=1.0)
            self.assertEqual(cost, 1.0)

        # Entering protected directly raises ValueError
        with self.assertRaises(ValueError):
            crop_safe_step_cost((2, 2), protected)

        # Factory function verification
        cost_fn = make_crop_safe_cost_fn(protected, risk_penalty=2.5, base_cost=1.0)
        self.assertEqual(cost_fn((0, 0), (1, 2)), 3.5)
        self.assertEqual(cost_fn((0, 0), (0, 1)), 1.0)

    def test_crop_safe_cost_rejects_invalid_penalty_and_base(self) -> None:
        with self.assertRaises(ValueError):
            crop_safe_step_cost((0, 0), set(), risk_penalty=-0.1)
        with self.assertRaises(ValueError):
            crop_safe_step_cost((0, 0), set(), base_cost=0.5)
        with self.assertRaises(ValueError):
            crop_safe_step_cost((0, 0), set(), risk_penalty=float("nan"))

    # -----------------------------------------------------------------------
    # Optimality vs Greedy Best-First Search Contrast
    # -----------------------------------------------------------------------

    def test_astar_cost_optimality_vs_gbfs(self) -> None:
        """Demonstrate that A* chooses the least-cost path while GBFS takes a higher-cost risky detour.

        Scenario:
        A direct route passes through cells adjacent to a protected crop band (risk surcharge 5.0 each).
        An alternate route goes around, adding 2 extra steps but avoiding all crop buffer penalties.
        A* computes f = g + h and selects the safer, lower-cost detour.
        GBFS considers only h(n) and greedily rushes straight through the expensive buffer cells.
        """
        # 6x6 grid, start=(1, 0), goal=(1, 5)
        # Protected crops at (2, 1), (2, 2), (2, 3), (2, 4)
        protected = {(2, 1), (2, 2), (2, 3), (2, 4)}
        field_map = FieldMap(height=6, width=6, blocked_cells=protected)
        start = (1, 0)
        goal = (1, 5)

        cost_fn = make_crop_safe_cost_fn(protected, risk_penalty=5.0, base_cost=1.0)

        astar_res = a_star_search(field_map, start, goal, cost_fn=cost_fn)
        gbfs_res = greedy_best_first_search(field_map, start, goal, cost_fn=cost_fn)

        self.assertTrue(astar_res.reached_goal)
        self.assertTrue(gbfs_res.reached_goal)

        # A* routes via row 0 (cost: 7.0, steps: 7)
        # GBFS routes via row 1 directly adjacent to crops (cost: 5 steps * (1+5) = 25+, steps: 5)
        self.assertLess(astar_res.cost, gbfs_res.cost)
        self.assertEqual(astar_res.cost, 7.0)
        self.assertGreaterEqual(gbfs_res.cost, 25.0)

    # -----------------------------------------------------------------------
    # Interoperability: FieldMap and Field
    # -----------------------------------------------------------------------

    def test_interoperability_fieldmap_and_field(self) -> None:
        """A* handles both model.Field and types.FieldMap with identical optimal route costs."""
        # Using model.Field
        field = Field(height=5, width=5, start=(0, 0), goal=(4, 4), protected={(2, 2)}, risk_penalty=2.0)
        res_field = a_star_search(field)

        # Using types.FieldMap
        field_map = FieldMap(height=5, width=5, blocked_cells=field.blocked)
        res_fmap = a_star_search(
            field_map,
            start=field.start,
            goal=field.goal,
            protected_cells=field.protected,
            risk_penalty=field.risk_penalty,
        )

        self.assertEqual(res_field.cost, res_fmap.cost)
        self.assertEqual(res_field.path_length, res_fmap.path_length)
        self.assertTrue(res_field.reached_goal)
        self.assertTrue(res_fmap.reached_goal)
        self.assertTrue(_is_valid_cardinal_path(res_field.path))
        self.assertTrue(_is_valid_cardinal_path(res_fmap.path))


if __name__ == "__main__":
    unittest.main(verbosity=2)
