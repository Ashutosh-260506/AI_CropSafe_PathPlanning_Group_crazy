"""Tests for field_scenario.py — Hitesh (Member 1) deliverable.

Verifies that:
- FIELD_CONFIG contains all required keys with correct types and values.
- make_field() constructs a valid Field consistent with model.py / demo_field().
- make_field_map() constructs a valid FieldMap consistent with types.py.
- make_field_map_with_hidden() includes the hidden obstacle in blocked_cells.
- Start and goal are traversable in both Field and FieldMap.
- Blocked cells (protected, obstacles) are non-traversable.
- Coordinate convention, movement directions, and cost model are correct.
- render_ascii() returns a non-empty string with expected symbols.
"""

from __future__ import annotations

import unittest

from cropsafe_pathplanning.field_scenario import (
    FIELD_CONFIG,
    SYMBOL_LEGEND,
    make_field,
    make_field_map,
    make_field_map_with_hidden,
    render_ascii,
)
from cropsafe_pathplanning.model import demo_field, demo_unknown_obstacle
from cropsafe_pathplanning.types import CARDINAL_DIRECTIONS


class TestFieldConfig(unittest.TestCase):
    """FIELD_CONFIG dictionary has all required keys and correct values."""

    def test_required_keys_present(self) -> None:
        required = {"height", "width", "start", "goal",
                    "crop_rows", "protected", "obstacles",
                    "risk_penalty", "hidden_obstacle"}
        self.assertTrue(required.issubset(FIELD_CONFIG.keys()))

    def test_grid_dimensions(self) -> None:
        self.assertEqual(FIELD_CONFIG["height"], 12)
        self.assertEqual(FIELD_CONFIG["width"], 18)

    def test_start_and_goal(self) -> None:
        self.assertEqual(FIELD_CONFIG["start"], (6, 1))
        self.assertEqual(FIELD_CONFIG["goal"], (6, 16))

    def test_risk_penalty_nonnegative(self) -> None:
        self.assertGreaterEqual(FIELD_CONFIG["risk_penalty"], 0.0)
        self.assertAlmostEqual(FIELD_CONFIG["risk_penalty"], 2.0)

    def test_hidden_obstacle_position(self) -> None:
        self.assertEqual(FIELD_CONFIG["hidden_obstacle"], (10, 9))

    def test_crop_rows_count(self) -> None:
        # row 3 cols 1-16 (16 cells) + row 8 cols 1-16 (16 cells) = 32
        self.assertEqual(len(FIELD_CONFIG["crop_rows"]), 32)

    def test_protected_count(self) -> None:
        # row 3 cols 2-15 (14 cells) + row 8 cols 2-15 (14 cells) = 28
        self.assertEqual(len(FIELD_CONFIG["protected"]), 28)

    def test_obstacles_count(self) -> None:
        self.assertEqual(len(FIELD_CONFIG["obstacles"]), 4)

    def test_protected_subset_of_crop_rows(self) -> None:
        self.assertTrue(
            set(FIELD_CONFIG["protected"]).issubset(set(FIELD_CONFIG["crop_rows"]))
        )

    def test_start_not_in_blocked(self) -> None:
        blocked = set(FIELD_CONFIG["protected"]) | set(FIELD_CONFIG["obstacles"])
        self.assertNotIn(FIELD_CONFIG["start"], blocked)

    def test_goal_not_in_blocked(self) -> None:
        blocked = set(FIELD_CONFIG["protected"]) | set(FIELD_CONFIG["obstacles"])
        self.assertNotIn(FIELD_CONFIG["goal"], blocked)

    def test_start_in_bounds(self) -> None:
        r, c = FIELD_CONFIG["start"]
        self.assertGreaterEqual(r, 0)
        self.assertLess(r, FIELD_CONFIG["height"])
        self.assertGreaterEqual(c, 0)
        self.assertLess(c, FIELD_CONFIG["width"])

    def test_goal_in_bounds(self) -> None:
        r, c = FIELD_CONFIG["goal"]
        self.assertGreaterEqual(r, 0)
        self.assertLess(r, FIELD_CONFIG["height"])
        self.assertGreaterEqual(c, 0)
        self.assertLess(c, FIELD_CONFIG["width"])


class TestMakeField(unittest.TestCase):
    """make_field() returns a Field consistent with model.demo_field()."""

    def setUp(self) -> None:
        self.field = make_field()
        self.demo = demo_field()

    def test_dimensions_match(self) -> None:
        self.assertEqual(self.field.height, self.demo.height)
        self.assertEqual(self.field.width, self.demo.width)

    def test_start_matches(self) -> None:
        self.assertEqual(self.field.start, self.demo.start)

    def test_goal_matches(self) -> None:
        self.assertEqual(self.field.goal, self.demo.goal)

    def test_protected_matches(self) -> None:
        self.assertEqual(self.field.protected, self.demo.protected)

    def test_obstacles_matches(self) -> None:
        self.assertEqual(self.field.obstacles, self.demo.obstacles)

    def test_risk_penalty_matches(self) -> None:
        self.assertAlmostEqual(self.field.risk_penalty, self.demo.risk_penalty)

    def test_start_traversable(self) -> None:
        self.assertNotIn(self.field.start, self.field.blocked)

    def test_goal_traversable(self) -> None:
        self.assertNotIn(self.field.goal, self.field.blocked)

    def test_protected_cells_blocked(self) -> None:
        for pos in self.field.protected:
            self.assertIn(pos, self.field.blocked)

    def test_obstacle_cells_blocked(self) -> None:
        for pos in self.field.obstacles:
            self.assertIn(pos, self.field.blocked)

    def test_step_cost_open_cell(self) -> None:
        # (0, 0) is open ground, not adjacent to any protected cell
        cost = self.field.step_cost((0, 0))
        self.assertAlmostEqual(cost, 1.0)

    def test_step_cost_adjacent_to_protected(self) -> None:
        # (2, 5) is adjacent to protected row 3; cost should be 1 + 2 = 3
        cost = self.field.step_cost((2, 5))
        self.assertAlmostEqual(cost, 3.0)

    def test_step_cost_all_legal_moves_at_least_one(self) -> None:
        """Every legal cell entry costs ≥ 1.0 (admissibility requirement)."""
        field = self.field
        for r in range(field.height):
            for c in range(field.width):
                pos = (r, c)
                if pos not in field.blocked:
                    self.assertGreaterEqual(field.step_cost(pos), 1.0)

    def test_blocked_cell_raises(self) -> None:
        blocked_pos = next(iter(self.field.protected))
        with self.assertRaises(ValueError):
            self.field.step_cost(blocked_pos)

    def test_four_directional_neighbors_only(self) -> None:
        """Neighbors must be exactly the four cardinal directions."""
        # (1, 1) has no obstacles nearby; expect up to 4 neighbors
        for neighbour in self.field.neighbors((1, 1)):
            r_n, c_n = neighbour
            r, c = 1, 1
            delta = (r_n - r, c_n - c)
            self.assertIn(delta, [(-1, 0), (1, 0), (0, -1), (0, 1)],
                          f"Unexpected diagonal or multi-step neighbour: {neighbour}")

    def test_hidden_obstacle_absent_from_known_field(self) -> None:
        hidden = demo_unknown_obstacle()
        self.assertNotIn(hidden, self.field.blocked)


class TestMakeFieldMap(unittest.TestCase):
    """make_field_map() returns a FieldMap with correct blocked cells."""

    def setUp(self) -> None:
        self.fmap = make_field_map()

    def test_dimensions(self) -> None:
        self.assertEqual(self.fmap.height, 12)
        self.assertEqual(self.fmap.width, 18)

    def test_start_traversable(self) -> None:
        self.assertTrue(self.fmap.is_traversable(FIELD_CONFIG["start"]))

    def test_goal_traversable(self) -> None:
        self.assertTrue(self.fmap.is_traversable(FIELD_CONFIG["goal"]))

    def test_protected_cells_blocked(self) -> None:
        for pos in FIELD_CONFIG["protected"]:
            self.assertTrue(self.fmap.is_blocked(pos), f"{pos} should be blocked")

    def test_obstacles_blocked(self) -> None:
        for pos in FIELD_CONFIG["obstacles"]:
            self.assertTrue(self.fmap.is_blocked(pos), f"{pos} should be blocked")

    def test_hidden_not_blocked_in_known_map(self) -> None:
        hidden = FIELD_CONFIG["hidden_obstacle"]
        self.assertFalse(self.fmap.is_blocked(hidden))

    def test_blocked_count(self) -> None:
        # protected (28) + obstacles (4) = 32 unique blocked cells
        self.assertEqual(len(self.fmap.blocked_cells), 32)

    def test_four_directional_neighbors(self) -> None:
        """get_neighbors must return only cardinal neighbours."""
        for n in self.fmap.get_neighbors((1, 1)):
            dr = n[0] - 1
            dc = n[1] - 1
            self.assertIn((dr, dc), [(-1, 0), (1, 0), (0, -1), (0, 1)])

    def test_out_of_bounds_not_traversable(self) -> None:
        self.assertFalse(self.fmap.is_traversable((-1, 0)))
        self.assertFalse(self.fmap.is_traversable((0, -1)))
        self.assertFalse(self.fmap.is_traversable((12, 0)))
        self.assertFalse(self.fmap.is_traversable((0, 18)))

    def test_cardinal_directions_constant(self) -> None:
        expected = {(-1, 0), (1, 0), (0, -1), (0, 1)}
        self.assertEqual(set(CARDINAL_DIRECTIONS), expected)
        self.assertEqual(len(CARDINAL_DIRECTIONS), 4)


class TestMakeFieldMapWithHidden(unittest.TestCase):
    """make_field_map_with_hidden() adds the hidden obstacle to blocked cells."""

    def setUp(self) -> None:
        self.known = make_field_map()
        self.true_map = make_field_map_with_hidden()
        self.hidden = FIELD_CONFIG["hidden_obstacle"]

    def test_hidden_blocked_in_true_map(self) -> None:
        self.assertTrue(self.true_map.is_blocked(self.hidden))

    def test_hidden_not_blocked_in_known_map(self) -> None:
        self.assertFalse(self.known.is_blocked(self.hidden))

    def test_true_map_one_more_blocked(self) -> None:
        self.assertEqual(len(self.true_map.blocked_cells),
                         len(self.known.blocked_cells) + 1)


class TestRenderAscii(unittest.TestCase):
    """render_ascii() produces correct output."""

    def test_returns_string(self) -> None:
        result = render_ascii()
        self.assertIsInstance(result, str)
        self.assertTrue(len(result) > 0)

    def test_contains_start_symbol(self) -> None:
        self.assertIn(" S ", render_ascii())

    def test_contains_goal_symbol(self) -> None:
        self.assertIn(" G ", render_ascii())

    def test_hidden_symbol_shown_by_default(self) -> None:
        self.assertIn(" H ", render_ascii(show_hidden=True))

    def test_hidden_symbol_hidden_when_false(self) -> None:
        result = render_ascii(show_hidden=False)
        self.assertNotIn(" H ", result)

    def test_protected_symbol_present(self) -> None:
        self.assertIn(" P ", render_ascii())

    def test_obstacle_symbol_present(self) -> None:
        self.assertIn(" X ", render_ascii())

    def test_symbol_legend_non_empty(self) -> None:
        self.assertIn("Safe ground", SYMBOL_LEGEND)
        self.assertIn("Crop row", SYMBOL_LEGEND)
        self.assertIn("Protected buffer", SYMBOL_LEGEND)
        self.assertIn("Known static obstacle", SYMBOL_LEGEND)
        self.assertIn("Hidden obstacle", SYMBOL_LEGEND)

    def test_path_symbol_shown(self) -> None:
        path = [(6, 2), (6, 3)]
        result = render_ascii(path=path)
        self.assertIn(" * ", result)

    def test_correct_row_count(self) -> None:
        lines = render_ascii().splitlines()
        # 2 header lines + 12 data rows + 1 footer = 15
        self.assertEqual(len(lines), 15)


if __name__ == "__main__":
    unittest.main()
