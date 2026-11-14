"""Automated tests for Manhattan distance heuristic function.

Covers Test 1 specifications:
    - Current position equals goal: result must be 0.
    - Different positions: result must be correct.
    - Result must never be negative.
Plus symmetry, triangle inequality, and invalid coordinate types.
"""

from __future__ import annotations

import sys
from pathlib import Path
import unittest

# Ensure src is on sys.path for direct test invocation
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from cropsafe_pathplanning.heuristic import manhattan_distance


class TestManhattanDistance(unittest.TestCase):
    """Test suite for manhattan_distance function."""

    def test_current_equals_goal(self) -> None:
        """Test 1.1: Current position equals goal -> distance must be 0."""
        self.assertEqual(manhattan_distance((0, 0), (0, 0)), 0)
        self.assertEqual(manhattan_distance((5, 10), (5, 10)), 0)
        self.assertEqual(manhattan_distance((-3, 4), (-3, 4)), 0)

    def test_different_positions(self) -> None:
        """Test 1.2: Different positions -> result must match formula |r1-r2| + |c1-c2|."""
        # Pure horizontal
        self.assertEqual(manhattan_distance((0, 0), (0, 5)), 5)
        self.assertEqual(manhattan_distance((0, 5), (0, 0)), 5)

        # Pure vertical
        self.assertEqual(manhattan_distance((0, 0), (7, 0)), 7)
        self.assertEqual(manhattan_distance((7, 0), (0, 0)), 7)

        # Mixed diagonal displacement
        # (1, 2) to (4, 6): |1-4| + |2-6| = 3 + 4 = 7
        self.assertEqual(manhattan_distance((1, 2), (4, 6)), 7)
        # Reverse
        self.assertEqual(manhattan_distance((4, 6), (1, 2)), 7)

        # Larger coordinates
        self.assertEqual(manhattan_distance((100, 250), (350, 500)), 500)

    def test_never_negative(self) -> None:
        """Test 1.3: Result must never be negative across varied coordinate pairs."""
        coords = [
            ((0, 0), (10, 10)),
            ((10, 10), (0, 0)),
            ((5, 2), (2, 8)),
            ((0, 100), (100, 0)),
            ((3, 3), (3, 3)),
        ]
        for a, b in coords:
            dist = manhattan_distance(a, b)
            self.assertGreaterEqual(dist, 0, f"Distance between {a} and {b} was negative: {dist}")

    def test_symmetry(self) -> None:
        """Verify metric symmetry: h(u, v) == h(v, u)."""
        pairs = [
            ((1, 4), (5, 9)),
            ((10, 2), (3, 7)),
            ((0, 0), (12, 18)),
        ]
        for u, v in pairs:
            self.assertEqual(
                manhattan_distance(u, v),
                manhattan_distance(v, u),
                f"Symmetry failed for {u} and {v}",
            )

    def test_triangle_inequality(self) -> None:
        """Verify triangle inequality: h(u, w) <= h(u, v) + h(v, w)."""
        u = (1, 2)
        v = (4, 7)
        w = (8, 3)
        self.assertLessEqual(
            manhattan_distance(u, w),
            manhattan_distance(u, v) + manhattan_distance(v, w),
        )

    def test_invalid_types_raise_type_error(self) -> None:
        """Verify TypeError is raised when coordinates are not integer pairs."""
        with self.assertRaises(TypeError):
            manhattan_distance((1.5, 2), (3, 4))  # type: ignore[arg-type]

        with self.assertRaises(TypeError):
            manhattan_distance("invalid", (3, 4))  # type: ignore[arg-type]

        with self.assertRaises(TypeError):
            manhattan_distance((1, 2, 3), (3, 4))  # type: ignore[arg-type]


if __name__ == "__main__":
    unittest.main()
