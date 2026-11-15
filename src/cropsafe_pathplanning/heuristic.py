"""Heuristic functions for informed search algorithms in CropSafe Path Planning.

This module provides the Manhattan distance heuristic function used by Greedy
Best-First Search (GBFS) and A* search algorithms for 4-directional grid pathfinding.
"""

from __future__ import annotations

from typing import Tuple

Coordinate = Tuple[int, int]


def manhattan_distance(current: Coordinate, goal: Coordinate) -> int:
    """Calculate the Manhattan (L1 norm / taxicab) distance between two grid coordinates.

    Formula:
        h(n) = |current_row - goal_row| + |current_col - goal_col|

    Parameters:
        current: A tuple `(row, col)` representing the current node coordinates.
        goal: A tuple `(row, col)` representing the goal node coordinates.

    Returns:
        The nonnegative integer Manhattan distance between `current` and `goal`.
        Returns 0 if and only if `current == goal`.

    Theoretical Foundations & Properties:
        1. Suitability for Four-Directional Grid Movement:
           In a 2D grid where an agent is restricted to four orthogonal moves
           (Up, Down, Left, Right) with uniform step cost (e.g., 1 cost unit per step),
           each move modifies either the row index by ±1 or the column index by ±1.
           No single move can change both row and column simultaneously.
           Consequently, the theoretical minimum number of steps required to travel
           between (r1, c1) and (r2, c2) in the absence of obstacles is exactly
           |r1 - r2| + |c1 - c2|.
           Thus, Manhattan distance is an admissible heuristic (never overestimates the
           true cost to the goal) and consistent (monotonic) for four-directional grids:
               h(n) <= c(n, a, n') + h(n')

        2. Heuristic Behavior under Diagonal Movement:
           If diagonal movements were permitted, Manhattan distance would overestimate
           the actual minimum step count because a diagonal move traverses both one row
           and one column in a single step:
           - Chebyshev Distance (L_infinity norm):
             If 8-directional movement is allowed with uniform step cost (cardinal cost = 1,
             diagonal cost = 1), the appropriate admissible heuristic is Chebyshev distance:
                 h_Chebyshev(n) = max(|current_row - goal_row|, |current_col - goal_col|)
           - Octile Distance:
             If diagonal steps cost sqrt(2) (~1.414) while cardinal steps cost 1, the
             exact shortest unconstrained path is given by Octile distance:
                 dr = |current_row - goal_row|
                 dc = |current_col - goal_col|
                 h_Octile(n) = (sqrt(2) - 1) * min(dr, dc) + max(dr, dc)
           - Euclidean Distance (L2 norm):
             If movement is continuous or not constrained to a discrete grid axis,
             Euclidean distance provides the straight-line lower bound:
                 h_Euclidean(n) = sqrt((current_row - goal_row)^2 + (current_col - goal_col)^2)

    Assumptions:
        - Coordinates are integers `(row, col)`.
        - Movement cost per cardinal step is 1.
    """
    if not (isinstance(current, (tuple, list)) and len(current) == 2):
        raise TypeError(f"current must be a 2-tuple of integers, got {type(current).__name__}: {current!r}")
    if not (isinstance(goal, (tuple, list)) and len(goal) == 2):
        raise TypeError(f"goal must be a 2-tuple of integers, got {type(goal).__name__}: {goal!r}")

    r1, c1 = current
    r2, c2 = goal

    if not (isinstance(r1, int) and isinstance(c1, int)):
        raise TypeError(f"current coordinate components must be integers, got ({type(r1).__name__}, {type(c1).__name__})")
    if not (isinstance(r2, int) and isinstance(c2, int)):
        raise TypeError(f"goal coordinate components must be integers, got ({type(r2).__name__}, {type(c2).__name__})")

    return abs(r1 - r2) + abs(c1 - c2)
