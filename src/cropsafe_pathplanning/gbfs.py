"""Greedy Best-First Search (GBFS) implementation for CropSafe Path Planning.

This module provides the Greedy Best-First Search algorithm for 2D grid pathfinding,
utilizing the Manhattan distance heuristic and deterministic priority-queue tie-breaking.
"""

from __future__ import annotations

import heapq
import itertools
from typing import Any, Callable, Sequence

from cropsafe_pathplanning.heuristic import manhattan_distance
from cropsafe_pathplanning.types import (
    CARDINAL_DIRECTIONS,
    Coordinate,
    FieldMap,
    SearchResult,
)


def _adapt_field(field_input: Any) -> FieldMap:
    """Adapt various field/grid representations to a FieldMap instance.

    Supports:
        - FieldMap instances (returned directly)
        - 2D sequences (list of lists, matrix of ints/strings/bools)
        - Objects with `height`, `width`, and `is_traversable` methods
    """
    if isinstance(field_input, FieldMap):
        return field_input

    if hasattr(field_input, "is_traversable") and hasattr(field_input, "height") and hasattr(field_input, "width"):
        # Duck-typed field object
        blocked: set[Coordinate] = set()
        for r in range(field_input.height):
            for c in range(field_input.width):
                if not field_input.is_traversable((r, c)):
                    blocked.add((r, c))
        return FieldMap(height=field_input.height, width=field_input.width, blocked_cells=blocked)

    if isinstance(field_input, (list, tuple)) and field_input and isinstance(field_input[0], (list, tuple)):
        return FieldMap.from_matrix(field_input)

    raise TypeError(
        f"Unsupported field representation of type {type(field_input).__name__}. "
        "Expected FieldMap or 2D matrix (sequence of sequences)."
    )


def validate_endpoints(
    field: FieldMap,
    start: Coordinate,
    goal: Coordinate,
    *,
    raise_on_invalid: bool = True,
) -> tuple[bool, str | None]:
    """Validate start and goal coordinates against field bounds and obstacles.

    Parameters:
        field: The FieldMap representing the grid.
        start: (row, col) start coordinate.
        goal: (row, col) goal coordinate.
        raise_on_invalid: If True, raise ValueError on validation failure.
                          If False, return (False, error_message).

    Returns:
        tuple (is_valid, error_message).

    Raises:
        ValueError: If `raise_on_invalid=True` and any validation check fails.
    """
    # Type checking
    for name, pt in [("Start", start), ("Goal", goal)]:
        if not (isinstance(pt, (tuple, list)) and len(pt) == 2):
            msg = f"{name} position must be a 2-tuple (row, col), got {pt!r}"
            if raise_on_invalid:
                raise TypeError(msg)
            return False, msg
        if not (isinstance(pt[0], int) and isinstance(pt[1], int)):
            msg = f"{name} coordinates must be integers, got ({type(pt[0]).__name__}, {type(pt[1]).__name__})"
            if raise_on_invalid:
                raise TypeError(msg)
            return False, msg

    # Bounds checking
    if not field.is_within_bounds(start):
        msg = f"Start position {start} is out of bounds for field size ({field.height}, {field.width})"
        if raise_on_invalid:
            raise ValueError(msg)
        return False, msg

    if not field.is_within_bounds(goal):
        msg = f"Goal position {goal} is out of bounds for field size ({field.height}, {field.width})"
        if raise_on_invalid:
            raise ValueError(msg)
        return False, msg

    # Blocked checking
    if field.is_blocked(start):
        msg = f"Start position {start} is blocked (obstacle or protected crop cell)"
        if raise_on_invalid:
            raise ValueError(msg)
        return False, msg

    if field.is_blocked(goal):
        msg = f"Goal position {goal} is blocked (obstacle or protected crop cell)"
        if raise_on_invalid:
            raise ValueError(msg)
        return False, msg

    return True, None


def greedy_best_first_search(
    field: FieldMap | Sequence[Sequence[Any]],
    start: Coordinate,
    goal: Coordinate,
    *,
    heuristic_fn: Callable[[Coordinate, Coordinate], int] = manhattan_distance,
    movement_directions: Sequence[Coordinate] = CARDINAL_DIRECTIONS,
    cost_fn: Callable[[Coordinate, Coordinate], float] | None = None,
    raise_on_invalid: bool = False,
) -> SearchResult:
    """Find a path from start to goal using Greedy Best-First Search (GBFS).

    Important Algorithm Distinction:
        Greedy Best-First Search evaluates nodes purely using their heuristic value:
            f(n) = h(n)
        It does NOT consider the accumulated path cost g(n) from the start node:
            f(n) != g(n) + h(n)   (that is A* Search)

        GBFS operates greedily by expanding the node that appears closest to the goal
        according to h(n). Consequently, while GBFS is often fast and memory-efficient
        in open terrain, it is NOT guaranteed to find the shortest or least-cost route.
        When obstacles obstruct the direct line of sight, GBFS may explore deep detours
        that minimize h(n) locally rather than finding the globally optimal path.

    Parameters:
        field: A `FieldMap` instance or a 2D matrix (sequence of sequences) where
               traversable cells are 0/'.'/False and blocked cells are 1/'#'/'C'/True.
        start: (row, col) coordinate of the starting cell.
        goal: (row, col) coordinate of the destination cell.
        heuristic_fn: Heuristic function estimating distance from node to goal.
                      Defaults to `manhattan_distance`.
        movement_directions: Sequence of legal movements. Defaults to 4 cardinal directions
                             (Up, Down, Left, Right) to ensure 4-connected grid movement.
        cost_fn: Optional step cost function (from_coord, to_coord) -> float.
                 If omitted, unit step cost of 1.0 is used for every transition.
        raise_on_invalid: If True, invalid start/goal inputs raise ValueError/TypeError.
                          If False, invalid inputs return a failure SearchResult.

    Returns:
        SearchResult containing:
            - "path": List of (row, col) tuples from start to goal (empty list on failure).
            - "cost": Total path cost (int or float; float('inf') on failure).
            - "expanded_nodes": List of (row, col) nodes in the exact order they were expanded.
            - "reached_goal": Boolean indicating whether the goal was reached.

    Deterministic Behavior:
        Tie-breaking among nodes with equal heuristic values is enforced using a monotonic
        insertion counter (FIFO order). Neighbors are evaluated in fixed directional order.
        Repeated runs on identical inputs yield identical paths and expanded-node sequences.
    """
    field_map = _adapt_field(field)

    # Validate inputs
    is_valid, error_msg = validate_endpoints(
        field_map, start, goal, raise_on_invalid=raise_on_invalid
    )
    if not is_valid:
        return SearchResult(
            path=[],
            cost=float("inf"),
            expanded_nodes=[],
            reached_goal=False,
            error=error_msg,
        )

    # Special case: Start equals Goal
    if start == goal:
        return SearchResult(
            path=[start],
            cost=0,
            expanded_nodes=[start],
            reached_goal=True,
        )

    # Priority queue: min-heap of tuples: (h_score, insertion_index, coordinate)
    # Using insertion_index as tie-breaker guarantees strict FIFO ordering for equal h values
    # and prevents heapq from comparing Coordinate tuples.
    counter = itertools.count()
    start_h = heuristic_fn(start, goal)
    open_heap: list[tuple[int, int, Coordinate]] = [(start_h, next(counter), start)]

    # Parent pointers for path reconstruction
    parent: dict[Coordinate, Coordinate | None] = {start: None}

    # Tracking discovered nodes prevents enqueuing duplicate entries in the priority queue
    visited: set[Coordinate] = {start}

    # Expanded nodes in the order they are popped and processed
    expanded_nodes: list[Coordinate] = []

    reached_goal = False

    while open_heap:
        h_val, _, current = heapq.heappop(open_heap)
        expanded_nodes.append(current)

        if current == goal:
            reached_goal = True
            break

        # Expand legal 4-directional neighbors in deterministic order
        r, c = current
        for dr, dc in movement_directions:
            neighbor: Coordinate = (r + dr, c + dc)

            if field_map.is_traversable(neighbor) and neighbor not in visited:
                visited.add(neighbor)
                parent[neighbor] = current
                h_neighbor = heuristic_fn(neighbor, goal)
                heapq.heappush(open_heap, (h_neighbor, next(counter), neighbor))

    # Path reconstruction
    if reached_goal:
        path: list[Coordinate] = []
        curr: Coordinate | None = goal
        while curr is not None:
            path.append(curr)
            curr = parent[curr]
        path.reverse()

        # Calculate actual path cost
        if cost_fn is None:
            # Standard unit-cost model: each step between cells costs 1
            total_cost: float | int = len(path) - 1
        else:
            total_cost = sum(
                cost_fn(path[i], path[i + 1]) for i in range(len(path) - 1)
            )

        return SearchResult(
            path=path,
            cost=total_cost,
            expanded_nodes=expanded_nodes,
            reached_goal=True,
        )

    # Goal is unreachable
    return SearchResult(
        path=[],
        cost=float("inf"),
        expanded_nodes=expanded_nodes,
        reached_goal=False,
    )
