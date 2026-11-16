"""A* Search algorithm and Crop-Safe route cost model.

Author: Ashutosh (Member 4)
Package: cropsafe_pathplanning

This module provides:
1. The A* pathfinding algorithm minimizing f(n) = g(n) + h(n).
2. The crop-safe cost model accounting for crop protection buffers/proximity penalties.
3. Admissible and consistent Manhattan distance heuristic integration.
4. Full interoperability with both `types.FieldMap` and `model.Field`.
"""

from __future__ import annotations

import heapq
import math
from time import perf_counter
from typing import Any, Callable, Sequence

from cropsafe_pathplanning.heuristic import manhattan_distance
from cropsafe_pathplanning.types import (
    CARDINAL_DIRECTIONS,
    Coordinate,
    FieldMap,
    SearchResult as BaseSearchResult,
)

__all__ = [
    "a_star_search",
    "crop_safe_step_cost",
    "make_crop_safe_cost_fn",
    "is_adjacent_to_protected",
    "validate_endpoints",
    "SearchResult",
    "ExpandedNodeList",
]


class ExpandedNodeList(list):
    """List of expanded node coordinates supporting both collection and integer interfaces.

    This dual interface enables seamless integration across:
    - Teammates' tests checking `isinstance(result.expanded_nodes, list)` and indexing.
    - Evaluation scripts computing `expanded_total += result.expanded_nodes` or formatting as an int.
    """

    def __init__(
        self,
        items: Sequence[Coordinate] | None = None,
        count: int | None = None,
    ) -> None:
        super().__init__(items or [])
        self._count = count if count is not None else len(self)

    def __int__(self) -> int:
        return self._count

    def __index__(self) -> int:
        return self._count

    def __str__(self) -> str:
        return str(self._count)

    def __repr__(self) -> str:
        return f"ExpandedNodeList(count={self._count}, nodes={super().__repr__()})"

    def __add__(self, other: Any) -> Any:
        if isinstance(other, int):
            return self._count + other
        return super().__add__(other)

    def __radd__(self, other: Any) -> Any:
        if isinstance(other, int):
            return other + self._count
        return other + list(self)

    def __format__(self, format_spec: str) -> str:
        if format_spec and any(c in format_spec for c in "doxXb"):
            return format(self._count, format_spec)
        try:
            return format(self._count, format_spec)
        except ValueError:
            return super().__format__(format_spec)


class SearchResult(BaseSearchResult):
    """Enhanced SearchResult with property accessors for evaluation and benchmarks.

    Conforms to the shared dict specification:
        {
            "path": list[Coordinate],
            "cost": float | int,
            "expanded_nodes": list[Coordinate],
            "reached_goal": bool,
            "algorithm": str,
            "elapsed_seconds": float,
        }
    """

    def __init__(
        self,
        path: list[Coordinate],
        cost: float | int,
        expanded_nodes: Sequence[Coordinate] | int,
        reached_goal: bool,
        algorithm: str = "A*",
        elapsed_seconds: float = 0.0,
        **kwargs: Any,
    ) -> None:
        if isinstance(expanded_nodes, int):
            node_list = ExpandedNodeList(count=expanded_nodes)
        else:
            node_list = ExpandedNodeList(expanded_nodes)

        super().__init__(
            path=list(path),
            cost=cost,
            expanded_nodes=node_list,
            reached_goal=bool(reached_goal),
            algorithm=algorithm,
            elapsed_seconds=elapsed_seconds,
            **kwargs,
        )
        # Ensure our custom ExpandedNodeList is preserved after BaseSearchResult init
        self["expanded_nodes"] = node_list

    @property
    def algorithm(self) -> str:
        """Name of the algorithm executed."""
        return self.get("algorithm", "A*")

    @property
    def elapsed_seconds(self) -> float:
        """Execution time in seconds."""
        return self.get("elapsed_seconds", 0.0)

    @property
    def path_length(self) -> int:
        """Number of movement steps in the path (0 if no path or single cell)."""
        p = self.path
        return max(0, len(p) - 1) if p else 0

    @property
    def expanded_count(self) -> int:
        """Total count of expanded nodes."""
        return len(self.expanded_nodes)


# ---------------------------------------------------------------------------
# Crop-Safe Route Cost Model
# ---------------------------------------------------------------------------

def is_adjacent_to_protected(coord: Coordinate, protected_cells: set[Coordinate]) -> bool:
    """Return True if `coord` is 4-directionally adjacent to any protected crop cell."""
    r, c = coord
    for dr, dc in CARDINAL_DIRECTIONS:
        if (r + dr, c + dc) in protected_cells:
            return True
    return False


def crop_safe_step_cost(
    destination: Coordinate,
    protected_cells: set[Coordinate],
    *,
    risk_penalty: float = 2.0,
    base_cost: float = 1.0,
) -> float:
    """Compute step transition cost entering `destination`.

    Cost Formulation:
        c(u, v) = base_cost + (risk_penalty if v is adjacent to a protected crop else 0.0)

    Parameters:
        destination: Cell being entered.
        protected_cells: Set of protected crop coordinates (strictly impassable).
        risk_penalty: Nonnegative penalty added when entering a crop buffer cell.
        base_cost: Standard movement cost for one grid step (default 1.0).

    Returns:
        Total transition cost as float.
    """
    if destination in protected_cells:
        raise ValueError(f"Cannot calculate transition into protected cell {destination}")
    if is_adjacent_to_protected(destination, protected_cells):
        return base_cost + risk_penalty
    return base_cost


def make_crop_safe_cost_fn(
    protected_cells: set[Coordinate],
    *,
    risk_penalty: float = 2.0,
    base_cost: float = 1.0,
) -> Callable[[Coordinate, Coordinate], float]:
    """Factory creating a (current, neighbor) -> float transition cost function."""
    protected_set = set(protected_cells)

    def cost_fn(current: Coordinate, neighbor: Coordinate) -> float:
        del current  # transition cost depends on entering the neighbor cell
        return crop_safe_step_cost(
            neighbor,
            protected_set,
            risk_penalty=risk_penalty,
            base_cost=base_cost,
        )

    return cost_fn


# ---------------------------------------------------------------------------
# Field Adaptation and Validation
# ---------------------------------------------------------------------------

def _adapt_field(field_input: Any) -> FieldMap:
    """Coerce various grid representations to a FieldMap instance."""
    if isinstance(field_input, FieldMap):
        return field_input

    # Duck-typed model.Field support
    if hasattr(field_input, "blocked") and hasattr(field_input, "height") and hasattr(field_input, "width"):
        return FieldMap(
            height=field_input.height,
            width=field_input.width,
            blocked_cells=set(field_input.blocked),
        )

    # Duck-typed with is_traversable
    if hasattr(field_input, "is_traversable") and hasattr(field_input, "height") and hasattr(field_input, "width"):
        blocked: set[Coordinate] = set()
        for r in range(field_input.height):
            for c in range(field_input.width):
                if not field_input.is_traversable((r, c)):
                    blocked.add((r, c))
        return FieldMap(
            height=field_input.height,
            width=field_input.width,
            blocked_cells=blocked,
        )

    # 2D sequence/matrix
    if isinstance(field_input, (list, tuple)) and field_input and isinstance(field_input[0], (list, tuple)):
        return FieldMap.from_matrix(field_input)

    raise TypeError(
        f"Unsupported field type {type(field_input).__name__}. "
        "Expected FieldMap, Field, or 2D matrix."
    )


def validate_endpoints(
    field: FieldMap,
    start: Coordinate,
    goal: Coordinate,
    *,
    raise_on_invalid: bool = False,
) -> tuple[bool, str | None]:
    """Validate that start and goal are valid coordinates within bounds and unblocked."""
    for label, pt in [("Start", start), ("Goal", goal)]:
        if not (isinstance(pt, (tuple, list)) and len(pt) == 2):
            msg = f"{label} must be a 2-tuple (row, col), got {pt!r}"
            if raise_on_invalid:
                raise TypeError(msg)
            return False, msg
        if not (isinstance(pt[0], int) and isinstance(pt[1], int)):
            msg = f"{label} coordinates must be integers, got ({type(pt[0]).__name__}, {type(pt[1]).__name__})"
            if raise_on_invalid:
                raise TypeError(msg)
            return False, msg

    if not field.is_within_bounds(start):
        msg = f"Start {start} is out of bounds for {field.height}x{field.width} field"
        if raise_on_invalid:
            raise ValueError(msg)
        return False, msg

    if not field.is_within_bounds(goal):
        msg = f"Goal {goal} is out of bounds for {field.height}x{field.width} field"
        if raise_on_invalid:
            raise ValueError(msg)
        return False, msg

    if field.is_blocked(start):
        msg = f"Start {start} is blocked (obstacle or protected crop cell)"
        if raise_on_invalid:
            raise ValueError(msg)
        return False, msg

    if field.is_blocked(goal):
        msg = f"Goal {goal} is blocked (obstacle or protected crop cell)"
        if raise_on_invalid:
            raise ValueError(msg)
        return False, msg

    return True, None


def _reconstruct_path(
    parent: dict[Coordinate, Coordinate | None],
    start: Coordinate,
    goal: Coordinate,
) -> list[Coordinate]:
    """Reconstruct path from parent pointers backwards from goal to start."""
    if goal != start and goal not in parent:
        return []
    path: list[Coordinate] = []
    current: Coordinate | None = goal
    while current is not None:
        path.append(current)
        current = parent.get(current)
    path.reverse()
    if not path or path[0] != start:
        return []
    return path


# ---------------------------------------------------------------------------
# A* Algorithm Implementation
# ---------------------------------------------------------------------------

def a_star_search(
    field: Any,
    start: Coordinate | None = None,
    goal: Coordinate | None = None,
    *,
    heuristic_fn: Callable[[Coordinate, Coordinate], float] = manhattan_distance,
    movement_directions: Sequence[Coordinate] = CARDINAL_DIRECTIONS,
    cost_fn: Callable[[Coordinate, Coordinate], float] | None = None,
    raise_on_invalid: bool = False,
    protected_cells: set[Coordinate] | None = None,
    risk_penalty: float | None = None,
) -> SearchResult:
    """Find the cost-optimal path from start to goal using the A* Search algorithm.

    A* evaluates frontier nodes by f(n) = g(n) + h(n):
        - g(n): Exact cost accumulated from start to node n.
        - h(n): Admissible heuristic estimate of remaining cost from n to goal.
        - f(n): Estimated total path cost through node n.

    Heuristic Admissibility & Consistency:
        Under 4-directional cardinal grid movement with minimum step cost c >= 1.0,
        the Manhattan distance h(n) = |r_n - r_g| + |c_n - c_g| is:
        1. Admissible: h(n) <= h*(n) because every step costs >= 1.0 and requires at least
           one coordinate decrement.
        2. Consistent (Monotonic): h(n) <= c(n, n') + h(n') because |h(n) - h(n')| <= 1 <= c(n, n').
        Since all risk penalties are nonnegative, adding crop-safety penalties only increases
        true costs (c(n, n') >= 1.0), preserving admissibility and guaranteeing cost-optimality.

    Parameters:
        field: A `FieldMap`, `model.Field`, or 2D matrix.
        start: (row, col) start coordinate (inferred from field.start if None).
        goal: (row, col) goal coordinate (inferred from field.goal if None).
        heuristic_fn: Heuristic function (n, goal) -> float. Defaults to Manhattan distance.
        movement_directions: Sequence of allowed movement vectors (default cardinal 4-way).
        cost_fn: Optional transition cost function (current, neighbor) -> float.
        raise_on_invalid: If True, invalid start/goal raises ValueError/TypeError.
        protected_cells: Optional set of protected crop coordinates.
        risk_penalty: Optional surcharge for moving adjacent to protected crops.

    Returns:
        SearchResult containing path, cost, expanded_nodes, reached_goal, algorithm, and elapsed_seconds.
    """
    started = perf_counter()

    # Resolve start and goal endpoints from field attributes if omitted
    resolved_start = start if start is not None else getattr(field, "start", None)
    resolved_goal = goal if goal is not None else getattr(field, "goal", None)

    field_map = _adapt_field(field)

    # Validate endpoints
    is_valid, error_msg = validate_endpoints(
        field_map,
        resolved_start,
        resolved_goal,
        raise_on_invalid=raise_on_invalid,
    )
    if not is_valid:
        return SearchResult(
            path=[],
            cost=math.inf,
            expanded_nodes=[],
            reached_goal=False,
            algorithm="A*",
            elapsed_seconds=perf_counter() - started,
            error=error_msg,
        )

    # Resolve cost function
    actual_cost_fn: Callable[[Coordinate, Coordinate], float]
    if cost_fn is not None:
        actual_cost_fn = cost_fn
    elif hasattr(field, "step_cost"):
        actual_cost_fn = lambda current, neighbor: field.step_cost(neighbor)
    else:
        # Check for protected cells from parameters or field
        resolved_protected = (
            protected_cells
            if protected_cells is not None
            else getattr(field, "protected", None)
        )
        resolved_penalty = (
            risk_penalty
            if risk_penalty is not None
            else getattr(field, "risk_penalty", 2.0)
        )
        if resolved_protected:
            actual_cost_fn = make_crop_safe_cost_fn(
                resolved_protected,
                risk_penalty=resolved_penalty,
                base_cost=1.0,
            )
        else:
            actual_cost_fn = lambda current, neighbor: 1.0

    # Start == Goal edge case
    if resolved_start == resolved_goal:
        return SearchResult(
            path=[resolved_start],
            cost=0.0,
            expanded_nodes=[resolved_start],
            reached_goal=True,
            algorithm="A*",
            elapsed_seconds=perf_counter() - started,
        )

    # Priority queue: (f_score, tie_breaker_serial, coordinate)
    serial = 0
    start_h = float(heuristic_fn(resolved_start, resolved_goal))
    frontier: list[tuple[float, int, Coordinate]] = [(start_h, serial, resolved_start)]

    g_score: dict[Coordinate, float] = {resolved_start: 0.0}
    parent: dict[Coordinate, Coordinate | None] = {resolved_start: None}
    closed: set[Coordinate] = set()
    expanded_nodes: list[Coordinate] = []

    while frontier:
        current_f, _, current = heapq.heappop(frontier)

        if current in closed:
            continue

        closed.add(current)
        expanded_nodes.append(current)

        if current == resolved_goal:
            path = _reconstruct_path(parent, resolved_start, resolved_goal)
            total_cost = g_score[resolved_goal]
            return SearchResult(
                path=path,
                cost=total_cost,
                expanded_nodes=expanded_nodes,
                reached_goal=True,
                algorithm="A*",
                elapsed_seconds=perf_counter() - started,
            )

        # Explore neighbors
        # If field provides custom neighbors generator (like model.Field), prefer it for consistency
        candidate_neighbors: Sequence[Coordinate]
        if hasattr(field, "neighbors"):
            candidate_neighbors = list(field.neighbors(current))
        else:
            r, c = current
            candidate_neighbors = [
                (r + dr, c + dc)
                for dr, dc in movement_directions
                if field_map.is_traversable((r + dr, c + dc))
            ]

        for neighbor in candidate_neighbors:
            if neighbor in closed or not field_map.is_traversable(neighbor):
                continue

            step = actual_cost_fn(current, neighbor)
            tentative_g = g_score[current] + step

            if tentative_g < g_score.get(neighbor, math.inf):
                g_score[neighbor] = tentative_g
                parent[neighbor] = current
                serial += 1
                h_val = float(heuristic_fn(neighbor, resolved_goal))
                f_val = tentative_g + h_val
                heapq.heappush(frontier, (f_val, serial, neighbor))

    # Goal unreachable
    return SearchResult(
        path=[],
        cost=math.inf,
        expanded_nodes=expanded_nodes,
        reached_goal=False,
        algorithm="A*",
        elapsed_seconds=perf_counter() - started,
    )
