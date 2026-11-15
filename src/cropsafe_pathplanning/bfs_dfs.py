"""Breadth-First Search (BFS) and Depth-First Search (DFS) for CropSafe Path Planning.


Both algorithms use the shared FieldMap / SearchResult interfaces defined in
``cropsafe_pathplanning.types`` and the Field / neighbor interface from
``cropsafe_pathplanning.model``.

Key properties
--------------
BFS
    Explores the frontier level-by-level (FIFO queue).  With uniform step
    costs it is guaranteed to return the **shortest path by number of steps**.
    When all step costs are equal (cost = 1 per move) the BFS path is also
    cost-optimal.  It may expand many nodes before finding the goal, but it
    will never return a longer-than-necessary path.

DFS
    Explores as deeply as possible before back-tracking (LIFO stack).  It is
    **complete** on finite, acyclic graphs (it will find *a* path if one
    exists), but it does **not** guarantee the shortest path.  The returned
    path length depends entirely on the order in which neighbours are visited.

Neither algorithm mutates the FieldMap passed to it.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Any, Callable, Sequence

from cropsafe_pathplanning.types import (
    CARDINAL_DIRECTIONS,
    Coordinate,
    FieldMap,
    SearchResult,
)


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _adapt_field(field_input: Any) -> FieldMap:
    """Coerce various grid representations to a :class:`FieldMap`.

    Accepts:
    * A ``FieldMap`` instance (returned as-is, **not** copied — no mutation).
    * A 2-D list / tuple of lists (forwarded to :meth:`FieldMap.from_matrix`).
    * Any duck-typed object exposing ``height``, ``width``, and
      ``is_traversable(coord)``.
    """
    if isinstance(field_input, FieldMap):
        return field_input

    if (
        hasattr(field_input, "is_traversable")
        and hasattr(field_input, "height")
        and hasattr(field_input, "width")
    ):
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

    if (
        isinstance(field_input, (list, tuple))
        and field_input
        and isinstance(field_input[0], (list, tuple))
    ):
        return FieldMap.from_matrix(field_input)

    raise TypeError(
        f"Unsupported field type {type(field_input).__name__}. "
        "Expected a FieldMap, a 2-D matrix (list of lists), or an object "
        "with height/width/is_traversable."
    )


def _validate_endpoints(
    field: FieldMap,
    start: Coordinate,
    goal: Coordinate,
    *,
    raise_on_invalid: bool,
) -> tuple[bool, str | None]:
    """Check that start and goal are in-bounds and unblocked.

    Returns ``(True, None)`` on success, or ``(False, message)`` on failure
    when ``raise_on_invalid=False``.  Raises ``ValueError`` / ``TypeError``
    when ``raise_on_invalid=True``.
    """
    for label, pt in [("Start", start), ("Goal", goal)]:
        if not (isinstance(pt, (tuple, list)) and len(pt) == 2):
            msg = f"{label} must be a 2-tuple (row, col), got {pt!r}"
            if raise_on_invalid:
                raise TypeError(msg)
            return False, msg
        if not (isinstance(pt[0], int) and isinstance(pt[1], int)):
            msg = (
                f"{label} coordinates must be integers, "
                f"got ({type(pt[0]).__name__}, {type(pt[1]).__name__})"
            )
            if raise_on_invalid:
                raise TypeError(msg)
            return False, msg

    if not field.is_within_bounds(start):
        msg = (
            f"Start {start} is out of bounds "
            f"for a {field.height}×{field.width} field"
        )
        if raise_on_invalid:
            raise ValueError(msg)
        return False, msg

    if not field.is_within_bounds(goal):
        msg = (
            f"Goal {goal} is out of bounds "
            f"for a {field.height}×{field.width} field"
        )
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
    """Walk the parent-pointer map backwards from *goal* to *start*."""
    path: list[Coordinate] = []
    current: Coordinate | None = goal
    while current is not None:
        path.append(current)
        current = parent.get(current)  # type: ignore[arg-type]
    path.reverse()
    # Safety: if path reconstruction produced something that doesn't start at
    # start, the graph was inconsistent — return empty to signal failure.
    if not path or path[0] != start:
        return []
    return path


def _failure_result(
    expanded_nodes: list[Coordinate],
    error: str | None = None,
) -> SearchResult:
    """Return a canonical failure SearchResult."""
    kwargs: dict[str, Any] = {}
    if error is not None:
        kwargs["error"] = error
    return SearchResult(
        path=[],
        cost=math.inf,
        expanded_nodes=expanded_nodes,
        reached_goal=False,
        **kwargs,
    )


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def breadth_first_search(
    field: FieldMap | Sequence[Sequence[Any]],
    start: Coordinate,
    goal: Coordinate,
    *,
    movement_directions: Sequence[Coordinate] = CARDINAL_DIRECTIONS,
    cost_fn: Callable[[Coordinate, Coordinate], float] | None = None,
    raise_on_invalid: bool = False,
) -> SearchResult:
    """Find a path from *start* to *goal* using **Breadth-First Search**.

    BFS explores the grid level by level (FIFO frontier).  When every step
    has equal cost it is guaranteed to return the **shortest path by number
    of steps** — no other uninformed search can do better on this metric.

    Parameters
    ----------
    field:
        A :class:`~cropsafe_pathplanning.types.FieldMap`, a 2-D matrix
        (list of lists with 0 = free, 1/'#' = blocked), or any duck-typed
        object exposing ``height``, ``width``, and ``is_traversable``.
        **The field is never mutated.**
    start:
        ``(row, col)`` start coordinate.
    goal:
        ``(row, col)`` goal coordinate.
    movement_directions:
        Ordered sequence of ``(Δrow, Δcol)`` pairs describing legal moves.
        Defaults to 4-directional cardinal movement (Up, Down, Left, Right).
    cost_fn:
        Optional ``(from_coord, to_coord) → float`` step-cost function.
        When omitted, every step costs 1 (unit-cost model) and the
        returned ``cost`` equals the path length minus one.
    raise_on_invalid:
        If ``True``, raise ``ValueError``/``TypeError`` on bad inputs.
        If ``False`` (default), return a failure :class:`SearchResult`.

    Returns
    -------
    SearchResult
        A dict-subclass with keys:

        * ``"path"`` – ordered list of ``(row, col)`` coordinates, start → goal;
          empty on failure.
        * ``"cost"`` – total path cost (``float("inf")`` on failure).
        * ``"expanded_nodes"`` – list of coordinates in expansion order.
        * ``"reached_goal"`` – ``True`` iff the goal was reached.

    Notes
    -----
    * BFS **guarantees the shortest path by step count** when all step costs
      are equal.  It does **not** optimise for weighted costs (use A* for
      that).
    * DFS, by contrast, finds *a* path but makes no optimality guarantee.
    """
    field_map = _adapt_field(field)

    is_valid, error_msg = _validate_endpoints(
        field_map, start, goal, raise_on_invalid=raise_on_invalid
    )
    if not is_valid:
        return _failure_result([], error=error_msg)

    # Special case: start is already the goal
    if start == goal:
        return SearchResult(
            path=[start],
            cost=0,
            expanded_nodes=[start],
            reached_goal=True,
        )

    # BFS core — FIFO queue
    frontier: deque[Coordinate] = deque([start])
    parent: dict[Coordinate, Coordinate | None] = {start: None}
    expanded_nodes: list[Coordinate] = []
    reached_goal = False

    while frontier:
        current = frontier.popleft()
        expanded_nodes.append(current)

        if current == goal:
            reached_goal = True
            break

        r, c = current
        for dr, dc in movement_directions:
            neighbour: Coordinate = (r + dr, c + dc)
            if field_map.is_traversable(neighbour) and neighbour not in parent:
                parent[neighbour] = current
                frontier.append(neighbour)

    if not reached_goal:
        return _failure_result(expanded_nodes)

    path = _reconstruct_path(parent, start, goal)
    if not path:
        return _failure_result(expanded_nodes)

    # Compute cost
    if cost_fn is None:
        total_cost: float | int = len(path) - 1
    else:
        total_cost = sum(cost_fn(path[i], path[i + 1]) for i in range(len(path) - 1))

    return SearchResult(
        path=path,
        cost=total_cost,
        expanded_nodes=expanded_nodes,
        reached_goal=True,
    )


def depth_first_search(
    field: FieldMap | Sequence[Sequence[Any]],
    start: Coordinate,
    goal: Coordinate,
    *,
    movement_directions: Sequence[Coordinate] = CARDINAL_DIRECTIONS,
    cost_fn: Callable[[Coordinate, Coordinate], float] | None = None,
    raise_on_invalid: bool = False,
) -> SearchResult:
    """Find a path from *start* to *goal* using **Depth-First Search**.

    DFS explores as far as possible before back-tracking (LIFO stack).  It
    is **complete** on finite graphs (it will find *a* path when one exists)
    but it does **not** return the shortest path in general.

    Parameters
    ----------
    field:
        Same as :func:`breadth_first_search`. **The field is never mutated.**
    start:
        ``(row, col)`` start coordinate.
    goal:
        ``(row, col)`` goal coordinate.
    movement_directions:
        Ordered sequence of legal ``(Δrow, Δcol)`` moves.  To preserve
        deterministic, left-to-right neighbour expansion, the stack pushes
        neighbours in *reversed* direction order so the first direction is
        popped first.
    cost_fn:
        Optional step-cost function.  When omitted, unit cost is used.
    raise_on_invalid:
        If ``True``, raise on bad inputs; otherwise return a failure result.

    Returns
    -------
    SearchResult
        Same schema as :func:`breadth_first_search`.

    Notes
    -----
    * DFS path length depends heavily on neighbour order and graph shape.
      It can be much longer than BFS's shortest path.
    * DFS uses less memory than BFS on wide grids (frontier is at most as
      deep as the longest path explored).
    * For shortest paths, prefer BFS (uniform costs) or A* (weighted costs).
    """
    field_map = _adapt_field(field)

    is_valid, error_msg = _validate_endpoints(
        field_map, start, goal, raise_on_invalid=raise_on_invalid
    )
    if not is_valid:
        return _failure_result([], error=error_msg)

    # Special case
    if start == goal:
        return SearchResult(
            path=[start],
            cost=0,
            expanded_nodes=[start],
            reached_goal=True,
        )

    # DFS core — LIFO stack
    # Push in reversed neighbour order so the *first* direction is examined
    # first (matches BFS deterministic ordering).
    stack: list[Coordinate] = [start]
    parent: dict[Coordinate, Coordinate | None] = {start: None}
    expanded_nodes: list[Coordinate] = []
    reached_goal = False

    while stack:
        current = stack.pop()
        expanded_nodes.append(current)

        if current == goal:
            reached_goal = True
            break

        r, c = current
        # Reverse so the first direction ends up on top of the stack
        for dr, dc in reversed(list(movement_directions)):
            neighbour: Coordinate = (r + dr, c + dc)
            if field_map.is_traversable(neighbour) and neighbour not in parent:
                parent[neighbour] = current
                stack.append(neighbour)

    if not reached_goal:
        return _failure_result(expanded_nodes)

    path = _reconstruct_path(parent, start, goal)
    if not path:
        return _failure_result(expanded_nodes)

    if cost_fn is None:
        total_cost: float | int = len(path) - 1
    else:
        total_cost = sum(cost_fn(path[i], path[i + 1]) for i in range(len(path) - 1))

    return SearchResult(
        path=path,
        cost=total_cost,
        expanded_nodes=expanded_nodes,
        reached_goal=True,
    )

