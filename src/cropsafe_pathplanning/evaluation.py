"""Common evaluation adapter for the team search implementations."""

from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter

from .astar import a_star_search
from .bfs_dfs import breadth_first_search, depth_first_search
from .field_scenario import make_field
from .gbfs import greedy_best_first_search
from .model import Field, Position
from .types import FieldMap


@dataclass
class SearchSummary:
    algorithm: str
    path: list[Position]
    cost: float
    expanded_nodes: int
    reached_goal: bool
    elapsed_seconds: float

    @property
    def path_length(self) -> int:
        return max(0, len(self.path) - 1)


def evaluate_algorithms(field: Field | None = None, field_map: FieldMap | None = None) -> list[SearchSummary]:
    """Run BFS, DFS, GBFS, and A* on the same known field and cost settings."""
    field = field or make_field()
    if field_map is None:
        field_map = FieldMap(field.height, field.width, blocked_cells=set(field.blocked))
    if (field.height, field.width) != (field_map.height, field_map.width):
        raise ValueError("Field and FieldMap dimensions must match")
    if field.blocked != field_map.blocked_cells:
        raise ValueError("Field and FieldMap must contain the same known blocked cells")

    cost_fn = lambda current, neighbor: field.step_cost(neighbor)
    calls = (
        ("BFS", lambda: breadth_first_search(field_map, field.start, field.goal, cost_fn=cost_fn)),
        ("DFS", lambda: depth_first_search(field_map, field.start, field.goal, cost_fn=cost_fn)),
        ("Greedy", lambda: greedy_best_first_search(field_map, field.start, field.goal, cost_fn=cost_fn)),
        ("A*", lambda: a_star_search(field)),
    )

    summaries: list[SearchSummary] = []
    for name, call in calls:
        started = perf_counter()
        result = call()
        elapsed = perf_counter() - started
        expanded = result.expanded_nodes
        expanded_count = expanded if isinstance(expanded, int) else len(expanded)
        summaries.append(
            SearchSummary(
                algorithm=name,
                path=list(result.path),
                cost=float(result.cost),
                expanded_nodes=expanded_count,
                reached_goal=bool(result.reached_goal),
                elapsed_seconds=elapsed,
            )
        )
    return summaries
