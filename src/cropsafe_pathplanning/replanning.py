"""Simulation of discovering an obstacle and replanning online."""

from __future__ import annotations

from dataclasses import dataclass, replace

from .model import Field, Position
from .search import SearchResult, a_star_search


@dataclass
class ReplanningResult:
    final_path: list[Position]
    replans: int
    detected_obstacles: list[Position]
    reached_goal: bool
    total_expanded_nodes: int
    total_planning_seconds: float


def simulate_replanning(known_field: Field, true_obstacles: set[Position]) -> ReplanningResult:
    """Follow planned routes until the next unknown obstacle is encountered.

    The simulated robot detects an obstacle when it is the next cell on its
    route. It adds that cell to its known map and runs A* again.
    """
    known_obstacles = set(known_field.obstacles)
    current = known_field.start
    full_route = [current]
    detected: list[Position] = []
    replans = expanded_total = 0
    planning_seconds = 0.0
    remaining_actual_obstacles = set(true_obstacles)
    while current != known_field.goal:
        current_map = replace(known_field, start=current, obstacles=set(known_obstacles))
        result: SearchResult = a_star_search(current_map)
        expanded_total += result.expanded_nodes
        planning_seconds += result.elapsed_seconds
        if not result.reached_goal:
            return ReplanningResult(full_route, replans, detected, False, expanded_total, planning_seconds)
        blocked_next = next((cell for cell in result.path[1:] if cell in remaining_actual_obstacles), None)
        if blocked_next is None:
            full_route.extend(result.path[1:])
            current = known_field.goal
            break
        segment = result.path[1:result.path.index(blocked_next)]
        full_route.extend(segment)
        if segment:
            current = segment[-1]
        known_obstacles.add(blocked_next)
        remaining_actual_obstacles.remove(blocked_next)
        detected.append(blocked_next)
        replans += 1
    return ReplanningResult(full_route, replans, detected, current == known_field.goal, expanded_total, planning_seconds)
