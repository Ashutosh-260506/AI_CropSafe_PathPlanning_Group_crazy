"""Classical graph-search algorithms for the shared field model."""

from __future__ import annotations

import heapq
from collections import deque
from dataclasses import dataclass
from time import perf_counter
from typing import Callable

from .model import Field, Position


@dataclass
class SearchResult:
    algorithm: str
    path: list[Position]
    cost: float
    expanded_nodes: int
    reached_goal: bool
    elapsed_seconds: float = 0.0

    @property
    def path_length(self) -> int:
        return max(0, len(self.path) - 1)


def manhattan(a: Position, b: Position) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _reconstruct(parent: dict[Position, Position], start: Position, goal: Position) -> list[Position]:
    if goal != start and goal not in parent:
        return []
    path = [goal]
    while path[-1] != start:
        path.append(parent[path[-1]])
    path.reverse()
    return path


def _result(name: str, field: Field, path: list[Position], expanded: int, started: float) -> SearchResult:
    cost = sum(field.step_cost(cell) for cell in path[1:]) if path else 0.0
    return SearchResult(name, path, cost, expanded, bool(path), perf_counter() - started)


def breadth_first_search(field: Field) -> SearchResult:
    started = perf_counter()
    frontier = deque([field.start])
    parent: dict[Position, Position] = {}
    discovered = {field.start}
    expanded = 0
    while frontier:
        current = frontier.popleft()
        expanded += 1
        if current == field.goal:
            break
        for neighbor in field.neighbors(current):
            if neighbor not in discovered:
                discovered.add(neighbor)
                parent[neighbor] = current
                frontier.append(neighbor)
    return _result("BFS", field, _reconstruct(parent, field.start, field.goal), expanded, started)


def depth_first_search(field: Field) -> SearchResult:
    started = perf_counter()
    frontier = [field.start]
    parent: dict[Position, Position] = {}
    discovered = {field.start}
    expanded = 0
    while frontier:
        current = frontier.pop()
        expanded += 1
        if current == field.goal:
            break
        for neighbor in reversed(tuple(field.neighbors(current))):
            if neighbor not in discovered:
                discovered.add(neighbor)
                parent[neighbor] = current
                frontier.append(neighbor)
    return _result("DFS", field, _reconstruct(parent, field.start, field.goal), expanded, started)


def _best_first(field: Field, heuristic: Callable[[Position, Position], float], *, use_cost: bool, name: str) -> SearchResult:
    started = perf_counter()
    serial = 0
    frontier: list[tuple[float, int, Position]] = [(heuristic(field.start, field.goal), serial, field.start)]
    parent: dict[Position, Position] = {}
    best_cost = {field.start: 0.0}
    closed: set[Position] = set()
    expanded = 0
    while frontier:
        _, _, current = heapq.heappop(frontier)
        if current in closed:
            continue
        closed.add(current)
        expanded += 1
        if current == field.goal:
            break
        for neighbor in field.neighbors(current):
            candidate_cost = best_cost[current] + field.step_cost(neighbor)
            if candidate_cost < best_cost.get(neighbor, float("inf")):
                best_cost[neighbor] = candidate_cost
                parent[neighbor] = current
                serial += 1
                priority = candidate_cost + heuristic(neighbor, field.goal) if use_cost else heuristic(neighbor, field.goal)
                heapq.heappush(frontier, (priority, serial, neighbor))
    return _result(name, field, _reconstruct(parent, field.start, field.goal), expanded, started)


def greedy_best_first_search(field: Field) -> SearchResult:
    return _best_first(field, manhattan, use_cost=False, name="Greedy")


def a_star_search(field: Field) -> SearchResult:
    return _best_first(field, manhattan, use_cost=True, name="A*")


ALGORITHMS = (breadth_first_search, depth_first_search, greedy_best_first_search, a_star_search)
