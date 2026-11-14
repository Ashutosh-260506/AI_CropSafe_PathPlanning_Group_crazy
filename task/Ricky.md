# Ricky — Member 3

## Scope

Implement Greedy Best-First Search and define the informed-search heuristic.

## Tasks

- Implement Greedy Best-First Search using the agreed map and result interfaces.
- Define Manhattan distance for the proposed four-directional grid.
- Document how movement rules affect heuristic choice and why Greedy search does not guarantee a least-cost route.
- Exclude blocked cells and report unreachable goals clearly.
- Supply reproducible examples and expanded-node data through the shared result format.

## Acceptance evidence

- The heuristic returns zero at the goal and a nonnegative estimate elsewhere (verified in `tests/test_heuristic.py`).
- The algorithm reaches valid goals when a route exists and handles no-route cases (verified in `tests/test_gbfs.py`).
- Its output strictly follows the shared result format (`path`, `cost`, `expanded_nodes`, `reached_goal`) and can be directly compared with BFS, DFS, and A* on identical scenarios.
- Comprehensive technical documentation completed in `docs/GREEDY_BEST_FIRST_SEARCH.md`.
- Reproducible demonstrations (Examples A and B) available via `python -m cropsafe_pathplanning`.

## Contribution score

Pending review: ___ / 100. Record reviewer and evidence in `TEAM_TASKS.md` after completion.
