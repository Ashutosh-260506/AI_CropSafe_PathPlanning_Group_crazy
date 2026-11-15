# Dushyant — Member 2

## Scope

Implement Breadth-First Search (BFS) and Depth-First Search (DFS).

## Tasks

- Use the agreed field and neighbor interfaces from Hitesh's problem model.
- Return a consistent result with path, cost/length, expanded-node count, and reached-goal status.
- Handle unreachable goals and invalid start/goal cells cleanly.
- Explain that BFS is shortest by steps under equal step costs; DFS does not generally return a shortest path.
- Provide reproducible reachable and unreachable examples.

## Acceptance evidence

- Both algorithms respect blocked cells and movement rules.
- Results use the shared interface and do not mutate the field map.
- A route can be traced from start to goal and checked against the map.

## Completion evidence

| Deliverable | File | Status |
|---|---|---|
| BFS implementation | `src/cropsafe_pathplanning/bfs_dfs.py` | ✅ Done |
| DFS implementation | `src/cropsafe_pathplanning/bfs_dfs.py` | ✅ Done |
| Shared result interface (`SearchResult`, `FieldMap`) | `src/cropsafe_pathplanning/types.py` | ✅ Used, not mutated |
| Reachable example | `tests/test_bfs_dfs.py` Tests 2, 14 | ✅ Passes |
| Unreachable example | `tests/test_bfs_dfs.py` Tests 4, 15 | ✅ Passes |
| BFS optimality explanation | `docs/BFS_DFS.md` | ✅ Done |
| DFS non-optimality explanation | `docs/BFS_DFS.md` | ✅ Done |
| All tests passing | 25/25 | ✅ `python3 -m unittest tests.test_bfs_dfs` |

### Key notes

- **BFS guarantees the shortest path by step count** when all step costs are equal.  
  On an open 5×5 grid from `(0,0)` to `(4,4)`, BFS always returns exactly 8 steps (= Manhattan distance).
- **DFS does not guarantee the shortest path.**  
  It finds *a* valid path (complete on finite graphs) but may return a much longer route.
- Neither algorithm mutates the `FieldMap`; `blocked_cells` is verified unchanged after every run.
- Both accept an optional `cost_fn=(from, to)→float` to plug in weighted-cost models (e.g. `Field.step_cost`).

## Contribution score

Pending peer review: 90 / 100 provisional. Record reviewer and evidence in `TEAM_TASKS.md` after peer review.
