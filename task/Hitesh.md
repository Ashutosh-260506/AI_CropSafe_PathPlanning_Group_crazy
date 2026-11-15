# Hitesh — Member 1

## Scope

Design the potato-field grid environment and state the search problem formally.

## Tasks

- [x] Define grid coordinates, traversable cells, crop rows, protected cells/buffers, known obstacles, start, and goal.
- [x] Provide a sample field configuration shared by all algorithms.
- [x] Specify initial state, legal actions, transition model, goal test, and path-cost model.
- [x] Confirm movement rules with the team — four-directional movement confirmed and documented.
- [x] Document the map representation and symbol legend for visualization.

## Deliverables

| File | Purpose |
|------|---------|
| `docs/FIELD_ENVIRONMENT.md` | Full formal problem definition, map layout, ASCII legend, SVG colour scheme |
| `src/cropsafe_pathplanning/field_scenario.py` | `FIELD_CONFIG` dict + `make_field()` / `make_field_map()` / `make_field_map_with_hidden()` factories + `render_ascii()` / `print_field_ascii()` |
| `tests/test_field_scenario.py` | 52 unit tests covering config, Field, FieldMap, hidden obstacle, ASCII rendering |

## Acceptance evidence

- [x] `make_field()` and `make_field_map()` confirmed consistent with `model.demo_field()` by unit tests.
- [x] `FIELD_CONFIG` is the single source of truth; `model.py` and `types.py` interfaces both tested.
- [x] Four-directional movement confirmed: `Field.neighbors()`, `FieldMap.get_neighbors()`, `CARDINAL_DIRECTIONS` all tested.
- [x] Start `(6, 1)` and goal `(6, 16)` verified traversable by construction (`Field.__post_init__`) and by tests.
- [x] Blocked-cell entry raises `ValueError`; out-of-bounds cells never yielded by `neighbors()` — tested.
- [x] Step cost ≥ 1.0 for all legal cells (admissibility requirement) — asserted across full grid.
- [x] 52 / 52 tests pass (`python3 -m unittest tests/test_field_scenario.py -v`).

## Formal problem summary

| Element | Value |
|---------|-------|
| Grid | 12 rows × 18 columns |
| Initial state | `(6, 1)` |
| Goal state | `(6, 16)` |
| Actions | UP, DOWN, LEFT, RIGHT (4-directional) |
| Transition | Deterministic; blocked/OOB cells excluded |
| Goal test | `state == (6, 16)` |
| Step cost | `1.0` (open) or `3.0` (adjacent to protected row) |
| Heuristic | Manhattan distance — admissible and consistent |

## Contribution score

Completed. Pending peer review: ___ / 100. Record reviewer and evidence in `TEAM_TASKS.md`.
