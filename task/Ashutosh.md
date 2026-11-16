# Ashutosh — Member 4

## Scope

Implement A* and the crop-safe route cost model.

## Tasks

- [x] Implement A* with `f(n) = g(n) + h(n)` using the agreed interfaces.
- [x] Add a documented configurable cost/penalty for routes near protected crop regions; protected cells remain forbidden.
- [x] Use a heuristic that stays a lower bound for the chosen movement-cost model, and state assumptions behind optimality.
- [x] Return path cost, expanded-node count, and reached-goal status consistently.
- [x] Demonstrate a case where a safer alternative is selected when its total cost is lower.

## Deliverables & Completion Evidence

| Deliverable | File | Status | Description |
|---|---|---|---|
| A* Search Implementation | `src/cropsafe_pathplanning/astar.py` | ✅ Done | `a_star_search` using $f(n) = g(n) + h(n)$, min-heap with deterministic FIFO tie-breaking, dual-nature `ExpandedNodeList`. |
| Crop-Safe Cost Model | `src/cropsafe_pathplanning/astar.py` | ✅ Done | `crop_safe_step_cost`, `make_crop_safe_cost_fn`, and `is_adjacent_to_protected` with configurable risk penalties. |
| Comprehensive Unit Tests | `tests/test_astar.py` | ✅ Done | 16/16 test methods passing covering edge cases, contracts, immutability, admissibility, and GBFS contrast. |
| Technical Documentation | `docs/ASTAR.md` | ✅ Done | Mathematical proofs of admissibility & consistency, cost model equations, case studies, and architecture bridge details. |
| Shared Result Interface | `src/cropsafe_pathplanning/astar.py` | ✅ Done | Standard dictionary contract + property accessors (`.path`, `.cost`, `.expanded_nodes`, `.reached_goal`, `.path_length`, `.algorithm`). |
| Replanning Integration | `src/cropsafe_pathplanning/astar.py` | ✅ Done | Fully interoperable with `model.Field` and Sharad's `simulate_replanning`. |

## Acceptance Evidence

- **Obstacle & Crop Avoidance:** A* strictly avoids obstacles and forbidden crop regions (`test_astar_avoids_blocked_cells`).
- **Cost Influence & Safer Route Preference:** Increasing `risk_penalty` from 0.0 to 3.0 induces A* to route wide around crop buffers (length 10, cost 10) instead of hugging crop rows (length 8, cost 8) (`test_safety_penalty_changes_route`).
- **GBFS vs A\* Contrast:** Demonstrates that GBFS ($f = h$) falls into expensive crop proximity buffers (>25 cost), while A* ($f = g + h$) selects the safe detour (cost 7.0) (`test_astar_cost_optimality_vs_gbfs`).
- **Interface Bridge:** `ExpandedNodeList` satisfies both list contract (`isinstance(..., list)`) and integer arithmetic (`expanded_total += result.expanded_nodes`).
- **Test Status:** 16/16 unit tests passing (`python -m unittest tests/test_astar.py -v`). Full test suite passes 110/110 tests.

## Contribution score

- **Claimed Score:** 100 / 100
- **Evidence:** Dedicated algorithm and cost module `src/cropsafe_pathplanning/astar.py`, comprehensive documentation `docs/ASTAR.md`, and 16 unit tests in `tests/test_astar.py`.
