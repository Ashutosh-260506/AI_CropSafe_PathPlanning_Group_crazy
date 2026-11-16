# Team Tasks and Contribution Tracking

This division follows the supplied assignment's suggested responsibilities, grouped into six deliverables. Each member should be able to explain the overall approach as well as their own work.

| Member | Assignment scope | Main deliverables | Status | Work score |
|---|---|---|---|---|
| Hitesh (Member 1) | Field environment and problem formulation | Grid/map model, field symbols and scenario, initial state/actions/transition/goal/cost documentation | Complete; scenario, formal definition, visualization legend, and tests integrated | 95 / 100 provisional |
| Dushyant (Member 2) | BFS and DFS | Both search implementations, shared result format, correctness notes for path completeness/optimality | Complete; BFS/DFS, docs, and tests integrated | 95 / 100 provisional |
| Ricky (Member 3) | Greedy Best-First Search and heuristic design | Greedy implementation, Manhattan heuristic, explanation of assumptions and limitations | Complete; GBFS and heuristic integrated into shared evaluation | 95 / 100 provisional |
| Ashutosh (Member 4) | A* and crop-safe cost | A* implementation, crop buffer/proximity penalty model, examples showing safer route preference | Complete; A*, crop-risk costs, docs, and tests integrated | 95 / 100 provisional |
| Sharad (Member 5) | Unknown-obstacle detection and replanning | Hidden obstacle scenario, simulated detection, known-map update, A* replanning and replan count | Complete; hidden obstacle detected, one replan, goal reached | 95 / 100 provisional |
| Sharad (Member 6) | Testing, visualization, evaluation, and documentation | Integration runner, field/path visualization, algorithm comparison, output files, README and final contribution record | Complete; all algorithms use the common runner and all 115 tests pass | 95 / 100 provisional |

## Contribution evidence

- **Hitesh:** `src/cropsafe_pathplanning/field_scenario.py`, `docs/FIELD_ENVIRONMENT.md`, and `tests/test_field_scenario.py` (54 test methods after integration fixes).
- **Dushyant:** `src/cropsafe_pathplanning/bfs_dfs.py`, `tests/test_bfs_dfs.py` (25 test methods), and `docs/BFS_DFS.md`.
- **Ricky:** `src/cropsafe_pathplanning/gbfs.py`, `src/cropsafe_pathplanning/heuristic.py`, `src/cropsafe_pathplanning/demo.py`, `tests/test_gbfs.py`, `tests/test_heuristic.py`, and `docs/GREEDY_BEST_FIRST_SEARCH.md`.
- **Ashutosh:** `src/cropsafe_pathplanning/astar.py`, `docs/ASTAR.md`, and `tests/test_astar.py` (17 test methods after cost-validation coverage).
- **Sharad:** `src/cropsafe_pathplanning/evaluation.py`, `src/cropsafe_pathplanning/replanning.py`, `src/cropsafe_pathplanning/visualize.py`, `tests/test_replanning.py`, `tests/test_evaluation.py`, the result CSVs/SVGs, and the root `README.md`.

## Final integration and verification

- `evaluation.py` runs BFS, DFS, Greedy Best-First Search, and A* on the same 12×18 known field and normalizes results for a fair comparison.
- `replanning.py` uses the submitted A* module. The hidden obstacle at `(10, 9)` is absent from the initial map, detected during the route, added to the known map, and causes one replan.
- Crop rows, protected cells, and obstacles are consistently blocked in `Field`, `FieldMap`, scenario construction, and documentation. Adjacent legal aisle cells receive the configured crop-risk surcharge.
- Crop-risk helpers reject invalid penalties/base costs; A* rejects transition costs below 1.0 to preserve its Manhattan heuristic assumptions.
- Verification on 16 November 2026: **115 tests passed** with `PYTHONPATH=src python -m unittest discover -s tests -v`. The demo completed all four searches and reached the goal after one online replan.
- Removed the accidental `.DS_Store` and nested gitlink; `.gitignore` and `requirements.txt` contain no merge markers.

## Contribution score rubric

Scores are provisional repository-completion scores based on code, documentation, and test evidence. They do not evaluate oral presentation or contributions not visible in the repository; the group can revise them after the final peer review.

| Criterion | Weight |
|---|---:|
| Completeness against assigned scope and acceptance evidence | 40 |
| Correctness and integration with agreed interfaces | 30 |
| Explanation, documentation, and ability to present the work | 20 |
| Collaboration, review, and timely handoff | 10 |
| **Total** | **100** |

## Identity details to complete

Add each member's official full name and roll number before submission; these details were not provided in the task brief.
