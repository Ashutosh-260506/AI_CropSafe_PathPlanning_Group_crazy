# Team Tasks and Contribution Tracking

This division follows the supplied assignment's suggested responsibilities, grouped into six deliverables. Each member should be able to explain the overall approach as well as their own work.

| Member | Assignment scope | Main deliverables | Status | Work score |
|---|---|---|---|---|
| Hitesh (Member 1) | Field environment and problem formulation | Grid/map model, field symbols and scenario, initial state/actions/transition/goal/cost documentation | No Hitesh-authored deliverable identified in the pulled commits; review pending | 0 / 100 repo-progress points |
| Dushyant (Member 2) | BFS and DFS | Both search implementations, shared result format, correctness notes for path completeness/optimality | BFS/DFS, docs, and 25 test methods present; integration review pending | 90 / 100 provisional |
| Ricky (Member 3) | Greedy Best-First Search and heuristic design | Greedy implementation, Manhattan heuristic, explanation of assumptions and limitations | GBFS, heuristic, demo, docs, and tests present; integration review pending | 85 / 100 provisional |
| Aashutosh (Member 4) | A* and crop-safe cost | A* implementation, crop buffer/proximity penalty model, examples showing safer route preference | A* tests present; no A* implementation change appears in the merged Aashutosh commit; review/ownership clarification pending | 50 / 100 provisional |
| Sharad (Member 5) | Unknown-obstacle detection and replanning | Hidden obstacle scenario, simulated detection, known-map update, A* replanning and replan count | Complete in starter runner: hidden obstacle detected; one replan; goal reached | 90 / 100 provisional |
| Sharad (Member 6) | Testing, visualization, evaluation, and documentation | Integration runner, field/path visualization, algorithm comparison, output files, README and focused checks | Core artifacts present; integration with contributed BFS/DFS/GBFS modules is incomplete | 70 / 100 provisional |

Sharad's evidence: `src/cropsafe_pathplanning/replanning.py`, `tests/test_replanning.py` (2 test methods), `results/algorithm_comparison.csv`, `results/replanning_summary.csv`, `results/screenshots/`, and the root `README.md`. The results runner still invokes the original algorithms in `src/cropsafe_pathplanning/search.py`.

Dushyant's evidence: `src/cropsafe_pathplanning/bfs_dfs.py` (BFS + DFS), `tests/test_bfs_dfs.py` (25 test methods present), and `docs/BFS_DFS.md`. Tests were not rerun as part of this static review.

Ricky's evidence: `src/cropsafe_pathplanning/gbfs.py`, `src/cropsafe_pathplanning/heuristic.py`, `src/cropsafe_pathplanning/demo.py`, `tests/test_gbfs.py`, `tests/test_heuristic.py`, and `docs/GREEDY_BEST_FIRST_SEARCH.md`. The GBFS module accepts the shared `FieldMap` interface; it is not currently called by the root evaluation runner.

Aashutosh's merged commit adds `tests/test_astar.py` and updates result CSVs. The A* implementation tested there is in the earlier `src/cropsafe_pathplanning/search.py` baseline, with no A* source change in that merged commit. Credit for implementing A* or its crop-safe cost model therefore remains subject to confirmation of authorship.

## Review findings

- **Integration gap:** `src/cropsafe_pathplanning/__main__.py` imports `ALGORITHMS` from `src/cropsafe_pathplanning/search.py`. It does not invoke the contributed `bfs_dfs.py` or `gbfs.py` implementations, so the checked-in comparison output is not yet a comparison using all teammates' submitted modules.
- **Interface mismatch:** the original runner uses `model.Field` and a `SearchResult` with an integer expanded-node count. The contributed BFS/DFS/GBFS use `types.FieldMap` and a `SearchResult` whose `expanded_nodes` field is a list. GBFS's duck-typed adapter expects `is_traversable`, which `model.Field` does not provide. Agree on one field/result interface before integrating.
- **Test status:** the repository contains 47 test methods across the listed test modules. No test suite was run during this review; status above records source/test presence, not a fresh passing result.
- **Repository hygiene:** the Dushyant merge added a macOS `.DS_Store` file; it should be removed from version control and ignored.

## Shared interface and integration expectations

- Agree on coordinate format, map representation, and movement rules before parallel implementation.
- Each search should use a consistent result structure containing at least `path`, `cost`, `expanded_nodes`, and `reached_goal`; elapsed time can be measured by the evaluation runner.
- Report no-path cases clearly. Do not count a failed route as a successful zero-length path.
- Use the same start, goal, known obstacles, and cost settings when comparing algorithms. Record any settings that differ.
- Keep the true map and robot-known map distinct for the unknown-obstacle experiment.
- Review each other's work and ensure the complete program runs from the documented setup before submission. Sharad's integration runner currently uses the shared starter modules; peer review against the members' implementations is pending.

## Score rubric (suggested)

The scores are provisional repo-review scores based on the merged files and apparent contribution evidence, not grades for work outside the repository. A score does not assert tests passed unless they were actually run. The team should confirm authorship and revise scores after code review, integration, and a shared test run.

| Criterion | Weight |
|---|---:|
| Completeness against assigned scope and acceptance evidence | 40 |
| Correctness and integration with agreed interfaces | 30 |
| Explanation, documentation, and ability to present the work | 20 |
| Collaboration, review, and timely handoff | 10 |
| **Total** | **100** |

Record reviewer notes or evidence beside each score when work is assessed. The score should reflect completed and reviewed work, not task difficulty alone.

## Identity details to complete

The assignment asks for full names and roll numbers. Add each member's official full name and roll number here before submission; only first names were provided in this request.
