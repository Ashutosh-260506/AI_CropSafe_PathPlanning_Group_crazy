# Team Tasks and Contribution Tracking

This division follows the supplied assignment's suggested responsibilities, grouped into six deliverables. Each member should be able to explain the overall approach as well as their own work.

| Member | Assignment scope | Main deliverables | Work score |
|---|---|---|---|
| Hitesh (Member 1) | Field environment and problem formulation | Grid/map model, field symbols and scenario, initial state/actions/transition/goal/cost documentation | Pending review: ___ / 100 |
| Dushyant (Member 2) | BFS and DFS | Both search implementations, shared result format, correctness notes for path completeness/optimality | Pending review: ___ / 100 |
| Ricky (Member 3) | Greedy Best-First Search and heuristic design | Greedy implementation, Manhattan heuristic, explanation of assumptions and limitations | Pending review: ___ / 100 |
| Aashutosh (Member 4) | A* and crop-safe cost | A* implementation, crop buffer/proximity penalty model, examples showing safer route preference | Pending review: ___ / 100 |
| Sharad (Member 5) | Unknown-obstacle detection and replanning | Hidden obstacle scenario, simulated detection, known-map update, A* replanning and replan count | Pending review: ___ / 100 |
| Sharad (Member 6) | Testing, visualization, evaluation, and documentation | Integrate modules, field/path visualization, algorithm comparison table, screenshots, README and final contribution record | Pending review: ___ / 100 |

## Shared interface and integration expectations

- Agree on coordinate format, map representation, and movement rules before parallel implementation.
- Each search should use a consistent result structure containing at least `path`, `cost`, `expanded_nodes`, and `reached_goal`; elapsed time can be measured by the evaluation runner.
- Report no-path cases clearly. Do not count a failed route as a successful zero-length path.
- Use the same start, goal, known obstacles, and cost settings when comparing algorithms. Record any settings that differ.
- Keep the true map and robot-known map distinct for the unknown-obstacle experiment.
- Review each other's work and ensure the complete program runs from the documented setup before submission.

## Score rubric (suggested)

Scores are left pending because the work has not been completed yet.

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
