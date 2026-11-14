# Aashutosh — Member 4

## Scope

Implement A* and the crop-safe route cost model.

## Tasks

- Implement A* with `f(n) = g(n) + h(n)` using the agreed interfaces.
- Add a documented configurable cost/penalty for routes near protected crop regions; protected cells remain forbidden.
- Use a heuristic that stays a lower bound for the chosen movement-cost model, and state assumptions behind optimality.
- Return path cost, expanded-node count, and reached-goal status consistently.
- Demonstrate a case where a safer alternative is selected when its total cost is lower.

## Acceptance evidence

- A* avoids obstacle and crop-protection cells.
- The safety penalty influences route choice as intended and is included in path cost.
- The implementation can be used by Sharad's replanning simulation.

## Contribution score

Pending review: ___ / 100. Record reviewer and evidence in `TEAM_TASKS.md` after completion.
