# Sharad — Members 5 and 6

## Scope

Own the unknown-obstacle simulation and A* replanning (Member 5), plus integration, testing, visualization, performance evaluation, and final documentation (Member 6).

## Tasks

### Member 5: detection and replanning

- Keep the true field map distinct from the robot's known map.
- Place at least one obstacle on a route that is initially unknown to the robot.
- Simulate detection during travel, update the known map, and invoke A* when the active route is blocked.
- Count replanning operations and record the final route/outcome.

### Member 6: integration and reporting

- Integrate the environment and four algorithms using their agreed interfaces.
- Visualize the field, crop rows/protection areas, obstacles, start/goal, and paths with a legend.
- Measure path length/cost, expanded nodes, execution time, and reached-goal status for each search on fixed scenarios.
- Produce the required comparison table and brief observations, including replanning count for the online-obstacle run.
- Save representative output screenshots in `results/screenshots/` and finish setup/run documentation in the root README.
- Maintain the team contribution record and add full official names and roll numbers once supplied.

## Completion record

- [x] The hidden obstacle is absent from the initial known map, discovered during the simulated run, and causes a map update and replan from the current position.
- [x] Evaluation records path length/cost, expanded nodes, execution time, goal status, and replanning count.
- [x] Field/path visualizations and CSV summaries are generated from the documented demo command.
- [x] README includes setup, run, output, and focused test instructions.
- [x] Integrated the submitted BFS, DFS, Greedy, and A* implementations through the common evaluation runner.
- [x] Ran the full suite: 115 tests pass; refreshed the algorithm/replanning CSVs and SVG figures.
- [ ] Add official names and roll numbers when the group provides them; contribution score remains provisional pending presentation review.

## Contribution score

Provisional repository score: 95 / 100. Evidence and rubric are recorded in `TEAM_TASKS.md`.
