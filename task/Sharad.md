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

## Acceptance evidence

- The hidden obstacle is absent from the initial known map, discovered during the simulated run, and causes a map update and replan when it blocks the route.
- Evaluation compares all algorithms on documented, repeatable inputs and reports failures honestly.
- A new teammate can set up and run the finished project from the README.

## Contribution score

Pending review: ___ / 100. Record reviewer and evidence in `TEAM_TASKS.md` after completion.
