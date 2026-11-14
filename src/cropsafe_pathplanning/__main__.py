<<<<<<< HEAD
"""Run the crop-safe path-planning demo and write evaluation artifacts."""

from __future__ import annotations

import csv
from pathlib import Path

from .model import demo_field, demo_unknown_obstacle
from .replanning import simulate_replanning
from .search import ALGORITHMS
from .visualize import save_field_plot, save_replanning_plot


ROOT = Path(__file__).resolve().parents[2]
RESULTS = ROOT / "results"


def main() -> None:
    field = demo_field()
    results = [algorithm(field) for algorithm in ALGORITHMS]
    RESULTS.mkdir(exist_ok=True)
    with (RESULTS / "algorithm_comparison.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["Algorithm", "Path length (steps)", "Path cost", "Expanded nodes", "Time (ms)", "Reached goal"])
        for result in results:
            writer.writerow([result.algorithm, result.path_length, f"{result.cost:.2f}", result.expanded_nodes, f"{result.elapsed_seconds * 1000:.3f}", result.reached_goal])
    print("Algorithm comparison (same known map)")
    print(f"{'Algorithm':<12} {'Steps':>7} {'Cost':>8} {'Expanded':>10} {'Time ms':>10} {'Reached':>9}")
    for result in results:
        print(f"{result.algorithm:<12} {result.path_length:>7} {result.cost:>8.2f} {result.expanded_nodes:>10} {result.elapsed_seconds * 1000:>10.3f} {str(result.reached_goal):>9}")
    save_field_plot(field, results, RESULTS / "screenshots" / "algorithm_paths.svg")

    hidden = demo_unknown_obstacle()
    online = simulate_replanning(field, {hidden})
    actual_field = field.with_obstacles(field.obstacles | {hidden})
    print("\nOnline obstacle experiment")
    print(f"Detected obstacles: {online.detected_obstacles}")
    print(f"Replanning operations: {online.replans}")
    print(f"Reached goal: {online.reached_goal}")
    print(f"Final route length: {max(0, len(online.final_path) - 1)} steps")
    save_replanning_plot(actual_field, online.final_path, [hidden], RESULTS / "screenshots" / "online_replanning.svg")
    with (RESULTS / "replanning_summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["Detected obstacles", "Replanning operations", "Final route length (steps)", "Expanded nodes", "Planning time (ms)", "Reached goal"])
        writer.writerow([";".join(map(str, online.detected_obstacles)), online.replans, max(0, len(online.final_path) - 1), online.total_expanded_nodes, f"{online.total_planning_seconds * 1000:.3f}", online.reached_goal])
    print(f"\nSaved CSV summaries and plots under {RESULTS.relative_to(ROOT)}/")

=======
"""Package entry point for `python -m cropsafe_pathplanning`."""

from cropsafe_pathplanning.demo import main
>>>>>>> ricky

if __name__ == "__main__":
    main()
