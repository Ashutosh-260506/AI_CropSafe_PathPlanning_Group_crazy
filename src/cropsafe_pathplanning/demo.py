"""Reproducible demonstration of Greedy Best-First Search for CropSafe Path Planning.

Provides two fixed examples:
    - Example A: A reachable field configuration with obstacles and crop zones.
    - Example B: An unreachable field scenario where the goal is completely blocked.
"""

from __future__ import annotations

from cropsafe_pathplanning.gbfs import greedy_best_first_search
from cropsafe_pathplanning.types import Coordinate, FieldMap


def run_example_a() -> None:
    """Example A: A grid where the goal is reachable through open field aisles."""
    print("=" * 70)
    print("EXAMPLE A: Reachable Goal with Crop Rows and Obstacles")
    print("=" * 70)

    # 7x9 field:
    # 0 = free soil/aisle
    # 1 = obstacle / potato crop row
    field_matrix = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 0, 1, 1, 0, 1, 0],
        [0, 1, 1, 0, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 1, 1, 0, 1, 1, 0, 0, 0],
        [0, 1, 1, 0, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
    ]

    start: Coordinate = (0, 0)
    goal: Coordinate = (6, 8)

    field = FieldMap.from_matrix(field_matrix)

    print("\nInitial Field Map:")
    print("Legend: S = Start, G = Goal, # = Blocked Crop/Obstacle, . = Free Cell")
    print(field.render(start=start, goal=goal))

    result = greedy_best_first_search(field, start, goal)

    print(f"\nStart Coordinate : {start}")
    print(f"Goal Coordinate  : {goal}")
    print(f"Goal Reached     : {result.reached_goal}")
    print(f"Actual Path Cost : {result.cost}")
    print(f"Path Length      : {len(result.path)} coordinates")
    print(f"Returned Path    : {result.path}")
    print(f"Expanded Nodes ({len(result.expanded_nodes)} total):")
    print(f"  {result.expanded_nodes}")

    print("\nField with Solved Path (*):")
    print(field.render(path=result.path, start=start, goal=goal))


def run_example_b() -> None:
    """Example B: A grid where obstacles surround the goal, making it unreachable."""
    print("\n" + "=" * 70)
    print("EXAMPLE B: Unreachable Goal (Blocked by Enclosed Obstacles)")
    print("=" * 70)

    # 5x5 field where goal at (2, 2) is completely surrounded by obstacles
    field_matrix = [
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 1, 0, 1, 0],  # Cell (2, 2) is enclosed by 1s
        [0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0],
    ]

    start: Coordinate = (0, 0)
    goal: Coordinate = (2, 2)

    field = FieldMap.from_matrix(field_matrix)

    print("\nInitial Field Map:")
    print("Legend: S = Start, G = Goal, # = Blocked Crop/Obstacle, . = Free Cell")
    print(field.render(start=start, goal=goal))

    result = greedy_best_first_search(field, start, goal)

    print(f"\nStart Coordinate : {start}")
    print(f"Goal Coordinate  : {goal}")
    print(f"Goal Reached     : {result.reached_goal}")
    print(f"Actual Path Cost : {result.cost}")
    print(f"Returned Path    : {result.path}")
    print(f"Expanded Nodes ({len(result.expanded_nodes)} total):")
    print(f"  {result.expanded_nodes}")

    print("\nField Exploration State (· = Expanded Nodes):")
    print(field.render(start=start, goal=goal, expanded=result.expanded_nodes))


def main() -> None:
    """Execute all demonstration scenarios."""
    run_example_a()
    run_example_b()


if __name__ == "__main__":
    main()
