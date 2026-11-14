# AI-Based Crop-Safe Path Planning

> A grid-based simulation of an autonomous robot navigating a potato field while protecting crops, avoiding obstacles, and replanning when it discovers an unknown obstacle.

> **Project status:** planning and repository setup. Implementation and measured results are not added yet.

## Assignment goals

The project follows the supplied Theory Coding Assignment. It will model a 2D field, formulate a search problem, implement BFS, DFS, Greedy Best-First Search, and A*, protect crop regions, simulate discovery of an unknown obstacle, and compare the algorithms.

## Planned repository layout

```text
.
├── README.md
├── requirements.txt
├── src/
│   └── cropsafe_pathplanning/
├── tests/
├── results/
│   └── screenshots/
└── task/
    ├── TEAM_TASKS.md
    ├── Hitesh.md
    ├── Dushyant.md
    ├── Ricky.md
    ├── Aashutosh.md
    └── Sharad.md
```

The `src` and `tests` directories are reserved for implementation and validation. Keep algorithms separate from field data, visualization, and evaluation so each algorithm can be compared on the same scenarios.

## Planned model and design

- Represent each traversable location as a grid coordinate `(row, column)`.
- Use four-directional movement (up, down, left, right) initially. Manhattan distance is suitable as a lower bound when each move costs at least one.
- Keep the true field map separate from the robot's known map. At least one obstacle begins hidden from the robot; when detected, update its known map and replan with A*.
- Treat crop-protection cells and known obstacles as blocked. Model a configurable safety buffer or proximity penalty around protected crop regions, and document its route-cost effect.
- Return comparable run data from each search: path, path cost/length, expanded-node count, elapsed time, and reached-goal status. Record replanning count for the dynamic-obstacle run.
- Fix and document scenario inputs when comparing algorithms. Report measured results rather than predicted performance.

## Requirements

Python 3.10 or newer is recommended. The initial implementation can use the Python standard library for search and timing. Add NumPy or Matplotlib to `requirements.txt` only if the team chooses them for the grid or plots.

## Setup and run

From the repository root, create and activate a virtual environment:

```bash
python -m venv .venv
```

On macOS/Linux:

```bash
source .venv/bin/activate
```

On Windows PowerShell:

```powershell
.venv\Scripts\Activate.ps1
```

Install dependencies once `requirements.txt` is populated:

```bash
python -m pip install -r requirements.txt
```

Run instructions will be added when the entry point is implemented. The intended command is `python -m cropsafe_pathplanning` with `src` on the import path or an installed package. Do not treat that command as runnable yet.

## Algorithms and evaluation

- **BFS:** baseline for equal-cost, four-directional movement; finds a minimum-step path when all step costs are equal.
- **DFS:** explores deeply and is included for comparison; it does not generally guarantee a shortest path.
- **Greedy Best-First Search:** prioritizes estimated distance to the goal; it can be fast but is not generally cost-optimal.
- **A\*:** prioritizes `g(n) + h(n)`, where `g` is cost so far and `h` estimates remaining cost. For four-directional movement, use Manhattan distance as the base heuristic. If risk penalties are nonnegative, this base movement-distance heuristic remains a lower bound; document the cost model and optimality assumptions.

The final report should include a comparison table and observations for a fixed field scenario, plus a separate online-obstacle run with replanning count. Screenshots of the rendered field and output belong in `results/screenshots/`.

## Team assignments and contribution scores

See [task/TEAM_TASKS.md](task/TEAM_TASKS.md) for the consolidated assignments, acceptance criteria, and contribution-score tracking. Replace member placeholders with full names and roll numbers before submission.

## References

- Peter Hart, Nils Nilsson, and Bertram Raphael, [A Formal Basis for the Heuristic Determination of Minimum Cost Paths (1968)](https://ai.stanford.edu/~nilsson/OnlinePubs-Nils/PublishedPapers/astar.pdf).
- Amit Patel, [Introduction to the A* Algorithm](https://www.redblobgames.com/pathfinding/a-star/introduction.html) and [A* implementation guide](https://www.redblobgames.com/pathfinding/a-star/implementation.html), for grid-search concepts and Manhattan distance.
- Python Packaging User Guide, [Installing packages using pip and virtual environments](https://packaging.python.org/en/latest/guides/installing-using-pip-and-virtual-environments/).

Credit any additional external references or code used in the final submission.
