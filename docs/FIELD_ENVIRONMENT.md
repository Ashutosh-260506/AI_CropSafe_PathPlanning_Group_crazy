# Field Environment and Formal Problem Definition

> **Author:** Hitesh (Member 1)  
> **Scope:** Grid environment design, sample field configuration, and formal search-problem specification for the AI-CropSafe Path Planning project.

---

## 1. Grid Coordinate System

The field is modelled as a **rectangular 2-D grid**.  
Every cell is identified by an integer tuple `(row, col)`:

| Axis | Range | Direction |
|------|-------|-----------|
| `row` | `0 … height − 1` | top → bottom (increasing downward) |
| `col` | `0 … width  − 1` | left → right (increasing rightward) |

> [!NOTE]
> `(0, 0)` is the **top-left** corner of the field.  
> All indices are **0-based**.

---

## 2. Cell Classification

Every cell in the grid belongs to exactly one of the categories below.

| Category | Symbol | Traversable | Description |
|----------|--------|-------------|-------------|
| Safe ground / path | `.` | ✅ Yes | Open soil or headland the robot may enter freely. |
| Crop row | `C` | ❌ No | Active potato-crop planting row; robot must not enter. |
| Protected buffer | `P` | ❌ No | One cell adjacent to a crop row reserved as a safety margin; counts as blocked. |
| Known obstacle | `X` | ❌ No | Pre-mapped static obstacle (stone, equipment, furrow edge). |
| Hidden / unknown obstacle | `H` | ❌ No (true map) | Obstacle absent from the robot's initial map; discovered at run-time. |
| Start cell | `S` | ✅ Yes | Robot's initial position; must be traversable. |
| Goal cell | `G` | ✅ Yes | Target delivery or treatment station; must be traversable. |
| Planned path cell | `*` | ✅ Yes | Cell on the algorithm's chosen route (visualisation only). |
| Expanded / visited cell | `·` | ✅ Yes | Cell opened during search but not on final path (visualisation only). |

> [!IMPORTANT]
> `blocked = protected ∪ obstacles`.  
> Neither the **start** nor the **goal** may be blocked.  
> A cell that is both a crop row **and** a protected cell is rendered as protected (more restrictive).

---

## 3. Sample Field Configuration (Shared Scenario)

All algorithms use the same **12 × 18** field defined in `model.py → demo_field()`.

```
Grid size:   12 rows × 18 columns   (height=12, width=18)
Start:       (6, 1)
Goal:        (6, 16)
```

### 3.1 Crop Rows

Potato plants occupy **two horizontal bands**:

| Band | Rows | Columns | Symbol |
|------|------|---------|--------|
| Band A | row 3 | cols 1–16 | `C` |
| Band B | row 8 | cols 1–16 | `C` |

```python
crop_rows = {(r, c) for r in (3, 8) for c in range(1, 17)}
```

### 3.2 Protected Cells (Safety Buffers)

The one-cell-wide safe-margin is placed along the **same rows** but over the full column extent that the search model enforces as blocked:

| Buffer | Rows | Columns |
|--------|------|---------|
| Buffer A | row 3 | cols 2–15 |
| Buffer B | row 8 | cols 2–15 |

```python
protected = {(r, c) for r in (3, 8) for c in range(2, 16)}
```

> [!NOTE]
> `protected ⊂ crop_rows` for the demo field. In general the two sets are independent: `crop_rows` is used for colour-coding in the visualiser, while `protected` determines which cells are blocked.

### 3.3 Known Obstacles

Four static obstacles form two symmetric pillar pairs:

| Obstacle | Position |
|----------|----------|
| O1 | `(5, 5)` |
| O2 | `(6, 5)` |
| O3 | `(5, 12)` |
| O4 | `(6, 12)` |

```python
obstacles = {(5, 5), (6, 5), (5, 12), (6, 12)}
```

### 3.4 Hidden / Unknown Obstacle (Online Experiment)

```python
hidden_obstacle = (10, 9)
```

This cell is **not** present in the robot's initial known map. It is revealed when the robot's planned route would step onto it.

### 3.5 Risk Penalty

```python
risk_penalty = 2.0
```

Any traversable cell **orthogonally adjacent** to a `protected` cell incurs an additional cost of `risk_penalty` on top of the base step cost of `1.0`.

---

## 4. ASCII Map (12 × 18 Demo Field)

Column indices:
```
     0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17
   +----------------------------------------------------+
 0 | .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  . |
 1 | .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  . |
 2 | .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  . |
 3 | .  C  P  P  P  P  P  P  P  P  P  P  P  P  P  P  C  . |
 4 | .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  . |
 5 | .  .  .  .  .  X  .  .  .  .  .  .  X  .  .  .  .  . |
 6 | .  S  .  .  .  X  .  .  .  .  .  .  X  .  .  .  G  . |
 7 | .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  . |
 8 | .  C  P  P  P  P  P  P  P  P  P  P  P  P  P  P  C  . |
 9 | .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  . |
10 | .  .  .  .  .  .  .  .  .  H  .  .  .  .  .  .  .  . |
11 | .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  . |
   +----------------------------------------------------+
```

**Legend:**

| Symbol | Meaning |
|--------|---------|
| `.` | Safe ground (traversable) |
| `C` | Crop row edge (blocked) |
| `P` | Protected buffer (blocked) |
| `X` | Known static obstacle (blocked) |
| `H` | Hidden obstacle — absent from robot's initial map |
| `S` | Start `(6, 1)` |
| `G` | Goal  `(6, 16)` |
| `*` | Robot's planned path (shown at runtime) |
| `·` | Expanded/visited node (shown at runtime) |

---

## 5. Formal Search Problem

### 5.1 State Space

A **state** is a grid cell `(row, col)` that is traversable (not blocked, within bounds).  
The total number of states is at most `height × width − |blocked|`.

For the demo field: `12 × 18 − |protected ∪ obstacles| = 216 − (28 + 4) = 184` reachable states.

### 5.2 Initial State

```
s₀ = (6, 1)      # Start cell; confirmed traversable at construction time
```

### 5.3 Legal Actions

The robot uses **four-directional (cardinal) movement**:

| Action | Δrow | Δcol | Mnemonic |
|--------|------|------|----------|
| UP | −1 | 0 | Move to row above |
| DOWN | +1 | 0 | Move to row below |
| LEFT | 0 | −1 | Move one column left |
| RIGHT | 0 | +1 | Move one column right |

An action `a` applied from state `(r, c)` is **legal** if and only if:

1. The resulting cell `(r + Δrow, c + Δcol)` is within grid bounds.
2. The resulting cell is not in `blocked`.

> [!IMPORTANT]
> **Diagonal movement is not permitted.** This matches the `Field.neighbors()` implementation in `model.py` and the `FieldMap.get_neighbors()` implementation in `types.py`.

### 5.4 Transition Model

```
T((r, c), a) = (r + Δrow(a), c + Δcol(a))
             if 0 ≤ r + Δrow(a) < height
             and 0 ≤ c + Δcol(a) < width
             and (r + Δrow(a), c + Δcol(a)) ∉ blocked
```

The transition function is **deterministic** (no stochastic effects). Movement order within `neighbors()` is fixed (UP → RIGHT → DOWN → LEFT) to ensure reproducible results across runs.

### 5.5 Goal Test

```
GOAL-TEST(s) = True   iff   s == (6, 16)
```

### 5.6 Path-Cost Model

The step cost of **entering** cell `d` from any adjacent cell is:

```
c(d) = 1.0 + risk_penalty   if d is orthogonally adjacent to any protected cell
     = 1.0                  otherwise
```

With `risk_penalty = 2.0`:

| Cell type | Step cost |
|-----------|-----------|
| Open ground, not near crop | **1.0** |
| Open ground, adjacent to protected row | **3.0** |
| Blocked cell | ∞ (illegal; never entered) |

**Path cost** is the sum of step costs for all cells entered after the start:

```
g(path) = Σ c(pathᵢ)   for i = 1 … len(path) − 1
```

> [!NOTE]
> `risk_penalty ≥ 0` is enforced at construction time.  
> Because `c(d) ≥ 1.0 > 0` for all legal moves, Manhattan distance `h(n) = |r − goal_r| + |c − goal_c|` is a **consistent (monotone) admissible heuristic** for A* under this cost model: each step costs at least 1, and the heuristic counts exactly 1 per step in a straight-line path with no obstacles or penalties.

### 5.7 Heuristic Function (Informed Search)

```
h(n) = manhattan_distance(n, goal)
     = |n.row − goal.row| + |n.col − goal.col|
```

Implemented in `heuristic.py → manhattan_distance()`.

### 5.8 Optimality Properties by Algorithm

| Algorithm | Optimal for step-count? | Optimal for weighted cost? | Complete? |
|-----------|------------------------|---------------------------|-----------|
| BFS | ✅ Yes (uniform cost) | ❌ No | ✅ Yes |
| DFS | ❌ No | ❌ No | ✅ Yes (finite grid, visited-set) |
| Greedy BFS | ❌ No | ❌ No | ✅ Yes (visited-set) |
| A* | ✅ Yes | ✅ Yes (admissible h) | ✅ Yes |

---

## 6. Blocked-Cell Behaviour

| Situation | Behaviour |
|-----------|-----------|
| `start ∈ blocked` | `Field.__post_init__` raises `ValueError` at construction time. |
| `goal ∈ blocked` | `Field.__post_init__` raises `ValueError` at construction time. |
| Any intermediate cell ∈ blocked | `Field.neighbors()` never yields it; `Field.step_cost()` raises `ValueError` if called on it. |
| `position` out of bounds | `Field.in_bounds()` returns `False`; never added to frontier. |
| Goal unreachable (all paths blocked) | Algorithm empties its frontier without reaching goal; `SearchResult.reached_goal = False`, `path = []`. |

---

## 7. Map Representation in Code

### 7.1 `model.py` — `Field` dataclass

```python
Field(
    height=12,            # number of rows
    width=18,             # number of columns
    start=(6, 1),         # (row, col) — robot start
    goal=(6, 16),         # (row, col) — robot goal
    crop_rows=set(...),   # visual annotation only (colour in SVG)
    protected=set(...),   # blocked cells — safety buffer
    obstacles=set(...),   # blocked cells — static obstacles
    risk_penalty=2.0,     # extra cost when adjacent to protected
)
```

**Key methods:**

| Method | Purpose |
|--------|---------|
| `in_bounds(pos)` | Returns `True` if `pos` is within the grid. |
| `blocked` (property) | Returns `protected ∪ obstacles`. |
| `neighbors(pos)` | Yields legal 4-directional neighbours not in `blocked`. |
| `step_cost(dest)` | Returns entry cost for `dest`; raises if `dest` is blocked. |
| `with_obstacles(obs)` | Returns a new `Field` with a replaced obstacle set (used for replanning). |

### 7.2 `types.py` — `FieldMap` class (BFS/DFS interface)

```python
FieldMap(
    height=12,
    width=18,
    blocked_cells=frozenset(protected | obstacles),
)
```

**Key methods:**

| Method | Purpose |
|--------|---------|
| `is_within_bounds(coord)` | Bounds check. |
| `is_blocked(coord)` | True if coord in `blocked_cells`. |
| `is_traversable(coord)` | In-bounds AND not blocked. |
| `get_neighbors(coord)` | Returns list of traversable 4-directional neighbours. |
| `from_matrix(matrix)` | Construct from a 2-D list; blocked symbols: `1`, `'#'`, `'C'`, `'X'`, `'O'`, `True`. |
| `render(path, start, goal, expanded)` | ASCII grid string for terminal debugging. |

### 7.3 Loading the Shared Scenario

```python
from cropsafe_pathplanning.model import demo_field, demo_unknown_obstacle
from cropsafe_pathplanning.field_scenario import make_field_map, FIELD_CONFIG

# Rich Field object (used by search.py algorithms)
field = demo_field()

# FieldMap object (used by bfs_dfs.py and gbfs.py)
fmap = make_field_map()

# Hidden obstacle position
hidden = demo_unknown_obstacle()   # (10, 9)

# Full configuration dictionary
config = FIELD_CONFIG
```

---

## 8. SVG Visualisation Colour Legend

The `visualize.py` module generates SVG figures with this colour scheme:

| Colour (hex) | Meaning |
|-------------|---------|
| `#f5f2e9` (cream) | Safe ground |
| `#4f8f4b` (green) | Crop row |
| `#76a9d8` (blue) | Protected buffer |
| `#31343b` (dark grey) | Known static obstacle |
| `#e6550d` (orange) | Detected hidden obstacle |
| `#2ca02c` (green circle) | Start marker `S` |
| `#ffe04b` (yellow circle) | Goal marker `G` |
| `#d62728` line | BFS path |
| `#9467bd` line | DFS path |
| `#ff7f0e` line | Greedy BFS path |
| `#1f77b4` line | A* path |

---

## 9. Integration Checklist

- [x] `model.py` — `Field` dataclass with `demo_field()` and `demo_unknown_obstacle()` exported.
- [x] `types.py` — `FieldMap`, `Coordinate`, `SearchResult`, `CARDINAL_DIRECTIONS` exported from package `__init__.py`.
- [x] `field_scenario.py` — `FIELD_CONFIG` dict and `make_field_map()` helper for other modules.
- [x] Start `(6, 1)` and goal `(6, 16)` verified traversable by `Field.__post_init__`.
- [x] Four-directional movement confirmed in `Field.neighbors()`, `FieldMap.get_neighbors()`, and `heuristic.py`.
- [x] Cost model (`step_cost`) returns ≥ 1.0 for all legal moves, preserving admissibility of Manhattan heuristic.
- [x] Blocked-cell behaviour documented and enforced at construction and search time.
- [x] ASCII map, symbol legend, and SVG colour scheme documented above.
