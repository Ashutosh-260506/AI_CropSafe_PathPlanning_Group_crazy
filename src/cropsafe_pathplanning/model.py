"""Grid and field model shared by the search algorithms."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable

Position = tuple[int, int]


@dataclass
class Field:
    """A rectangular four-connected field.

    Protected cells are forbidden. Cells adjacent to a protected cell receive
    a configurable nonnegative risk surcharge to encourage crop clearance.
    """

    height: int
    width: int
    start: Position
    goal: Position
    crop_rows: set[Position] = field(default_factory=set)
    protected: set[Position] = field(default_factory=set)
    obstacles: set[Position] = field(default_factory=set)
    risk_penalty: float = 2.0

    def __post_init__(self) -> None:
        if self.height <= 0 or self.width <= 0:
            raise ValueError("Field dimensions must be positive")
        if self.risk_penalty < 0:
            raise ValueError("risk_penalty must be nonnegative")
        for position in {self.start, self.goal} | self.crop_rows | self.protected | self.obstacles:
            if not self.in_bounds(position):
                raise ValueError(f"Position outside field: {position}")
        if self.start in self.blocked or self.goal in self.blocked:
            raise ValueError("Start and goal must be traversable")

    @property
    def blocked(self) -> set[Position]:
        return self.protected | self.obstacles

    def in_bounds(self, position: Position) -> bool:
        row, col = position
        return 0 <= row < self.height and 0 <= col < self.width

    def neighbors(self, position: Position) -> Iterable[Position]:
        row, col = position
        # Fixed order makes runs deterministic and easier to compare.
        for candidate in ((row - 1, col), (row, col + 1), (row + 1, col), (row, col - 1)):
            if self.in_bounds(candidate) and candidate not in self.blocked:
                yield candidate

    def step_cost(self, destination: Position) -> float:
        if destination in self.blocked:
            raise ValueError(f"Cannot enter blocked cell {destination}")
        adjacent_protected = any(
            (destination[0] + dr, destination[1] + dc) in self.protected
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1))
        )
        return 1.0 + (self.risk_penalty if adjacent_protected else 0.0)

    def with_obstacles(self, obstacles: set[Position]) -> "Field":
        return Field(
            height=self.height,
            width=self.width,
            start=self.start,
            goal=self.goal,
            crop_rows=set(self.crop_rows),
            protected=set(self.protected),
            obstacles=set(obstacles),
            risk_penalty=self.risk_penalty,
        )


def demo_field(*, known_map: bool = False) -> Field:
    """Return the fixed 12x18 scenario used by the command-line demo.

    ``known_map=False`` gives the robot only confirmed obstacles; the true
    obstacle returned by :func:`demo_unknown_obstacle` is withheld from it.
    """
    del known_map
    protected = {(r, c) for r in (3, 8) for c in range(2, 16)}
    crop_rows = {(r, c) for r in (3, 8) for c in range(1, 17)}
    obstacles = {(5, 5), (6, 5), (5, 12), (6, 12)}
    return Field(12, 18, (6, 1), (6, 16), crop_rows, protected, obstacles, risk_penalty=2.0)


def demo_unknown_obstacle() -> Position:
    return (10, 9)
