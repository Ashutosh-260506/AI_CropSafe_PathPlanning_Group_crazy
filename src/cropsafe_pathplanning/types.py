"""Type definitions and shared data models for CropSafe Path Planning.

This module defines common types and data structures shared across search algorithms
in the cropsafe_pathplanning package, adhering to the project's coordinate and
result interface specifications.
"""

from __future__ import annotations

from typing import Any, Callable, Iterator, Sequence

# Grid coordinate representation: (row, column)
Coordinate = tuple[int, int]

# Four standard legal movement directions: Up, Down, Left, Right
CARDINAL_DIRECTIONS: list[Coordinate] = [
    (-1, 0),  # Up
    (1, 0),   # Down
    (0, -1),  # Left
    (0, 1),   # Right
]


class SearchResult(dict):
    """Standardized search result dictionary.

    Inherits from `dict` so it can be used directly as a dictionary conforming
    to the project's agreed schema:
        {
            "path": list of (row, column) coordinates,
            "cost": float or int,
            "expanded_nodes": list of (row, column) coordinates,
            "reached_goal": bool,
        }

    Also provides property accessors (.path, .cost, .expanded_nodes, .reached_goal)
    for convenience and readability.
    """

    def __init__(
        self,
        path: list[Coordinate],
        cost: float | int,
        expanded_nodes: list[Coordinate],
        reached_goal: bool,
        **kwargs: Any,
    ) -> None:
        super().__init__(
            path=list(path),
            cost=cost,
            expanded_nodes=list(expanded_nodes),
            reached_goal=bool(reached_goal),
            **kwargs,
        )

    @property
    def path(self) -> list[Coordinate]:
        """Ordered (row, column) coordinates from start to goal."""
        return self["path"]

    @property
    def cost(self) -> float | int:
        """Actual cost of the returned path."""
        return self["cost"]

    @property
    def expanded_nodes(self) -> list[Coordinate]:
        """Coordinates expanded during search, in expansion order."""
        return self["expanded_nodes"]

    @property
    def reached_goal(self) -> bool:
        """Boolean indicating whether the search successfully reached the goal."""
        return self["reached_goal"]

    def to_dict(self) -> dict[str, Any]:
        """Return a plain dictionary copy of the search result."""
        return dict(self)


class FieldMap:
    """Two-dimensional grid map representing the agricultural field.

    Coordinates are represented as `(row, column)` tuples with 0-based indexing:
        - row: 0 to height - 1 (top to bottom)
        - col: 0 to width - 1 (left to right)

    Cells can be traversable (free path/soil) or non-traversable (obstacles or
    protected crop regions).
    """

    def __init__(
        self,
        height: int,
        width: int,
        blocked_cells: set[Coordinate] | None = None,
    ) -> None:
        if height <= 0 or width <= 0:
            raise ValueError(f"Field dimensions must be positive, got {height}x{width}")
        self.height: int = height
        self.width: int = width
        self.blocked_cells: set[Coordinate] = set(blocked_cells or set())

    @property
    def rows(self) -> int:
        """Alias for height."""
        return self.height

    @property
    def cols(self) -> int:
        """Alias for width."""
        return self.width

    def is_within_bounds(self, coord: Coordinate) -> bool:
        """Check if coordinate is within grid boundaries."""
        r, c = coord
        return 0 <= r < self.height and 0 <= c < self.width

    def is_blocked(self, coord: Coordinate) -> bool:
        """Check if coordinate is an obstacle or protected cell."""
        return coord in self.blocked_cells

    def is_traversable(self, coord: Coordinate) -> bool:
        """Check if coordinate is within bounds and not blocked."""
        return self.is_within_bounds(coord) and not self.is_blocked(coord)

    def block_cell(self, coord: Coordinate) -> None:
        """Mark a cell as blocked."""
        if not self.is_within_bounds(coord):
            raise ValueError(f"Coordinate {coord} is out of bounds for {self.height}x{self.width} field")
        self.blocked_cells.add(coord)

    def unblock_cell(self, coord: Coordinate) -> None:
        """Unmark a blocked cell."""
        self.blocked_cells.discard(coord)

    def get_neighbors(
        self,
        coord: Coordinate,
        directions: Sequence[Coordinate] = CARDINAL_DIRECTIONS,
    ) -> list[Coordinate]:
        """Return legal, traversable 4-directional neighbor coordinates.

        Neighbors are generated in deterministic order (Up, Down, Left, Right
        by default).
        """
        r, c = coord
        neighbors: list[Coordinate] = []
        for dr, dc in directions:
            neighbor = (r + dr, c + dc)
            if self.is_traversable(neighbor):
                neighbors.append(neighbor)
        return neighbors

    @classmethod
    def from_matrix(
        cls,
        matrix: Sequence[Sequence[Any]],
        blocked_symbols: set[Any] | None = None,
    ) -> FieldMap:
        """Construct a FieldMap from a 2D matrix/list of lists.

        By default, values `1`, `'#'`, `'C'`, `'X'`, `'O'`, or `True` are treated
        as blocked cells. Values `0`, `'.'`, `' '`, or `False` are traversable.
        """
        if not matrix or not matrix[0]:
            raise ValueError("Matrix must be non-empty and 2-dimensional")

        height = len(matrix)
        width = len(matrix[0])
        default_blocked = {1, "#", "C", "c", "X", "x", "O", "o", True, "OBSTACLE", "CROP"}
        check_blocked = blocked_symbols if blocked_symbols is not None else default_blocked

        blocked_cells: set[Coordinate] = set()
        for r, row in enumerate(matrix):
            if len(row) != width:
                raise ValueError(f"Inconsistent row length at row {r}: expected {width}, got {len(row)}")
            for c, val in enumerate(row):
                if val in check_blocked:
                    blocked_cells.add((r, c))

        return cls(height=height, width=width, blocked_cells=blocked_cells)

    def render(
        self,
        path: Sequence[Coordinate] | None = None,
        start: Coordinate | None = None,
        goal: Coordinate | None = None,
        expanded: Sequence[Coordinate] | None = None,
    ) -> str:
        """Render a plain-text visual ASCII representation of the field."""
        path_set = set(path or [])
        expanded_set = set(expanded or [])

        lines: list[str] = []
        # Header with column indices
        col_header = "    " + " ".join(f"{c:2d}" for c in range(self.width))
        lines.append(col_header)
        lines.append("   +" + "--" * self.width + "-+")

        for r in range(self.height):
            row_chars: list[str] = []
            for c in range(self.width):
                coord = (r, c)
                if coord == start:
                    ch = " S"
                elif coord == goal:
                    ch = " G"
                elif coord in self.blocked_cells:
                    ch = " #"
                elif coord in path_set:
                    ch = " *"
                elif coord in expanded_set:
                    ch = " ·"
                else:
                    ch = " ."
                row_chars.append(ch)
            lines.append(f"{r:2d} |" + "".join(row_chars) + " |")

        lines.append("   +" + "--" * self.width + "-+")
        return "\n".join(lines)

    def __repr__(self) -> str:
        return f"FieldMap(height={self.height}, width={self.width}, blocked_count={len(self.blocked_cells)})"
