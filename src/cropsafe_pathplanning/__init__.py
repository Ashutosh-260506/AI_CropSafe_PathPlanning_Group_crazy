<<<<<<< HEAD
"""Crop-safe grid path-planning demonstration."""

__version__ = "0.1.0"
=======
"""CropSafe Path Planning: AI search algorithms for agricultural field robotics.

This package provides grid representations, heuristics, and search algorithms
(BFS, DFS, GBFS, A*) designed to navigate farm fields while avoiding obstacles
and respecting crop-protection zones.
"""

from cropsafe_pathplanning.gbfs import (
    greedy_best_first_search,
    validate_endpoints,
)
from cropsafe_pathplanning.heuristic import manhattan_distance
from cropsafe_pathplanning.types import (
    CARDINAL_DIRECTIONS,
    Coordinate,
    FieldMap,
    SearchResult,
)

__version__ = "0.1.0"

__all__ = [
    "greedy_best_first_search",
    "validate_endpoints",
    "manhattan_distance",
    "FieldMap",
    "SearchResult",
    "Coordinate",
    "CARDINAL_DIRECTIONS",
]
>>>>>>> ricky
