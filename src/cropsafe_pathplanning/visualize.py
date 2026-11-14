"""Dependency-free SVG field and route visualizations."""

from __future__ import annotations

from pathlib import Path

from .model import Field, Position
from .search import SearchResult

COLORS = {"ground": "#f5f2e9", "crop": "#4f8f4b", "protected": "#76a9d8", "obstacle": "#31343b", "detected": "#e6550d"}


def _svg(field: Field, path: list[Position], output_path: Path, title: str, *, detected: list[Position] | None = None, routes: list[SearchResult] | None = None) -> None:
    cell, left, top = 42, 64, 70
    width, height = left + field.width * cell + 20, top + field.height * cell + 90
    elements = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">', '<rect width="100%" height="100%" fill="white"/>', f'<text x="{width/2}" y="34" text-anchor="middle" font-family="sans-serif" font-size="21" font-weight="bold">{title}</text>']
    detected = detected or []
    for row in range(field.height):
        for col in range(field.width):
            pos = (row, col)
            color = COLORS["ground"]
            if pos in field.crop_rows: color = COLORS["crop"]
            if pos in field.protected: color = COLORS["protected"]
            if pos in field.obstacles: color = COLORS["obstacle"]
            if pos in detected: color = COLORS["detected"]
            x, y = left + col * cell, top + row * cell
            elements.append(f'<rect x="{x}" y="{y}" width="{cell}" height="{cell}" fill="{color}" stroke="#aaa" stroke-width="1"/>')
            elements.append(f'<text x="{x+cell/2}" y="{y+cell/2+4}" text-anchor="middle" font-family="sans-serif" font-size="9" fill="#555">{row},{col}</text>')
    if routes:
        route_colors = {"BFS": "#d62728", "DFS": "#9467bd", "Greedy": "#ff7f0e", "A*": "#1f77b4"}
        for result in routes:
            if result.path:
                points = " ".join(f"{left+c*cell+cell/2},{top+r*cell+cell/2}" for r, c in result.path)
                elements.append(f'<polyline points="{points}" fill="none" stroke="{route_colors[result.algorithm]}" stroke-width="3" opacity="0.85"/>')
    elif path:
        points = " ".join(f"{left+c*cell+cell/2},{top+r*cell+cell/2}" for r, c in path)
        elements.append(f'<polyline points="{points}" fill="none" stroke="#1f77b4" stroke-width="4"/>')
    for position, label, color in ((field.start, "S", "#2ca02c"), (field.goal, "G", "#ffe04b")):
        row, col = position
        x, y = left + col * cell + cell/2, top + row * cell + cell/2
        elements.append(f'<circle cx="{x}" cy="{y}" r="13" fill="{color}" stroke="black" stroke-width="2"/>')
        elements.append(f'<text x="{x}" y="{y+5}" text-anchor="middle" font-family="sans-serif" font-size="14" font-weight="bold">{label}</text>')
    legend = [("Safe ground", COLORS["ground"]), ("Crop row", COLORS["crop"]), ("Protected", COLORS["protected"]), ("Obstacle", COLORS["obstacle"])]
    if detected: legend.append(("Detected hidden obstacle", COLORS["detected"]))
    if routes: legend.extend((f"{result.algorithm}: {result.path_length} steps", {"BFS":"#d62728","DFS":"#9467bd","Greedy":"#ff7f0e","A*":"#1f77b4"}[result.algorithm]) for result in routes)
    legend_y = top + field.height * cell + 32
    for i, (label, color) in enumerate(legend):
        x = left + (i % 4) * (width - left) / 4
        y = legend_y + (i // 4) * 24
        elements.append(f'<rect x="{x}" y="{y-12}" width="14" height="14" fill="{color}" stroke="#777"/>')
        elements.append(f'<text x="{x+19}" y="{y}" font-family="sans-serif" font-size="11">{label}</text>')
    elements.append('</svg>')
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(elements), encoding="utf-8")


def save_field_plot(field: Field, results: list[SearchResult], output_path: Path, *, title: str = "Search algorithm routes") -> None:
    _svg(field, [], output_path, title, routes=results)


def save_replanning_plot(field: Field, path: list[Position], detected: list[Position], output_path: Path) -> None:
    _svg(field, path, output_path, "Online obstacle discovery and A* replanning", detected=detected)
