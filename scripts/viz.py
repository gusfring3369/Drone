"""
Shared matplotlib drawing helpers (top-down map, east on the x-axis, north on the y-axis).
Used by planner.py (preview image), live_map.py (live view) and mission.py (final flight map).
"""
from __future__ import annotations

import matplotlib.patches as patches

from planner import Box, obstacles_from_config


def _rect(ax, box: Box, margin: float = 0.0, **kw):
    w = box.size_e + 2 * margin
    h = box.size_n + 2 * margin
    r = patches.Rectangle((box.east - w / 2, box.north - h / 2), w, h, **kw)
    ax.add_patch(r)
    return r


def draw_static(ax, cfg: dict) -> None:
    """Flight boundary, buildings (+ safety margin), hospital and village pads."""
    b = cfg["map_bounds"]
    m = cfg["safety_margin"]
    ax.add_patch(patches.Rectangle((b["east_min"], b["north_min"]),
                                   b["east_max"] - b["east_min"], b["north_max"] - b["north_min"],
                                   fill=False, ls="-.", lw=1.2, ec="purple", label="flight boundary"))
    for k, box in enumerate(obstacles_from_config(cfg)):
        _rect(ax, box, m, fill=False, ls=":", ec="grey", lw=0.8,
              label="safety margin" if k == 0 else None)
        _rect(ax, box, 0, fc="dimgrey", ec="black", label="building" if k == 0 else None)
        ax.text(box.east, box.north, f"{box.name}\n{box.height:.0f} m", ha="center", va="center",
                fontsize=6, color="white")
    h, v = cfg["hospital"], cfg["village"]
    ax.add_patch(patches.Circle((h["east"], h["north"]), 3, fc="red", ec="darkred", label=h["name"]))
    ax.add_patch(patches.Circle((v["east"], v["north"]), 3, fc="limegreen", ec="darkgreen", label=v["name"]))
    ax.set_xlim(b["east_min"] - 5, b["east_max"] + 5)
    ax.set_ylim(b["north_min"] - 5, b["north_max"] + 5)
    ax.set_aspect("equal")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.grid(alpha=0.25)


def draw_path(ax, path, label, color="tab:blue", style="-", lw=2.0):
    if not path:
        return None
    (line,) = ax.plot([p[1] for p in path], [p[0] for p in path], style, color=color, lw=lw,
                      marker="o", ms=3, label=label)
    return line


def draw_blockage(ax, box: Box, margin: float, active: bool = True):
    if active:
        _rect(ax, box, margin, fill=False, ls=":", ec="red", lw=0.8)
        return _rect(ax, box, 0, fc="red", ec="darkred", alpha=0.8, hatch="//", label="temporary blockage")
    return _rect(ax, box, 0, fill=False, ec="red", ls="--", label="blockage (cleared)")
