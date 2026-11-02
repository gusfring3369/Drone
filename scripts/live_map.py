#!/usr/bin/env python3
"""
Live 2-D mission map. Run it in its own terminal while mission.py is flying:

    python3 scripts/live_map.py

It re-reads logs/state.json (written by mission.py) twice per second and shows the
buildings, the planned / re-planned route, the drone, its track, the blockage and the
sensor range, plus the current mission state and recent events.
"""
from __future__ import annotations

import json
import os
import sys

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches

import planner
import viz
from planner import Box

STATE_FILE = os.path.join(planner.ROOT, "logs", "state.json")


def main() -> int:
    cfg = planner.load_config(sys.argv[1] if len(sys.argv) > 1 else planner.DEFAULT_CONFIG)
    fig, ax = plt.subplots(figsize=(9, 9))
    fig.canvas.manager.set_window_title("Medical delivery drone - live map")

    def draw(_frame):
        try:
            with open(STATE_FILE) as f:
                s = json.load(f)
        except (OSError, ValueError):
            s = None
        ax.cla()
        viz.draw_static(ax, cfg)
        if not s:
            ax.set_title("Waiting for mission.py ... (logs/state.json not found yet)")
            return
        if s.get("original_route"):
            viz.draw_path(ax, s["original_route"], "original A* route", color="tab:blue", style=":", lw=1.5)
        if s.get("route") and s["route"] != s.get("original_route"):
            viz.draw_path(ax, s["route"], "current route", color="tab:green", style="--")
        b = s.get("blockage")
        if b:
            box = Box("blockage", b["north"], b["east"], b["size"], b["size"])
            viz.draw_blockage(ax, box, cfg["safety_margin"], active=b["active"])
        if s.get("track"):
            tr = s["track"]
            ax.plot([p[1] for p in tr], [p[0] for p in tr], "-", color="darkorange", lw=2, label="flight track")
        d = s.get("drone")
        if d:
            ax.plot(d[1], d[0], marker="o", ms=11, color="black", mec="yellow", mew=2, label="drone", zorder=5)
            if b and b["active"]:
                ax.add_patch(patches.Circle((d[1], d[0]), b["sensor_range"], fill=False, ec="orange",
                                            ls="--", lw=1, label="sensor range"))
            ax.text(0.01, 0.99, f"alt {d[2]:.1f} m", transform=ax.transAxes, va="top", fontsize=9)
        ax.set_title(f"t={s['t']:.0f}s   STATE: {s['state']}   leg: {s.get('leg') or '-'}   "
                     f"scenario: {s.get('scenario')}", fontsize=10)
        ev = "\n".join(s.get("events", [])[-5:])
        if ev:
            ax.text(0.01, 0.01, ev, transform=ax.transAxes, fontsize=7, family="monospace", va="bottom",
                    bbox=dict(fc="white", alpha=0.85, ec="grey"))
        ax.legend(loc="upper right", fontsize=7)

    anim = FuncAnimation(fig, draw, interval=500, cache_frame_data=False)  # noqa: F841 (keep reference)
    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
