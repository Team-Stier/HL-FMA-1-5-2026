#!/usr/bin/env python3
"""Render confirmed mission markers over the production Yongin RDDF catalogue."""
import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE.parents[1]
RDDF = REPOSITORY / "src" / "localization" / "rddf"
CONFIG = PACKAGE / "config" / "missions.json"
OUTPUT = PACKAGE / "docs" / "yongin-mission-landmarks.png"


def load_route(name):
    with (RDDF / ("yongin_" + name + ".csv")).open(encoding="utf-8-sig", newline="") as stream:
        return [(float(row["east_m"]), float(row["north_m"]), float(row["path_yaw_rad"]))
                for row in csv.DictReader(stream)]


def route_distance(points):
    values = [0.0]
    for a, b in zip(points, points[1:]):
        values.append(values[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    return values


def pose_at(points, distances, target):
    target = min(distances[-1], max(0.0, target))
    for index in range(len(points) - 1):
        if target <= distances[index + 1]:
            length = distances[index + 1] - distances[index]
            ratio = (target - distances[index]) / length if length else 0.0
            a, b = points[index], points[index + 1]
            yaw_delta = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
            return (a[0] + ratio * (b[0] - a[0]), a[1] + ratio * (b[1] - a[1]),
                    a[2] + ratio * yaw_delta)
    return points[-1]


def line_across(axis, pose, width=3.0, **style):
    x, y, yaw = pose
    dx, dy = -math.sin(yaw) * width / 2, math.cos(yaw) * width / 2
    axis.plot([x - dx, x + dx], [y - dy, y + dy], **style)


def main():
    with CONFIG.open(encoding="utf-8") as stream:
        config = json.load(stream)
    route_files = sorted(RDDF.glob("yongin_*.csv"))
    routes = {}
    for path in route_files:
        name = path.stem.removeprefix("yongin_")
        routes[name] = load_route(name)

    figure = plt.figure(figsize=(14, 9), constrained_layout=True)
    grid = figure.add_gridspec(2, 2, width_ratios=(1.45, 1.0))
    overview = figure.add_subplot(grid[:, 0])
    hill = figure.add_subplot(grid[0, 1])
    signals = figure.add_subplot(grid[1, 1])

    for name, points in routes.items():
        xs, ys = zip(*[(p[0], p[1]) for p in points])
        overview.plot(xs, ys, color="#bdc4cc", linewidth=0.8, alpha=0.7)
    highlighted = {"1_left": "#d97706", "1_right": "#f59e0b",
                   "2": "#2563eb", "4": "#7c3aed", "7": "#0891b2"}
    for name, color in highlighted.items():
        points = routes[name]
        xs, ys = zip(*[(p[0], p[1]) for p in points])
        overview.plot(xs, ys, color=color, linewidth=2.0)

    hill_context = []
    for name in ("1_left", "1_right"):
        points = routes[name]
        distances = route_distance(points)
        start = config["landmarks"][name]["hill_zone_start_s"]
        end = config["landmarks"][name]["hill_zone_end_s"]
        middle = (start + end) / 2
        subset = [pose_at(points, distances, start)]
        subset.extend(point for point, distance in zip(points, distances) if start < distance < end)
        subset.append(pose_at(points, distances, end))
        context_start, context_end = max(0.0, start - 5.0), min(distances[-1], end + 5.0)
        context = [pose_at(points, distances, context_start)]
        context.extend(point for point, distance in zip(points, distances)
                       if context_start < distance < context_end)
        context.append(pose_at(points, distances, context_end))
        hill_context.extend(context)
        hill.plot([p[0] for p in context], [p[1] for p in context], color="#9ca3af", linewidth=1.2)
        hill.plot([p[0] for p in subset], [p[1] for p in subset], color=highlighted[name], linewidth=5)
        start_pose, end_pose, middle_pose = (pose_at(points, distances, value) for value in (start, end, middle))
        line_across(hill, start_pose, color="#111827", linewidth=5, solid_capstyle="butt")
        line_across(hill, start_pose, color="white", linewidth=3, solid_capstyle="butt")
        line_across(hill, end_pose, color="#111827", linewidth=5, solid_capstyle="butt")
        line_across(hill, end_pose, color="white", linewidth=3, solid_capstyle="butt")
        hill.scatter([middle_pose[0]], [middle_pose[1]], marker="*", s=180,
                     color="#dc2626", edgecolor="white", linewidth=0.8, zorder=5)
        overview.scatter([middle_pose[0]], [middle_pose[1]], marker="*", s=90,
                         color="#dc2626", edgecolor="white", linewidth=0.6, zorder=5)
        offset = (18, -34) if name == "1_left" else (18, 16)
        hill.annotate("%s midpoint\ns = %.2f m" % (name, middle), middle_pose[:2],
                      xytext=offset, textcoords="offset points", fontsize=9,
                      arrowprops={"arrowstyle": "-", "color": "#6b7280", "linewidth": .8})

    for name in ("2", "4", "7"):
        points = routes[name]
        distances = route_distance(points)
        stop = config["landmarks"][name]["stop_line_s"]
        pose = pose_at(points, distances, stop)
        xs, ys = zip(*[(p[0], p[1]) for p in points])
        signals.plot(xs, ys, color=highlighted[name], linewidth=1.8, label="Section " + name)
        line_across(signals, pose, color="#dc2626", linewidth=3)
        signals.scatter([pose[0]], [pose[1]], marker="s", s=45, color="#dc2626", zorder=5)
        signals.annotate("S%s stop line\ns = %.2f m" % (name, stop), pose[:2],
                         xytext=(6, 6), textcoords="offset points", fontsize=9)
        overview.scatter([pose[0]], [pose[1]], marker="s", s=45, color="#dc2626", zorder=5)
        overview.annotate("S%s" % name, pose[:2], xytext=(5, 5),
                          textcoords="offset points", fontsize=8)

    overview.set_title("Global RDDF and confirmed mission markers", fontweight="medium")
    hill.set_title("Section 1 hill white-line boundaries", fontweight="medium")
    signals.set_title("Traffic stop lines projected on global RDDF", fontweight="medium")
    for axis in (overview, hill, signals):
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("East from RDDF origin (m)")
        axis.set_ylabel("North from RDDF origin (m)")
        axis.grid(True, color="#d1d5db", linewidth=0.5, alpha=0.7)
    overview.legend(handles=[
        Line2D([0], [0], color="#bdc4cc", label="All global RDDF routes"),
        Line2D([0], [0], marker="s", linestyle="none", color="#dc2626", label="Traffic stop line"),
        Line2D([0], [0], marker="*", linestyle="none", color="#dc2626", markersize=10,
               label="Derived hill stop midpoint")], loc="lower left", fontsize=9)
    signals.legend(loc="best", fontsize=8)
    hill.set_xlim(min(p[0] for p in hill_context) - 2, max(p[0] for p in hill_context) + 2)
    hill.set_ylim(min(p[1] for p in hill_context) - 2, max(p[1] for p in hill_context) + 2)
    figure.suptitle("Yongin mission landmark integration | origin 37.288731, 127.1072336",
                    fontsize=14, fontweight="medium")
    figure.savefig(OUTPUT, dpi=180, metadata={"Software": "Team Stier landmark renderer"})
    print(OUTPUT)


if __name__ == "__main__":
    main()
