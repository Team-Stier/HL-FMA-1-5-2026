"""RDDF corridor construction and point filtering without ROS dependencies."""

from dataclasses import dataclass
import csv
import json
import math
from pathlib import Path
import re

import numpy as np

from .spatial import nearest_within_radius


@dataclass(frozen=True)
class Route:
    name: str
    points: np.ndarray
    cumulative: np.ndarray
    section: int
    side: str


@dataclass(frozen=True)
class Connection:
    successor: str
    progress_m: float
    distance_m: float


def _section(name):
    match = re.match(r"^(\d+)", name)
    if match is None:
        raise ValueError("RDDF route name must start with a section number: " + name)
    return int(match.group(1))


def _side(name):
    lowered = name.lower()
    if "left" in lowered:
        return "left"
    if "right" in lowered:
        return "right"
    return ""


def _project_to_polyline(point, route):
    starts = route.points[:-1]
    vectors = route.points[1:] - starts
    lengths_squared = np.einsum("ij,ij->i", vectors, vectors)
    fractions = np.zeros(len(vectors), dtype=float)
    valid = lengths_squared > 1e-12
    fractions[valid] = np.clip(
        np.einsum("ij,ij->i", point - starts[valid], vectors[valid])
        / lengths_squared[valid],
        0.0,
        1.0,
    )
    projected = starts + fractions[:, None] * vectors
    distances = np.linalg.norm(projected - point, axis=1)
    index = int(np.argmin(distances))
    segment_length = route.cumulative[index + 1] - route.cumulative[index]
    progress = route.cumulative[index] + fractions[index] * segment_length
    return float(distances[index]), float(progress)


def _point_at(route, progress_m):
    progress = min(max(float(progress_m), 0.0), float(route.cumulative[-1]))
    index = min(int(np.searchsorted(route.cumulative, progress, side="right")) - 1,
                len(route.points) - 2)
    index = max(index, 0)
    length = route.cumulative[index + 1] - route.cumulative[index]
    fraction = 0.0 if length <= 1e-12 else (progress - route.cumulative[index]) / length
    return route.points[index] + fraction * (route.points[index + 1] - route.points[index])


def _route_slice(route, start_m, end_m):
    start = min(max(float(start_m), 0.0), float(route.cumulative[-1]))
    end = min(max(float(end_m), start), float(route.cumulative[-1]))
    middle = route.points[(route.cumulative > start) & (route.cumulative < end)]
    return np.vstack((_point_at(route, start), middle, _point_at(route, end)))


class RddfRouteNetwork:
    """Load sampled RDDF routes and connect consecutive numbered sections."""

    def __init__(self, routes, route_groups, connection_distance_m=1.0):
        self.routes = dict(routes)
        self.route_groups = dict(route_groups)
        self.connection_distance_m = float(connection_distance_m)
        if not math.isfinite(self.connection_distance_m) or self.connection_distance_m <= 0:
            raise ValueError("connection_distance_m must be positive")
        if not self.routes:
            raise ValueError("RDDF route network is empty")
        self.connections = self._build_connections()

    @classmethod
    def from_directory(cls, directory, connection_distance_m=1.0):
        directory = Path(directory)
        project = json.loads(
            (directory / "yongin_route_project.json").read_text(encoding="utf-8-sig")
        )
        routes = {}
        for path in sorted(directory.glob("*.csv")):
            with path.open(encoding="utf-8-sig", newline="") as stream:
                rows = list(csv.DictReader(stream))
            if len(rows) < 2:
                continue
            name = rows[0]["route_name"]
            if name in routes or any(row["route_name"] != name for row in rows):
                raise ValueError("duplicated or mixed RDDF route: " + name)
            points = np.asarray(
                [(float(row["east_m"]), float(row["north_m"])) for row in rows],
                dtype=float,
            )
            if not np.isfinite(points).all():
                raise ValueError("non-finite RDDF route: " + name)
            distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
            if np.any(distances <= 1e-9):
                raise ValueError("RDDF route contains duplicate adjacent points: " + name)
            cumulative = np.r_[0.0, np.cumsum(distances)]
            routes[name] = Route(name, points, cumulative, _section(name), _side(name))
        return cls(routes, project.get("route_groups", {}), connection_distance_m)

    def _build_connections(self):
        result = {name: [] for name in self.routes}
        for route in self.routes.values():
            candidates = [candidate for candidate in self.routes.values()
                          if candidate.section == route.section + 1]
            if route.side:
                candidates = [candidate for candidate in candidates
                              if not candidate.side or candidate.side == route.side]
            for candidate in candidates:
                distance, progress = _project_to_polyline(candidate.points[0], route)
                if distance <= self.connection_distance_m:
                    result[route.name].append(Connection(candidate.name, progress, distance))
            result[route.name].sort(key=lambda item: (item.progress_m, item.successor))
        return result

    def route_names(self, public_name):
        return list(self.route_groups.get(public_name, [public_name]))

    def corridor(self, source_route, segment_index, segment_fraction,
                 lookahead_m, lookbehind_m=0.0, max_depth=4):
        """Return polylines covering the current route and connected successors."""
        if source_route not in self.routes:
            raise ValueError("unknown RDDF source route: " + str(source_route))
        route = self.routes[source_route]
        if (isinstance(segment_index, bool) or not isinstance(segment_index, int)
                or not 0 <= segment_index < len(route.points) - 1):
            raise ValueError("invalid RDDF segment index")
        fraction = float(segment_fraction)
        ahead, behind = float(lookahead_m), float(lookbehind_m)
        if (not math.isfinite(fraction) or not 0.0 <= fraction <= 1.0
                or not math.isfinite(ahead) or ahead <= 0
                or not math.isfinite(behind) or behind < 0
                or isinstance(max_depth, bool) or not isinstance(max_depth, int) or max_depth < 0):
            raise ValueError("invalid RDDF corridor parameters")
        segment_length = route.cumulative[segment_index + 1] - route.cumulative[segment_index]
        current = route.cumulative[segment_index] + fraction * segment_length
        start = max(0.0, current - behind)
        budget = ahead + current - start
        polylines = []
        best_budget = {}

        def walk(name, progress, remaining, depth):
            key = (name, round(progress, 3))
            if best_budget.get(key, -1.0) >= remaining:
                return
            best_budget[key] = remaining
            branch = self.routes[name]
            end = min(float(branch.cumulative[-1]), progress + remaining)
            polylines.append(_route_slice(branch, progress, end))
            if depth >= max_depth:
                return
            for connection in self.connections[name]:
                travel = connection.progress_m - progress
                if -1e-6 <= travel <= remaining + 1e-6:
                    walk(connection.successor, 0.0, max(0.0, remaining - max(0.0, travel)),
                         depth + 1)

        walk(source_route, start, budget, 0)
        return polylines


def circle_union_mask(points, centers, radius_m):
    """Return a Boolean mask for points inside any equal-radius ROI circle."""
    points = np.asarray(points, dtype=float)
    centers = np.asarray(centers, dtype=float)
    radius = float(radius_m)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("points must have shape (N, 2)")
    if centers.ndim != 2 or centers.shape[1] != 2 or len(centers) == 0:
        raise ValueError("centers must have shape (M, 2) and not be empty")
    if not np.isfinite(points).all() or not np.isfinite(centers).all():
        raise ValueError("ROI input contains non-finite coordinates")
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError("radius_m must be positive")
    return nearest_within_radius(points, centers, radius)


def lidar_forward_mask(points, lidar_origin_x_m, min_forward_m):
    """Keep points ahead of the LiDAR origin along base_link +X."""
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("points must have shape (N, 2)")
    if not np.isfinite(points).all():
        raise ValueError("LiDAR forward ROI input contains non-finite coordinates")
    origin = float(lidar_origin_x_m)
    minimum = float(min_forward_m)
    if not math.isfinite(origin):
        raise ValueError("LiDAR origin X must be finite")
    if not math.isfinite(minimum) or minimum < 0:
        raise ValueError("LiDAR forward distance must be nonnegative")
    return points[:, 0] >= origin + minimum


def angular_roi_mask(points, min_angle_deg, max_angle_deg):
    """Return points inside a base_link angular sector.

    ``base_link`` follows REP-103: +X is 0 degrees and +Y is +90 degrees.
    Sector boundaries are included. If the minimum angle is greater than the
    maximum angle, the sector wraps across -180/180 degrees.
    """
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("points must have shape (N, 2)")
    if not np.isfinite(points).all():
        raise ValueError("angular ROI input contains non-finite coordinates")
    minimum, maximum = float(min_angle_deg), float(max_angle_deg)
    if (not math.isfinite(minimum) or not math.isfinite(maximum)
            or not -180.0 <= minimum <= 180.0
            or not -180.0 <= maximum <= 180.0
            or minimum == maximum):
        raise ValueError(
            "angular ROI angles must be distinct and within [-180, 180] degrees")
    angles = np.degrees(np.arctan2(points[:, 1], points[:, 0]))
    if minimum < maximum:
        return (angles >= minimum) & (angles <= maximum)
    return (angles >= minimum) | (angles <= maximum)


def vehicle_exclusion_mask(points, rear_m, front_m, right_m, left_m):
    """Return True for points outside a base_link-aligned vehicle box.

    ``base_link`` follows REP-103: +X is forward and +Y is left. The box spans
    ``[-rear_m, front_m]`` in X and ``[-right_m, left_m]`` in Y. Points on the
    box boundary are treated as vehicle returns and removed.
    """
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("points must have shape (N, 2)")
    if not np.isfinite(points).all():
        raise ValueError("vehicle exclusion input contains non-finite coordinates")
    extents = tuple(float(value) for value in (rear_m, front_m, right_m, left_m))
    if any(not math.isfinite(value) or value <= 0 for value in extents):
        raise ValueError("vehicle exclusion extents must be positive")
    rear, front, right, left = extents
    inside = ((points[:, 0] >= -rear) & (points[:, 0] <= front)
              & (points[:, 1] >= -right) & (points[:, 1] <= left))
    return ~inside
