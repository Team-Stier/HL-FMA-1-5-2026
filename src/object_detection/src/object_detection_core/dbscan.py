"""Deterministic 2D DBSCAN with KD-tree and spatial-hash neighbor search."""

from collections import defaultdict, deque
import math

import numpy as np

from .spatial import radius_neighbors


UNVISITED = -2
NOISE = -1


def dbscan(points, eps=0.3, min_samples=4):
    """Return a cluster label for every XY point; noise points are ``-1``.

    ``min_samples`` includes the point itself. Cluster identifiers start at zero
    and follow the first core point encountered in input order.
    """
    points = np.asarray(points, dtype=float)
    if points.shape == (0,):
        points = points.reshape(0, 2)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError('points must have shape (N, 2)')
    if not np.isfinite(points).all():
        raise ValueError('points must contain only finite coordinates')
    if not isinstance(eps, (int, float)) or isinstance(eps, bool) or not math.isfinite(eps) or eps <= 0:
        raise ValueError('eps must be a positive finite number')
    if not isinstance(min_samples, int) or isinstance(min_samples, bool) or min_samples < 1:
        raise ValueError('min_samples must be a positive integer')

    count = len(points)
    labels = np.full(count, UNVISITED, dtype=np.int32)
    if count == 0:
        return labels

    adjacency = radius_neighbors(points, eps)
    if adjacency is None:
        # Dependency-free fallback for systems where SciPy is not installed.
        # Python floats avoid a tiny NumPy allocation for every distance check.
        xs = points[:, 0].tolist()
        ys = points[:, 1].tolist()
        cells = defaultdict(list)
        for index, (x, y) in enumerate(zip(xs, ys)):
            cells[(math.floor(x / eps), math.floor(y / eps))].append(index)
        eps_squared = eps * eps
        adjacency = [[index] for index in range(count)]

        def connect(first, second):
            dx = xs[first] - xs[second]
            dy = ys[first] - ys[second]
            if dx * dx + dy * dy <= eps_squared:
                adjacency[first].append(second)
                adjacency[second].append(first)

        # Build each undirected neighbor pair once. Only the current cell and
        # four forward neighbors are needed; the other four were already seen.
        for cell, members in cells.items():
            for offset, first in enumerate(members):
                for second in members[offset + 1:]:
                    connect(first, second)
            cell_x, cell_y = cell
            for dx, dy in ((0, 1), (1, -1), (1, 0), (1, 1)):
                for first in members:
                    for second in cells.get((cell_x + dx, cell_y + dy), ()):
                        connect(first, second)

    cluster = 0
    for index in range(count):
        if labels[index] != UNVISITED:
            continue
        nearby = adjacency[index]
        if len(nearby) < min_samples:
            labels[index] = NOISE
            continue

        labels[index] = cluster
        queue = deque(nearby)
        queued = set(nearby)
        while queue:
            candidate = queue.popleft()
            if labels[candidate] == NOISE:
                labels[candidate] = cluster
            if labels[candidate] != UNVISITED:
                continue
            labels[candidate] = cluster
            candidate_neighbors = adjacency[candidate]
            if len(candidate_neighbors) >= min_samples:
                for neighbor in candidate_neighbors:
                    if neighbor not in queued:
                        queued.add(neighbor)
                        queue.append(neighbor)
        cluster += 1
    return labels


def filter_clusters_by_extent(points, labels, max_extent_m):
    """Relabel clusters wider or longer than ``max_extent_m`` as noise.

    The extent is the larger of the cluster's X and Y spans. A value of zero
    disables this filter. Cluster identifiers are preserved for accepted
    clusters so marker colors remain stable within the scan.
    """
    points = np.asarray(points, dtype=float)
    labels = np.asarray(labels)
    if (points.ndim != 2 or points.shape[1] < 2 or not np.isfinite(points).all()):
        raise ValueError("points must be a finite Nx2-or-greater array")
    if (labels.ndim != 1 or len(labels) != len(points)):
        raise ValueError("labels must be a vector matching points")
    if (not isinstance(max_extent_m, (int, float)) or isinstance(max_extent_m, bool)
            or not math.isfinite(max_extent_m) or max_extent_m < 0):
        raise ValueError("max_extent_m must be a nonnegative finite number")

    filtered = labels.astype(np.int32, copy=True)
    if max_extent_m == 0 or len(points) == 0:
        return filtered
    for cluster_id in set(filtered.tolist()) - {NOISE}:
        member_mask = filtered == cluster_id
        spans = np.ptp(points[member_mask, :2], axis=0)
        if float(np.max(spans)) > max_extent_m:
            filtered[member_mask] = NOISE
    return filtered


def voxel_downsample(points, voxel_size_m):
    """Replace points in each XY voxel with their centroid."""
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] not in (2, 3):
        raise ValueError("points must have shape (N, 2) or (N, 3)")
    if not np.isfinite(points).all():
        raise ValueError("points must contain only finite coordinates")
    if (not isinstance(voxel_size_m, (int, float)) or isinstance(voxel_size_m, bool)
            or not math.isfinite(voxel_size_m) or voxel_size_m <= 0):
        raise ValueError("voxel_size_m must be a positive finite number")
    if len(points) == 0:
        return points.copy()
    cells = np.floor(points[:, :2] / float(voxel_size_m)).astype(np.int64)
    _, inverse = np.unique(cells, axis=0, return_inverse=True)
    count = int(inverse.max()) + 1
    sums = np.zeros((count, points.shape[1]), dtype=float)
    samples = np.bincount(inverse, minlength=count)
    np.add.at(sums, inverse, points)
    return sums / samples[:, None]
