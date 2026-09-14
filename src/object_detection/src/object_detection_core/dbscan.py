"""Small deterministic 2D DBSCAN implementation using a spatial hash grid."""

from collections import defaultdict, deque
import math

import numpy as np


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

    # Python floats avoid allocating a tiny NumPy vector for every distance
    # check. This matters for dense angle-compensated LaserScan clouds.
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

    # Build each undirected neighbor pair once. Only the current cell and four
    # forward neighboring cells are needed; the other four were already seen.
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
