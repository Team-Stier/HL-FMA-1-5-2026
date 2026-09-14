"""Optional SciPy KD-tree helpers with dependency-free fallbacks."""

import numpy as np

try:
    from scipy.spatial import cKDTree
except ImportError:  # Keep the node usable until python3-scipy is installed.
    cKDTree = None


def spatial_index_backend():
    """Return the spatial search implementation selected in this process."""
    return "kd_tree" if cKDTree is not None else "numpy_fallback"


def radius_neighbors(points, radius):
    """Return KD-tree radius-neighbor lists, or None when SciPy is unavailable."""
    if cKDTree is None:
        return None
    return cKDTree(points).query_ball_point(points, radius)


def nearest_within_radius(points, centers, radius, block_size=2048):
    """Return whether every point is within radius of its nearest center."""
    if cKDTree is not None:
        distances, _ = cKDTree(centers).query(
            points, k=1, distance_upper_bound=radius)
        return np.isfinite(distances)

    # A bounded NumPy fallback is faster than Python point/center loops and
    # avoids allocating the full N x M matrix for unusually long corridors.
    radius_squared = radius * radius
    mask = np.zeros(len(points), dtype=bool)
    for start in range(0, len(points), block_size):
        stop = min(start + block_size, len(points))
        differences = points[start:stop, None, :] - centers[None, :, :]
        squared = np.einsum("ijk,ijk->ij", differences, differences)
        mask[start:stop] = np.any(squared <= radius_squared, axis=1)
    return mask
