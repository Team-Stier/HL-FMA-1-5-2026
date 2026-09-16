import unittest

import numpy as np

from object_detection_core import (
    NOISE,
    dbscan,
    filter_clusters_by_extent,
    voxel_downsample,
)


class DbscanTest(unittest.TestCase):
    def test_two_clusters_and_noise(self):
        points = [(0, 0), (.1, 0), (0, .1), (5, 5), (5.1, 5), (5, 5.1), (20, 20)]
        self.assertEqual(dbscan(points, .2, 3).tolist(), [0, 0, 0, 1, 1, 1, NOISE])

    def test_density_reachability_expands_cluster(self):
        points = [(0, 0), (.15, 0), (.30, 0), (.45, 0), (.60, 0)]
        self.assertEqual(dbscan(points, .16, 2).tolist(), [0, 0, 0, 0, 0])

    def test_border_point_previously_marked_noise_is_absorbed(self):
        points = [(0, 0), (.2, 0), (.2, .1), (.2, -.1)]
        self.assertEqual(dbscan(points, .21, 4).tolist(), [0, 0, 0, 0])

    def test_duplicate_points_count_as_samples(self):
        self.assertEqual(dbscan([(1, 1)] * 3, .1, 3).tolist(), [0, 0, 0])

    def test_empty_input(self):
        labels = dbscan(np.empty((0, 2)), .3, 4)
        self.assertEqual(labels.dtype, np.int32)
        self.assertEqual(labels.shape, (0,))

    def test_invalid_arguments(self):
        for points in ([(0, 0, 0)], [(0, float('nan'))]):
            with self.assertRaises(ValueError):
                dbscan(points)
        for eps in (0, -1, float('inf'), True):
            with self.assertRaises(ValueError):
                dbscan([(0, 0)], eps, 1)
        for minimum in (0, 1.5, True):
            with self.assertRaises(ValueError):
                dbscan([(0, 0)], .3, minimum)

    def test_voxel_downsample_returns_xyz_centroids(self):
        result = voxel_downsample(
            np.asarray(((0.01, 0.01, 1.0), (0.04, 0.03, 3.0), (0.11, 0.0, 5.0))),
            0.05,
        )
        np.testing.assert_allclose(result, ((0.025, 0.02, 2.0), (0.11, 0.0, 5.0)))

    def test_voxel_downsample_validates_input(self):
        for points, size in (([(0, 0, 0, 0)], .05), ([(0, float("nan"))], .05),
                             ([(0, 0)], 0), ([(0, 0)], True)):
            with self.assertRaises(ValueError):
                voxel_downsample(points, size)

    def test_filter_clusters_by_extent_rejects_only_oversized_cluster(self):
        points = np.asarray(((0, 0), (0.5, 0), (1.0, 0),
                             (5, 5), (5.4, 5.3), (5.8, 5.6)))
        labels = np.asarray((0, 0, 0, 1, 1, 1), dtype=np.int32)
        result = filter_clusters_by_extent(points, labels, 0.9)
        self.assertEqual(result.tolist(), [NOISE, NOISE, NOISE, 1, 1, 1])

    def test_filter_clusters_by_extent_zero_disables_filter(self):
        labels = np.asarray((0, 0), dtype=np.int32)
        result = filter_clusters_by_extent(((0, 0), (100, 0)), labels, 0)
        self.assertEqual(result.tolist(), labels.tolist())

    def test_filter_clusters_by_extent_validates_input(self):
        invalid_cases = (
            ([(0, 0, 0)], [0, 0], 1.0),
            ([(0, float("nan"))], [0], 1.0),
            ([(0, 0)], [0], -1.0),
            ([(0, 0)], [0], True),
        )
        for points, labels, maximum in invalid_cases:
            with self.assertRaises(ValueError):
                filter_clusters_by_extent(points, labels, maximum)


if __name__ == '__main__':
    unittest.main()
