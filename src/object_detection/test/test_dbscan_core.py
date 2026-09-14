import unittest

import numpy as np

from object_detection_core import NOISE, dbscan


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


if __name__ == '__main__':
    unittest.main()
