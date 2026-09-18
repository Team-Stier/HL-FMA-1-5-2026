#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import unittest

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'rddf_offset_planner_node.py'
SPEC = importlib.util.spec_from_file_location('rddf_offset_planner_node', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def cluster(x, y):
    marker = Marker()
    marker.ns = 'dbscan_clusters'
    marker.action = Marker.ADD
    marker.type = Marker.POINTS
    marker.points = [Point(x, y, 0.0)]
    return marker


class RddfOffsetPlannerTest(unittest.TestCase):
    path = [(float(x), 0.0, 0.0) for x in range(11)]

    def test_left_obstacle_selects_right_side(self):
        side = MODULE.nearest_obstacle_side(self.path, [cluster(4.0, .3)], 1.0, 8.0)
        self.assertEqual(side, 1)
        shifted = MODULE.offset_path(self.path, -1.0)
        self.assertAlmostEqual(shifted[4][1], -1.0)

    def test_right_obstacle_selects_left_side(self):
        side = MODULE.nearest_obstacle_side(self.path, [cluster(4.0, -.3)], 1.0, 8.0)
        self.assertEqual(side, -1)
        shifted = MODULE.offset_path(self.path, 1.0)
        self.assertAlmostEqual(shifted[4][1], 1.0)

    def test_no_obstacle_on_path_keeps_rddf(self):
        self.assertEqual(
            MODULE.nearest_obstacle_side(self.path, [cluster(4.0, 2.0)], 1.0, 8.0),
            0)

    def test_short_projected_prefix_does_not_reverse_after_offset(self):
        path = [(0.0, 0.0, 0.0), (0.001, 0.01, 0.0), (0.1, 0.5, 0.0)]
        shifted = MODULE.offset_path(path, 0.5)
        source_dx = path[1][0] - path[0][0]
        source_dy = path[1][1] - path[0][1]
        shifted_dx = shifted[1][0] - shifted[0][0]
        shifted_dy = shifted[1][1] - shifted[0][1]
        self.assertGreater(source_dx*shifted_dx + source_dy*shifted_dy, 0.0)


if __name__ == '__main__':
    unittest.main()
