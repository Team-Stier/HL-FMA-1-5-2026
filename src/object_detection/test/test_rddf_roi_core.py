#!/usr/bin/env python3

import unittest
from pathlib import Path

import numpy as np

from object_detection_core.rddf_roi import RddfRouteNetwork, circle_union_mask


class RddfRoiCoreTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rddf = Path(__file__).resolve().parents[2] / "localization" / "rddf"
        cls.network = RddfRouteNetwork.from_directory(rddf, 1.0)

    def test_branch_successors_are_discovered_at_real_route_contacts(self):
        self.assertEqual(
            {item.successor for item in self.network.connections["4"]},
            {"5_T-left-in", "5_T-right-in"},
        )
        self.assertEqual(
            {item.successor for item in self.network.connections["12"]},
            {"13_left", "13_right"},
        )

    def test_side_specific_parking_exit_does_not_cross_connect(self):
        self.assertEqual(
            [item.successor for item in self.network.connections["5_T-left-in"]],
            ["6-T-left-out"],
        )
        self.assertEqual(
            [item.successor for item in self.network.connections["10_parallel-right-in"]],
            ["11_parallel-right-out"],
        )

    def test_corridor_reaches_next_rddf_before_current_route_changes(self):
        route = self.network.routes["4"]
        polylines = self.network.corridor(
            "4", len(route.points) - 2, 0.5, lookahead_m=15.0,
            lookbehind_m=1.0, max_depth=2,
        )
        points = np.vstack(polylines)
        for successor in ("5_T-left-in", "5_T-right-in"):
            expected = self.network.routes[successor].points[0]
            self.assertLess(np.min(np.linalg.norm(points - expected, axis=1)), 1e-6)

    def test_circle_union_mask(self):
        mask = circle_union_mask(
            np.asarray(((0.0, 0.0), (1.9, 0.0), (2.1, 0.0), (5.0, 5.0))),
            np.asarray(((0.0, 0.0),)),
            2.0,
        )
        np.testing.assert_array_equal(mask, (True, True, False, False))


if __name__ == "__main__":
    unittest.main()
