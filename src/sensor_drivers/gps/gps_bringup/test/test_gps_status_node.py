#!/usr/bin/env python3
"""GpsStatus가 서로 다른 navigation epoch를 합치지 않는지 확인한다."""

import importlib.machinery
import importlib.util
from pathlib import Path
import unittest


PACKAGE_DIR = Path(__file__).resolve().parents[1]
NODE_PATH = PACKAGE_DIR / "scripts" / "gps_status_node.py"
LOADER = importlib.machinery.SourceFileLoader("gps_status_node_test", str(NODE_PATH))
SPEC = importlib.util.spec_from_loader(LOADER.name, LOADER)
GPS_STATUS_NODE = importlib.util.module_from_spec(SPEC)
LOADER.exec_module(GPS_STATUS_NODE)


class GpsStatusNodeTest(unittest.TestCase):
    def test_same_epoch_has_zero_distance(self):
        self.assertEqual(0, GPS_STATUS_NODE.GpsStatusNode._itow_distance_ms(1234, 1234))

    def test_different_epoch_distance_is_measured_in_milliseconds(self):
        self.assertEqual(501, GPS_STATUS_NODE.GpsStatusNode._itow_distance_ms(1000, 1501))

    def test_gps_week_rollover_uses_short_distance(self):
        week = GPS_STATUS_NODE.GPS_WEEK_MILLISECONDS
        self.assertEqual(200, GPS_STATUS_NODE.GpsStatusNode._itow_distance_ms(week - 100, 100))


if __name__ == "__main__":
    unittest.main()
