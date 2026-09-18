#!/usr/bin/env python3
"""RDDF placement regression: offline geometry only, never vehicle control."""
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'scripts'))
from prepare_test_rddf import Placement, gps_heading, prepare
from rddf_initialization_core import RddfRouteMap
from rddf_route_provider import load_catalogue


class TestPlacement(unittest.TestCase):
    def setUp(self):
        self.source = PACKAGE / 'test_data/hongik_s_rddf'
        self.original = RddfRouteMap(self.source)
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.output = Path(self.temp.name) / 'placed'
        self.lat, self.lon, self.yaw = 37.5510307, 126.9247017, math.radians(7.7846065375)
        self.report = prepare(self.source, self.output, '3_s-static-obstacle',
                              self.lat, self.lon, self.yaw, .65, .10)
        self.placed = RddfRouteMap(self.output)

    def test_antenna_offset_and_s_start_heading(self):
        antenna = np.array(self.original.project_gps(self.lat, self.lon))
        lever = np.array([.65 * math.cos(self.yaw) - .10 * math.sin(self.yaw),
                          .65 * math.sin(self.yaw) + .10 * math.cos(self.yaw)])
        np.testing.assert_allclose(self.placed.routes['3_s-static-obstacle'][0],
                                   antenna - lever, atol=1e-8)
        delta = np.diff(self.placed.routes['3_s-static-obstacle'][:2], axis=0)[0]
        self.assertAlmostEqual(math.atan2(delta[1], delta[0]), self.yaw, places=7)

    def test_all_lengths_directions_and_gps_coordinates_preserved(self):
        self.assertEqual(set(self.original.routes), set(self.placed.routes))
        self.assertEqual(self.original.route_directions, self.placed.route_directions)
        for name in self.original.routes:
            np.testing.assert_allclose(
                np.linalg.norm(np.diff(self.original.routes[name], axis=0), axis=1),
                np.linalg.norm(np.diff(self.placed.routes[name], axis=0), axis=1), atol=2e-8)
        for path in self.output.rglob('*.csv'):
            with path.open() as stream:
                rows = list(csv.DictReader(stream))
            for row in rows:
                np.testing.assert_allclose(self.placed.project_gps(float(row['latitude']), float(row['longitude'])),
                    [float(row['east_m']), float(row['north_m'])], atol=1e-6)
            original = self.source / path.relative_to(self.output)
            with original.open() as stream:
                original_rows = list(csv.DictReader(stream))
            self.assertEqual([r['distance_m'] for r in rows], [r['distance_m'] for r in original_rows])

    def test_shared_datum_and_all_consumer_configurations(self):
        origin, catalogue = load_catalogue(self.output)
        self.assertEqual(origin, self.original.origin)
        self.assertEqual(len(catalogue), 19)
        init = yaml.safe_load((self.output / 'initialization.yaml').read_text())
        viewer = yaml.safe_load((self.output / 'viewer.yaml').read_text())
        self.assertEqual(init['initialization']['rddf_directory'], str(self.output))
        self.assertEqual(viewer['rddf_directory'], str(self.output))
        self.assertEqual(init['reference']['latitude_deg'], origin['lat'])
        self.assertEqual(init['reference']['longitude_deg'], origin['lng'])

    def test_markers_move_but_station_and_branch_semantics_do_not(self):
        old = json.loads((self.source / 'yongin_mission_landmarks.json').read_text())
        new = json.loads((self.output / 'yongin_mission_landmarks.json').read_text())
        self.assertEqual(old['origin'], new['origin'])
        for route in old['landmarks']:
            for key, value in old['landmarks'][route].items():
                if isinstance(value, dict):
                    moved = new['landmarks'][route][key]
                    self.assertEqual(value['projected_s_m'], moved['projected_s_m'])
                    self.assertNotEqual(value['east_m'], moved['east_m'])
                    np.testing.assert_allclose(self.placed.project_gps(moved['latitude'], moved['longitude']),
                                               [moved['east_m'], moved['north_m']], atol=1e-6)

    def test_original_files_and_existing_placement_are_not_overwritten(self):
        before = {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in self.source.rglob('*') if p.is_file()}
        with self.assertRaises(FileExistsError):
            prepare(self.source, self.output, '3_s-static-obstacle', self.lat, self.lon, self.yaw, .65, 0.)
        self.assertEqual(before, {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in before})

    def test_measured_gps_heading_is_not_the_old_imu_heading(self):
        yaw, distance = gps_heading(self.original, 37.5510243, 126.9246429, self.lat, self.lon)
        self.assertAlmostEqual(math.degrees(yaw), 7.7846065375)
        self.assertAlmostEqual(distance, 5.2441965196)


if __name__ == '__main__':
    unittest.main()
