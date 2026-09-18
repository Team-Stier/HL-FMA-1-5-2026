#!/usr/bin/env python3
"""Offline school-course checks; no ROS master, sensors or vehicle commands."""
import json
from pathlib import Path
import sys
import unittest

REPOSITORY = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(REPOSITORY / 'src/localization/scripts'),
               str(REPOSITORY / 'src/state_manager/src'),
               str(REPOSITORY / 'src/state_manager/examples')]

from rddf_route_provider import load_catalogue
from stier_state_manager.geometry import Route
from stier_state_manager.mission import MissionEngine, PARKING_ROUTES
from replay_missions import replay


class SchoolCourseConfig(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((REPOSITORY / 'src/state_manager/config/missions_hongik.json').read_text())
        self.origin, catalogue = load_catalogue(REPOSITORY / 'src/localization/test_data/hongik_rddf')
        self.routes = {name: Route(name, points, direction) for name, direction, points in catalogue}

    def test_all_thirteen_sections_and_parking_branches_are_present(self):
        self.assertEqual(len(self.routes), 19)
        self.assertEqual({route.section for route in self.routes.values()}, set(range(1, 14)))
        for variants in PARKING_ROUTES.values():
            for pair in variants.values():
                for name in pair:
                    self.assertIn(name, self.routes)
        self.assertEqual(self.config['map_origin']['latitude'], self.origin['lat'])
        self.assertEqual(self.config['map_origin']['longitude'], self.origin['lng'])
        self.assertEqual(self.config['start_branch'], 'left')
        self.assertAlmostEqual(self.routes['1_left'].start[0], 0.)
        self.assertAlmostEqual(self.routes['1_left'].start[1], 0.)

    def test_mandatory_stop_markers_fit_the_actual_routes(self):
        engine = MissionEngine(self.config)
        for name, route in self.routes.items():
            with self.subTest(route=name):
                _, error = engine._landmarks(
                    {'route': name, 'landmarks': self.config['landmarks']},
                    route.section, route.length)
                self.assertIsNone(error)
        self.assertEqual(engine.rules['hill_hold_s'], 3.5)
        self.assertEqual(engine.rules['traffic_force_departure_s'], 20.)
        self.assertEqual(self.config['finish_fallback_branch'], 'left')

    def test_recorded_parallel_gear_changes_fit_both_branches(self):
        for name, profile in self.config['parallel_parking_profiles'].items():
            previous = 0.
            for change in profile['changes']:
                self.assertGreater(change['s'], previous)
                self.assertLess(change['s'], self.routes[name].length)
                previous = change['s']

    def test_synthetic_full_mission_logic_can_finish_both_branches(self):
        # Separate from actual map checks above: synthetic mission messages,
        # not a vehicle/traction simulation or proof of physical course fit.
        for side in ('left', 'right'):
            with self.subTest(side=side):
                result = replay(side, side)
                self.assertEqual(result['sections'], list(range(1, 14)))
                self.assertIn('finish', result['completed_missions'])
                self.assertFalse(result['vehicle_output'])

    def test_camera_free_full_mission_stops_then_departs_at_all_three_signals(self):
        for parking in ('left', 'right'):
            for finish in ('left', 'right'):
                with self.subTest(parking=parking, finish=finish):
                    result = replay(parking, finish, camera_enabled=False)
                    self.assertEqual(result['sections'], list(range(1, 14)))
                    self.assertIn('hill', result['completed_missions'])
                    self.assertIn('finish', result['completed_missions'])
                    departures = [e for e in result['events']
                                  if e['reason'] == 'TRAFFIC_FORCE_DEPARTURE_AFTER_TIMEOUT']
                    self.assertEqual([e['route'] for e in departures], ['2', '4', '7'])
                    self.assertFalse(result['camera_enabled'])
                    self.assertFalse(result['vehicle_output'])


if __name__ == '__main__':
    unittest.main()
