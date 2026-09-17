"""Parallel-parking RDDF gear-profile runtime tests."""

import importlib.util
import json
from pathlib import Path
import sys
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'src'))
from stier_state_manager.geometry import Route, project
from stier_state_manager.runtime import MissionRuntime


class ParkingRuntimeTests(unittest.TestCase):
    route_name = '10_parallel-right-in'

    def setUp(self):
        self.route = Route(self.route_name, [(float(x), 0.0, 0.0) for x in range(21)])
        self.config = {
            'start_route': self.route_name,
            'landmarks_validated': True,
            'parking_branches': {'t': 'left', 'parallel': 'right'},
            'parallel_parking_profiles': {
                self.route_name: {
                    'initial_direction': 1,
                    'changes': [{'s': 5.0, 'direction': -1},
                                {'s': 15.0, 'direction': 1}],
                },
            },
            'vehicle': {'validated': True, 'front_m': .5,
                        'deceleration_mps2': 1.0, 'reaction_s': .2,
                        'max_speed_mps': 2.0},
            'stop_buffer_m': .05,
            'landmarks': {},
        }
        self.runtime = MissionRuntime({self.route_name: self.route}, self.config)

    def data(self, now, x=0.0, speed=0.0, y=0.0):
        matched = project(self.route, x, y)
        segment = matched['segment']
        segment_length = self.route.s[segment + 1] - self.route.s[segment]
        fraction = ((matched['s'] - self.route.s[segment]) / segment_length
                    if segment_length > 1e-9 else 0.0)
        candidate = {'route': self.route_name, 'segment_index': segment,
                     'segment_fraction': fraction, 'distance_m': matched['distance']}
        return {
            'odom': {'stamp': now, 'frame': 'map', 'child_frame': 'base_link',
                     'x': x, 'y': y, 'yaw': 0.0, 'speed': speed,
                     'yaw_rate': 0.0},
            'localization': {'stamp': now, 'valid': True},
            'rddf_match': {'stamp': now, 'received': now, 'pose_stamp': now,
                           'frame': 'map', 'matched': True, **candidate,
                           'candidates': [candidate]},
        }

    def candidate(self, now):
        route, mode, direction = self.runtime.request
        return {'stamp': now, 'receipt_stamp': now,
                'decision_id': self.runtime.decision_id, 'route': route,
                'source': mode, 'direction': direction, 'ready': True,
                'reason': 'PATH_READY', 'path_fingerprint': 'rddf-profile'}

    def prime(self, now=1.0, x=0.0, speed=0.0):
        return self.runtime.step(now, self.data(now, x, speed), {})

    def hold(self, start, x, speed=0.0):
        result = None
        for index in range(5):
            now = start + index * .25
            result = self.runtime.step(now, self.data(now, x, speed),
                                       self.candidate(now))
        return result

    def test_rddf_path_is_clipped_to_current_gear_leg(self):
        result = self.prime()
        self.assertEqual((result['path_mode'], result['direction']), ('RDDF', 1))
        self.assertAlmostEqual(self.runtime.rddf_points(1.1, {})[-1][0], 5.0)

        switched = self.hold(1.1, 4.9)
        self.assertEqual((switched['direction'], switched['decision_id']), (-1, 2))
        points = self.runtime.rddf_points(2.2, {})
        self.assertAlmostEqual(points[0][0], 5.0)
        self.assertAlmostEqual(points[-1][0], 15.0)

    def test_gear_change_waits_for_actual_standstill_and_new_path(self):
        self.prime()
        moving = self.runtime.step(1.1, self.data(1.1, 4.9, .6), self.candidate(1.1))
        self.assertEqual((moving['reason'], moving['direction'], moving['decision_id']),
                         ('PARALLEL_GEAR_CHANGE', 1, 1))
        stopped = self.hold(1.2, 4.9)
        self.assertEqual((stopped['reason'], stopped['direction'], stopped['decision_id']),
                         ('WAIT_NEW_PATH', -1, 2))
        ready = self.runtime.step(2.3, self.data(2.3, 6.0, -.6), self.candidate(2.3))
        self.assertFalse(ready['stop_requested'], ready)

    def test_direct_midroute_start_selects_matching_gear_leg(self):
        result = self.prime(x=10.0)
        self.assertEqual((result['direction'], result['parking_leg_index']), (-1, 1))
        self.assertEqual(self.runtime.decision_id, 2)

    def test_second_change_selects_forward_leg(self):
        self.prime(x=10.0)
        switched = self.hold(1.1, 14.9)
        self.assertEqual((switched['direction'], switched['parking_leg_index']), (1, 2))

    def test_real_parallel_right_route_uses_saved_profile_without_planner(self):
        source = PACKAGE.parent / 'localization' / 'scripts' / 'rddf_route_provider.py'
        spec = importlib.util.spec_from_file_location('parking_route_provider', source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        _, catalogue = module.load_catalogue(PACKAGE.parent / 'localization' / 'rddf')
        name, direction, points = next(item for item in catalogue if item[0] == self.route_name)
        route = Route(name, points, direction)
        config = json.loads((PACKAGE / 'config' / 'missions.json').read_text())
        config.update(start_route=name, landmarks_validated=True)
        config['parking_branches']['parallel'] = 'right'
        runtime = MissionRuntime({name: route}, config)
        self.route, self.runtime = route, runtime
        x, y, _ = route.start
        result = runtime.step(1.0, self.data(1.0, x=x, y=y), {})
        self.assertTrue(result['valid'])
        self.assertEqual((result['path_mode'], result['direction']), ('RDDF', 1))


if __name__ == '__main__':
    unittest.main()
