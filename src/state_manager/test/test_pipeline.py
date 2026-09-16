"""Mission Runtime -> Selector integration without a separate command gate."""

import math
from pathlib import Path
import sys
import unittest


PACKAGES = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(PACKAGES / name / 'src') for name in
                ('state_manager', 'selector')]

from selector.core import Candidate, Pose, SelectorCore, State
from stier_state_manager.geometry import Route, scan_to_geometry
from stier_state_manager.runtime import MissionRuntime


def straight(name, start, end):
    return Route(name, [(float(x), 0.0, 0.0) for x in range(start, end + 1)])


def fixture_config(start='1_right'):
    return {'start_route': start, 'landmarks_validated': True,
            'vehicle': {'validated': True, 'curb_visibility_validated': True,
                        'front_m': .5, 'rear_m': .5, 'width_m': 1.0,
                        'max_speed_mps': 2.0, 'max_yaw_rate_rps': 1.0,
                        'deceleration_mps2': 1.0, 'reaction_s': .2, 'margin_m': .1},
            'stop_buffer_m': .05,
            'landmarks': {'1_right': {'hill_start_s': 1.0, 'hill_stop_s': 4.0,
                                      'hill_top_s': 8.0},
                          '2': {'stop_line_s': 5.0, 'intersection_exit_s': 8.0},
                          '4': {'stop_line_s': 5.0, 'intersection_exit_s': 8.0}}}


class Pipeline:
    def __init__(self):
        routes = {'1_right': straight('1_right', 0, 10),
                  '2': straight('2', 10, 20),
                  '3_s-static-obstacle': straight('3_s-static-obstacle', 20, 30),
                  '4': straight('4', 30, 40)}
        self.routes = routes
        self.runtime = MissionRuntime(routes, fixture_config())
        self.selector = SelectorCore()
        self.now = 1.0

    def observation(self, x, speed=0.0):
        hits, rays = scan_to_geometry([math.inf] * 361, -math.pi,
                                      math.pi / 180, .01, 60.0,
                                      scanner_pose=(x, 0.0, 0.0))
        return {'odom': {'stamp': self.now, 'frame': 'map',
                         'child_frame': 'base_link', 'x': x, 'y': 0.0,
                         'yaw': 0.0, 'speed': speed, 'yaw_rate': 0.0,
                         'position_variance': .01, 'yaw_variance': .01},
                'localization': {'stamp': self.now, 'valid': True},
                'scan': {'stamp': self.now, 'valid': True,
                         'hits': hits, 'rays': rays}}

    def candidates(self):
        name, mode, direction = self.runtime.request or (
            self.runtime.tracker.route_name, 'RDDF', 1)
        yaw = math.pi if direction < 0 else 0.0
        poses = tuple(Pose('map', (x, y, 0.0),
                           (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
                      for x, y, _ in self.routes[name].points)
        return {mode: Candidate(self.now, self.now,
                                self.runtime.decision_id or 1, name, direction,
                                'map', 'map', self.now, poses)}

    def step(self, x=0.0, speed=0.0, edit=None, candidates=None):
        self.now = round(self.now + .1, 8)
        data = self.observation(x, speed)
        if edit:
            edit(data)
        choices = self.candidates() if candidates is None else candidates
        decision = self.runtime.step(self.now, data, choices)
        state = State(self.now, self.now, decision['decision_id'],
                      decision['route'], decision['path_mode'],
                      decision['direction'], decision['valid'],
                      decision['stop_requested'])
        selected = self.selector.evaluate(state, choices, self.now)
        commandable = (decision['valid'] and not decision['finished'] and
                       not decision['stop_requested'] and
                       decision['speed_limit'] > 0.0 and selected.ready)
        return decision, selected, commandable


class DirectPipelineTests(unittest.TestCase):
    def test_lidar_dropout_stops_and_fresh_data_recovers(self):
        pipeline = Pipeline()
        self.assertTrue(pipeline.step()[2])
        decision, selected, commandable = pipeline.step(
            edit=lambda data: data['scan'].update(valid=False))
        self.assertFalse(decision['valid'])
        self.assertFalse(commandable)
        self.assertTrue(selected.ready)
        self.assertTrue(pipeline.step()[2])

    def test_missing_requested_local_path_never_falls_back_to_rddf(self):
        pipeline = Pipeline()
        pipeline.runtime.tracker.route_name = '3_s-static-obstacle'
        pipeline.runtime.request = ('3_s-static-obstacle', 'LOCAL', 1)
        decision, selected, commandable = pipeline.step(x=20.0, candidates={})
        self.assertEqual(decision['path_mode'], 'LOCAL')
        self.assertFalse(selected.ready)
        self.assertFalse(commandable)

    def test_full_replay_exercises_reverse_command_contract(self):
        from importlib.util import module_from_spec, spec_from_file_location
        source = PACKAGES / 'state_manager' / 'examples' / 'replay_missions.py'
        spec = spec_from_file_location('synthetic_mission_replay', source)
        replay_module = module_from_spec(spec)
        spec.loader.exec_module(replay_module)
        report = replay_module.replay('left', 'right')
        self.assertTrue(report['reverse_output_supported'])
        self.assertEqual(report['sections'], list(range(1, 14)))
        reverse = [row for row in report['events']
                   if row['phase'] == 'REVERSE_ENTRY']
        self.assertTrue(reverse)
        self.assertTrue(any(row['control_allowed'] for row in reverse))


if __name__ == '__main__':
    unittest.main()
