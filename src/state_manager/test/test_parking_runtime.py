"""Replay parking plan epochs through localization matches and the runtime."""

import importlib.util
import math
from pathlib import Path
import sys
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(PACKAGE / 'src'), str(PACKAGE.parent / 'selector' / 'src')]
from stier_state_manager.geometry import Route, project
from stier_state_manager.runtime import MissionRuntime


class ParkingRuntimeTests(unittest.TestCase):
    route_name = '10_parallel-right-in'

    def setUp(self):
        self.route = Route(self.route_name, [(x, 0, 0) for x in range(11)])
        self.config = {
            'start_route': self.route_name, 'landmarks_validated': True,
            'vehicle': {'validated': True, 'curb_visibility_validated': True,
                        'front_m': .5, 'rear_m': .5, 'width_m': 1.0,
                        'max_speed_mps': 2.0, 'max_yaw_rate_rps': 1.0,
                        'deceleration_mps2': 1.0, 'reaction_s': .2, 'margin_m': .1},
            'stop_buffer_m': .05,
            'landmarks': {self.route_name: {'parking_confirm_s': 8.0}},
        }
        self.runtime = self.make_runtime()

    def make_runtime(self):
        runtime = MissionRuntime({self.route_name: self.route}, self.config)
        # This replay begins after section 9 has selected its LiDAR-cleared bay.
        # Branch selection itself is covered by the course and mission replays.
        runtime.engine.branches['parallel'] = 'right'
        return runtime

    def data(self, now, x=0.0, y=0.0, yaw=0.0, speed=0.0, maneuver=None):
        result = {
            'odom': {'stamp': now, 'frame': 'map', 'child_frame': 'base_link',
                     'x': x, 'y': y, 'yaw': yaw, 'speed': speed, 'yaw_rate': 0.0,
                     'position_variance': .01, 'yaw_variance': .01},
            'localization': {'stamp': now, 'valid': True},
        }
        route = self.runtime.source_routes[self.runtime.active_route_name]
        matched = project(route, x, y)
        segment = matched['segment']
        length = route.s[segment + 1] - route.s[segment]
        fraction = ((matched['s'] - route.s[segment]) / length
                    if length > 1e-9 else 0.0)
        candidate = {'route': route.name, 'segment_index': segment,
                     'segment_fraction': fraction,
                     'distance_m': matched['distance']}
        result['rddf_match'] = {
            'stamp': now, 'received': now, 'pose_stamp': now, 'frame': 'map',
            'matched': True, **candidate, 'candidates': [candidate]}
        if maneuver is not None:
            result['parking_maneuver'] = maneuver
        return result

    def maneuver(self, now, decision_id=None, reverse=False):
        return dict(stamp=now, route=self.route_name,
                    decision_id=self.runtime.decision_id if decision_id is None else decision_id,
                    leg_index=1 if reverse else 0,
                    phase='REVERSE_ENTRY' if reverse else 'FORWARD_APPROACH',
                    direction=-1 if reverse else 1, start_s=2.0 if reverse else 0.0,
                    target_s=8.0 if reverse else 2.0, final_leg=reverse)

    def candidate(self, now, decision_id=None, direction=None, body_yaw=None):
        direction = self.runtime.request[2] if direction is None else direction
        decision_id = self.runtime.decision_id if decision_id is None else decision_id
        return {'stamp': now, 'receipt_stamp': now,
                'decision_id': decision_id, 'route': self.route_name,
                'source': 'PARKING', 'direction': direction,
                'ready': True, 'reason': 'PATH_READY',
                'path_fingerprint': 'parking-test-path'}

    def accept_forward(self):
        self.runtime.step(1.0, self.data(1.0), {})
        self.assertEqual(self.runtime.decision_id, 1)
        accepted = self.runtime.step(1.1, self.data(1.1, maneuver=self.maneuver(1.1)),
                                     self.candidate(1.1))
        self.assertTrue(accepted['stop_requested'])
        self.assertEqual(accepted['decision_id'], 2)
        self.assertEqual(accepted['direction'], 1)
        self.assertEqual(accepted['parking_leg_phase'], 'FORWARD_APPROACH')
        return accepted

    def advance_to_forward_target(self):
        for now, x in ((1.2, 0.0), (1.3, 1.0), (1.4, 2.0)):
            result = self.runtime.step(now, self.data(now, x=x, maneuver=self.maneuver(now)),
                                       self.candidate(now))
        return result

    def accept_reverse(self):
        self.accept_forward()
        self.advance_to_forward_target()
        result = self.runtime.step(1.5, self.data(1.5, x=2, maneuver=self.maneuver(1.5, reverse=True)),
                                   self.candidate(1.5))
        self.assertEqual(result['decision_id'], 3)
        self.assertEqual(result['direction'], -1)
        self.assertTrue(result['stop_requested'])
        return result

    def test_real_parallel_right_route_localizes_before_any_gear_plan(self):
        source = PACKAGE.parent / 'localization' / 'scripts' / 'rddf_route_provider.py'
        spec = importlib.util.spec_from_file_location('parking_route_provider', source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        _, routes = module.load_catalogue(PACKAGE.parent / 'localization' / 'rddf')
        name, direction, points = next(route for route in routes if route[0] == self.route_name)
        self.route = Route(name, points, direction)
        self.config['landmarks'][name]['parking_confirm_s'] = self.route.length
        self.runtime = self.make_runtime()
        x, y, yaw = self.route.start
        result = self.runtime.step(1, self.data(1, x=x, y=y, yaw=yaw), {})
        self.assertTrue(result['tracking']['healthy'])
        self.assertTrue(result['valid'])
        self.assertTrue(result['stop_requested'])
        self.assertEqual(result['direction'], 1)
        self.assertEqual(result['reason'], 'PARKING_MANEUVER_STALE_OR_MISMATCHED')

    def test_first_forward_leg_changes_epoch_even_without_direction_change(self):
        accepted = self.accept_forward()
        self.assertEqual(accepted['reason'], 'WAIT_NEW_PATH')
        # A new path cannot make an old-epoch maneuver advertise current intent.
        old = self.runtime.step(1.2, self.data(1.2, maneuver=self.maneuver(1.2, decision_id=1)),
                                self.candidate(1.2))
        self.assertTrue(old['stop_requested'])
        self.assertEqual(old['decision_id'], 2)
        for now in (1.3, 1.4, 1.5):
            ready = self.runtime.step(now, self.data(now, maneuver=self.maneuver(now)), self.candidate(now))
            self.assertTrue(ready['valid'])
            self.assertFalse(ready['safety']['stop'], ready)
            self.assertFalse(ready['stop_requested'], ready)
            self.assertEqual(ready['decision_id'], 2)  # No accepted-leg epoch loop.
            self.assertGreater(ready['speed_limit'], 0)

    def test_gear_change_rejects_motion_and_requires_new_path_epoch_when_stopped(self):
        self.accept_forward()
        at_target = self.advance_to_forward_target()
        self.assertEqual(at_target['remaining_stop_m'], 0)
        self.assertTrue(at_target['stop_requested'])
        moving = self.runtime.step(1.5, self.data(1.5, x=2, speed=.2,
                                                maneuver=self.maneuver(1.5, reverse=True)),
                                   self.candidate(1.5))
        self.assertEqual(moving['reason'], 'WAIT_STANDSTILL_FOR_PARKING_LEG')
        self.assertEqual((moving['decision_id'], moving['direction']), (2, 1))
        accepted = self.runtime.step(1.6, self.data(1.6, x=2, maneuver=self.maneuver(1.6, reverse=True)),
                                     self.candidate(1.6))
        self.assertEqual((accepted['decision_id'], accepted['direction']), (3, -1))
        self.assertTrue(accepted['stop_requested'])
        old_path = self.runtime.step(1.7, self.data(1.7, x=2, maneuver=self.maneuver(1.7, reverse=True)),
                                     self.candidate(1.7, decision_id=2, direction=-1))
        self.assertTrue(old_path['stop_requested'])
        self.assertEqual(old_path['reason'], 'REQUESTED_PATH_UNAVAILABLE')
        ready = self.runtime.step(1.8, self.data(1.8, x=2, maneuver=self.maneuver(1.8, reverse=True)),
                                  self.candidate(1.8))
        self.assertFalse(ready['stop_requested'], ready)
        self.assertFalse(ready['safety']['stop'], ready)
        self.assertEqual(ready['remaining_stop_m'], 6)
        self.assertGreater(ready['speed_limit'], 0)  # Old setup stop target was cleared.

    def test_old_or_stale_maneuver_cannot_revive_reverse_preview(self):
        self.accept_reverse()
        for now, observation in (
            (1.6, self.maneuver(1.6, decision_id=2, reverse=True)),
            (2.2, self.maneuver(1.5, decision_id=3, reverse=True)),
        ):
            result = self.runtime.step(now, self.data(now, x=2, maneuver=observation), self.candidate(now))
            self.assertTrue(result['stop_requested'])
            self.assertEqual(result['reason'], 'PARKING_MANEUVER_STALE_OR_MISMATCHED')
            self.assertEqual((result['decision_id'], result['direction']), (3, -1))

    def test_parking_path_is_not_blocked_by_removed_global_heading_gate(self):
        self.accept_forward()
        result = self.runtime.step(1.2, self.data(1.2, yaw=math.pi, maneuver=self.maneuver(1.2)),
                                   self.candidate(1.2))
        self.assertTrue(result['tracking']['healthy'])
        self.assertFalse(result['safety']['stop'], result)
        self.assertEqual(result['safety']['reason'], 'PATH_ACCEPTED')
        self.assertFalse(result['stop_requested'], result)


if __name__ == '__main__':
    unittest.main()
