"""Replay actual runtime -> selector -> command gate, without ROS or a vehicle.

Coordinates, vehicle dimensions and free-space observations below are synthetic
test fixtures, not calibration for the Yongin course. No component under test is
mocked, and tests never publish a vehicle command.
"""
from dataclasses import replace
import importlib.util
import math
from pathlib import Path
import sys
import unittest

PACKAGES = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(PACKAGES / name / 'src') for name in
                ('state_manager', 'selector', 'vehicle_safety')]

from selector.core import Candidate, Pose, SelectorCore, State, path_fingerprint
from stier_state_manager.geometry import Route, scan_to_geometry
from stier_state_manager.runtime import MissionRuntime
from vehicle_safety.core import Mission, PathReady, RawCommand, SafetyGate, SafetyInput


def straight(name, start, end):
    return Route(name, [(float(x), 0.0, 0.0) for x in range(start, end + 1)])


def fixture_config(start='1_right'):
    return {'start_route': start, 'landmarks_validated': True,
            'vehicle': {'validated': True, 'curb_visibility_validated': True,
                        'front_m': .5, 'rear_m': .5, 'width_m': 1.0,
                        'max_speed_mps': 2.0, 'max_yaw_rate_rps': 1.0,
                        'deceleration_mps2': 1.0, 'reaction_s': .2, 'margin_m': .1},
            'stop_buffer_m': .05,
            'landmarks': {'1_right': {'hill_start_s': 1.0, 'hill_stop_s': 4.0, 'hill_top_s': 8.0},
                          '2': {'stop_line_s': 5.0, 'intersection_exit_s': 8.0},
                          '4': {'stop_line_s': 5.0, 'intersection_exit_s': 8.0}}}


class Pipeline:
    def __init__(self, routes=None, config=None):
        self.routes = routes or {'1_right': straight('1_right', 0, 10),
                                  '2': straight('2', 10, 20),
                                  '3_s-static-obstacle': straight('3_s-static-obstacle', 20, 30),
                                  '4': straight('4', 30, 40)}
        self.runtime = MissionRuntime(self.routes, config or fixture_config())
        self.selector, self.gate = SelectorCore(), SafetyGate()
        self.now = 1.0

    def observation(self, x, y=0.0, yaw=0.0, speed=0.0):
        hits, rays = scan_to_geometry([math.inf] * 361, -math.pi, math.pi / 180,
                                      .01, 60.0, scanner_pose=(x, y, 0))
        return {'odom': {'stamp': self.now, 'frame': 'map', 'child_frame': 'base_link',
                         'x': x, 'y': y, 'yaw': yaw, 'speed': speed, 'yaw_rate': 0.0,
                         'position_variance': .01, 'yaw_variance': .01},
                'localization': {'stamp': self.now, 'valid': True},
                'scan': {'stamp': self.now, 'valid': True, 'hits': hits, 'rays': rays}}

    def candidates(self):
        name, mode, direction = self.runtime.request or (
            self.runtime.tracker.route_name, 'RDDF', 1)
        poses = tuple(Pose('map', (x, y, 0.0), (0.0, 0.0,
                     math.sin((yaw + (math.pi if direction < 0 else 0)) / 2),
                     math.cos((yaw + (math.pi if direction < 0 else 0)) / 2)))
                      for x, y, yaw in self.routes[name].points)
        return {mode: Candidate(self.now, self.now, self.runtime.decision_id or 1,
                                name, direction, 'map', 'map', self.now, poses)}

    def deliver(self, decision, candidates, localization=True, raw=True):
        state = State(self.now, self.now, decision['decision_id'], decision['route'],
                      decision['path_mode'], decision['direction'], decision['valid'],
                      decision['stop_requested'])
        selected = self.selector.evaluate(state, candidates, self.now)
        self.gate.update_mission(Mission(self.now, state.decision_id, state.route_name,
            state.path_mode, state.direction, decision['speed_limit'], state.stop_requested,
            state.valid, decision['finished']), self.now)
        check = decision['safety']
        self.gate.update_safety(SafetyInput(self.now, check['stop'], check['sensor_valid'],
            check['reason'], check['clearance_m'], check['path_fingerprint']), self.now)
        self.gate.update_path(PathReady(self.now, state.decision_id, state.route_name,
            selected.source, state.direction, selected.ready,
            path_fingerprint(selected.candidate) if selected.ready else ''), self.now)
        self.gate.update_localization(localization, self.now)
        if raw:
            self.gate.update_raw(RawCommand(10, 0, 0), self.now)
        return selected, self.gate.evaluate(self.now)

    def step(self, x=0.0, speed=0.0, edit=None, candidates=None, raw=True):
        self.now = round(self.now + .1, 8)
        data = self.observation(x, speed=speed)
        if edit:
            edit(data)
        choices = self.candidates() if candidates is None else candidates
        decision = self.runtime.step(self.now, data, choices)
        selected, command = self.deliver(decision, choices, data['localization']['valid'], raw)
        return decision, selected, command


class PipelineTests(unittest.TestCase):
    def test_lidar_preview_selects_clear_parking_alternative_in_both_approaches(self):
        from stier_state_manager.mission import PARKING_ROUTES
        for kind, approach in (('t', '4'), ('parallel', '9')):
            for clear_side in ('left', 'right'):
                with self.subTest(kind=kind, clear_side=clear_side):
                    routes = {approach: straight(approach, 0, 10)}
                    for side, (entry, exit_route) in PARKING_ROUTES[kind].items():
                        y = 4.0 if side == 'left' else -4.0
                        routes[entry] = Route(entry, [(10.0, 0.0, math.atan2(y, 1)),
                                                      (11.0, y, 0.0), (12.0, y, 0.0)])
                        routes[exit_route] = Route(exit_route, [(12.0, y, 0.0), (13.0, y, 0.0)])
                    config = fixture_config(approach)
                    config['preferred_parking_branch'] = 'right' if clear_side == 'left' else 'left'
                    pipeline = Pipeline(routes, config)
                    def perception(data):
                        data['signal'] = {'stamp': pipeline.now, 'route': approach, 'value': 'GREEN'}
                        # Explicit synthetic cone return lies on the other bay.
                        data['scan']['hits'].append((11.0, -4.0 if clear_side == 'left' else 4.0))
                    for index in range(100):
                        decision, _, command = pipeline.step(x=index / 10, speed=.5, edit=perception)
                        if decision['route'] != approach:
                            break
                    self.assertEqual(decision['route'], PARKING_ROUTES[kind][clear_side][0])
                    self.assertFalse(command.allowed)
                    self.assertEqual(decision['phase'], 'HANDOFF')

    def test_full_thirteen_section_message_replay_covers_both_branch_choices(self):
        source = PACKAGES / 'state_manager' / 'examples' / 'replay_missions.py'
        spec = importlib.util.spec_from_file_location('synthetic_mission_replay', source)
        replay_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(replay_module)
        for parking in ('left', 'right'):
            for lane in ('left', 'right'):
                with self.subTest(parking=parking, lane=lane):
                    report = replay_module.replay(parking, lane)
                    self.assertTrue(report['simulation_only'])
                    self.assertFalse(report['vehicle_output'])
                    self.assertEqual(report['sections'], list(range(1, 14)))
                    events = report['events']
                    expected = {'hill', 'hill_clearance', 'static', 'dynamic', 'dynamic_hold',
                                'intersection:2', 'intersection:4', 'intersection:7',
                                'parking:t:entry', 'parking:t:exit',
                                'parking:parallel:entry', 'parking:parallel:exit', 'finish'}
                    self.assertEqual(set(report['completed_missions']), expected)
                    for section in (5, 6, 10, 11):
                        self.assertTrue(all(parking in row['route'] for row in events
                                            if row['section'] == section))
                    self.assertEqual(events[-1]['route'], '13_' + lane)
                    self.assertEqual(events[-1]['gate_reason'], 'MISSION_FINISHED')
                    self.assertEqual([row['decision_id'] for row in events],
                                     sorted(row['decision_id'] for row in events))
                    reverse = [row for row in events if row['phase'] == 'REVERSE_ENTRY']
                    self.assertEqual(len(reverse), 2)
                    self.assertTrue(all(row['gate_reason'] == 'REVERSE_INTERFACE_UNAVAILABLE'
                                        and not row['gate_allowed'] for row in reverse))
                    for section, completion in ((1, 'hill'), (8, 'dynamic_hold')):
                        start = next(row['time_s'] for row in events
                                     if row['section'] == section and row['phase'] == 'HOLD')
                        self.assertGreaterEqual(report['completed_missions'][completion] - start, 3.0)

    def test_lidar_dropout_stops_pipeline_and_recovery_requires_new_command(self):
        pipeline = Pipeline()
        self.assertTrue(pipeline.step()[2].allowed)
        decision, _, command = pipeline.step(edit=lambda data: data['scan'].update(valid=False))
        self.assertFalse(decision['valid'])
        self.assertEqual((command.kph, command.brake), (0, 1))
        decision, selected, command = pipeline.step(raw=False)
        self.assertTrue(decision['valid'])
        self.assertTrue(selected.ready)
        self.assertFalse(command.allowed)
        self.assertEqual(command.reason, 'FRESH_RAW_COMMAND_REQUIRED')
        self.assertTrue(pipeline.step()[2].allowed)

    def test_geometry_change_cannot_reuse_previous_safety_approval(self):
        pipeline = Pipeline()
        decision, _, command = pipeline.step()
        self.assertTrue(command.allowed)
        old = pipeline.candidates()['RDDF']
        changed = replace(old, poses=tuple(replace(p, position=(p.position[0], .1, 0.0))
                                            for p in old.poses))
        self.assertNotEqual(path_fingerprint(old), path_fingerprint(changed))
        selected, command = pipeline.deliver(decision, {'RDDF': changed})
        self.assertTrue(selected.ready)
        self.assertFalse(command.allowed)
        self.assertEqual(command.reason, 'PATH_FINGERPRINT_MISMATCH')
        # Runtime inspects the new geometry, after which a fresh command may run.
        decision = pipeline.runtime.step(pipeline.now, pipeline.observation(0), {'RDDF': changed})
        _, command = pipeline.deliver(decision, {'RDDF': changed}, raw=False)
        self.assertFalse(command.allowed)
        _, command = pipeline.deliver(decision, {'RDDF': changed}, raw=True)
        self.assertTrue(command.allowed, command)
        refreshed = replace(changed, stamp=changed.stamp + .01, received=changed.received + .01,
                            path_stamp=changed.path_stamp + .01)
        self.assertEqual(path_fingerprint(refreshed), path_fingerprint(changed))

    def test_hill_signal_and_static_route_handoffs_preserve_request_identity(self):
        pipeline = Pipeline()
        pipeline.step()
        for index in range(1, 41):
            pipeline.step(x=index / 10, speed=1.0 if index < 40 else 0.0)
        for _ in range(31):
            decision, _, command = pipeline.step(x=4.0)
        self.assertIn('hill', decision['completed_missions'])
        self.assertTrue(command.allowed)
        for index in range(41, 101):
            old_choices = pipeline.candidates()
            decision, selected, command = pipeline.step(x=index / 10, speed=.5)
            if decision['route'] == '2':
                break
        self.assertEqual(decision['phase'], 'HANDOFF')
        self.assertFalse(decision['valid'])
        self.assertFalse(command.allowed)
        self.assertEqual(selected.reason, 'PATH_REQUEST_MISMATCH')
        # Even a freshly timestamped old route cannot satisfy the new epoch.
        stale_request = {key: replace(value, stamp=pipeline.now + .1, received=pipeline.now + .1,
                                      path_stamp=pipeline.now + .1)
                         for key, value in old_choices.items()}
        decision, selected, command = pipeline.step(x=10.0, candidates=stale_request)
        self.assertFalse(selected.ready)
        self.assertFalse(command.allowed)
        pipeline.step(x=10.0)
        red = lambda data: data.update(signal={'stamp': pipeline.now, 'route': '2', 'value': 'RED'})
        for index in range(101, 146):
            decision, _, command = pipeline.step(x=index / 10, speed=.5 if index < 145 else 0.0, edit=red)
        self.assertEqual(decision['phase'], 'WAIT_SIGNAL')
        self.assertFalse(command.allowed)
        green = lambda data: data.update(signal={'stamp': pipeline.now, 'route': '2', 'value': 'GREEN'})
        self.assertTrue(pipeline.step(x=14.5, edit=green)[2].allowed)
        pipeline.step(x=14.6, speed=.5, edit=green)
        # A signal changing after authorized entry cannot trigger a signal stop.
        decision, _, command = pipeline.step(x=14.7, speed=.5, edit=red)
        self.assertEqual(decision['phase'], 'CROSSING')
        self.assertTrue(command.allowed)
        for index in range(148, 201):
            decision, _, command = pipeline.step(x=index / 10, speed=.5, edit=red)
            if decision['route'] == '3_s-static-obstacle':
                break
        self.assertEqual(decision['path_mode'], 'LOCAL')
        self.assertFalse(command.allowed)
        # An RDDF source cannot stand in for the required static-avoidance plan.
        decision, selected, command = pipeline.step(x=20.0, candidates={})
        self.assertFalse(selected.ready)
        self.assertFalse(command.allowed)
        decision, selected, command = pipeline.step(x=20.0)
        self.assertEqual(selected.source, 'LOCAL')
        self.assertTrue(command.allowed, (decision, command))

    def test_dynamic_hold_survives_lidar_collision_veto_and_clears_after_three_seconds(self):
        name = '8_dynamic-obstacle'
        pipeline = Pipeline({name: straight(name, 0, 10)}, fixture_config(name))
        def central(data):
            data['dynamic'] = {'stamp': pipeline.now, 'blocked': True, 'central_stopped': True}
            data['scan']['hits'].append((.7, 0.0))
        for _ in range(30):
            decision, _, command = pipeline.step(edit=central)
            self.assertNotIn('dynamic_hold', decision['completed_missions'])
            self.assertFalse(command.allowed)
        for _ in range(2):
            decision, _, command = pipeline.step(edit=central)
        self.assertIn('dynamic_hold', decision['completed_missions'])
        self.assertTrue(decision['safety']['stop'])
        self.assertFalse(command.allowed)
        clear = lambda data: data.update(dynamic={'stamp': pipeline.now, 'blocked': False, 'central_stopped': False})
        decision, _, command = pipeline.step(edit=clear, raw=False)
        self.assertIn('dynamic', decision['completed_missions'])
        self.assertFalse(command.allowed)
        self.assertTrue(pipeline.step(edit=clear)[2].allowed)


if __name__ == '__main__':
    unittest.main()
