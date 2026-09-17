"""Exercise adapter logic with minimal message/clock fakes, without ROS.

Only the node's definitions are loaded; ROS startup/imports stay outside these
tests. These checks complement, rather than claim to replace, catkin/ROS tests.
"""
import ast
import copy
import math
import secrets
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from stier_state_manager.geometry import Route
from stier_state_manager.calibration import validate_landmarks
from stier_state_manager.mission import PARKING_ROUTES, ROUTES


class Stamp:
    def __init__(self, value):
        self.value = value

    def to_sec(self):
        return self.value

    def is_zero(self):
        return self.value == 0


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        def fake_runtime(routes, config):
            runtime = NS(decision_id=0, start_route=config['start_route'], initial_s=None)
            runtime.tracker = NS(set_initial_progress=lambda value: setattr(runtime, 'initial_s', value))
            return runtime
        fake_ros = NS(Time=NS(now=lambda: Stamp(self.now)), Duration=lambda value: value,
                      logwarn_throttle=lambda *args: None, logerr_throttle=lambda *args: None,
                      loginfo=lambda *args: None)
        namespace = {'math': math, 'copy': copy, 'rospy': fake_ros, 'secrets': secrets,
                     'Route': Route, 'ROUTES': ROUTES, 'PARKING_ROUTES': PARKING_ROUTES,
                     'validate_landmarks': validate_landmarks,
                     'MissionRuntime': fake_runtime,
                     'Marker': NS(ADD=0, DELETEALL=3, POINTS=8)}
        source = Path(__file__).resolve().parents[1] / 'scripts' / 'state_manager_node'
        parsed = ast.parse(source.read_text())
        definitions = ast.Module(body=[n for n in parsed.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))],
                                 type_ignores=[])
        exec(compile(definitions, str(source), 'exec'), namespace)
        self.node = namespace['StateManagerNode'].__new__(namespace['StateManagerNode'])
        self.node.lock = threading.RLock()
        self.node.data = {}
        self.node.last_clock, self.node.clock_fault = None, ''
        self.node.config = {'input_timeout_s': .5}
        self.node.routes, self.node.runtime, self.node.map_fingerprint = {}, None, None
        self.node.active_match_route, self.node.rddf_match = '', None
        self.node.last_decision = None
        self.node.configuration_fault, self.node.map_fault = '', ''

    def header(self, stamp=9.9, frame='map'):
        return NS(stamp=Stamp(stamp), frame_id=frame)

    def test_general_green_is_preserved_distinct_from_left_arrow(self):
        self.node.on_signal(NS(header=self.header(), route_name='7', junction_id='left_turn',
                               value='GREEN', confidence=.9))
        self.assertEqual(self.node.data['signal']['value'], 'GREEN')
        self.assertNotEqual(self.node.data['signal']['value'], 'LEFT_ARROW')

    def cluster_marker(self, namespace='dbscan_clusters', action=0, points=None,
                       stamp=9.9, frame='map'):
        return NS(header=self.header(stamp, frame), ns=namespace, action=action, type=8,
                  pose=NS(position=NS(x=0, y=0, z=0),
                          orientation=NS(x=0, y=0, z=0, w=1)),
                  points=[NS(x=x, y=y, z=0) for x, y in (points or [])])

    def test_stamped_dbscan_clusters_are_preserved_for_runtime(self):
        clear = self.cluster_marker(namespace='', action=3)
        cluster = self.cluster_marker(points=[(1, -.2), (1.2, .2)])
        self.node.on_clusters(NS(markers=[clear, cluster]))
        observation = self.node.data['clusters']
        self.assertTrue(observation['valid'], observation)
        self.assertEqual(observation['stamp'], 9.9)
        self.assertEqual(observation['clusters'][0], [(1.0, -.2), (1.2, .2)])

    def test_headerless_dbscan_clear_is_invalid(self):
        clear = self.cluster_marker(namespace='', action=3, stamp=0, frame='')
        self.node.on_clusters(NS(markers=[clear]))
        self.assertFalse(self.node.data['clusters']['valid'])

    def test_clock_regression_clears_observations_and_latches_fault(self):
        self.node.clock_now()
        self.node.data['signal'] = {'value': 'GREEN'}
        self.now = 9
        self.node.clock_now()
        self.assertEqual(self.node.data, {})
        self.assertEqual(self.node.clock_fault, 'CLOCK_REGRESSION_RESTART_REQUIRED')

    def test_selector_status_is_the_only_path_readiness_input(self):
        message = NS(header=self.header(), decision_id=42,
                     route_name='3_s-static-obstacle', source='LOCAL',
                     direction=1, ready=True, reason='PATH_READY',
                     path_fingerprint='abc123')
        self.node.on_selector_status(message)
        self.assertEqual(self.node.data['selector_status'], {
            'stamp': 9.9, 'receipt_stamp': 10.0, 'decision_id': 42,
            'route': '3_s-static-obstacle', 'source': 'LOCAL',
            'direction': 1, 'ready': True, 'reason': 'PATH_READY',
            'path_fingerprint': 'abc123',
        })

    def test_invalid_quaternion_invalidates_odometry(self):
        message = NS(header=self.header(), child_frame_id='base_link',
                     pose=NS(pose=NS(orientation=NS(x=0, y=0, z=0, w=0))))
        self.node.on_odom(message)
        self.assertTrue(math.isnan(self.node.data['odom']['x']))

    def test_localization_state_is_stored_with_receipt_time(self):
        self.node.on_localization_state(NS(data='DEAD_RECKONING'))
        self.assertEqual(self.node.data['localization_state'], {
            'stamp': 10.0, 'state': 'DEAD_RECKONING'
        })

    def route_map(self):
        names = set(ROUTES.values()) | {'1_left', '1_right', '13_left', '13_right'}
        names.update(name for sides in PARKING_ROUTES.values() for pair in sides.values() for name in pair)
        routes = []
        for name in sorted(names):
            poses = [NS(header=self.header(), pose=NS(position=NS(x=x, y=0, z=0),
                         orientation=NS(x=0, y=0, z=0, w=1))) for x in (0, 30)]
            section = Route(name, [(0, 0, 0), (30, 0, 0)]).section
            routes.append(NS(name=name, section=section, direction=1,
                             path=NS(header=self.header(), poses=poses)))
        return NS(header=self.header(), origin_latitude=37.0, origin_longitude=127.0, routes=routes)

    def rddf_match(self, route='1_right', segment=0, fraction=.0, matched=True):
        nearest = NS(source_route_name=route, segment_index=segment,
                     segment_fraction=fraction, distance_m=.1)
        return NS(header=self.header(), pose_stamp=Stamp(9.9), matched=matched,
                  reason='MATCHED' if matched else 'AMBIGUOUS_ROUTE',
                  source_route_name=route if matched else '',
                  segment_index=segment if matched else -1, nearest=nearest,
                  candidates=[nearest])

    def test_runtime_waits_for_match_and_uses_matched_route_and_progress(self):
        self.node.config['map_origin'] = {'latitude': 37.0, 'longitude': 127.0}
        self.node.on_route_map(self.route_map())
        self.assertIsNone(self.node.runtime)
        self.node.on_current_rddf(self.rddf_match('3_s-static-obstacle', fraction=.5))
        self.assertEqual(self.node.runtime.start_route, '3_s-static-obstacle')
        self.assertAlmostEqual(self.node.runtime.initial_s, 15.0)

    def test_new_matched_route_starts_independent_mission_epoch(self):
        self.node.config['map_origin'] = {'latitude': 37.0, 'longitude': 127.0}
        self.node.on_route_map(self.route_map())
        self.node.on_current_rddf(self.rddf_match('3_s-static-obstacle'))
        first_runtime = self.node.runtime
        self.node.on_current_rddf(self.rddf_match('3_s-static-obstacle', fraction=.2))
        self.assertIs(self.node.runtime, first_runtime)
        self.node.on_current_rddf(self.rddf_match('8_dynamic-obstacle'))
        self.assertIsNot(self.node.runtime, first_runtime)
        self.assertEqual(self.node.runtime.start_route, '8_dynamic-obstacle')

    def test_invalid_validated_overlay_retains_map_but_blocks_runtime(self):
        self.node.config.update(map_origin={'latitude': 37.0, 'longitude': 127.0},
                                landmarks_validated=True, landmarks={})
        self.node.on_route_map(self.route_map())
        self.assertEqual(len(self.node.routes), 19)
        self.assertTrue(self.node.configuration_fault.startswith('LANDMARK_CALIBRATION_INVALID:'))
        self.assertIsNone(self.node.runtime)
        self.node.on_current_rddf(self.rddf_match())
        self.assertIsNotNone(self.node.runtime)
        output, steps = [], []
        self.node.runtime.step = lambda *args: steps.append(args)
        self.node.fault_decision = lambda reason: {'valid': False, 'stop_requested': True, 'reason': reason}
        self.node.publish = lambda decision, now: output.append(decision)
        self.node.tick(None)
        self.assertEqual(steps, [])
        self.assertFalse(output[0]['valid'])
        self.assertTrue(output[0]['stop_requested'])

    def test_datum_mismatch_rejects_route_map(self):
        self.node.config['map_origin'] = {'latitude': 37.01, 'longitude': 127.0}
        self.node.on_route_map(self.route_map())
        self.assertIsNone(self.node.runtime)
        self.assertTrue(self.node.map_fault.startswith('ROUTE_MAP_INVALID:'))

    def test_changed_route_map_requires_restart(self):
        self.node.config['map_origin'] = {'latitude': 37.0, 'longitude': 127.0}
        message = self.route_map()
        self.node.on_route_map(message)
        self.assertIsNone(self.node.runtime)
        self.node.on_current_rddf(self.rddf_match())
        self.assertIsNotNone(self.node.runtime)
        message.routes[0].path.poses[1].pose.position.x += .1
        self.node.on_route_map(message)
        self.assertEqual(self.node.map_fault, 'ROUTE_MAP_CHANGED_RESTART_REQUIRED')


if __name__ == '__main__':
    unittest.main()
