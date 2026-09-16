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
from stier_state_manager.geometry import Route, transform_scan_to_geometry
from stier_state_manager.calibration import validate_landmarks
from stier_state_manager.mission import PARKING_ROUTES, ROUTES


class Stamp:
    def __init__(self, value):
        self.value = value

    def to_sec(self):
        return self.value


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        fake_ros = NS(Time=NS(now=lambda: Stamp(self.now)), Duration=lambda value: value,
                      logwarn_throttle=lambda *args: None, logerr_throttle=lambda *args: None)
        namespace = {'math': math, 'copy': copy, 'rospy': fake_ros, 'secrets': secrets,
                     'Route': Route, 'ROUTES': ROUTES, 'PARKING_ROUTES': PARKING_ROUTES,
                     'validate_landmarks': validate_landmarks,
                     'MissionRuntime': lambda routes, config: NS(decision_id=0),
                     'transform_scan_to_geometry': transform_scan_to_geometry,
                     'tf2_ros': NS(LookupException=LookupError, ConnectivityException=ConnectionError,
                                   ExtrapolationException=TimeoutError)}
        source = Path(__file__).resolve().parents[1] / 'scripts' / 'state_manager_node'
        parsed = ast.parse(source.read_text())
        definitions = ast.Module(body=[n for n in parsed.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))],
                                 type_ignores=[])
        exec(compile(definitions, str(source), 'exec'), namespace)
        self.node = namespace['StateManagerNode'].__new__(namespace['StateManagerNode'])
        self.node.lock = threading.RLock()
        self.node.data, self.node.candidates = {}, {}
        self.node.last_clock, self.node.clock_fault = None, ''
        self.node.config = {'input_timeout_s': .5, 'max_scan_duration_s': .2}
        self.node.routes, self.node.runtime, self.node.map_fingerprint = {}, None, None
        self.node.configuration_fault, self.node.map_fault = '', ''
        transform = NS(transform=NS(translation=NS(x=0, y=0, z=.3), rotation=NS(x=1, y=0, z=0, w=0)))
        self.node.tf_buffer = NS(lookup_transform=lambda *args: transform)

    def header(self, stamp=9.9, frame='map'):
        return NS(stamp=Stamp(stamp), frame_id=frame)

    def scan(self, stamp=9.9):
        return NS(header=self.header(stamp, 'laser'), ranges=[math.inf] * 721,
                  time_increment=.1 / 720, scan_time=.1, angle_min=-math.pi,
                  angle_increment=math.pi / 360, range_min=.05, range_max=20)

    def test_scan_retains_source_time_and_roll_flipped_tf(self):
        self.node.on_scan(self.scan())
        observation = self.node.data['scan']
        self.assertTrue(observation['valid'])
        self.assertEqual(observation['stamp'], 9.9)
        self.assertEqual(observation['rays'].plane_normal, (0, 0, -1))

    def test_expired_scan_cannot_be_refreshed_by_callback(self):
        self.node.on_scan(self.scan(9))
        self.assertFalse(self.node.data['scan']['valid'])
        self.assertEqual(self.node.data['scan']['stamp'], 9)

    def test_future_last_beam_is_rejected(self):
        self.node.on_scan(self.scan(9.95))
        self.assertFalse(self.node.data['scan']['valid'])

    def test_unknown_acquisition_duration_is_rejected(self):
        scan = self.scan()
        scan.time_increment = scan.scan_time = 0
        self.node.on_scan(scan)
        self.assertFalse(self.node.data['scan']['valid'])

    def test_general_green_is_preserved_distinct_from_left_arrow(self):
        self.node.on_signal(NS(header=self.header(), route_name='7', junction_id='left_turn',
                               value='GREEN', confidence=.9))
        self.assertEqual(self.node.data['signal']['value'], 'GREEN')
        self.assertNotEqual(self.node.data['signal']['value'], 'LEFT_ARROW')

    def test_clock_regression_clears_observations_and_latches_fault(self):
        self.node.clock_now()
        self.node.data['signal'] = {'value': 'GREEN'}
        self.node.candidates['PARKING'] = object()
        self.now = 9
        self.node.clock_now()
        self.assertEqual(self.node.data, {})
        self.assertEqual(self.node.candidates, {})
        self.assertEqual(self.node.clock_fault, 'CLOCK_REGRESSION_RESTART_REQUIRED')

    def test_invalid_quaternion_invalidates_odometry(self):
        message = NS(header=self.header(), child_frame_id='base_link',
                     pose=NS(pose=NS(orientation=NS(x=0, y=0, z=0, w=0))))
        self.node.on_odom(message)
        self.assertTrue(math.isnan(self.node.data['odom']['x']))

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

    def test_invalid_validated_overlay_retains_map_but_blocks_runtime(self):
        self.node.config.update(map_origin={'latitude': 37.0, 'longitude': 127.0},
                                landmarks_validated=True, landmarks={})
        self.node.on_route_map(self.route_map())
        self.assertEqual(len(self.node.routes), 19)
        self.assertTrue(self.node.configuration_fault.startswith('LANDMARK_CALIBRATION_INVALID:'))
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
        self.assertIsNotNone(self.node.runtime)
        message.routes[0].path.poses[1].pose.position.x += .1
        self.node.on_route_map(message)
        self.assertEqual(self.node.map_fault, 'ROUTE_MAP_CHANGED_RESTART_REQUIRED')


if __name__ == '__main__':
    unittest.main()
