import copy
import importlib.util
import math
from pathlib import Path
import sys
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(PACKAGE / 'src'), str(PACKAGE.parent / 'selector' / 'src')]
from selector.core import Candidate, Pose
from stier_state_manager.geometry import Route, scan_to_geometry
from stier_state_manager.runtime import MissionRuntime


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.routes = {'1_right': Route('1_right', [(x, 0, 0) for x in range(11)]),
                       '2': Route('2', [(x, 0, 0) for x in range(10, 21)]),
                       '8_dynamic-obstacle': Route('8_dynamic-obstacle', [(x, 0, 0) for x in range(11)])}
        self.config = {'start_route': '1_right', 'landmarks_validated': True,
                       'vehicle': {'validated': True, 'curb_visibility_validated': True,
                                   'front_m': .5, 'rear_m': .5, 'width_m': 1.0,
                                   'max_speed_mps': 2.0, 'max_yaw_rate_rps': 1.0,
                                   'deceleration_mps2': 1.0, 'reaction_s': .2, 'margin_m': .1},
                       'stop_buffer_m': .05,
                       'landmarks': {'1_right': {'hill_start_s': 1, 'hill_stop_s': 4, 'hill_top_s': 8},
                                     '2': {'stop_line_s': 5, 'intersection_exit_s': 8}}}

    def data(self, now, x=0.0, speed=0.0):
        hits, rays = scan_to_geometry([math.inf]*361, -math.pi, math.pi/180, 0.01, 30,
                                      scanner_pose=(x, 0, 0))
        return {'odom': {'stamp': now, 'frame': 'map', 'child_frame': 'base_link',
                         'x': x, 'y': 0, 'yaw': 0, 'speed': speed, 'yaw_rate': 0,
                         'position_variance': .01, 'yaw_variance': .01},
                'localization': {'stamp': now, 'valid': True},
                'scan': {'stamp': now, 'valid': True, 'hits': hits, 'rays': rays}}

    def candidate(self, runtime, now):
        name, mode, direction = runtime.request or (runtime.tracker.route_name, 'RDDF', 1)
        points = self.routes[name].points
        return {mode: Candidate(now, now, runtime.decision_id or 1, name, direction, 'map', 'map', now,
                                 [Pose('map', (x, y, 0), (0, 0, math.sin(yaw/2), math.cos(yaw/2)))
                                  for x, y, yaw in points])}

    def run_step(self, runtime, now, x=0, speed=0):
        return runtime.step(now, self.data(now, x, speed), self.candidate(runtime, now))

    def test_unvalidated_vehicle_stops(self):
        self.config['vehicle']['validated'] = False
        runtime = MissionRuntime(self.routes, self.config)
        result = self.run_step(runtime, 1)
        self.assertFalse(result['valid'])
        self.assertTrue(result['stop_requested'])

    def test_calibration_session_never_drives(self):
        self.config['calibration_mode'] = True
        result = self.run_step(MissionRuntime(self.routes, self.config), 1)
        self.assertFalse(result['valid'])

    def test_hill_hold_and_coherent_successor_epoch(self):
        runtime = MissionRuntime(self.routes, self.config)
        initial = self.run_step(runtime, 1)
        self.assertFalse(initial['safety']['stop'], initial)
        self.assertFalse(initial['stop_requested'])
        for i in range(32):
            result = self.run_step(runtime, 2 + i/10, x=4)
        self.assertIn('hill', result['completed_missions'])
        self.run_step(runtime, 6, x=8)
        result = self.run_step(runtime, 7, x=10)
        self.assertEqual((result['route'], result['decision_id'], result['section']), ('2', 2, 2))
        self.assertEqual(runtime.request, ('2', 'RDDF', 1))
        self.assertEqual(result['tracking']['route'], result['route'])
        self.assertTrue(result['stop_requested'])
        self.assertFalse(result['valid'])

    def test_raw_lidar_collision_still_stops_on_dynamic_section(self):
        self.config['start_route'] = '8_dynamic-obstacle'
        runtime = MissionRuntime(self.routes, self.config)
        data = self.data(1)
        data['scan']['hits'].append((.7, 0))
        result = runtime.step(1, data, self.candidate(runtime, 1))
        self.assertEqual(result['mission'], 'DYNAMIC_OBSTACLE')
        self.assertTrue(result['safety']['stop'])
        self.assertTrue(result['stop_requested'])

    def cluster_data(self, now, points):
        data = self.data(now)
        data['clusters'] = {'stamp': now, 'receipt_stamp': now, 'frame': 'map',
                            'valid': True, 'reason': 'OK', 'clusters': [points]}
        return data

    def test_dynamic_route_cluster_on_rddf_requests_estop(self):
        self.config['start_route'] = '8_dynamic-obstacle'
        self.config['dynamic_obstacle'] = {'route_token': 'dynamic', 'input_timeout_s': .5,
                                           'lookahead_m': 10.0, 'path_margin_m': .25}
        runtime = MissionRuntime(self.routes, self.config)
        data = self.cluster_data(1, [(2.0, -.2), (2.2, .2)])
        result = runtime.step(1, data, self.candidate(runtime, 1))
        self.assertTrue(result['emergency_stop_requested'])
        self.assertTrue(result['stop_requested'])
        self.assertEqual(result['reason'], 'DYNAMIC_OBSTACLE_ON_RDDF')
        self.assertEqual(result['phase'], 'EMERGENCY_STOP')

        clear = self.cluster_data(1.1, [])
        clear['clusters']['clusters'] = []
        released = runtime.step(1.1, clear, self.candidate(runtime, 1.1))
        self.assertFalse(released['emergency_stop_requested'])
        self.assertFalse(released['stop_requested'], released)

    def test_dynamic_route_cluster_off_rddf_does_not_request_estop(self):
        self.config['start_route'] = '8_dynamic-obstacle'
        runtime = MissionRuntime(self.routes, self.config)
        data = self.cluster_data(1, [(2.0, 2.0), (2.2, 2.1)])
        result = runtime.step(1, data, self.candidate(runtime, 1))
        self.assertFalse(result['emergency_stop_requested'])
        self.assertFalse(result['stop_requested'], result)
        self.assertEqual(result['dynamic_obstacle']['reason'], 'DYNAMIC_OBSTACLE_CLEAR')

    def test_cluster_on_non_dynamic_route_never_requests_estop(self):
        runtime = MissionRuntime(self.routes, self.config)
        data = self.cluster_data(1, [(2.0, 0.0)])
        result = runtime.step(1, data, self.candidate(runtime, 1))
        self.assertFalse(result['emergency_stop_requested'])
        self.assertFalse(result['dynamic_obstacle']['required'])

    def test_stale_dynamic_clusters_do_not_assert_estop(self):
        self.config['start_route'] = '8_dynamic-obstacle'
        runtime = MissionRuntime(self.routes, self.config)
        data = self.cluster_data(1, [(2.0, 0.0)])
        data['clusters']['stamp'] = .1
        result = runtime.step(1, data, self.candidate(runtime, 1))
        self.assertFalse(result['emergency_stop_requested'])
        self.assertFalse(result['dynamic_obstacle']['valid'])
        self.assertTrue(result['stop_requested'])
        self.assertEqual(result['phase'], 'WAIT_DYNAMIC_OBSERVATION')

    def test_lidar_dropout_then_recovery_resets_continuous_hold(self):
        runtime = MissionRuntime(self.routes, self.config)
        self.run_step(runtime, 1)
        for i in range(20):
            self.run_step(runtime, 2+i/10, 4)
        data = self.data(4, 4)
        data['scan']['stamp'] = 1
        result = runtime.step(4, data, self.candidate(runtime, 4))
        self.assertFalse(result['valid'])
        for i in range(20):
            result = self.run_step(runtime, 4.1+i/10, 4)
        self.assertNotIn('hill', result['completed_missions'])

    def test_wrong_frame_and_clock_regression_stop(self):
        runtime = MissionRuntime(self.routes, self.config)
        self.run_step(runtime, 2)
        data = self.data(2.1)
        data['odom']['frame'] = 'odom'
        self.assertFalse(runtime.step(2.1, data, self.candidate(runtime, 2.1))['valid'])
        self.assertFalse(self.run_step(runtime, 1)['valid'])
        self.assertFalse(self.run_step(runtime, 3)['valid'])

    def test_stale_previous_path_cannot_match_new_decision(self):
        runtime = MissionRuntime(self.routes, self.config)
        old = self.candidate(runtime, 1)
        runtime._set_request('1_right', 'LOCAL', 1)
        result = runtime.step(1, self.data(1), old)
        self.assertTrue(result['stop_requested'])

    def test_minimum_command_resolution_must_fit_stop_tolerance(self):
        self.config['vehicle']['reaction_s'] = 2.0
        runtime = MissionRuntime(self.routes, self.config)
        self.assertFalse(self.run_step(runtime, 1)['valid'])

    def test_motion_beyond_measured_scan_bounds_stops(self):
        runtime = MissionRuntime(self.routes, self.config)
        result = self.run_step(runtime, 1, speed=3.0)
        self.assertFalse(result['valid'])
        self.assertEqual(result['safety']['reason'], 'MOTION_OUTSIDE_SCAN_CALIBRATION')

    def test_scan_motion_expands_collision_envelope(self):
        runtime = MissionRuntime(self.routes, self.config)
        data = self.data(1)
        data['scan']['hits'] = [(1, 1.0)]
        before = runtime._corridor([(0, 0, 0), (1, 0, 0)], data)
        self.assertEqual(before['status'], 'CLEAR')
        data['scan'].update(duration=.1, range_max=30)
        after = runtime._corridor([(0, 0, 0), (1, 0, 0)], data)
        self.assertEqual(after['status'], 'BLOCKED')

    def test_real_catalogue_has_nineteen_routes_and_twelve_left_fork(self):
        source = PACKAGE.parent / 'localization' / 'scripts' / 'rddf_route_provider.py'
        spec = importlib.util.spec_from_file_location('route_provider', source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        origin, routes = module.load_catalogue(PACKAGE.parent / 'localization' / 'rddf')
        catalogue = {name: Route(name, points, direction) for name, direction, points in routes}
        self.assertEqual(len(catalogue), 19)
        self.assertEqual(origin['lat'], 37.288731)
        self.assertLess(math.hypot(*(a-b for a,b in zip(catalogue['12'].pose_at(13.258)[:2], catalogue['13_left'].start[:2]))), .01)

    def traffic_runtime(self, name='2'):
        self.routes[name] = Route(name, [(x, 0, 0) for x in range(11)])
        self.config['start_route'] = name
        self.config['landmarks'][name] = {'stop_line_s': 5, 'intersection_exit_s': 8}
        return MissionRuntime(self.routes, self.config)

    def planned_prefix(self, runtime, now, signal):
        points = runtime.rddf_points(now, signal)
        return {'RDDF': Candidate(now, now, runtime.decision_id or 1,
                                 runtime.tracker.route_name, 1, 'map', 'map', now,
                                 [Pose('map', (x,y,0), (0,0,math.sin(yaw/2),math.cos(yaw/2)))
                                  for x,y,yaw in points])}

    def test_red_approach_path_stops_before_front_bumper_and_buffer(self):
        runtime = self.traffic_runtime()
        signal = {'stamp': 1, 'route': '2', 'value': 'RED'}
        candidate = self.planned_prefix(runtime, 1, signal)
        self.assertAlmostEqual(candidate['RDDF'].poses[-1].position[0], 4.45)
        result = runtime.step(1, dict(self.data(1), signal=signal), candidate)
        self.assertFalse(result['stop_requested'], result)
        self.assertTrue(result['virtual_stop']['active'])
        self.assertEqual(result['virtual_stop']['pose'], (5., 0., 0.))

    def test_green_red_green_replans_and_rejects_old_unrestricted_geometry(self):
        runtime = self.traffic_runtime()
        green = {'stamp': 1, 'route': '2', 'value': 'GREEN'}
        path = self.planned_prefix(runtime, 1, green)
        result = runtime.step(1, dict(self.data(1), signal=green), path)
        self.assertFalse(result['virtual_stop']['active'])
        self.assertFalse(result['stop_requested'], result)
        red = dict(green, stamp=1.1, value='RED')
        # A planner ignoring the new wall is vetoed even before reaching it.
        result = runtime.step(1.1, dict(self.data(1.1), signal=red), self.candidate(runtime, 1.1))
        self.assertEqual(result['safety']['reason'], 'PATH_CROSSES_VIRTUAL_STOP')
        self.assertTrue(result['stop_requested'])
        result = runtime.step(1.2, dict(self.data(1.2), signal=red), self.planned_prefix(runtime, 1.2, red))
        self.assertFalse(result['stop_requested'], result)
        green['stamp'] = 1.3
        result = runtime.step(1.3, dict(self.data(1.3), signal=green), self.planned_prefix(runtime, 1.3, green))
        self.assertFalse(result['stop_requested'], result)
        self.assertEqual(runtime.rddf_points(1.3, green)[-1][0], 10)

    def test_unknown_stale_wrong_route_future_or_wrong_turn_keeps_wall(self):
        for name in ('2', '4', '7'):
            runtime = self.traffic_runtime(name)
            required = 'LEFT_ARROW' if name == '7' else 'GREEN'
            for signal in ({}, {'stamp': 1, 'route': name, 'value': 'YELLOW'},
                           {'stamp': 1, 'route': name, 'value': 'RED'},
                           {'stamp': .1, 'route': name, 'value': required},
                           {'stamp': 2, 'route': name, 'value': required},
                           {'stamp': 1, 'route': 'wrong', 'value': required},
                           {'stamp': 1, 'route': name, 'value': 'GREEN' if name == '7' else 'LEFT_ARROW'}):
                with self.subTest(name=name, signal=signal):
                    self.assertAlmostEqual(runtime.rddf_points(1, signal)[-1][0], 4.45)
            self.assertEqual(runtime.rddf_points(1, {'stamp': 1, 'route': name, 'value': required})[-1][0], 10)

    def test_traffic_without_marker_produces_no_path_even_on_green(self):
        runtime = self.traffic_runtime()
        self.config['landmarks']['2']['stop_line_s'] = None
        self.assertEqual(runtime.rddf_points(1, {'stamp': 1, 'route': '2', 'value': 'GREEN'}), [])

    def test_red_after_authorized_entry_does_not_recreate_wall(self):
        runtime = self.traffic_runtime()
        for i in range(46):
            now = 1+i/10
            green = {'stamp': now, 'route': '2', 'value': 'GREEN'}
            result = runtime.step(now, dict(self.data(now, x=i/10), signal=green), self.planned_prefix(runtime, now, green))
        self.assertEqual(result['phase'], 'CROSSING')
        red = {'stamp': 5.6, 'route': '2', 'value': 'RED'}
        result = runtime.step(5.6, dict(self.data(5.6, x=4.6), signal=red), self.planned_prefix(runtime, 5.6, red))
        self.assertFalse(result['virtual_stop']['active'])
        self.assertFalse(result['stop_requested'], result)

    def test_midpoint_is_on_curved_rddf_and_not_chord_midpoint(self):
        route = Route('1_right', [(0,0,0), (4,0,math.pi/2), (4,6,math.pi/2)])
        self.routes['1_right'] = route
        self.config['landmarks']['1_right'] = {'hill_zone_start_s': 2, 'hill_zone_end_s': 8}
        from stier_state_manager.mission import hill_target
        self.assertEqual(route.pose_at(hill_target(self.config['landmarks']['1_right'])[1])[:2], (4,1))


if __name__ == '__main__':
    unittest.main()
