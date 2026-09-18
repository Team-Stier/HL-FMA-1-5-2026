import copy
import importlib.util
import math
from pathlib import Path
import sys
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(PACKAGE / 'src'), str(PACKAGE.parent / 'selector' / 'src')]
from stier_state_manager.geometry import Route, project
from stier_state_manager.runtime import MissionRuntime


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.routes = {'1_right': Route('1_right', [(x, 0, 0) for x in range(11)]),
                       '2': Route('2', [(x, 0, 0) for x in range(10, 21)]),
                       '8_dynamic-obstacle': Route('8_dynamic-obstacle', [(x, 0, 0) for x in range(11)])}
        self.config = {'start_route': '1_right', 'landmarks_validated': True,
                       'input_timeout_s': 2.0,
                       'vehicle': {'validated': True, 'curb_visibility_validated': True,
                                   'front_m': .5, 'rear_m': .5, 'width_m': 1.0,
                                   'max_speed_mps': 2.0, 'max_yaw_rate_rps': 1.0,
                                   'deceleration_mps2': 1.0, 'reaction_s': .2, 'margin_m': .1},
                       'stop_buffer_m': .05,
                       'dynamic_obstacle': {'route_token': 'dynamic', 'input_timeout_s': 2.0,
                                            'lookahead_m': 10.0, 'corridor_half_width_m': .65},
                       'landmarks': {'1_right': {'hill_start_s': 1, 'hill_stop_s': 4, 'hill_top_s': 8},
                                     '2': {'stop_line_s': 5}}}

    def match(self, runtime, now, x=0.0, y=0.0):
        route = runtime.source_routes[runtime.active_route_name]
        matched = project(route, x, y)
        segment = matched['segment']
        length = route.s[segment + 1] - route.s[segment]
        fraction = ((matched['s'] - route.s[segment]) / length
                    if length > 1e-9 else 0.0)
        candidate = {'route': route.name, 'segment_index': segment,
                     'segment_fraction': fraction,
                     'distance_m': matched['distance']}
        return {'stamp': now, 'received': now, 'pose_stamp': now,
                'frame': 'map', 'matched': True, **candidate,
                'candidates': [candidate]}

    def data(self, now, x=0.0, speed=0.0, runtime=None):
        runtime = runtime or MissionRuntime(self.routes, self.config)
        return {'odom': {'stamp': now, 'frame': 'map', 'child_frame': 'base_link',
                         'x': x, 'y': 0, 'yaw': 0, 'speed': speed, 'yaw_rate': 0,
                         'position_variance': .01, 'yaw_variance': .01},
                'localization': {'stamp': now, 'valid': True},
                'localization_state': {'stamp': now, 'state': 'TRACKING'},
                'rddf_match': self.match(runtime, now, x)}

    def candidate(self, runtime, now):
        name, mode, direction = runtime.request or (runtime.active_route_name, 'RDDF', 1)
        return {'stamp': now, 'receipt_stamp': now,
                'decision_id': runtime.decision_id or 1,
                'route': name, 'source': mode, 'direction': direction,
                'ready': True, 'reason': 'PATH_READY',
                'path_fingerprint': 'test-path'}

    def run_step(self, runtime, now, x=0, speed=0):
        return runtime.step(now, self.data(now, x, speed, runtime), self.candidate(runtime, now))

    def test_rddf_preview_crosses_seam_but_respects_next_stop_line(self):
        runtime = MissionRuntime(self.routes, self.config)
        runtime.progress_s = 9.0
        points = runtime.rddf_points(10.0, None)
        self.assertGreater(points[-1][0], 10.0)
        self.assertLessEqual(points[-1][0], 14.45 + 1e-6)

    def test_rounded_cusp_does_not_add_reverse_micrometre_segment(self):
        for yaw in (0.0, 0.69, 2.35, -2.8):
            with self.subTest(yaw=yaw):
                cosine, sine = math.cos(yaw), math.sin(yaw)
                route = Route('10_parallel-right-in',
                              [(x*cosine, x*sine, yaw)
                               for x in (0.0, 1.0, 2.0, 1.0, 0.0)])
                runtime = MissionRuntime(
                    {route.name: route}, dict(self.config, start_route=route.name))
                runtime.rddf_bounds = (route.name, 0.0, 2.0+0.5e-6)

                points = runtime.rddf_points(10.0, {})

                self.assertGreater(len(points), 2)
                self.assertTrue(all(
                    math.hypot(b[0]-a[0], b[1]-a[1]) > 1e-6
                    for a, b in zip(points, points[1:])))

    def test_rddf_keeps_real_reversal_and_removes_only_duplicate_positions(self):
        route = Route('10_parallel-right-in',
                      [(x, 0.0, 0.0) for x in (0.0, 1.0, 1.0, 2.0, 1.9, 1.0)])
        runtime = MissionRuntime(
            {route.name: route}, dict(self.config, start_route=route.name))

        points = runtime.rddf_points(10.0, {})

        self.assertEqual(points[0][:2], (0.0, 0.0))
        self.assertEqual(points[-1][:2], (1.0, 0.0))
        self.assertTrue(any(b[0] < a[0] for a, b in zip(points, points[1:])))
        self.assertTrue(all(
            math.hypot(b[0]-a[0], b[1]-a[1]) > 1e-6
            for a, b in zip(points, points[1:])))

    def test_prevalidated_next_rddf_handoff_keeps_drive_and_traffic_stop(self):
        runtime = MissionRuntime(self.routes, self.config)
        self.run_step(runtime, 10)
        previous = self.candidate(runtime, 10)
        runtime.active_source_routes = ('1_right', '2')
        expected = (runtime.decision_id + 1, '2', 'RDDF', 1)
        prepared = dict(previous, decision_id=expected[0], route='2')
        runtime.activate_route('2')
        data = self.data(10.1, x=10, runtime=runtime)
        data['prefetch_statuses'] = {expected: prepared}
        result = runtime.step(10.1, data, previous)
        self.assertFalse(result['stop_requested'], result)
        self.assertEqual(result['decision_id'], expected[0])
        # A prepared path never authorizes crossing a red signal.
        data = self.data(10.2, x=14.5, runtime=runtime)
        data['prefetch_statuses'] = {expected: prepared}
        stopped = runtime.step(10.2, data, previous)
        self.assertTrue(stopped['stop_requested'])
        self.assertEqual(stopped['phase'], 'WAIT_SIGNAL')

    def test_missing_stale_or_wrong_prefetch_cannot_skip_path_wait(self):
        for failure in ('missing', 'stale', 'wrong', 'rejected'):
            runtime = MissionRuntime(self.routes, self.config)
            self.run_step(runtime, 10)
            previous = self.candidate(runtime, 10)
            expected = (runtime.decision_id + 1, '2', 'RDDF', 1)
            runtime.active_source_routes = ('1_right', '2')
            runtime.activate_route('2')
            prepared = dict(previous, route='2', decision_id=expected[0])
            if failure == 'stale': prepared['stamp'] = 8.0
            if failure == 'wrong': prepared['decision_id'] += 1
            if failure == 'rejected': prepared['ready'] = False
            data = self.data(10.1, x=10, runtime=runtime)
            data['prefetch_statuses'] = {} if failure == 'missing' else {expected: prepared}
            result = runtime.step(10.1, data, previous)
            self.assertTrue(result['stop_requested'], (failure, result))

    def test_rddf_preview_does_not_append_parking(self):
        routes = dict(self.routes)
        routes['4'] = Route('4', [(0, 0, 0), (10, 0, 0)])
        routes['5_T-left-in'] = Route('5_T-left-in', [(10, 0, 0), (20, 0, 0)], -1)
        config = dict(self.config, start_route='4')
        config['landmarks'] = dict(self.config['landmarks'], **{'4': {'stop_line_s': 5}})
        runtime = MissionRuntime(routes, config)
        runtime.engine.states['4'] = {'authorized': True}
        runtime.progress_s = 9.0
        self.assertLessEqual(runtime.rddf_points(10.0, None)[-1][0], 10.0)

    def test_localization_match_is_progress_authority(self):
        runtime = MissionRuntime(self.routes, self.config)
        data = self.data(1, x=50, runtime=runtime)
        # Deliberately impossible pose/yaw for the current RDDF: Localization's
        # accepted projection, not State Manager reprojection, owns progress.
        data['odom'].update(x=50, y=50, yaw=math.pi)
        data['rddf_match'] = self.match(runtime, 1, x=3)
        result = runtime.step(1, data, self.candidate(runtime, 1))
        self.assertTrue(result['tracking']['healthy'], result)
        self.assertAlmostEqual(result['distance_m'], 3.0)

    def test_localization_route_change_preserves_mission_state(self):
        runtime = MissionRuntime(self.routes, self.config)
        engine = runtime.engine
        runtime.engine.completed_missions['hill'] = 1.0
        runtime.request = ('1_right', 'RDDF', 1)

        self.assertTrue(runtime.activate_route('2', 3.0))

        self.assertIs(runtime.engine, engine)
        self.assertEqual(runtime.engine.completed_missions['hill'], 1.0)
        self.assertEqual(runtime.active_route_name, '2')
        self.assertAlmostEqual(runtime.progress_s, 3.0)
        self.assertIsNone(runtime.request)

    def test_global_validation_flags_do_not_stop_a_configured_route(self):
        self.config['vehicle']['validated'] = False
        self.config['landmarks_validated'] = False
        runtime = MissionRuntime(self.routes, self.config)
        result = self.run_step(runtime, 1)
        self.assertTrue(result['valid'], result)
        self.assertFalse(result['stop_requested'], result)
        self.assertEqual(result['safety']['reason'], 'PATH_ACCEPTED')

    def test_current_route_landmarks_are_still_required(self):
        self.config['landmarks_validated'] = False
        self.config['landmarks']['1_right'] = {}
        result = self.run_step(MissionRuntime(self.routes, self.config), 1)
        self.assertFalse(result['valid'])
        self.assertTrue(result['stop_requested'])
        self.assertEqual(result['reason'], 'CALIBRATION_REQUIRED:hill_start_s')

    def test_supervisor_valid_is_not_rejected_by_position_covariance(self):
        runtime = MissionRuntime(self.routes, self.config)
        data = self.data(1)
        data['localization_state']['state'] = 'DEAD_RECKONING'
        data['odom']['position_variance'] = 20.0
        healthy, reason = runtime._health(data, 1)
        self.assertTrue(healthy, reason)

    def test_tracking_does_not_recheck_position_covariance(self):
        runtime = MissionRuntime(self.routes, self.config)
        data = self.data(1)
        data.pop('localization_state')
        data['odom']['position_variance'] = 20.0
        data['odom']['yaw_variance'] = 20.0
        self.assertEqual(runtime._health(data, 1), (True, 'OK'))

    def test_supervisor_valid_is_not_rejected_by_yaw_covariance(self):
        runtime = MissionRuntime(self.routes, self.config)
        data = self.data(1)
        data['localization_state']['state'] = 'DEAD_RECKONING'
        data['odom']['position_variance'] = 20.0
        data['odom']['yaw_variance'] = 0.1
        self.assertEqual(runtime._health(data, 1), (True, 'OK'))

    def test_calibration_session_never_drives(self):
        self.config['calibration_mode'] = True
        result = self.run_step(MissionRuntime(self.routes, self.config), 1)
        self.assertFalse(result['valid'])

    def test_hill_completion_does_not_override_localization_route(self):
        runtime = MissionRuntime(self.routes, self.config)
        initial = self.run_step(runtime, 1)
        self.assertFalse(initial['safety']['stop'], initial)
        self.assertFalse(initial['stop_requested'])
        for i in range(32):
            result = self.run_step(runtime, 2 + i/10, x=4)
        self.assertIn('hill', result['completed_missions'])
        self.run_step(runtime, 6, x=8)
        result = self.run_step(runtime, 7, x=10)
        self.assertEqual((result['route'], result['section']), ('1_right', 1))
        self.assertEqual(result['next_route'], '2')
        self.assertEqual(runtime.request, ('1_right', 'RDDF', 1))
        self.assertEqual(result['tracking']['route'], result['route'])
        self.assertFalse(result['stop_requested'])
        self.assertTrue(result['valid'])

    def test_raw_lidar_is_not_a_state_manager_input(self):
        runtime = MissionRuntime(self.routes, self.config)
        data = self.data(1)
        self.assertNotIn('scan', data)
        result = runtime.step(1, data, self.candidate(runtime, 1))
        self.assertEqual(result['mission'], 'HILL_STOP')
        self.assertFalse(result['safety']['stop'], result)
        self.assertFalse(result['stop_requested'], result)

    def test_unvalidated_vehicle_does_not_block_dynamic_path_check(self):
        self.config['start_route'] = '8_dynamic-obstacle'
        self.config['vehicle']['validated'] = False
        runtime = MissionRuntime(self.routes, self.config)
        result = runtime.step(1, self.cluster_data(1, []), self.candidate(runtime, 1))
        self.assertTrue(result['valid'])
        self.assertFalse(result['stop_requested'], result)
        self.assertEqual(result['dynamic_obstacle']['reason'], 'DYNAMIC_OBSTACLE_CLEAR')

    def cluster_data(self, now, points):
        data = self.data(now)
        data['clusters'] = {'stamp': now, 'receipt_stamp': now, 'frame': 'map',
                            'valid': True, 'reason': 'OK', 'clusters': [points]}
        return data

    def test_dynamic_route_cluster_on_rddf_requests_estop(self):
        self.config['start_route'] = '8_dynamic-obstacle'
        self.config['dynamic_obstacle'] = {'route_token': 'dynamic', 'input_timeout_s': .5,
                                           'lookahead_m': 10.0, 'corridor_half_width_m': .65}
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

    def test_dynamic_corridor_uses_configured_half_width_only(self):
        self.config['start_route'] = '8_dynamic-obstacle'
        self.config['vehicle']['validated'] = False
        inside = MissionRuntime(self.routes, self.config)
        result = inside.step(1, self.cluster_data(1, [(2.0, .64)]), self.candidate(inside, 1))
        self.assertTrue(result['emergency_stop_requested'])
        outside = MissionRuntime(self.routes, self.config)
        result = outside.step(1, self.cluster_data(1, [(2.0, .66)]), self.candidate(outside, 1))
        self.assertFalse(result['emergency_stop_requested'])
        self.assertFalse(result['stop_requested'], result)

    def test_cluster_on_non_dynamic_route_never_requests_estop(self):
        runtime = MissionRuntime(self.routes, self.config)
        data = self.cluster_data(1, [(2.0, 0.0)])
        result = runtime.step(1, data, self.candidate(runtime, 1))
        self.assertFalse(result['emergency_stop_requested'])
        self.assertFalse(result['dynamic_obstacle']['required'])

    def test_stale_dynamic_clusters_do_not_assert_estop(self):
        self.config['start_route'] = '8_dynamic-obstacle'
        runtime = MissionRuntime(self.routes, self.config)
        data = self.cluster_data(3, [(2.0, 0.0)])
        data['clusters']['stamp'] = .1
        result = runtime.step(3, data, self.candidate(runtime, 3))
        self.assertFalse(result['emergency_stop_requested'])
        self.assertFalse(result['dynamic_obstacle']['valid'])
        self.assertTrue(result['stop_requested'])
        self.assertEqual(result['phase'], 'WAIT_DYNAMIC_OBSERVATION')

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

    def test_stop_precision_check_is_not_a_global_motion_gate(self):
        self.config['vehicle']['reaction_s'] = 2.0
        runtime = MissionRuntime(self.routes, self.config)
        self.assertTrue(runtime.vehicle_ok)
        self.assertFalse(runtime.vehicle_dynamics_ok)
        result = self.run_step(runtime, 1)
        self.assertTrue(result['valid'], result)
        self.assertFalse(result['stop_requested'], result)

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

    def test_finish_route_is_extended_three_metres_and_stops_at_runout_end(self):
        self.routes['13_left'] = Route('13_left', [(x, 0, 0) for x in range(11)])
        self.config.update(start_route='13_left', finish_fallback_branch='left')
        self.config.setdefault('rules', {})['finish_runout_m'] = 3.0
        runtime = MissionRuntime(self.routes, self.config)
        self.assertAlmostEqual(runtime.active_route.length, 13.0)
        self.assertEqual(runtime.active_route.end, (13.0, 0.0, 0.0))
        self.assertEqual(runtime.rddf_points(1, {})[-1], (13.0, 0.0, 0.0))
        runtime.progress_s = 9.0
        before = runtime.step(1, self.data(1, x=9, runtime=runtime), self.candidate(runtime, 1))
        self.assertFalse(before['finished'])
        almost = runtime.step(1.5, self.data(1.5, x=12.7, runtime=runtime), self.candidate(runtime, 1.5))
        self.assertFalse(almost['finished'])
        finished = runtime.step(1.6, self.data(1.6, x=12.8, runtime=runtime), self.candidate(runtime, 1.6))
        self.assertTrue(finished['finished'])
        self.assertEqual(finished['reason'], 'COURSE_COMPLETE')

    def traffic_runtime(self, name='2'):
        self.routes[name] = Route(name, [(x, 0, 0) for x in range(11)])
        self.config['start_route'] = name
        self.config['landmarks'][name] = {'stop_line_s': 5}
        return MissionRuntime(self.routes, self.config)

    def planned_prefix(self, runtime, now, signal):
        return self.candidate(runtime, now)

    def test_red_approach_keeps_full_steering_path(self):
        runtime = self.traffic_runtime()
        signal = {'stamp': 1, 'route': '2', 'value': 'RED'}
        candidate = self.planned_prefix(runtime, 1, signal)
        self.assertAlmostEqual(runtime.rddf_points(1, signal)[-1][0], 10.0)
        result = runtime.step(1, dict(self.data(1), signal=signal), candidate)
        self.assertFalse(result['stop_requested'], result)
        self.assertTrue(result['virtual_stop']['active'])
        self.assertEqual(result['virtual_stop']['pose'], (5., 0., 0.))

    def test_green_red_green_updates_virtual_stop_without_global_path_veto(self):
        runtime = self.traffic_runtime()
        green = {'stamp': 1, 'route': '2', 'value': 'GREEN'}
        path = self.planned_prefix(runtime, 1, green)
        result = runtime.step(1, dict(self.data(1), signal=green), path)
        self.assertFalse(result['virtual_stop']['active'])
        self.assertFalse(result['stop_requested'], result)
        red = dict(green, stamp=1.1, value='RED')
        # The removed vehicle gate no longer vetoes the complete path.  The
        # traffic mission still publishes the stop line and remaining distance.
        result = runtime.step(1.1, dict(self.data(1.1), signal=red), self.candidate(runtime, 1.1))
        self.assertEqual(result['safety']['reason'], 'PATH_ACCEPTED')
        self.assertFalse(result['stop_requested'], result)
        self.assertAlmostEqual(result['remaining_stop_m'], 4.5)
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
                    self.assertAlmostEqual(runtime.rddf_points(1, signal)[-1][0], 10.0)
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

    def test_nonpermitted_signal_forces_departure_after_twenty_stopped_seconds(self):
        for signal_value in ('RED', 'YELLOW', 'UNKNOWN'):
            with self.subTest(signal_value=signal_value):
                runtime = self.traffic_runtime()
                for index in range(201):
                    now = 1.0 + index / 10.0
                    signal = {'stamp': now, 'route': '2', 'value': signal_value}
                    data = dict(self.data(now, x=4.5, speed=0.0, runtime=runtime),
                                signal=signal)
                    result = runtime.step(now, data, self.candidate(runtime, now))
                    if index < 200:
                        self.assertTrue(result['stop_requested'], result)
                self.assertFalse(result['stop_requested'], result)
                self.assertEqual(result['phase'], 'CROSSING')
                self.assertEqual(result['reason'], 'TRAFFIC_FORCE_DEPARTURE_AFTER_TIMEOUT')
                self.assertFalse(result['virtual_stop']['active'])

    def test_force_departure_timer_survives_speed_noise_after_first_stop(self):
        runtime = self.traffic_runtime()
        result = None
        for index in range(201):
            now = 1.0 + index / 10.0
            speed = 0.1 if index == 50 else 0.0
            signal = {'stamp': now, 'route': '2', 'value': 'UNKNOWN'}
            data = dict(self.data(now, x=4.5, speed=speed, runtime=runtime), signal=signal)
            result = runtime.step(now, data, self.candidate(runtime, now))
        self.assertFalse(result['stop_requested'], result)
        self.assertEqual(result['traffic_wait_elapsed_s'], 20.0)

    def test_left_turn_green_uses_same_force_departure_timeout(self):
        runtime = self.traffic_runtime('7')
        for index in range(201):
            now = 1.0 + index / 10.0
            signal = {'stamp': now, 'route': '7', 'value': 'GREEN'}
            data = dict(self.data(now, x=4.5, speed=0.0, runtime=runtime), signal=signal)
            result = runtime.step(now, data, self.candidate(runtime, now))
        self.assertFalse(result['stop_requested'], result)
        self.assertEqual(result['reason'], 'TRAFFIC_FORCE_DEPARTURE_AFTER_TIMEOUT')

    def test_midpoint_is_on_curved_rddf_and_not_chord_midpoint(self):
        route = Route('1_right', [(0,0,0), (4,0,math.pi/2), (4,6,math.pi/2)])
        self.routes['1_right'] = route
        self.config['landmarks']['1_right'] = {'hill_zone_start_s': 2, 'hill_zone_end_s': 8}
        from stier_state_manager.mission import hill_target
        self.assertEqual(route.pose_at(hill_target(self.config['landmarks']['1_right'])[1])[:2], (4,1))


if __name__ == '__main__':
    unittest.main()
