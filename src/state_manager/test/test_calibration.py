import copy
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from stier_state_manager.calibration import (atomic_update, merge_overlay, record_landmark,
                                            route_catalogue_digest, snap_landmark, validate_landmarks)
from stier_state_manager.geometry import Route
from stier_state_manager.mission import REQUIRED_LANDMARKS


def route(name, start=0, length=20):
    return Route(name, [(start + i, 0., 0.) for i in range(length + 1)])


def catalogue():
    names = ('1_left', '1_right', '2', '3_s-static-obstacle', '4',
             '5_T-left-in', '5_T-right-in', '6-T-left-out', '6_T-right-out',
             '7', '8_dynamic-obstacle', '9', '10_parallel-left-in',
             '10_parallel-right-in', '11-parallel-left-out', '11_parallel-right-out',
             '12', '13_left', '13_right')
    return {name: route(name, 13.258 if name == '13_left' else 20 if name == '13_right' else 0)
            for name in names}


def measured(routes):
    config = {'vehicle': {'unchanged': 'value'}, 'landmarks': {}, 'landmarks_validated': False}
    for name, item in routes.items():
        marks = {key: 10. for key in REQUIRED_LANDMARKS.get(item.section, ())}
        if item.section == 1:
            marks.update(hill_start_s=2., hill_stop_s=5., hill_top_s=9.)
        if item.section in (2, 4, 7):
            marks.update(stop_line_s=5.)
        if item.section == 5:
            marks.update(parking_confirm_s=item.length, parking_yaw_rad=math.pi)
        if marks:
            config['landmarks'][name] = marks
    return config


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.routes = catalogue()
        self.config = measured(self.routes)

    def test_snap_actual_waypoint_with_zero_based_index(self):
        point = snap_landmark(self.routes, '2', 'stop_line_s', 4.2, .3)
        self.assertEqual(point, {'index': 4, 'east_m': 4., 'north_m': 0., 'distance_m': 4.})

    def test_click_rejects_unknown_route_wrong_key_yaw_and_far_points(self):
        for args in [('unknown', 'stop_line_s', 4, 0), ('3_s-static-obstacle', 'stop_line_s', 4, 0),
                     ('5_T-left-in', 'parking_yaw_rad', 4, 0), ('2', 'stop_line_s', 4, 2.01),
                     ('2', 'stop_line_s', math.nan, 0), ('2', 'stop_line_s', 4, math.inf)]:
            with self.subTest(args=args), self.assertRaises(ValueError):
                snap_landmark(self.routes, *args)

    def test_record_preserves_config_and_other_routes_but_revokes_validation(self):
        self.config['landmarks_validated'] = True
        original = copy.deepcopy(self.config)
        point = snap_landmark(self.routes, '2', 'stop_line_s', 4, 0)
        changed = record_landmark(self.config, self.routes, '2', 'stop_line_s', point)
        self.assertEqual(self.config, original)
        self.assertEqual(changed['vehicle'], original['vehicle'])
        self.assertEqual(changed['landmarks']['1_left'], original['landmarks']['1_left'])
        self.assertNotIn('intersection_exit_s', changed['landmarks']['2'])
        self.assertEqual(changed['landmarks']['2']['stop_line_s'], 4.)
        self.assertEqual(changed['landmark_points']['2']['stop_line_s'], point)
        self.assertFalse(changed['landmarks_validated'])

    def test_record_rejects_tampered_metadata(self):
        point = snap_landmark(self.routes, '2', 'stop_line_s', 4, 0)
        point['distance_m'] = 17.
        with self.assertRaises(ValueError):
            record_landmark(self.config, self.routes, '2', 'stop_line_s', point)

    def test_all_measured_routes_validate_without_implicitly_changing_flag(self):
        self.assertEqual(validate_landmarks(self.config, self.routes), [])
        self.assertFalse(self.config['landmarks_validated'])

    def test_missing_any_branch_landmark_blocks_validation(self):
        del self.config['landmarks']['1_right']['hill_stop_s']
        errors = validate_landmarks(self.config, self.routes)
        self.assertTrue(any('1_right' in error and 'hill_stop_s' in error for error in errors))

    def test_nonfinite_boolean_out_of_route_and_unknown_fields(self):
        for value in (math.inf, math.nan, True, -1, 21):
            config = copy.deepcopy(self.config)
            config['landmarks']['2']['stop_line_s'] = value
            self.assertTrue(validate_landmarks(config, self.routes))
        config = copy.deepcopy(self.config)
        config['landmarks']['missing'] = {'stop_line_s': 1}
        config['landmarks']['2']['typo'] = 1
        errors = validate_landmarks(config, self.routes)
        self.assertTrue(any('Unknown landmark route' in e for e in errors))
        self.assertTrue(any('unknown landmark typo' in e for e in errors))

    def test_hill_minimum_one_metre_zone(self):
        self.config['landmarks']['1_left']['hill_stop_s'] = 2.8
        errors = validate_landmarks(self.config, self.routes)
        self.assertTrue(any('ramp boundaries' in e for e in errors))

    def test_parallel_parking_uses_rddf_without_landmarks(self):
        for name in ('10_parallel-left-in', '10_parallel-right-in',
                     '11-parallel-left-out', '11_parallel-right-out'):
            self.config['landmarks'].pop(name, None)
        self.assertEqual(validate_landmarks(self.config, self.routes), [])

    def test_finish_branch_is_derived_from_rddf_geometry(self):
        self.routes['13_left'] = Route('13_left', [(100., 100., 0.), (101., 100., 0.)])
        errors = validate_landmarks(self.config, self.routes)
        self.assertTrue(any('fork does not match RDDF geometry' in e for e in errors))

    def test_fork_missing_or_invalid_tolerance_fails(self):
        del self.routes['13_left']
        self.config['tracker'] = {'end_tolerance_m': 3}
        errors = validate_landmarks(self.config, self.routes)
        self.assertTrue(any('required to verify' in e for e in errors))
        self.assertTrue(any('end_tolerance_m' in e for e in errors))

    def test_obsolete_finish_landmark_is_rejected(self):
        self.config['landmarks']['13_left'] = {'finish_s': 19.0}
        errors = validate_landmarks(self.config, self.routes)
        self.assertTrue(any('13_left: unknown landmark finish_s' in e for e in errors))

    def test_overlay_preserves_actual_base_rules_and_only_replaces_selected_fields(self):
        base = {'rules': {'finish_runout_m': 3., 'rear_axle_offset_m': -.5},
                'tracker': {'end_tolerance_m': .8}, 'vehicle': {'width_m': 2.}}
        overlay = copy.deepcopy(self.config)
        overlay['rules'] = {'finish_runout_m': 4.}
        combined = merge_overlay(base, overlay)
        self.assertEqual(combined['rules'], {'finish_runout_m': 4., 'rear_axle_offset_m': -.5})
        self.assertEqual(combined['vehicle']['width_m'], 2.)
        self.assertEqual(combined['vehicle']['unchanged'], 'value')
        self.assertEqual(base['rules']['finish_runout_m'], 3.)
        self.assertNotIn('rear_axle_offset_m', overlay['rules'])
        self.assertEqual(validate_landmarks(combined, self.routes), [])

    def test_validated_config_requires_matching_catalogue_digest(self):
        self.config['landmarks_validated'] = True
        self.assertTrue(any('not bound' in e for e in validate_landmarks(self.config, self.routes)))
        self.config['route_catalogue_digest'] = route_catalogue_digest(self.routes)
        self.assertEqual(validate_landmarks(self.config, self.routes), [])
        changed = dict(self.routes)
        changed['2'] = route('2', start=.5)
        self.assertEqual(changed['2'].length, self.routes['2'].length)
        self.assertTrue(any('not bound' in e for e in validate_landmarks(self.config, changed)))

    def test_digest_is_order_stable_and_covers_direction_and_geometry(self):
        digest = route_catalogue_digest(self.routes)
        self.assertEqual(route_catalogue_digest(dict(reversed(list(self.routes.items())))), digest)
        changed = dict(self.routes)
        changed['2'] = Route('2', self.routes['2'].points, direction=-1)
        self.assertNotEqual(route_catalogue_digest(changed), digest)

    def test_explicit_revalidation_rejects_existing_points_from_changed_map(self):
        point = snap_landmark(self.routes, '2', 'stop_line_s', 5, 0)
        config = record_landmark(self.config, self.routes, '2', 'stop_line_s', point)
        config.update(landmarks_validated=True, route_catalogue_digest=route_catalogue_digest(self.routes))
        changed = dict(self.routes)
        changed['2'] = route('2', start=.5)
        errors = validate_landmarks(config, changed, explicit_validation=True)
        self.assertTrue(any('remeasure this recorded point' in e for e in errors))

    def test_explicit_manual_validation_is_possible_without_inventing_point_metadata(self):
        self.config['landmarks_validated'] = True
        self.assertEqual(validate_landmarks(self.config, self.routes, explicit_validation=True), [])
        self.assertNotIn('route_catalogue_digest', self.config)
        self.assertNotIn('landmark_points', self.config)

    def test_point_metadata_must_match_current_distance_and_index(self):
        point = snap_landmark(self.routes, '2', 'stop_line_s', 5, 0)
        config = record_landmark(self.config, self.routes, '2', 'stop_line_s', point)
        config['landmarks']['2']['stop_line_s'] = 6
        self.assertTrue(any('remeasure this recorded point' in e
                            for e in validate_landmarks(config, self.routes, explicit_validation=True)))
        config['landmark_points']['2']['stop_line_s']['index'] = 1000
        self.assertTrue(any('invalid landmark_points index' in e
                            for e in validate_landmarks(config, self.routes, explicit_validation=True)))

    def test_atomic_update_preserves_existing_data_and_failure_keeps_file(self):
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / 'landmarks.json'
            atomic_update(filename, lambda _: self.config)
            point = snap_landmark(self.routes, '2', 'stop_line_s', 4, 0)
            result = atomic_update(filename, lambda cfg: record_landmark(
                cfg, self.routes, '2', 'stop_line_s', point))
            self.assertEqual(json.loads(filename.read_text()), result)
            self.assertEqual(result['vehicle'], self.config['vehicle'])
            before = filename.read_bytes()
            with self.assertRaises(ValueError):
                atomic_update(filename, lambda cfg: dict(cfg, invalid=math.nan))
            self.assertEqual(filename.read_bytes(), before)
            self.assertFalse(list(Path(directory).glob('*.tmp')))

    def test_atomic_update_rejects_malformed_existing_json_and_symlink(self):
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / 'landmarks.json'
            filename.write_text('invalid json')
            with self.assertRaises(ValueError):
                atomic_update(filename, lambda cfg: {})
            self.assertEqual(filename.read_text(), 'invalid json')
            link = Path(directory) / 'link.json'
            link.symlink_to(filename)
            with self.assertRaises(ValueError):
                atomic_update(link, lambda cfg: {})

    def test_zone_pair_clicks_replace_legacy_hill_target_and_bind_to_rddf(self):
        for name in ('1_left', '1_right'):
            self.config['landmarks'][name] = {}
            for key, x in (('hill_zone_start_s', 4), ('hill_zone_end_s', 5)):
                point = snap_landmark(self.routes, name, key, x, 0)
                self.config = record_landmark(self.config, self.routes, name, key, point)
        self.assertEqual(validate_landmarks(self.config, self.routes), [])
        self.config['landmarks']['1_left']['hill_zone_end_s'] = 3
        self.assertTrue(validate_landmarks(self.config, self.routes))

    def test_one_zone_marker_never_uses_existing_legacy_stop(self):
        self.config['landmarks']['1_left']['hill_zone_start_s'] = 4
        self.assertTrue(any('hill_zone_end_s' in e for e in validate_landmarks(self.config, self.routes)))


if __name__ == '__main__':
    unittest.main()
