import math
import os
import sys
import unittest
from dataclasses import replace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from selector.core import Candidate, Pose, SelectorCore, State, path_fingerprint


class SelectorTests(unittest.TestCase):
    def setUp(self):
        self.core = SelectorCore()
        self.state = State(10, 10, 1, '3_s-static-obstacle', 'RDDF', 1)
        self.path = Candidate(10, 10, 1, self.state.route_name, 1, 'map', 'map', 10,
                              (Pose('map', (0., 0., 0.), (0., 0., 0., 1.)),
                               Pose('map', (1., 0., 0.), (0., 0., 0., 1.))))

    def evaluate(self, state=None, path=None, now=10.1):
        return self.core.evaluate(state or self.state, {'RDDF': path or self.path}, now)

    def test_valid_path_and_waiting_state_geometry(self):
        self.assertTrue(self.evaluate().ready)
        waiting = replace(self.state, valid=False, stop_requested=True)
        self.assertTrue(self.evaluate(state=waiting).ready)

    def test_missing_requested_source_never_falls_back(self):
        for mode in ('LOCAL', 'PARKING'):
            result = self.evaluate(state=replace(self.state, path_mode=mode))
            self.assertEqual(result.reason, 'REQUESTED_PATH_MISSING')

    def test_mismatched_old_request_route_or_direction(self):
        for change in ({'decision_id': 0}, {'route_name': 'other'}, {'direction': -1}):
            self.assertEqual(self.evaluate(path=replace(self.path, **change)).reason,
                             'PATH_REQUEST_MISMATCH')

    def test_state_and_path_stamps_are_fresh_and_not_future(self):
        for stamp in (0., 9., 11., math.nan):
            for field in ('stamp', 'received'):
                self.assertFalse(self.evaluate(state=replace(self.state, **{field: stamp})).ready)
            for field in ('stamp', 'received', 'path_stamp'):
                self.assertFalse(self.evaluate(path=replace(self.path, **{field: stamp})).ready)

    def test_invalid_mode_and_missing_state(self):
        self.assertFalse(self.evaluate(state=replace(self.state, path_mode='AVOIDANCE')).ready)
        self.assertFalse(self.core.evaluate(None, {}, 10).ready)

    def test_invalid_frames_positions_and_quaternions(self):
        for field in ('frame_id', 'path_frame_id'):
            self.assertFalse(self.evaluate(path=replace(self.path, **{field: 'odom'})).ready)
        first, second = self.path.poses
        for pose in (replace(second, frame_id=''),
                     replace(second, position=(math.nan, 0, 0)),
                     replace(second, orientation=(0, 0, 0, 0)),
                     replace(second, orientation=(0, 0, 0, 2))):
            self.assertFalse(self.evaluate(path=replace(self.path, poses=(first, pose))).ready)

    def test_empty_duplicate_and_discontinuous_paths(self):
        first, second = self.path.poses
        for poses in ((), (first,), (first, first),
                      (first, replace(second, position=(4, 0, 0))),
                      (first, replace(second, position=(1e308, 0, 0)))):
            self.assertFalse(self.evaluate(path=replace(self.path, poses=poses)).ready)

    def test_reverse_geometry_can_be_ready_for_preview(self):
        poses = tuple(replace(pose, orientation=(0., 0., 1., 0.)) for pose in self.path.poses)
        self.assertTrue(self.evaluate(state=replace(self.state, direction=-1),
                                      path=replace(self.path, direction=-1, poses=poses)).ready)

    def test_changing_direction_tag_does_not_make_forward_geometry_reverse(self):
        result = self.evaluate(state=replace(self.state, direction=-1),
                               path=replace(self.path, direction=-1))
        self.assertEqual(result.reason, 'PATH_BODY_DIRECTION_MISMATCH')

    def test_reversed_positions_can_keep_forward_facing_body(self):
        self.assertTrue(self.evaluate(state=replace(self.state, direction=-1),
                                      path=replace(self.path, direction=-1,
                                                   poses=tuple(reversed(self.path.poses)))).ready)

    def test_both_segment_endpoints_must_face_the_motion_direction(self):
        first, second = self.path.poses
        wrong = (0., 0., math.sin(math.pi / 4), math.cos(math.pi / 4))
        for poses in ((replace(first, orientation=wrong), second),
                      (first, replace(second, orientation=wrong))):
            self.assertEqual(self.evaluate(path=replace(self.path, poses=poses)).reason,
                             'PATH_BODY_DIRECTION_MISMATCH')

    def test_curved_path_checks_each_local_tangent_not_a_global_heading(self):
        # A gentle quarter circle has local tangent headings between 0 and pi/2.
        poses = []
        for index in range(7):
            angle = index * math.pi / 12
            poses.append(Pose('map', (3 * math.sin(angle), 3 * (1 - math.cos(angle)), 0.),
                              (0., 0., math.sin(angle / 2), math.cos(angle / 2))))
        self.assertTrue(self.evaluate(path=replace(self.path, poses=tuple(poses))).ready)
        inconsistent = tuple(replace(pose, orientation=(0., 0., 0., 1.)) for pose in poses)
        self.assertEqual(self.evaluate(path=replace(self.path, poses=inconsistent)).reason,
                         'PATH_BODY_DIRECTION_MISMATCH')

    def test_yaw_wrap_around_pi_is_not_a_direction_error(self):
        yaw = -math.pi + .01
        orientation = (0., 0., math.sin(yaw / 2), math.cos(yaw / 2))
        poses = (Pose('map', (0., 0., 0.), orientation),
                 Pose('map', (-1., .01, 0.), orientation))
        self.assertTrue(self.evaluate(path=replace(self.path, poses=poses)).ready)

    def test_vertical_only_path_is_not_a_ground_vehicle_trajectory(self):
        first, second = self.path.poses
        path = replace(self.path, poses=(first, replace(second, position=(0., 0., 1.))))
        self.assertEqual(self.evaluate(path=path).reason, 'PATH_DEGENERATE')

    def test_heading_tolerance_cannot_allow_perpendicular_or_opposite_motion(self):
        for tolerance in (0, -1, math.pi / 2, math.pi, math.nan, math.inf):
            with self.subTest(tolerance=tolerance), self.assertRaises(ValueError):
                SelectorCore(max_path_heading_error_rad=tolerance)

    def test_fingerprint_agrees_for_identical_geometry_with_refreshed_stamps(self):
        self.assertEqual(path_fingerprint(self.path), path_fingerprint(
            replace(self.path, received=10.1, stamp=10.1, path_stamp=10.1)))

    def test_fingerprint_binds_exact_pose_frames_and_identity(self):
        first, second = self.path.poses
        fingerprint = path_fingerprint(self.path)
        changes = ({'decision_id': 2},
                   {'route_name': '4'}, {'direction': -1}, {'frame_id': 'odom'},
                   {'poses': (first, replace(second, position=(1., .5, 0.)))},
                   {'poses': (first, replace(second, orientation=(0., 0., 1., 0.)))})
        for fields in changes:
            self.assertNotEqual(fingerprint, path_fingerprint(replace(self.path, **fields)))


if __name__ == '__main__':
    unittest.main()
