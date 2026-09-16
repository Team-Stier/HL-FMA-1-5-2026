import math
import os
import sys
import unittest
from dataclasses import replace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from vehicle_safety.core import Mission, PathReady, RawCommand, SafetyGate, SafetyInput


class GateTests(unittest.TestCase):
    def setUp(self):
        self.gate = SafetyGate()
        self.mission = Mission(10, 1, '8_dynamic-obstacle', 'RDDF', 1, 2.)
        self.path = PathReady(10, 1, self.mission.route_name, 'RDDF', 1, True, 'plan-a')
        self.safety = SafetyInput(10, False, True, clearance_m=3., path_fingerprint='plan-a')
        self.healthy(10)
        self.gate.update_raw(RawCommand(10, 40, 0), 10.01)

    def healthy(self, now):
        self.gate.update_mission(replace(self.mission, stamp=now), now)
        self.gate.update_localization(True, now)
        self.gate.update_safety(replace(self.safety, stamp=now), now)
        self.gate.update_path(replace(self.path, stamp=now), now)

    def test_speed_is_floored_and_steering_bounded(self):
        output = self.gate.evaluate(10.02)
        self.assertTrue(output.allowed)
        self.assertEqual((output.kph, output.deg, output.brake), (7, 25, 0))

    def test_positive_speed_caps_change_without_requiring_a_new_raw(self):
        self.gate.update_mission(replace(self.mission, speed_limit_mps=1.5), 10.05)
        output = self.gate.evaluate(10.06)
        self.assertTrue(output.allowed)
        self.assertEqual(output.kph, 5)
        self.gate.update_mission(replace(self.mission, speed_limit_mps=2.5), 10.07)
        output = self.gate.evaluate(10.08)
        self.assertTrue(output.allowed)
        self.assertEqual(output.kph, 9)

    def test_zero_cap_and_recovery_still_require_a_new_raw(self):
        self.gate.update_mission(replace(self.mission, speed_limit_mps=0), 10.05)
        self.gate.update_mission(self.mission, 10.06)
        self.assertEqual(self.gate.evaluate(10.07).reason, 'FRESH_RAW_COMMAND_REQUIRED')
        self.gate.update_raw(RawCommand(5, 0, 0), 10.08)
        self.assertTrue(self.gate.evaluate(10.09).allowed)

    def test_missing_inputs_and_missing_raw_stop(self):
        self.assertFalse(SafetyGate().evaluate(10).allowed)
        self.gate = SafetyGate()
        self.healthy(10)
        self.assertFalse(self.gate.evaluate(10).allowed)

    def test_raw_expiry(self):
        self.assertEqual(self.gate.evaluate(10.3).reason, 'RAW_COMMAND_MISSING_OR_STALE')

    def test_stale_and_future_stamped_dependencies(self):
        for kind, message in (('mission', self.mission), ('path', self.path), ('safety', self.safety)):
            for stamp in (0, 9, 11, math.nan):
                self.setUp()
                self.healthy(10.1)
                getattr(self.gate, 'update_' + kind)(replace(message, stamp=stamp), 10.1)
                self.gate.update_raw(RawCommand(5, 0, 0), 10.11)
                self.assertFalse(self.gate.evaluate(10.12).allowed)

    def test_unknown_mission_fields_rejected(self):
        for fields in ({'speed_limit_mps': math.nan}, {'speed_limit_mps': -1},
                       {'speed_limit_mps': 0.1}, {'path_mode': 'UNKNOWN'},
                       {'route_name': ''}, {'direction': 0}, {'valid': False},
                       {'finished': True}, {'stop_requested': True}):
            self.setUp()
            self.healthy(10.1)
            self.gate.update_mission(replace(self.mission, stamp=10.1, **fields), 10.1)
            self.gate.update_raw(RawCommand(5, 0, 0), 10.11)
            self.assertFalse(self.gate.evaluate(10.12).allowed)

    def test_unsigned_reverse_is_always_blocked(self):
        self.gate.update_mission(replace(self.mission, direction=-1), 10.1)
        self.gate.update_path(replace(self.path, direction=-1), 10.1)
        self.gate.update_raw(RawCommand(1, 0, 0), 10.11)
        self.assertEqual(self.gate.evaluate(10.12).reason, 'REVERSE_INTERFACE_UNAVAILABLE')

    def test_path_identity_and_ready_must_match(self):
        for fields in ({'ready': False}, {'decision_id': 2}, {'source': 'PARKING'},
                       {'route_name': 'other'}, {'direction': -1}):
            self.setUp()
            self.healthy(10.1)
            self.gate.update_path(replace(self.path, stamp=10.1, **fields), 10.1)
            self.gate.update_raw(RawCommand(5, 0, 0), 10.11)
            self.assertFalse(self.gate.evaluate(10.12).allowed)

    def test_replanned_path_cannot_use_previous_paths_safety_approval(self):
        self.gate.update_path(replace(self.path, path_fingerprint='plan-b'), 10.05)
        self.assertEqual(self.gate.evaluate(10.06).reason, 'PATH_FINGERPRINT_MISMATCH')
        # Receiving raw while B lacks approval does not arm motion later.
        self.gate.update_raw(RawCommand(5, 0, 0), 10.07)
        self.gate.update_safety(replace(self.safety, path_fingerprint='plan-b'), 10.08)
        self.assertEqual(self.gate.evaluate(10.09).reason, 'FRESH_RAW_COMMAND_REQUIRED')
        self.gate.update_raw(RawCommand(5, 0, 0), 10.10)
        self.assertTrue(self.gate.evaluate(10.11).allowed)

    def test_approval_before_replanned_path_also_invalidates_previous_raw(self):
        self.gate.update_safety(replace(self.safety, path_fingerprint='plan-b'), 10.05)
        self.gate.update_path(replace(self.path, path_fingerprint='plan-b'), 10.06)
        self.assertEqual(self.gate.evaluate(10.07).reason, 'FRESH_RAW_COMMAND_REQUIRED')

    def test_unbound_path_or_safety_approval_never_allows_motion(self):
        self.gate.update_path(replace(self.path, path_fingerprint=''), 10.05)
        self.assertEqual(self.gate.evaluate(10.06).reason, 'PATH_FINGERPRINT_MISSING')
        self.gate.update_path(self.path, 10.07)
        self.gate.update_safety(replace(self.safety, path_fingerprint=''), 10.08)
        self.assertEqual(self.gate.evaluate(10.09).reason, 'SAFETY_PATH_FINGERPRINT_MISSING')

    def test_transient_safety_stop_requires_command_after_recovery(self):
        self.gate.update_safety(replace(self.safety, stop=True), 10.05)
        # No timer tick during this stop: callbacks must still invalidate raw.
        self.gate.update_raw(RawCommand(5, 0, 0), 10.06)
        self.gate.update_safety(self.safety, 10.07)
        self.assertEqual(self.gate.evaluate(10.08).reason, 'FRESH_RAW_COMMAND_REQUIRED')
        self.gate.update_raw(RawCommand(5, 0, 0), 10.09)
        self.assertTrue(self.gate.evaluate(10.1).allowed)

    def test_localization_loss_recovery_does_not_revive_raw(self):
        self.gate.update_localization(False, 10.05)
        self.gate.update_localization(True, 10.06)
        self.assertEqual(self.gate.evaluate(10.07).reason, 'FRESH_RAW_COMMAND_REQUIRED')
        self.gate.update_raw(RawCommand(5, 0, 0), 10.08)
        self.assertTrue(self.gate.evaluate(10.09).allowed)

    def test_mission_stop_and_release_do_not_revive_raw(self):
        self.gate.update_mission(replace(self.mission, stop_requested=True), 10.05)
        self.gate.update_mission(self.mission, 10.06)
        self.assertEqual(self.gate.evaluate(10.07).reason, 'FRESH_RAW_COMMAND_REQUIRED')
        self.gate.update_raw(RawCommand(5, 0, 0), 10.08)
        self.assertTrue(self.gate.evaluate(10.09).allowed)

    def test_stale_dependency_refresh_does_not_revive_raw(self):
        gate = SafetyGate(input_timeout=0.05, command_timeout=0.25)
        self.gate = gate
        self.healthy(10)
        self.gate.update_raw(RawCommand(5, 0, 0), 10.01)
        # There is no evaluate call while inputs are stale.
        self.healthy(10.06)
        self.assertEqual(self.gate.evaluate(10.07).reason, 'FRESH_RAW_COMMAND_REQUIRED')

    def test_new_decision_and_new_path_require_new_raw(self):
        self.gate.update_mission(replace(self.mission, decision_id=2), 10.05)
        self.gate.update_path(replace(self.path, decision_id=2), 10.05)
        self.assertEqual(self.gate.evaluate(10.06).reason, 'FRESH_RAW_COMMAND_REQUIRED')
        self.gate.update_raw(RawCommand(5, 0, 0), 10.07)
        self.assertTrue(self.gate.evaluate(10.08).allowed)

    def test_clock_reset_requires_all_fresh_inputs(self):
        self.assertFalse(self.gate.evaluate(1).allowed)
        self.gate.update_raw(RawCommand(5, 0, 0), 1.01)
        self.assertFalse(self.gate.evaluate(1.02).allowed)
        self.healthy(1.03)
        self.assertFalse(self.gate.evaluate(1.04).allowed)
        self.gate.update_raw(RawCommand(5, 0, 0), 1.05)
        self.assertTrue(self.gate.evaluate(1.06).allowed)

    def test_invalid_raw_and_controller_stop(self):
        for cmd in (RawCommand(-1, 0, 0), RawCommand(1, 0, 256),
                    RawCommand(1.2, 0, 0), RawCommand(1, 0, 1), RawCommand(0, 0, 0)):
            self.setUp()
            self.gate.update_raw(cmd, 10.05)
            output = self.gate.evaluate(10.06)
            self.assertFalse(output.allowed)
            self.assertEqual((output.kph, output.brake), (0, 1))

    def test_lidar_unknown_stops(self):
        self.gate.update_safety(replace(self.safety, sensor_valid=False), 10.05)
        self.assertEqual(self.gate.evaluate(10.06).reason, 'LIDAR_INVALID')

    def test_raw_kept_fresh_cannot_hide_localization_expiry(self):
        self.gate.update_mission(replace(self.mission, stamp=10.6), 10.6)
        self.gate.update_safety(replace(self.safety, stamp=10.6), 10.6)
        self.gate.update_path(replace(self.path, stamp=10.6), 10.6)
        self.gate.update_raw(RawCommand(5, 0, 0), 10.61)
        self.assertEqual(self.gate.evaluate(10.62).reason, 'LOCALIZATION_MISSING_OR_STALE')


if __name__ == '__main__':
    unittest.main()
