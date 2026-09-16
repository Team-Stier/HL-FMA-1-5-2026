#!/usr/bin/env python3
"""Rules regression tests using fake time and explicit perception snapshots."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from stier_state_manager.mission import MissionEngine


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.engine = MissionEngine()

    def snap(self, section=1, route=None, now=0.0, **changes):
        route = route or {
            1: "1_left", 2: "2", 3: "3_s-static-obstacle", 4: "4",
            5: "5_T-left-in", 6: "6-T-left-out", 7: "7",
            8: "8_dynamic-obstacle", 9: "9", 10: "10_parallel-left-in",
            11: "11-parallel-left-out", 12: "12", 13: "13_left",
        }[section]
        marks = {
            "hill_start_s": 2.0, "hill_stop_s": 5.0, "hill_top_s": 9.0,
            "stop_line_s": 10.0, "intersection_exit_s": 15.0,
            "parking_confirm_s": 18.0, "parking_exit_s": 19.0,
            "finish_branch_s": 13.258, "finish_s": 19.0,
        }
        value = {
            "now": now, "healthy": True, "reason": "", "route": route, "decision_id": 1,
            "section": section, "s": 0.0, "length": 20.0, "at_end": False,
            "speed": 1.0, "yaw": 0.0, "calibrated": True,
            "landmarks": {route: marks}, "path_ready": True, "collision": False,
            "signal": {"stamp": now, "route": route, "value": "UNKNOWN"},
            "parking": {"stamp": now, "left": "UNKNOWN", "right": "UNKNOWN"},
        }
        value.update(changes)
        return value

    def run_at(self, section=1, **kwargs):
        return self.engine.update(self.snap(section, **kwargs))

    def poll(self, section=1, start=0.0, end=3.0, **kwargs):
        """Provide continuous quarter-second observations across a fake hold."""
        result = None
        for index in range(int(round((end - start) * 4)) + 1):
            now = start + index / 4.0
            values = dict(kwargs)
            for name in ("parking_maneuver",):
                if name in values:
                    values[name] = dict(values[name], stamp=now)
            result = self.run_at(section, now=now, **values)
        return result

    def select_parking(self, kind="t", side="left", start=0.0):
        section = 4 if kind == "t" else 9
        for offset in (0.0, 0.1, 0.2):
            now = start + offset
            parking = {"stamp": now, "left": "BLOCKED", "right": "BLOCKED"}
            parking[side] = "CLEAR"
            result = self.run_at(section, now=now, parking=parking)
        return result

    def test_hill_requires_three_seconds_of_continuous_standstill(self):
        self.assertTrue(self.run_at(now=0.0, s=5.0, speed=0.0)["stop_requested"])
        self.poll(start=0.25, end=2.75, s=5.0, speed=0.0)
        self.assertTrue(self.run_at(now=2.999, s=5.0, speed=0.0)["stop_requested"])
        released = self.run_at(now=3.0, s=5.0, speed=0.0)
        self.assertFalse(released["stop_requested"])
        self.assertEqual(released["completed_missions"]["hill"], 3.0)
        self.assertEqual(self.run_at(now=4.0, s=20.0, at_end=True)["next_route"], "2")

    def test_hill_movement_resets_hold(self):
        self.run_at(now=0, s=5.0, speed=0.0)
        self.run_at(now=2, s=5.0, speed=0.2)
        self.run_at(now=3, s=5.0, speed=0.0)
        self.poll(start=3.25, end=5.75, s=5.0, speed=0.0)
        self.assertTrue(self.run_at(now=5.9, s=5.0, speed=0.0)["stop_requested"])
        self.assertFalse(self.run_at(now=6, s=5.0, speed=0.0)["stop_requested"])

    def test_localization_loss_restarts_hill_dwell(self):
        self.run_at(now=0, s=5.0, speed=0.0)
        lost = self.run_at(now=2, s=5.0, speed=0.0, healthy=False, reason="STALE_ODOM")
        self.assertEqual(lost["reason"], "STALE_ODOM")
        self.run_at(now=3, s=5.0, speed=0.0)
        self.poll(start=3.25, end=5.75, s=5.0, speed=0.0)
        self.assertTrue(self.run_at(now=5.9, s=5.0, speed=0.0)["stop_requested"])
        self.assertFalse(self.run_at(now=6, s=5.0, speed=0.0)["stop_requested"])

    def test_collision_restarts_hill_dwell(self):
        self.run_at(now=0, s=5.0, speed=0.0)
        self.run_at(now=2, s=5.0, speed=0.0, collision=True)
        self.run_at(now=3, s=5.0, speed=0.0)
        self.poll(start=3.25, end=5.75, s=5.0, speed=0.0)
        self.assertTrue(self.run_at(now=5.9, s=5.0, speed=0.0)["stop_requested"])

    def test_hill_uses_raw_progress_to_detect_rollback_and_latches_fault(self):
        self.run_at(now=0, s=5.0, raw_s=5.0)
        rolled = self.run_at(now=1, s=5.0, raw_s=4.49)
        self.assertEqual(rolled["reason"], "HILL_ROLLBACK_LIMIT_EXCEEDED")
        again = self.run_at(now=2, s=5.1, raw_s=5.1)
        self.assertTrue(again["stop_requested"])
        self.assertEqual(again["counters"]["hill_rollback"], 1)

    def test_hill_rule_zone_requires_one_metre_from_ends(self):
        snap = self.snap()
        snap["landmarks"]["1_left"]["hill_stop_s"] = 2.5
        self.assertEqual(self.engine.update(snap)["reason"], "HILL_STOP_OUTSIDE_RULE_ZONE")

    def test_hill_stop_cannot_be_silently_skipped(self):
        result = self.run_at(s=6.0)
        self.assertEqual(result["reason"], "HILL_STOP_ZONE_MISSED")
        self.assertIsNone(result["next_route"])

    def test_hill_clearance_timeout_does_not_override_safety_stop(self):
        self.poll(s=5, speed=0)
        result = self.run_at(now=34, s=6, speed=0)
        self.assertIn("HILL_CLEARANCE_TIMEOUT", result["diagnostics"])
        stopped = self.run_at(now=35, s=6, collision=True)
        self.assertTrue(stopped["stop_requested"])

    def test_signal_red_stops_at_line_and_provides_approach_distance(self):
        approach = self.run_at(2, s=6)
        self.assertFalse(approach["stop_requested"])
        self.assertEqual(approach["remaining_stop_m"], 4.0)
        self.assertTrue(self.run_at(2, now=0.1, s=10)["stop_requested"])

    def test_signal_front_bumper_offset(self):
        self.engine = MissionEngine({"rules": {"front_bumper_offset_m": 1.2}})
        result = self.run_at(2, s=8.8)
        self.assertEqual(result["remaining_stop_m"], 0)
        self.assertTrue(result["stop_requested"])

    def test_stale_future_and_wrong_route_signals_do_not_authorize(self):
        for signal in (
            {"stamp": 8, "route": "2", "value": "GREEN"},
            {"stamp": 10.01, "route": "2", "value": "GREEN"},
            {"stamp": 10, "route": "4", "value": "GREEN"},
        ):
            with self.subTest(signal=signal):
                self.engine = MissionEngine()
                self.assertTrue(self.run_at(2, now=10, s=10, signal=signal)["stop_requested"])

    def test_green_before_line_does_not_latch_until_entry(self):
        result = self.run_at(2, now=0, s=9, signal={"stamp": 0, "route": "2", "value": "GREEN"})
        self.assertFalse(result["stop_requested"])
        self.assertIsNone(result["remaining_stop_m"])
        red = self.run_at(2, now=0.1, s=10, signal={"stamp": 0.1, "route": "2", "value": "RED"})
        self.assertTrue(red["stop_requested"])

    def test_green_authorization_survives_red_inside_intersection(self):
        green = {"stamp": 0, "route": "2", "value": "GREEN"}
        self.assertFalse(self.run_at(2, s=10, signal=green)["stop_requested"])
        red = {"stamp": 0.1, "route": "2", "value": "RED"}
        self.assertFalse(self.run_at(2, now=0.1, s=12, signal=red)["stop_requested"])
        self.assertEqual(self.run_at(2, now=1, s=20, at_end=True)["next_route"], "3_s-static-obstacle")

    def test_left_turn_requires_arrow_and_never_uses_plain_green(self):
        self.assertTrue(self.run_at(7, s=10, signal={"stamp": 0, "route": "7", "value": "GREEN"})["stop_requested"])
        result = self.run_at(7, now=0.1, s=10, signal={"stamp": 0.1, "route": "7", "value": "LEFT_ARROW"})
        self.assertFalse(result["stop_requested"])
        self.assertEqual(self.run_at(7, now=1, s=20, at_end=True)["next_route"], "8_dynamic-obstacle")

    def test_intersection_stop_penalty_timers_never_force_motion(self):
        for index in range(81):
            now = index / 4.0
            result = self.run_at(2, now=now, s=10, speed=0,
                                 signal={"stamp": now, "route": "2", "value": "GREEN"})
        self.assertEqual(result["counters"]["intersection_stop_penalty"], 1)
        self.assertEqual(result["counters"]["intersection_stop_timeout"], 1)
        unsafe = self.run_at(2, now=20.1, s=10, collision=True)
        self.assertTrue(unsafe["stop_requested"])

    def test_intersection_clearance_uses_rear_axle_and_30_second_limit(self):
        self.engine = MissionEngine({"rules": {"rear_axle_offset_m": -1}})
        self.run_at(2, now=0, s=10, signal={"stamp": 0, "route": "2", "value": "GREEN"})
        not_clear = self.run_at(2, now=1, s=15)
        self.assertNotIn("intersection:2", not_clear["completed_missions"])
        late_clear = self.run_at(2, now=30.1, s=16)
        self.assertIn("INTERSECTION_CLEARANCE_TIMEOUT", late_clear["diagnostics"])
        self.assertIn("intersection:2", late_clear["completed_missions"])

    def test_static_requires_local_path_and_has_no_rddf_fallback(self):
        result = self.run_at(3, path_ready=False, at_end=True, s=20)
        self.assertEqual(result["path_mode"], "LOCAL")
        self.assertTrue(result["stop_requested"])
        self.assertIsNone(result["next_route"])

    def test_unknown_parking_is_not_free(self):
        result = self.run_at(9, s=20, at_end=True)
        self.assertEqual(result["phase"], "WAIT_SPACE")
        self.assertIsNone(result["selected_branch"])

    def test_replayed_parking_stamp_is_one_observation(self):
        for now in (0.0, 0.1, 0.2, 0.3):
            result = self.run_at(9, now=now, parking={"stamp": 0, "left": "CLEAR", "right": "BLOCKED"})
        self.assertIsNone(result["selected_branch"])

    def test_parking_branch_is_stable_and_latched(self):
        result = self.select_parking("parallel", "right")
        self.assertEqual(result["selected_branch"], "right")
        changed = self.run_at(9, now=0.3, s=20, at_end=True,
                              parking={"stamp": 0.3, "left": "CLEAR", "right": "BLOCKED"})
        self.assertEqual(changed["selected_branch"], "right")
        self.assertTrue(changed["stop_requested"])
        clear = self.run_at(9, now=0.4, s=20, at_end=True,
                            parking={"stamp": 0.4, "left": "BLOCKED", "right": "CLEAR"})
        self.assertEqual(clear["next_route"], "10_parallel-right-in")

    def maneuver(self, route, now, phase="FORWARD_APPROACH", leg_index=0,
                 start_s=0.0, target_s=5.0, final_leg=False, **changes):
        value = dict(stamp=now, decision_id=1, route=route, phase=phase,
                     leg_index=leg_index, start_s=start_s, target_s=target_s,
                     direction=-1 if phase == "REVERSE_ENTRY" else 1,
                     final_leg=final_leg)
        value.update(changes)
        return value

    def test_both_parking_missions_replay_forward_reverse_and_exit_legs(self):
        for kind, entry_section, exit_section, next_route, entry, exit_route in (
            ("t", 5, 6, "7", "5_T-left-in", "6-T-left-out"),
            ("parallel", 10, 11, "12", "10_parallel-right-in", "11_parallel-right-out"),
        ):
            with self.subTest(kind=kind):
                self.engine = MissionEngine()
                side = "left" if kind == "t" else "right"
                self.select_parking(kind, side)
                accepted = self.run_at(entry_section, route=entry, now=1, s=0, speed=0,
                                      parking_maneuver=self.maneuver(entry, 1))
                self.assertEqual(accepted["reason"], "PARKING_LEG_ACCEPTED")
                self.assertEqual(accepted["direction"], 1)
                forward = self.run_at(entry_section, route=entry, now=1.1, s=2,
                                     parking_maneuver=self.maneuver(entry, 1.1))
                self.assertFalse(forward["stop_requested"])
                self.assertEqual(forward["phase"], "FORWARD_APPROACH")
                stop = self.run_at(entry_section, route=entry, now=1.2, s=5, speed=0,
                                  parking_maneuver=self.maneuver(entry, 1.2))
                self.assertEqual(stop["phase"], "WAIT_GEAR_CHANGE")
                reverse = dict(phase="REVERSE_ENTRY", leg_index=1, start_s=5,
                               target_s=18, final_leg=True)
                accepted = self.run_at(entry_section, route=entry, now=1.3, s=5, speed=0,
                                      parking_maneuver=self.maneuver(entry, 1.3, **reverse))
                self.assertTrue(accepted["stop_requested"])
                self.assertEqual(accepted["direction"], -1)
                reversing = self.run_at(entry_section, route=entry, now=1.4, s=10, speed=-0.5,
                                       parking_maneuver=self.maneuver(entry, 1.4, **reverse))
                self.assertFalse(reversing["stop_requested"])
                self.assertEqual(reversing["phase"], "REVERSE_ENTRY")
                confirmed = self.poll(entry_section, start=2, end=2.5, route=entry, s=18, speed=0,
                                      parking_maneuver=self.maneuver(entry, 2, **reverse))
                self.assertEqual(confirmed["next_route"], exit_route)
                exit_leg = dict(phase="FORWARD_EXIT", target_s=19, final_leg=True)
                self.run_at(exit_section, route=exit_route, now=3, s=0, speed=0,
                            parking_maneuver=self.maneuver(exit_route, 3, **exit_leg))
                exited = self.run_at(exit_section, route=exit_route, now=3.1, s=20, at_end=True,
                                     parking_maneuver=self.maneuver(exit_route, 3.1, **exit_leg))
                self.assertEqual(exited["direction"], 1)
                self.assertEqual(exited["next_route"], next_route)

    def test_parking_without_leg_never_infers_reverse_from_section(self):
        self.select_parking("parallel", "right")
        result = self.run_at(10, route="10_parallel-right-in", now=1, speed=0)
        self.assertTrue(result["stop_requested"])
        self.assertEqual(result["direction"], 1)
        self.assertEqual(result["parking_leg_index"], -1)
        self.assertNotIn("parking:parallel:entry", result["completed_missions"])

    def test_t_parking_cannot_skip_forward_entry_and_parallel_can_start_reverse(self):
        for kind, section, route in (("t", 5, "5_T-left-in"),
                                     ("parallel", 10, "10_parallel-left-in")):
            self.engine = MissionEngine()
            self.select_parking(kind)
            leg = self.maneuver(route, 1, phase="REVERSE_ENTRY", target_s=18, final_leg=True)
            result = self.run_at(section, now=1, speed=0, parking_maneuver=leg)
            self.assertEqual(result["reason"], "PARKING_INITIAL_LEG_INVALID" if kind == "t"
                             else "PARKING_LEG_ACCEPTED")

    def test_direction_switch_requires_standstill_at_prior_leg_end(self):
        route = "5_T-left-in"
        self.select_parking()
        self.run_at(5, now=1, speed=0, parking_maneuver=self.maneuver(route, 1))
        for now, s, raw_s, speed, expected in (
            (1.1, 5, 5, .5, "WAIT_STANDSTILL_FOR_PARKING_LEG"),
            (1.2, 5, 3, 0, "PARKING_LEG_START_POSE_MISMATCH"),
        ):
            reverse = self.maneuver(route, now, phase="REVERSE_ENTRY", leg_index=1,
                                    start_s=5, target_s=18, final_leg=True)
            result = self.run_at(5, now=now, s=s, raw_s=raw_s, speed=speed,
                                 parking_maneuver=reverse)
            self.assertEqual(result["reason"], expected)
            self.assertEqual(result["direction"], 1)
            self.assertEqual(result["parking_leg_index"], 0)

    def test_planner_cannot_move_active_leg_target_or_skip_leg_index(self):
        route = "5_T-left-in"
        self.select_parking()
        self.run_at(5, now=1, speed=0, parking_maneuver=self.maneuver(route, 1))
        result = self.run_at(5, now=1.1, speed=0,
                             parking_maneuver=self.maneuver(route, 1.1, target_s=7))
        self.assertEqual(result["reason"], "PARKING_ACTIVE_LEG_CHANGED")
        result = self.run_at(5, now=1.2, s=5, speed=0,
                             parking_maneuver=self.maneuver(route, 1.2, phase="REVERSE_ENTRY",
                                                           leg_index=2, start_s=5, target_s=18, final_leg=True))
        self.assertEqual(result["reason"], "PARKING_LEG_SEQUENCE_INVALID")

    def test_planner_target_and_completion_flag_cannot_override_measured_checkline(self):
        self.select_parking("parallel")
        route = "10_parallel-left-in"
        bad = self.maneuver(route, 1, phase="REVERSE_ENTRY", target_s=17, final_leg=True, completed=True)
        result = self.run_at(10, now=1, speed=0, parking_maneuver=bad)
        self.assertEqual(result["reason"], "PARKING_MANEUVER_CHECKPOINT_INVALID")
        good = dict(phase="REVERSE_ENTRY", target_s=18, final_leg=True, completed=True)
        self.run_at(10, now=1.1, speed=0, parking_maneuver=self.maneuver(route, 1.1, **good))
        result = self.poll(10, start=2, end=3, s=18, raw_s=16, speed=0,
                           parking_maneuver=self.maneuver(route, 2, **good))
        self.assertNotIn("parking:parallel:entry", result["completed_missions"])
        self.assertIsNone(result["next_route"])

    def test_stale_wrong_route_or_old_epoch_stops_active_reverse_without_changing_direction(self):
        route = "10_parallel-left-in"
        self.select_parking("parallel")
        reverse = dict(phase="REVERSE_ENTRY", target_s=18, final_leg=True)
        self.run_at(10, now=1, speed=0, parking_maneuver=self.maneuver(route, 1, **reverse))
        for observation in (
            self.maneuver(route, 1.0, **reverse),
            self.maneuver("10_parallel-right-in", 2, **reverse),
            self.maneuver(route, 2, decision_id=0, **reverse),
            self.maneuver(route, 2.1, **reverse),
        ):
            result = self.run_at(10, now=2, s=18, speed=0, parking_maneuver=observation)
            self.assertTrue(result["stop_requested"])
            self.assertEqual(result["direction"], -1)
            self.assertNotIn("parking:parallel:entry", result["completed_missions"])

    def test_parking_dwell_requires_fresh_plan_after_epoch_change(self):
        route = "10_parallel-left-in"
        self.select_parking("parallel")
        reverse = dict(phase="REVERSE_ENTRY", target_s=18, final_leg=True)
        self.run_at(10, now=1, speed=0, parking_maneuver=self.maneuver(route, 1, **reverse))
        self.run_at(10, now=1.1, s=18, speed=0,
                    parking_maneuver=self.maneuver(route, 1.1, **reverse))
        result = self.run_at(10, now=1.4, s=18, speed=0, decision_id=2,
                             parking_maneuver=self.maneuver(route, 1.4, **reverse))
        self.assertEqual(result["reason"], "PARKING_MANEUVER_STALE_OR_MISMATCHED")
        result = self.poll(10, start=1.5, end=2, s=18, speed=0, decision_id=2,
                           parking_maneuver=self.maneuver(route, 1.5, decision_id=2, **reverse))
        self.assertEqual(result["next_route"], "11-parallel-left-out")

    def test_overshooting_forward_setup_latches_parking_fault(self):
        self.select_parking()
        route = "5_T-left-in"
        self.run_at(5, now=1, speed=0, parking_maneuver=self.maneuver(route, 1))
        result = self.run_at(5, now=1.1, s=5.3, parking_maneuver=self.maneuver(route, 1.1))
        self.assertEqual(result["reason"], "PARKING_LEG_TARGET_MISSED")
        result = self.run_at(5, now=1.2, s=5, speed=0,
                             parking_maneuver=self.maneuver(route, 1.2, phase="REVERSE_ENTRY",
                                                           leg_index=1, start_s=5, target_s=18, final_leg=True))
        self.assertEqual(result["phase"], "FAULT")
        self.assertEqual(result["direction"], 1)

    def test_parking_branch_cannot_be_invented_by_route_change(self):
        self.select_parking("t", "left")
        result = self.run_at(5, now=1, route="5_T-right-in")
        self.assertEqual(result["reason"], "PARKING_BRANCH_NOT_AUTHORIZED")

    def test_parking_can_reselect_confirmed_alternative_before_entry(self):
        self.select_parking("parallel", "left")
        for now in (0.3, 0.4, 0.5):
            result = self.run_at(9, now=now, s=20, at_end=True,
                                 parking={"stamp": now, "left": "BLOCKED", "right": "CLEAR"})
        self.assertEqual(result["selected_branch"], "right")
        self.assertEqual(result["next_route"], "10_parallel-right-in")

    def test_committed_parking_branch_cannot_switch_during_manoeuvre(self):
        self.select_parking("parallel", "left")
        self.run_at(10, now=0.3, s=1)
        for now in (0.4, 0.5, 0.6):
            result = self.run_at(9, now=now, s=20, at_end=True,
                                 parking={"stamp": now, "left": "BLOCKED", "right": "CLEAR"})
        self.assertEqual(result["selected_branch"], "left")
        self.assertTrue(result["stop_requested"])

    def test_parking_exit_cannot_skip_confirmation(self):
        self.select_parking("t", "left")
        result = self.run_at(6, now=0.3, s=20, at_end=True)
        self.assertEqual(result["reason"], "PARKING_ENTRY_NOT_COMPLETED")

    def test_section_eight_keeps_rddf_mode_for_dynamic_monitoring(self):
        before = self.run_at(8, s=10)
        self.assertEqual(before["mission"], "DYNAMIC_OBSTACLE")
        self.assertEqual(before["phase"], "MONITORING_DYNAMIC")
        self.assertFalse(before["stop_requested"])
        result = self.run_at(8, s=20, at_end=True)
        self.assertEqual(result["next_route"], "9")
        self.assertIn("section_8_transit", result["completed_missions"])

    def test_healthy_false_blocks_every_segment_transition(self):
        for section in range(1, 14):
            with self.subTest(section=section):
                result = self.run_at(section, s=20, at_end=True, healthy=False)
                self.assertTrue(result["stop_requested"])
                self.assertIsNone(result["next_route"])

    def test_collision_overrides_every_mission(self):
        for section in range(1, 14):
            result = self.run_at(section, s=20, at_end=True, collision=True)
            self.assertTrue(result["stop_requested"])
            self.assertEqual(result["speed_limit"], 0)
            self.assertIsNone(result["next_route"])

    def test_calibration_null_or_out_of_route_prevents_execution(self):
        for section in (1, 2, 4, 5, 6, 7, 10, 11, 12, 13):
            result = self.run_at(section, landmarks={})
            self.assertTrue(result["stop_requested"])
            self.assertTrue(result["reason"].startswith("CALIBRATION_REQUIRED:"))
        result = self.run_at(3, calibrated=False)
        self.assertTrue(result["stop_requested"])

    def test_default_left_finish_branch_uses_internal_junction(self):
        before = self.run_at(12, now=0.3, s=13.2)
        self.assertEqual(before["selected_branch"], "left")
        self.assertIsNone(before["next_route"])
        result = self.run_at(12, now=0.4, s=13.258)
        self.assertEqual(result["next_route"], "13_left")

    def test_configured_right_finish_branch_stays_on_twelve_until_end(self):
        self.engine = MissionEngine({"finish_branch": "right"})
        before = self.run_at(12, now=0.3, s=13.258)
        self.assertEqual(before["selected_branch"], "right")
        self.assertIsNone(before["next_route"])
        result = self.run_at(12, now=0.4, s=20, at_end=True)
        self.assertEqual(result["next_route"], "13_right")

    def test_invalid_finish_branch_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            MissionEngine({"finish_branch": "camera"})

    def test_finish_requires_rear_axle_to_cross_calibrated_line(self):
        self.engine = MissionEngine({"rules": {"rear_axle_offset_m": -1.0}})
        before = self.run_at(13, now=1, s=19.9)
        self.assertFalse(before["stop_requested"])
        self.assertNotIn("finish", before["completed_missions"])
        finish = self.run_at(13, now=2, s=20)
        self.assertEqual(finish["reason"], "COURSE_COMPLETE")
        self.assertEqual(finish["completed_missions"]["finish"], 2)

    def test_race_clock_freezes_when_rear_axle_finishes(self):
        self.run_at(12, now=0, s=0, speed=1)
        finished = self.run_at(13, now=10, s=19)
        self.assertEqual(finished["elapsed_time_s"], 10)
        parked = self.run_at(13, now=900, s=19, speed=0)
        self.assertEqual(parked["elapsed_time_s"], 10)
        self.assertNotIn("MISSION_DEADLINE_EXCEEDED", parked["diagnostics"])
        self.assertNotIn("NO_MOTION_TIMEOUT", parked["diagnostics"])

    def test_completed_hill_timestamp_survives_route_reentry(self):
        self.poll(s=5, speed=0)
        self.run_at(2, now=4)
        result = self.run_at(now=5, s=6)
        self.assertFalse(result["stop_requested"])
        self.assertEqual(result["completed_missions"]["hill"], 3)

    def test_deadline_and_no_motion_only_report_and_preserve_safety(self):
        self.run_at(3, now=0, speed=0, run_started=True)
        result = self.run_at(3, now=481, speed=0, collision=True)
        self.assertTrue(result["stop_requested"])
        self.assertIn("MISSION_DEADLINE_EXCEEDED", result["diagnostics"])
        self.assertIn("NO_MOTION_TIMEOUT", result["diagnostics"])

    def test_regulatory_red_wait_within_one_metre_excluded_from_deadline(self):
        for index in range(2001):
            now = index / 4.0
            result = self.run_at(2, now=now, s=9.5, speed=0,
                                 run_started=True,
                                 signal={"stamp": now, "route": "2", "value": "RED"})
        self.assertEqual(result["excluded_signal_wait_s"], 500)
        self.assertEqual(result["elapsed_time_s"], 0)
        self.assertNotIn("MISSION_DEADLINE_EXCEEDED", result["diagnostics"])
        self.assertNotIn("NO_MOTION_TIMEOUT", result["diagnostics"])

    def test_distant_or_stale_red_wait_does_not_exclude_time(self):
        for s, stale in ((8.9, False), (9.5, True)):
            self.engine = MissionEngine()
            for index in range(5):
                now = index / 4.0
                stamp = now - 1.0 if stale else now
                result = self.run_at(2, now=now, s=s, speed=0,
                                     run_started=True,
                                     signal={"stamp": stamp, "route": "2", "value": "RED"})
            self.assertEqual(result["excluded_signal_wait_s"], 0)

    def test_startup_wait_never_consumes_competition_time(self):
        self.run_at(3, now=0, speed=0, healthy=False)
        for values in ({"healthy": False, "speed": 1}, {"calibrated": False, "speed": 1},
                       {"speed": 0}):
            result = self.run_at(3, now=900, **values)
            self.assertFalse(result["run_started"])
            self.assertEqual(result["elapsed_time_s"], 0)
            self.assertNotIn("MISSION_DEADLINE_EXCEEDED", result["diagnostics"])
            self.assertNotIn("NO_MOTION_TIMEOUT", result["diagnostics"])
        started = self.run_at(3, now=901, speed=1)
        self.assertTrue(started["run_started"])
        self.assertEqual(started["elapsed_time_s"], 0)
        running = self.run_at(3, now=905, speed=1)
        self.assertEqual(running["elapsed_time_s"], 4)

    def test_explicit_start_requires_healthy_calibrated_state(self):
        rejected = self.run_at(3, now=100, speed=0, healthy=False, run_started=True)
        self.assertFalse(rejected["run_started"])
        rejected = self.run_at(3, now=101, speed=0, calibrated=False, run_started=True)
        self.assertFalse(rejected["run_started"])
        started = self.run_at(3, now=102, speed=0, run_started=True)
        self.assertTrue(started["run_started"])
        stopped = self.run_at(3, now=162, speed=0)
        self.assertIn("NO_MOTION_TIMEOUT", stopped["diagnostics"])

    def test_hill_clearance_timer_starts_at_first_stop_not_release(self):
        self.poll(s=5, speed=0)
        result = self.run_at(now=30.1, s=9)
        self.assertIn("HILL_CLEARANCE_TIMEOUT", result["diagnostics"])

    def test_clock_regression_does_not_complete_hold(self):
        self.run_at(now=5, s=5, speed=0)
        result = self.run_at(now=4, s=5, speed=0)
        self.assertEqual(result["reason"], "CLOCK_REGRESSION")
        self.assertNotIn("hill", result["completed_missions"])

    def test_missed_update_interval_does_not_count_as_observed_hold(self):
        self.run_at(now=0, s=5, speed=0)
        result = self.run_at(now=3, s=5, speed=0)
        self.assertTrue(result["stop_requested"])
        self.assertNotIn("hill", result["completed_missions"])

    def test_configuration_cannot_relax_mandatory_rules(self):
        for rule, value in (("hill_hold_s", 2.9),
                            ("hill_rollback_limit_m", 0.6), ("mission_deadline_s", 500),
                            ("hill_clearance_timeout_s", 31), ("no_motion_timeout_s", 61),
                            ("parking_stable_observations", 0)):
            with self.subTest(rule=rule), self.assertRaises(ValueError):
                MissionEngine({"rules": {rule: value}})

    def test_paired_hill_markers_target_arc_length_midpoint_without_extra_inset(self):
        marks = {'1_left': {'hill_zone_start_s': 4.0, 'hill_zone_end_s': 5.0}}
        result = self.poll(s=4.5, raw_s=4.5, speed=0, landmarks=marks)
        self.assertEqual(result['hill_target_s'], 4.5)
        self.assertEqual(result['completed_missions']['hill'], 3.0)
        self.assertEqual(result['hill_hold_elapsed_s'], 3.0)
        cleared = self.run_at(now=3.1, s=5.01, landmarks=marks)
        self.assertIn('hill_clearance', cleared['completed_missions'])

    def test_partial_or_reversed_hill_pair_cannot_fall_back_to_legacy(self):
        for end in (None, 3.0, 4.0, float('nan'), 21):
            snap = self.snap(s=4.5, speed=0)
            snap['landmarks']['1_left'].update(hill_zone_start_s=4., hill_zone_end_s=end)
            result = self.engine.update(snap)
            self.assertEqual(result['phase'], 'UNAVAILABLE')
            self.assertTrue(result['stop_requested'])

    def test_small_hill_rollback_and_lateral_drift_reset_hold(self):
        for change in ({'raw_s': 4.97}, {'x': 5., 'y': .03}):
            with self.subTest(change=change):
                self.engine = MissionEngine()
                self.poll(end=2.75, s=5, raw_s=5, x=5, y=0, speed=0)
                values = dict(s=5, raw_s=5, x=5, y=0, speed=0)
                values.update(change)
                result = self.run_at(now=3, **values)
                self.assertNotIn('hill', result['completed_missions'])
                self.assertEqual(result['hill_hold_elapsed_s'], 0)
                result = self.poll(start=3.25, end=6, **values)
                self.assertIn('hill', result['completed_missions'])

    def test_negative_speed_never_completes_hill_hold_even_below_deadband(self):
        result = self.poll(s=5, raw_s=5, speed=-.001)
        self.assertNotIn('hill', result['completed_missions'])
        self.assertEqual(result['hill_hold_elapsed_s'], 0)

    def test_exact_half_metre_hill_rollback_is_a_fault(self):
        self.run_at(now=0, s=5, raw_s=5, speed=0)
        result = self.run_at(now=.1, s=5, raw_s=4.5, speed=0)
        self.assertEqual(result['reason'], 'HILL_ROLLBACK_LIMIT_EXCEEDED')


if __name__ == "__main__":
    unittest.main()
