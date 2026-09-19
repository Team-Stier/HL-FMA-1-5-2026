#!/usr/bin/env python3
"""Rules regression tests using fake time and explicit perception snapshots."""

import json
import math
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
            "stop_line_s": 10.0,
            "parking_confirm_s": 18.0, "parking_exit_s": 19.0,
        }
        value = {
            "now": now, "healthy": True, "reason": "", "route": route, "decision_id": 1,
            "section": section, "s": 0.0, "length": 20.0, "at_end": False,
            "speed": 1.0, "yaw": 0.0, "calibrated": True,
            "landmarks": {route: marks}, "path_ready": True, "collision": False,
            "finish_branch_s": 13.258,
            "signal": {"stamp": now, "route": route, "value": "UNKNOWN"},
            "lane": {"stamp": now, "route": route,
                     "left": "UNKNOWN", "right": "UNKNOWN"},
            "parking": {"stamp": now, "left": "UNKNOWN", "right": "UNKNOWN"},
        }
        value.update(changes)
        return value

    def run_at(self, section=1, **kwargs):
        return self.engine.update(self.snap(section, **kwargs))

    def test_normal_and_parking_speed_limits(self):
        normal = self.run_at(8, now=0.0, s=1.0)
        parking = self.run_at(5, now=0.1, s=1.0)
        self.assertAlmostEqual(normal["speed_limit"] * 3.6, 8.0)
        self.assertAlmostEqual(parking["speed_limit"] * 3.6, 6.0)

    def test_hongik_speed_limits_command_eight_kph(self):
        config_path = os.path.join(os.path.dirname(__file__), "..", "config", "missions_hongik.json")
        with open(config_path, encoding="utf-8") as config_file:
            config = json.load(config_file)
        for zone, speed_mps in config["speeds"].items():
            with self.subTest(zone=zone):
                self.assertEqual(min(8, math.floor(speed_mps * 3.6)), 8)

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
        branches = {"t": "left", "parallel": "left"}
        branches[kind] = side
        self.engine = MissionEngine({"parking_branches": branches})
        section = 4 if kind == "t" else 9
        return self.run_at(section, now=start)

    def test_hill_releases_three_and_half_seconds_after_first_stop(self):
        self.engine = MissionEngine({'rules': {'hill_hold_s': 3.5}})
        self.assertTrue(self.run_at(now=0.0, s=5.0, speed=0.0)["stop_requested"])
        self.poll(start=0.25, end=3.25, s=5.0, speed=0.0)
        self.assertTrue(self.run_at(now=3.499, s=5.0, speed=0.0)["stop_requested"])
        released = self.run_at(now=3.5, s=5.0, speed=0.0)
        self.assertFalse(released["stop_requested"])
        self.assertEqual(released["completed_missions"]["hill"], 3.5)
        self.assertEqual(self.run_at(now=4.0, s=20.0, at_end=True)["next_route"], "2")

    def test_hill_stop_accepts_lateral_offset_within_longitudinal_tolerance(self):
        for s in (4.9, 5.0, 5.1):
            with self.subTest(s=s):
                self.engine = MissionEngine()
                for i in range(13):
                    snap = self.snap(now=i*.25, s=s, raw_s=s,
                                     x=s, y=3.0, speed=-.01)
                    snap['landmarks']['1_left'].update(
                        hill_zone_start_s=4., hill_zone_end_s=6.)
                    result = self.engine.update(snap)
                self.assertIn('hill', result['completed_missions'])
                self.assertFalse(result['stop_requested'])

    def test_hill_zone_keeps_approach_before_target_and_holds_after(self):
        for s in (4.2, 5.8):
            self.engine = MissionEngine()
            snap = self.snap(s=s, raw_s=s, speed=0.0)
            snap['landmarks']['1_left'].update(
                hill_zone_start_s=4., hill_zone_end_s=6.)
            result = self.engine.update(snap)
            self.assertNotIn('hill', result['completed_missions'])
            self.assertEqual(result['reason'] == 'HILL_REQUIRED_HOLD', s > 5.2)

    def test_hill_hold_releases_after_three_and_half_seconds_outside_zone(self):
        self.engine = MissionEngine({'rules': {'hill_hold_s': 3.5}})
        marks = {'hill_zone_start_s': 4., 'hill_zone_end_s': 6.}
        self.run_at(now=0, s=5, raw_s=5, speed=1,
                    landmarks={'1_left': marks})
        self.assertTrue(self.run_at(now=1, s=7, raw_s=7, speed=0,
                                    landmarks={'1_left': marks})['stop_requested'])
        self.poll(start=1.25, end=4.25, s=7, raw_s=7, speed=0,
                  landmarks={'1_left': marks})
        self.assertTrue(self.run_at(now=4.49, s=7, raw_s=7, speed=0,
                                    landmarks={'1_left': marks})['stop_requested'])
        released = self.run_at(now=4.5, s=7, raw_s=7, speed=0,
                               landmarks={'1_left': marks})
        self.assertFalse(released['stop_requested'])
        self.assertEqual(released['completed_missions']['hill'], 4.5)

    def test_hill_movement_does_not_reset_hold(self):
        self.engine = MissionEngine({'rules': {'hill_hold_s': 3.5}})
        self.run_at(now=0, s=5.0, speed=0.0)
        self.run_at(now=2, s=5.0, speed=0.6)
        self.run_at(now=3, s=5.0, speed=0.0)
        self.assertTrue(self.run_at(now=3.49, s=5.0, speed=0.0)["stop_requested"])
        self.assertFalse(self.run_at(now=3.5, s=5.0, speed=0.0)["stop_requested"])

    def test_localization_loss_does_not_reset_hill_hold(self):
        self.engine = MissionEngine({'rules': {'hill_hold_s': 3.5}})
        self.run_at(now=0, s=5.0, speed=0.0)
        lost = self.run_at(now=2, s=5.0, speed=0.0, healthy=False, reason="STALE_ODOM")
        self.assertEqual(lost["reason"], "STALE_ODOM")
        self.run_at(now=3, s=5.0, speed=0.0)
        self.assertFalse(self.run_at(now=3.5, s=5.0, speed=0.0)["stop_requested"])

    def test_collision_does_not_reset_hill_hold(self):
        self.run_at(now=0, s=5.0, speed=0.0)
        self.run_at(now=2, s=5.0, speed=0.0, collision=True)
        self.run_at(now=3, s=5.0, speed=0.0)
        self.assertFalse(self.run_at(now=3.0, s=5.0, speed=0.0)["stop_requested"])

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

    def test_hill_stop_after_target_requires_hold(self):
        result = self.run_at(s=6.0)
        self.assertEqual(result["reason"], "HILL_REQUIRED_HOLD")
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

    def test_signal_stops_tracking_with_half_metre_remaining(self):
        self.assertFalse(self.run_at(2, now=0.0, s=9.49)["stop_requested"])
        result = self.run_at(2, now=0.1, s=9.5)
        self.assertTrue(result["stop_requested"])
        self.assertEqual(result["phase"], "WAIT_SIGNAL")

    def test_signal_wait_latches_across_stop_threshold_position_jitter(self):
        for i in range(201):
            # Regression: crossing 9.8 used to alternate APPROACH/WAIT_SIGNAL
            # and reset the timeout every time the estimate moved backward.
            s = 9.85 if i % 2 == 0 else 9.75
            result = self.run_at(2, now=i*.1, s=s, speed=0.0)
            if i < 200:
                self.assertTrue(result['stop_requested'])
                self.assertEqual(result['phase'], 'WAIT_SIGNAL')
            self.assertAlmostEqual(result['traffic_wait_elapsed_s'], i*.1)
        self.assertEqual(result['phase'], 'CROSSING')
        self.assertFalse(result['virtual_stop']['active'])

    def test_signal_wait_does_not_start_timer_before_first_standstill(self):
        for i in range(201):
            result = self.run_at(2, now=i*.1, s=9.9, speed=1.0)
        self.assertEqual(result['traffic_wait_elapsed_s'], 0.0)
        self.assertTrue(result['stop_requested'])

    def test_signal_wait_green_authorizes_before_line_after_stopping(self):
        self.run_at(2, now=1, s=9.9, speed=0.0)
        result = self.run_at(2, now=1.1, s=9.75, speed=0.0,
                             signal={'stamp': 1.1, 'route': '2', 'value': 'GREEN'})
        self.assertFalse(result['stop_requested'])
        red = self.run_at(2, now=1.2, s=9.75, speed=0.0)
        self.assertFalse(red['stop_requested'])

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

    def test_intersection_completes_at_rddf_end_and_reports_30_second_limit(self):
        self.run_at(2, now=0, s=10, signal={"stamp": 0, "route": "2", "value": "GREEN"})
        not_clear = self.run_at(2, now=1, s=15)
        self.assertNotIn("intersection:2", not_clear["completed_missions"])
        late_clear = self.run_at(2, now=30.1, s=20, at_end=True)
        self.assertIn("INTERSECTION_CLEARANCE_TIMEOUT", late_clear["diagnostics"])
        self.assertIn("intersection:2", late_clear["completed_missions"])

    def test_static_requires_local_path_and_has_no_rddf_fallback(self):
        result = self.run_at(3, path_ready=False, at_end=True, s=20)
        self.assertEqual(result["path_mode"], "LOCAL")
        self.assertTrue(result["stop_requested"])
        self.assertIsNone(result["next_route"])

    def test_default_parallel_parking_branch_is_left_without_observation(self):
        result = self.run_at(9, s=20, at_end=True)
        self.assertEqual(result["selected_branch"], "left")
        self.assertEqual(result["next_route"], "10_parallel-left-in")

    def test_parking_observation_does_not_override_configured_branch(self):
        for now in (0.0, 0.1, 0.2, 0.3):
            result = self.run_at(9, now=now, parking={"stamp": 0, "left": "CLEAR", "right": "BLOCKED"})
        self.assertEqual(result["selected_branch"], "left")

    def test_configured_parking_branch_is_stable_and_latched(self):
        result = self.select_parking("parallel", "right")
        self.assertEqual(result["selected_branch"], "right")
        changed = self.run_at(9, now=0.3, s=20, at_end=True,
                              parking={"stamp": 0.3, "left": "CLEAR", "right": "BLOCKED"})
        self.assertEqual(changed["selected_branch"], "right")
        self.assertEqual(changed["next_route"], "10_parallel-right-in")
        clear = self.run_at(9, now=0.4, s=20, at_end=True,
                            parking={"stamp": 0.4, "left": "BLOCKED", "right": "CLEAR"})
        self.assertEqual(clear["next_route"], "10_parallel-right-in")

    def test_parallel_left_uses_forward_then_reverse_rddf(self):
        self.select_parking("parallel", "left")
        forward = self.run_at(10, route="10_parallel-left-in", now=1, s=5)
        self.assertEqual((forward["path_mode"], forward["direction"]), ("RDDF", 1))
        self.assertAlmostEqual(forward["parking_leg_target_s"], 9.337439695228316)
        moving = self.run_at(10, route="10_parallel-left-in", now=1.1,
                             s=9.2, speed=.6)
        self.assertEqual((moving["reason"], moving["direction"]),
                         ("PARALLEL_GEAR_CHANGE", 1))
        switched = self.poll(10, start=1.2, end=2.2,
                             route="10_parallel-left-in", s=9.2, speed=0)
        self.assertEqual((switched["reason"], switched["direction"]),
                         ("PARALLEL_GEAR_CHANGE", -1))
        reversing = self.run_at(10, route="10_parallel-left-in", now=2.3,
                                s=10, speed=-.5)
        self.assertFalse(reversing["stop_requested"])
        self.assertEqual(reversing["direction"], -1)

    def test_parallel_right_uses_forward_reverse_forward_rddf(self):
        self.select_parking("parallel", "right")
        route = "10_parallel-right-in"
        self.assertEqual(self.run_at(10, route=route, now=1, s=2)["direction"], 1)
        first = self.poll(10, start=1.1, end=2.1, route=route, s=6.5, speed=0)
        self.assertEqual((first["direction"], first["parking_leg_index"]), (-1, 1))
        self.assertEqual(self.run_at(10, route=route, now=2.2, s=10,
                                     speed=-.5)["direction"], -1)
        second = self.poll(10, start=2.3, end=3.3, route=route, s=16.4, speed=0)
        self.assertEqual((second["direction"], second["parking_leg_index"]), (1, 2))
        self.assertFalse(self.run_at(10, route=route, now=3.4, s=17,
                                     speed=.5)["stop_requested"])

    def test_parallel_out_routes_reverse_then_forward(self):
        for side, route, change in (
            ("left", "11-parallel-left-out", .7236489020465036),
            ("right", "11_parallel-right-out", 2.237988918723955),
        ):
            with self.subTest(side=side):
                self.select_parking("parallel", side)
                reverse = self.run_at(11, route=route, now=1, s=0, speed=-.2)
                self.assertEqual((reverse["path_mode"], reverse["direction"]), ("RDDF", -1))
                switched = self.poll(11, start=1.1, end=2.1, route=route,
                                     s=change - .1, speed=0)
                self.assertEqual(switched["direction"], 1)
                forward = self.run_at(11, route=route, now=2.2, s=change + .5)
                self.assertFalse(forward["stop_requested"])
                self.assertEqual(forward["direction"], 1)
                complete = self.run_at(11, route=route, now=2.3, s=20, at_end=True)
                self.assertEqual(complete["next_route"], "12")

    def test_parallel_profile_can_start_mid_route_for_section_testing(self):
        self.select_parking("parallel", "right")
        reverse = self.run_at(10, route="10_parallel-right-in", now=1, s=10)
        self.assertEqual(reverse["direction"], -1)
        self.engine = MissionEngine({"parking_branches": {"t": "left", "parallel": "right"}})
        forward = self.run_at(10, route="10_parallel-right-in", now=1, s=17)
        self.assertEqual(forward["direction"], 1)

    def test_t_parking_uses_reverse_and_forward_rddf_without_maneuver(self):
        self.select_parking("t")
        reversing = self.run_at(5, now=1, route="5_T-left-in", s=5, speed=-0.5)
        self.assertFalse(reversing["stop_requested"])
        self.assertEqual((reversing["path_mode"], reversing["direction"]), ("RDDF", -1))
        entry_end = self.poll(5, start=1.1, end=2.1, route="5_T-left-in", s=20,
                              at_end=True, speed=0)
        self.assertTrue(entry_end["stop_requested"])
        self.assertEqual(entry_end["next_route"], "6-T-left-out")

        forward = self.run_at(6, now=2.2, route="6-T-left-out", s=0, speed=0)
        self.assertFalse(forward["stop_requested"], forward)
        self.assertEqual((forward["path_mode"], forward["direction"]), ("RDDF", 1))
        exited = self.run_at(6, now=2.3, route="6-T-left-out", s=20, at_end=True)
        self.assertEqual(exited["next_route"], "7")

    def test_t_reverse_handoff_brakes_until_continuous_standstill(self):
        self.engine = MissionEngine({
            "parking_branches": {"t": "left", "parallel": "left"},
            "rules": {"standstill_speed_mps": 0.5,
                      "parking_hold_s": 0.5},
        })
        self.run_at(4, now=1.0, s=10.0,
                    signal={"stamp": 1.0, "route": "4", "value": "GREEN"})

        rolling = self.run_at(4, now=1.1, s=20.0, at_end=True, speed=0.6)
        self.assertTrue(rolling["stop_requested"])
        self.assertEqual((rolling["phase"], rolling["reason"], rolling["direction"]),
                         ("WAIT_GEAR_CHANGE", "T_PARKING_REVERSE_HOLD", 1))
        self.assertIsNone(rolling["next_route"])

        for now in (1.2, 1.4, 1.6):
            held = self.run_at(4, now=now, s=20.0, at_end=True, speed=0.5)
        self.assertTrue(held["stop_requested"])
        self.assertEqual(held["next_route"], None)
        ready = self.run_at(4, now=1.7, s=20.0, at_end=True, speed=0.5)
        self.assertTrue(ready["stop_requested"])
        self.assertEqual(ready["next_route"], "5_T-left-in")

        reverse = self.run_at(5, now=1.8, route="5_T-left-in", s=0.0, speed=0.0)
        self.assertFalse(reverse["stop_requested"])
        self.assertEqual((reverse["path_mode"], reverse["direction"], reverse["phase"]),
                         ("RDDF", -1, "REVERSE_ENTRY"))

    def test_t_reverse_handoff_stays_latched_when_endpoint_projection_jitters(self):
        self.engine = MissionEngine({
            "parking_branches": {"t": "left", "parallel": "left"},
            "rules": {"parking_hold_s": 0.5},
        })
        first = self.run_at(4, now=1.0, s=20.0, at_end=True, speed=0.6,
                            signal={"stamp": 1.0, "route": "4", "value": "GREEN"})
        self.assertEqual(first["reason"], "T_PARKING_REVERSE_HOLD")

        # The next projection is no longer at_end, but the stop and dwell must
        # remain active instead of returning to CROSSING motion.
        jittered = self.run_at(4, now=1.1, s=19.0, at_end=False, speed=0.0)
        self.assertTrue(jittered["stop_requested"])
        self.assertEqual(jittered["reason"], "T_PARKING_REVERSE_HOLD")
        self.assertIsNone(jittered["next_route"])
        ready = self.run_at(4, now=1.6, s=19.0, at_end=False, speed=0.0)
        self.assertTrue(ready["stop_requested"])
        self.assertEqual(ready["next_route"], "5_T-left-in")

    def test_t_entry_does_not_request_forward_exit_while_still_rolling(self):
        self.select_parking("t")
        rolling = self.run_at(5, now=1.0, route="5_T-left-in", s=20,
                              at_end=True, speed=-0.6)
        self.assertTrue(rolling["stop_requested"])
        self.assertIsNone(rolling["next_route"])
        self.assertNotIn("parking:t:entry", rolling["completed_missions"])

        stopped = self.poll(5, start=1.1, end=2.1, route="5_T-left-in", s=20,
                            at_end=True, speed=0.0)
        self.assertTrue(stopped["stop_requested"])
        self.assertEqual(stopped["next_route"], "6-T-left-out")
        self.assertIn("parking:t:entry", stopped["completed_missions"])

    def test_t_entry_endpoint_stop_stays_latched_across_projection_jitter(self):
        self.select_parking("t")
        first = self.run_at(5, now=1.0, route="5_T-left-in", s=20,
                            at_end=True, speed=-0.6)
        self.assertTrue(first["stop_requested"])
        jittered = self.run_at(5, now=1.1, route="5_T-left-in", s=19,
                               at_end=False, speed=-0.6)
        self.assertTrue(jittered["stop_requested"])
        self.assertIsNone(jittered["next_route"])
        stopped = self.poll(5, start=1.2, end=2.2, route="5_T-left-in", s=19,
                            at_end=False, speed=0.0)
        self.assertTrue(stopped["stop_requested"])
        self.assertEqual(stopped["next_route"], "6-T-left-out")

    def test_parallel_ignores_removed_parking_maneuver_input(self):
        self.select_parking("parallel", "left")
        result = self.run_at(10, route="10_parallel-left-in", now=1, s=5,
                             parking_maneuver={"direction": -1})
        self.assertFalse(result["stop_requested"])
        self.assertEqual(result["direction"], 1)

    def test_parking_branch_cannot_be_invented_by_route_change(self):
        self.select_parking("t", "left")
        result = self.run_at(5, now=1, route="5_T-right-in")
        self.assertEqual(result["reason"], "PARKING_BRANCH_NOT_AUTHORIZED")

    def test_parking_cannot_reselect_from_observation(self):
        self.select_parking("parallel", "left")
        for now in (0.3, 0.4, 0.5):
            result = self.run_at(9, now=now, s=20, at_end=True,
                                 parking={"stamp": now, "left": "BLOCKED", "right": "CLEAR"})
        self.assertEqual(result["selected_branch"], "left")
        self.assertEqual(result["next_route"], "10_parallel-left-in")

    def test_committed_parking_branch_cannot_switch_during_manoeuvre(self):
        self.select_parking("parallel", "left")
        self.run_at(10, now=0.3, s=1)
        for now in (0.4, 0.5, 0.6):
            result = self.run_at(9, now=now, s=20, at_end=True,
                                 parking={"stamp": now, "left": "BLOCKED", "right": "CLEAR"})
        self.assertEqual(result["selected_branch"], "left")
        self.assertEqual(result["next_route"], "10_parallel-left-in")

    def test_t_and_parallel_parking_branches_are_independently_configurable(self):
        self.engine = MissionEngine({
            "parking_branches": {"t": "right", "parallel": "left"},
            "rules": {"parking_hold_s": 0.1},
        })
        self.run_at(4, s=20, at_end=True, speed=0.0,
                    signal={"stamp": 0, "route": "4", "value": "GREEN"})
        t_result = self.run_at(4, now=0.1, s=20, at_end=True, speed=0.0)
        self.assertEqual(t_result["selected_branch"], "right")
        self.assertEqual(t_result["next_route"], "5_T-right-in")
        parallel_result = self.run_at(9, now=0.1, s=20, at_end=True)
        self.assertEqual(parallel_result["selected_branch"], "left")
        self.assertEqual(parallel_result["next_route"], "10_parallel-left-in")

    def test_invalid_parking_branch_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            MissionEngine({"parking_branches": {"t": "camera", "parallel": "left"}})

    def test_t_exit_direct_start_uses_forward_rddf(self):
        self.select_parking("t", "left")
        result = self.run_at(6, now=0.3, s=0, speed=0)
        self.assertFalse(result["stop_requested"])
        self.assertEqual((result["path_mode"], result["direction"]), ("RDDF", 1))

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
        for section in (1, 2, 4, 7):
            result = self.run_at(section, landmarks={})
            self.assertTrue(result["stop_requested"])
            self.assertTrue(result["reason"].startswith("CALIBRATION_REQUIRED:"))
        self.assertFalse(self.run_at(5, landmarks={})["stop_requested"])
        self.assertFalse(self.run_at(10, landmarks={})["stop_requested"])
        self.assertFalse(self.run_at(11, landmarks={})["stop_requested"])
        result = self.run_at(3, calibrated=False)
        self.assertTrue(result["stop_requested"])

    def test_down_sign_selects_left_finish_branch_at_internal_junction(self):
        for now in (0.1, 0.2, 0.3):
            before = self.run_at(12, now=now, s=12.9, lane={
                "stamp": now, "route": "12", "left": "DOWN", "right": "X"})
        self.assertEqual(before["selected_branch"], "left")
        self.assertIsNone(before["next_route"])
        result = self.run_at(12, now=0.4, s=13.258, lane={
            "stamp": 0.4, "route": "12", "left": "DOWN", "right": "X"})
        self.assertEqual(result["next_route"], "13_left")

    def test_down_sign_selects_right_finish_branch_until_route_end(self):
        for now in (0.1, 0.2, 0.3):
            before = self.run_at(12, now=now, s=12.9, lane={
                "stamp": now, "route": "12", "left": "X", "right": "DOWN"})
        self.assertEqual(before["selected_branch"], "right")
        self.assertIsNone(before["next_route"])
        result = self.run_at(12, now=0.4, s=20, at_end=True, lane={
            "stamp": 0.4, "route": "12", "left": "X", "right": "DOWN"})
        self.assertEqual(result["next_route"], "13_right")

    def test_unconfirmed_finish_sign_uses_configured_fallback_at_fork(self):
        result = self.run_at(12, now=0.3, s=13.2)
        self.assertFalse(result["stop_requested"])
        self.assertEqual(result["selected_branch"], "left")
        self.assertEqual(result["reason"], "FINISH_SIGN_FALLBACK")

    def test_finish_sign_fallback_branch_is_configurable(self):
        self.engine = MissionEngine({"finish_fallback_branch": "right"})
        result = self.run_at(12, now=0.3, s=13.2)
        self.assertEqual(result["selected_branch"], "right")
        self.assertEqual(result["phase"], "FINISH_FALLBACK_SELECTED")

    def test_invalid_finish_branch_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            MissionEngine({"finish_fallback_branch": "camera"})

    def test_finish_occurs_at_extended_route_endpoint(self):
        before = self.run_at(13, now=1, s=22.7, length=23.0)
        self.assertFalse(before["stop_requested"])
        self.assertNotIn("finish", before["completed_missions"])
        finish = self.run_at(13, now=2, s=22.8, length=23.0)
        self.assertEqual(finish["reason"], "COURSE_COMPLETE")
        self.assertEqual(finish["completed_missions"]["finish"], 2)

    def test_race_clock_freezes_when_rear_axle_finishes(self):
        self.run_at(12, now=0, s=0, speed=1)
        finished = self.run_at(13, now=10, s=20)
        self.assertEqual(finished["elapsed_time_s"], 10)
        parked = self.run_at(13, now=900, s=20, speed=0)
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

    def test_elapsed_time_counts_without_intermediate_updates(self):
        self.run_at(now=0, s=5, speed=0)
        result = self.run_at(now=3, s=5, speed=0)
        self.assertFalse(result["stop_requested"])
        self.assertIn("hill", result["completed_missions"])

    def test_configuration_cannot_relax_mandatory_rules(self):
        for rule, value in (("hill_hold_s", 2.9),
                            ("hill_rollback_limit_m", 1.6), ("mission_deadline_s", 500),
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

    def test_small_hill_rollback_and_lateral_drift_do_not_reset_hold(self):
        for change in ({'raw_s': 4.97}, {'x': 5., 'y': .03}):
            with self.subTest(change=change):
                self.engine = MissionEngine()
                self.poll(end=2.75, s=5, raw_s=5, x=5, y=0, speed=0)
                values = dict(s=5, raw_s=5, x=5, y=0, speed=0)
                values.update(change)
                result = self.run_at(now=3, **values)
                self.assertIn('hill', result['completed_missions'])
                self.assertEqual(result['hill_hold_elapsed_s'], 3)

    def test_hill_accepts_signed_speed_within_standstill_range(self):
        for speed in (-.5, -.1, 0., .1, .5):
            with self.subTest(speed=speed):
                self.engine = MissionEngine()
                result = self.poll(s=5, raw_s=5, speed=speed)
                self.assertIn('hill', result['completed_missions'])
                self.assertFalse(result['stop_requested'])

    def test_hill_rejects_speed_outside_standstill_range(self):
        for speed in (-.501, .501):
            with self.subTest(speed=speed):
                self.engine = MissionEngine()
                result = self.poll(s=5, raw_s=5, speed=speed)
                self.assertNotIn('hill', result['completed_missions'])
                self.assertEqual(result['hill_hold_elapsed_s'], 0)

    def test_exact_half_metre_hill_rollback_is_a_fault(self):
        self.run_at(now=0, s=5, raw_s=5, speed=0)
        result = self.run_at(now=.1, s=5, raw_s=4.5, speed=0)
        self.assertEqual(result['reason'], 'HILL_ROLLBACK_LIMIT_EXCEEDED')


if __name__ == "__main__":
    unittest.main()
