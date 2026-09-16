#!/usr/bin/env python3
"""Pure Python checks; no ROS runtime is needed."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from stier_state_manager.geometry import (  # noqa: E402
    Route, RouteTracker, braking_clearance, corridor_status, scan_to_geometry,
    transform_scan_to_geometry,
)


def line(name, start=0.0, end=10.0, y=0.0, direction=1):
    return Route(name, [(start, y, 0), (end, y, 0)], direction=direction)


class RouteTests(unittest.TestCase):
    def test_arrival_and_transition_agree_at_decimal_tolerance_boundary(self):
        routes = {'1_right': line('1_right'), '2': line('2', 10, 20)}
        tracker = RouteTracker(routes)
        tracker.update(0, 0, 0, 1)
        tracker.update(8, 0, 0, 2)
        self.assertTrue(tracker.update(9.2, 0, 0, 3)['at_end'])
        self.assertTrue(tracker.transition('2'), tracker.transition_reason)

    def test_lengths_interpolation_and_slice(self):
        route = Route("1_right", [(0, 0, 0), (3, 0, 0), (3, 4, math.pi / 2)])
        self.assertEqual(route.s, [0, 3, 7])
        self.assertEqual(route.length, 7)
        self.assertEqual(route.pose_at(1)[:2], (1, 0))
        self.assertEqual(route.slice(2, 5)[0][:2], (2, 0))
        self.assertEqual(route.slice(2, 5)[-1][:2], (3, 2))

    def test_invalid_route(self):
        with self.assertRaises(ValueError):
            Route("1_right", [(0, 0, 0), (math.nan, 0, 0)])
        with self.assertRaises(ValueError):
            Route("1_right", [(0, 0, 0), (0, 0, 0)])

    def test_initial_acquisition_cannot_pick_middle(self):
        tracker = RouteTracker({"1_right": line("1_right", end=100)})
        result = tracker.update(50, 0, 0, 0)
        self.assertFalse(result["healthy"])
        self.assertEqual(result["reason"], "outside_start_acquisition")
        self.assertEqual(result["s"], 0)

    def test_matched_progress_allows_independent_middle_acquisition(self):
        tracker = RouteTracker({"7": line("7", end=100)}, "7")
        tracker.set_initial_progress(50)
        result = tracker.update(50, 0, 0, 1)
        self.assertTrue(result["healthy"], result)
        self.assertEqual(result["route"], "7")
        self.assertAlmostEqual(result["s"], 50)

    def test_invalid_matched_progress_is_rejected(self):
        tracker = RouteTracker({"7": line("7", end=100)}, "7")
        for value in (-1, 101, math.nan):
            with self.subTest(value=value), self.assertRaises(ValueError):
                tracker.set_initial_progress(value)

    def test_progress_is_monotonic_but_raw_retains_rollback(self):
        tracker = RouteTracker({"1_right": line("1_right")})
        tracker.update(0, 0, 0, 0)
        tracker.update(2, 0, 0, 1)
        result = tracker.update(1.7, 0, 0, 1.1)
        self.assertTrue(result["healthy"])
        self.assertEqual(result["s"], 2)
        self.assertAlmostEqual(result["raw_s"], 1.7)

    def test_position_jump_rejected(self):
        tracker = RouteTracker({"1_right": line("1_right", end=100)})
        tracker.update(0, 0, 0, 0)
        result = tracker.update(30, 0, 0, 0.1)
        self.assertFalse(result["healthy"])
        self.assertEqual(result["reason"], "position_jump")
        self.assertEqual(result["s"], 0)

    def test_elapsed_outage_does_not_allow_large_jump(self):
        tracker = RouteTracker({"1_right": line("1_right", end=100)})
        tracker.update(0, 0, 0, 0)
        self.assertFalse(tracker.update(30, 0, 0, 100)["healthy"])

    def test_unrelated_crossing_never_changes_section(self):
        routes = {"1_right": line("1_right"), "7": line("7")}
        tracker = RouteTracker(routes)
        tracker.update(0, 0, 0, 0)
        self.assertEqual(tracker.update(4, 0, 0, 1)["route"], "1_right")
        self.assertFalse(tracker.transition("7"))

    def test_same_route_crossing_uses_progress_window(self):
        route = Route("1_right", [(0, 0, 0), (3, 0, 0), (3, 20, math.pi / 2),
                                  (1, 20, math.pi), (1, -1, -math.pi / 2)])
        tracker = RouteTracker({"1_right": route})
        tracker.update(0, 0, 0, 0)
        result = tracker.update(1, 0, 0, .5)
        self.assertTrue(result["healthy"])
        self.assertAlmostEqual(result["s"], 1)

    def test_reverse_vehicle_heading(self):
        routes = {"5_T-left-in": line("5_T-left-in", direction=-1)}
        tracker = RouteTracker(routes, "5_T-left-in")
        self.assertFalse(tracker.update(0, 0, 0, 0)["healthy"])
        self.assertTrue(tracker.update(0, 0, math.pi, 0)["healthy"])
        self.assertTrue(tracker.update(2, 0, math.pi, 1)["healthy"])

    def test_validated_parking_leg_can_override_default_body_direction(self):
        routes = {"5_T-left-in": line("5_T-left-in", direction=1)}
        tracker = RouteTracker(routes, "5_T-left-in")
        self.assertTrue(tracker.update(0, 0, math.pi, 0, body_direction=-1)["healthy"])
        self.assertFalse(tracker.update(0, 0, math.pi, 1, body_direction=0)["healthy"])

    def test_small_endpoint_overshoot_can_transition(self):
        routes = {"1_right": line("1_right", end=3), "2": line("2", start=3, end=6)}
        tracker = RouteTracker(routes)
        tracker.update(0, 0, 0, 0)
        self.assertTrue(tracker.update(3.4, 0, 0, 1)["at_end"])
        self.assertTrue(tracker.transition("2"))
        self.assertEqual(tracker.route_name, "2")
        self.assertTrue(tracker.update(3.4, 0, 0, 1.1)["healthy"])

    def test_early_transition_is_rejected(self):
        routes = {"1_right": line("1_right"), "2": line("2", start=10, end=20)}
        tracker = RouteTracker(routes)
        tracker.update(0, 0, 0, 0)
        self.assertFalse(tracker.transition("2"))
        self.assertEqual(tracker.transition_reason, "boundary_not_reached")

    def test_parking_requires_matching_exit_branch(self):
        routes = {"5_T-left-in": line("5_T-left-in", end=2),
                  "6-T-left-out": line("6-T-left-out", start=2, end=4),
                  "6_T-right-out": line("6_T-right-out", start=2, end=4)}
        tracker = RouteTracker(routes, "5_T-left-in")
        tracker.update(0, 0, 0, 0)
        tracker.update(2, 0, 0, 1)
        self.assertFalse(tracker.transition("6_T-right-out"))
        self.assertTrue(tracker.transition("6-T-left-out"))

    def test_left_lane_fork_is_inside_section_twelve(self):
        routes = {"12": line("12", end=20), "13_left": line("13_left", start=13.258, end=30),
                  "13_right": line("13_right", start=20, end=30)}
        tracker = RouteTracker(routes, "12")
        tracker.update(0, 0, 0, 0)
        tracker.update(6, 0, 0, 1)
        tracker.update(13.258, 0, 0, 2)
        self.assertFalse(tracker.transition("13_right"))
        self.assertTrue(tracker.transition("13_left"))

    def test_left_lane_fork_cannot_be_taken_after_passing_it(self):
        routes = {"12": line("12", end=20), "13_left": line("13_left", start=13.258, end=30)}
        tracker = RouteTracker(routes, "12")
        tracker.update(0, 0, 0, 0)
        tracker.update(8, 0, 0, 1)
        tracker.update(16, 0, 0, 2)
        self.assertFalse(tracker.transition("13_left"))

    def test_invalid_pose_and_time_regression(self):
        tracker = RouteTracker({"1_right": line("1_right")})
        tracker.update(0, 0, 0, 1)
        self.assertFalse(tracker.update(math.nan, 0, 0, 2)["healthy"])
        self.assertFalse(tracker.update(0, 0, 0, 0)["healthy"])


class CorridorTests(unittest.TestCase):
    def scan(self, fill=math.inf, pose=(0, 0, 0), start=-math.pi, count=721):
        return scan_to_geometry([fill] * count, start, math.pi / 360, .05, 20, pose)

    def corridor(self, points, hits=(), rays=(), **kwargs):
        return corridor_status(points, hits, rays, vehicle_width=.8, margin=.1,
                               front=.5, rear=.2, sample_step=.25, **kwargs)

    def test_no_data_is_unknown_not_clear(self):
        self.assertEqual(self.corridor([(1, 0, 0), (3, 0, 0)])["status"], "UNKNOWN")

    def test_full_observed_free_corridor_is_clear(self):
        hits, rays = self.scan()
        result = self.corridor([(1, 0, 0), (3, 0, 0)], hits, rays)
        self.assertEqual(result["status"], "CLEAR")
        self.assertEqual(result["coverage"], 1)

    def test_nan_scan_stays_unknown(self):
        hits, rays = self.scan(fill=math.nan)
        self.assertEqual(hits, [])
        self.assertEqual(rays, [])
        self.assertEqual(self.corridor([(1, 0, 0), (3, 0, 0)], hits, rays)["status"], "UNKNOWN")

    def test_inf_requires_finite_valid_sensor_max_range(self):
        hits, rays = scan_to_geometry([math.inf] * 10, 0, .1, .05, math.inf)
        self.assertEqual((hits, rays), ([], []))

    def test_out_of_view_is_unknown(self):
        hits, rays = self.scan(start=-math.pi / 4, count=181)
        self.assertEqual(self.corridor([(-3, 0, 0), (-1, 0, 0)], hits, rays)["status"], "UNKNOWN")

    def test_occlusion_is_unknown_behind_return(self):
        hits, rays = self.scan(fill=2)
        self.assertEqual(self.corridor([(4, 0, 0), (6, 0, 0)], hits, rays)["status"], "UNKNOWN")

    def test_missing_single_ray_is_not_interpolated_as_clear(self):
        ranges = [math.inf] * 721
        ranges[360] = math.nan
        hits, rays = scan_to_geometry(ranges, -math.pi, math.pi / 360, .05, 20)
        self.assertEqual(self.corridor([(1, 0, 0), (3, 0, 0)], hits, rays)["status"], "UNKNOWN")

    def test_sub_grid_missing_beam_wedge_is_not_clear(self):
        ranges = [math.inf] * 3601
        ranges[1807] = math.nan
        hits, rays = scan_to_geometry(ranges, -math.pi, math.pi / 1800, .05, 20)
        result = self.corridor([(4.013, 0.007, 0), (4.313, 0.007, 0)], hits, rays)
        self.assertEqual(result["status"], "UNKNOWN")
        self.assertLess(result["coverage"], 1)

    def test_collision_between_coarse_route_samples(self):
        result = self.corridor([(1, 0, 0), (9, 0, 0)], hits=[(5, .4)])
        self.assertEqual(result["status"], "BLOCKED")
        self.assertIsNotNone(result["nearest_obstacle_s"])

    def test_coarse_sweep_groups_never_remove_a_fine_sweep_collision(self):
        points = [(0, 0, 0), (2, 0, 0), (2, 2, math.pi / 2), (4, 2, 0)]
        for ix in range(-1, 10):
            for iy in range(-2, 7):
                hit = (ix * .5, iy * .5)
                fine = self.corridor(points, [hit], sweep_chunk_m=.25)
                if fine['status'] == 'BLOCKED':
                    coarse = self.corridor(points, [hit], sweep_chunk_m=2)
                    self.assertEqual(coarse['status'], 'BLOCKED', hit)

    def test_free_space_cache_revalidates_mutated_scan_metadata(self):
        hits, rays = self.scan()
        points = [(1, 0, 0), (3, 0, 0)]
        self.assertEqual(self.corridor(points, hits, rays)['status'], 'CLEAR')
        rays.min_range = 10
        self.assertEqual(self.corridor(points, hits, rays)['status'], 'UNKNOWN')

    def test_obstacle_outside_margin_does_not_block(self):
        hits, rays = self.scan()
        result = self.corridor([(1, 0, 0), (3, 0, 0)], [(2, .51)], rays)
        self.assertEqual(result["status"], "CLEAR")

    def test_front_overhang_is_checked(self):
        self.assertEqual(self.corridor([(1, 0, 0)], [(1.59, 0)])["status"], "BLOCKED")
        self.assertEqual(self.corridor([(1, 0, 0)], [(1.61, 0)])["status"], "UNKNOWN")

    def test_rear_overhang_is_checked(self):
        self.assertEqual(self.corridor([(1, 0, 0)], [(.71, 0)])["status"], "BLOCKED")
        self.assertEqual(self.corridor([(1, 0, 0)], [(.69, 0)])["status"], "UNKNOWN")

    def test_reverse_overhangs_follow_vehicle_orientation(self):
        self.assertEqual(self.corridor([(1, 0, 0)], [(.41, 0)], direction=-1)["status"], "BLOCKED")
        self.assertEqual(self.corridor([(1, 0, 0)], [(1.59, 0)], direction=-1)["status"], "UNKNOWN")

    def test_rotated_footprint_and_swept_turn(self):
        result = self.corridor([(0, 0, math.pi / 2)], [(0, .59)])
        self.assertEqual(result["status"], "BLOCKED")
        result = self.corridor([(0, 0, 0), (0, 0, math.pi / 2)], [(.5, .5)])
        self.assertEqual(result["status"], "BLOCKED")

    def test_scan_transform_and_finite_hit(self):
        hits, rays = scan_to_geometry([2], 0, .1, .05, 20, (10, 5, math.pi / 2))
        self.assertAlmostEqual(hits[0][0], 10)
        self.assertAlmostEqual(hits[0][1], 7)
        self.assertEqual(rays[0][:2], (10, 5))

    def test_roll_flipped_laser_uses_full_rotation(self):
        hits, rays = transform_scan_to_geometry([2], math.pi / 2, .1, .05, 20,
                                                 translation=(10, 5, .3), rotation=(1, 0, 0, 0))
        self.assertAlmostEqual(hits[0][0], 10)
        self.assertAlmostEqual(hits[0][1], 3)
        self.assertAlmostEqual(rays.endpoint_z[0], .3)
        self.assertEqual(rays.plane_normal, (0, 0, -1))

    def test_hill_pitch_projects_actual_xy_and_records_height(self):
        q = (0, math.sin(math.pi / 6), 0, math.cos(math.pi / 6))
        hits, rays = transform_scan_to_geometry([2], 0, .1, .05, 20,
                                                 translation=(10, 5, 1), rotation=q)
        self.assertAlmostEqual(hits[0][0], 11)
        self.assertAlmostEqual(hits[0][1], 5)
        self.assertAlmostEqual(rays.endpoint_z[0], 1 - math.sqrt(3))

    def test_transformed_missing_beam_keeps_adjacency(self):
        ranges = [math.inf] * 3601
        ranges[1807] = math.nan
        hits, rays = transform_scan_to_geometry(ranges, -math.pi, math.pi / 1800,
                                                 .05, 20, rotation=(1, 0, 0, 0))
        result = self.corridor([(4, 0, 0), (5, 0, 0)], hits, rays)
        self.assertEqual(result["status"], "UNKNOWN")

    def test_invalid_quaternion_rejects_evidence(self):
        self.assertEqual(transform_scan_to_geometry([2], 0, .1, .05, 20,
                                                     rotation=(0, 0, 0, 0)), ([], []))

    def test_range_cap_is_not_an_obstacle(self):
        hits, rays = scan_to_geometry([10, math.inf], 0, .1, .05, 20, max_range=5)
        self.assertEqual(hits, [])
        self.assertAlmostEqual(math.hypot(rays[0][2], rays[0][3]), 5)

    def test_sensor_near_blind_zone_is_unknown(self):
        hits, rays = self.scan()
        self.assertEqual(self.corridor([(0, 0, 0), (1, 0, 0)], hits, rays)["status"], "UNKNOWN")

    def test_known_chassis_excludes_coverage_only_never_hits(self):
        hits, rays = self.scan()
        body = [(-.2, -.4), (.5, -.4), (.5, .4), (-.2, .4)]
        result = self.corridor([(0, 0, 0), (1, 0, 0)], hits, rays, coverage_exclusion=body)
        self.assertEqual(result["status"], "CLEAR")
        result = self.corridor([(0, 0, 0), (1, 0, 0)], [(0, 0)], rays, coverage_exclusion=body)
        self.assertEqual(result["status"], "BLOCKED")

    def test_degenerate_chassis_exclusion_cannot_clear_unknown_space(self):
        for body in ([(0, 0)], [(0, 0), (1, 0), (2, 0)], [(0, 0), (1, 0), (0, math.nan)]):
            result = self.corridor([(0, 0, 0), (1, 0, 0)], coverage_exclusion=body)
            self.assertEqual(result['status'], 'UNKNOWN')
            self.assertEqual(result['reason'], 'invalid_geometry')

    def test_braking_clearance(self):
        self.assertEqual(braking_clearance(4, 2, .5, 1), 7)
        self.assertEqual(braking_clearance(-4, 2, .5, 1), 7)
        with self.assertRaises(ValueError):
            braking_clearance(4, 0, .5)


if __name__ == "__main__":
    unittest.main()
