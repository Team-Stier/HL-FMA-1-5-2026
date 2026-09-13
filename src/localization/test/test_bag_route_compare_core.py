#!/usr/bin/env python3
"""Offline contracts for manual-route loading and raw NavPVT replay comparison.

Run directly with Python; no ROS installation, master, recordings, or browser
profile is needed. All route fixtures are isolated temporary files.
"""

import copy
import csv
import importlib.util
import json
import math
from collections import deque
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest


PACKAGE_DIR = Path(__file__).resolve().parents[1]
CORE_PATH = PACKAGE_DIR / "src" / "localization_replay" / "core.py"
SPEC = importlib.util.spec_from_file_location("bag_route_compare_core_tested", CORE_PATH)
CORE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = CORE
SPEC.loader.exec_module(CORE)

ORIGIN = (37.288731, 127.1072336)
CSV_FIELDS = [
    "route_id", "route_name", "closed", "index", "latitude", "longitude",
    "east_m", "north_m", "distance_m", "path_yaw_rad",
]


def navpvt(lat=ORIGIN[0], lon=ORIGIN[1], **changes):
    values = dict(
        lat=round(lat * 1e7), lon=round(lon * 1e7), fixType=3,
        flags=1 | (2 << 6), hAcc=150, numSV=23, iTOW=123456,
    )
    values.update(changes)
    return SimpleNamespace(**values)


def csv_row(route_id="b", name="경로, B", closed="false", index=0,
            lat=ORIGIN[0], lon=ORIGIN[1], **changes):
    row = dict(
        route_id=route_id, route_name=name, closed=closed, index=index,
        latitude=lat, longitude=lon, east_m="wrong origin", north_m="NaN",
        distance_m="unused", path_yaw_rad="unused",
    )
    row.update(changes)
    return row


def editor_project():
    return {
        "version": 1, "origin": {"lat": 37.289, "lng": 127.108},
        "spacingM": 0.5,
        "routes": [
            {"id": "b", "name": "경로 B", "closed": True, "points": [
                {"lat": 37.2890486775, "lng": 127.1073736623},
                {"lat": 37.2887311234, "lng": 127.1072336789},
                {"lat": 37.2890486775, "lng": 127.1073736623},
            ]},
            {"id": "empty", "name": "빈 경로", "closed": False, "points": []},
            {"id": "a", "name": "경로 A", "closed": False, "points": [
                {"lat": 37.2888, "lng": 127.1073},
            ]},
        ],
    }


class ProjectionTest(unittest.TestCase):
    def test_default_origin_is_the_route_editor_origin(self):
        self.assertEqual(ORIGIN, CORE.DEFAULT_ORIGIN)
        self.assertEqual((0.0, 0.0), CORE.project_point(*ORIGIN, ORIGIN))

    def test_matches_editor_local_east_north_regression_coordinates(self):
        # Fixed results from the editor's WGS84 curvature projection. These
        # distinguish it from spherical Mercator, UTM, or a rotated frame.
        cases = [
            ((37.288831, 127.1073336), (8.867397426498712, 11.098306151194366)),
            ((37.2890486775, 127.1073736623), (12.41988078574227, 35.2568215222903)),
            ((37.2885, 127.1069), (-29.581637814235165, -25.63708720834423)),
        ]
        for point, expected in cases:
            with self.subTest(point=point):
                x, y = CORE.project_point(*point, ORIGIN)
                self.assertAlmostEqual(expected[0], x, places=7)
                self.assertAlmostEqual(expected[1], y, places=7)

    def test_explicit_origin_is_used(self):
        other = (36.0, 128.0)
        self.assertEqual((0.0, 0.0), CORE.project_point(*other, other))
        x, y = CORE.project_point(36.0001, 128.0001, other)
        self.assertGreater(x, 8.9)
        self.assertLess(x, 9.1)
        self.assertGreater(y, 11.0)
        self.assertLess(y, 11.2)

    def test_invalid_coordinates_fail_before_projection(self):
        for point in [(math.nan, 127.0), (37.0, math.inf), (90.1, 0.0), (0.0, -180.1)]:
            with self.subTest(point=point), self.assertRaises(ValueError):
                CORE.project_point(*point, ORIGIN)


class RouteLoadingTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="bag-route-core-test-")
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)

    def write_json(self, value, name="project.json"):
        path = self.directory / name
        path.write_text(json.dumps(value, ensure_ascii=False), encoding="utf-8")
        return path

    def write_csv(self, rows, fields=CSV_FIELDS, name="routes.csv", bom=False):
        path = self.directory / name
        with path.open("w", encoding="utf-8-sig" if bom else "utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)
        return path

    def test_json_preserves_origin_order_empty_routes_and_exact_coordinates(self):
        source = editor_project()
        path = self.write_json(source)
        original = path.read_bytes()
        project = CORE.load_routes(path)
        self.assertIsInstance(project, CORE.RouteProject)
        self.assertEqual((37.289, 127.108), project.origin)
        self.assertIsInstance(project.routes, tuple)
        self.assertEqual(["b", "empty", "a"], [route.id for route in project.routes])
        for raw, route in zip(source["routes"], project.routes):
            self.assertIsInstance(route, CORE.Route)
            self.assertEqual(raw["name"], route.name)
            self.assertIs(raw["closed"], route.closed)
            self.assertEqual(tuple((p["lat"], p["lng"]) for p in raw["points"]), route.points)
        self.assertEqual(original, path.read_bytes())

    def test_json_empty_project_remains_empty(self):
        source = editor_project()
        source["routes"] = []
        self.assertEqual((), CORE.load_routes(self.write_json(source)).routes)

    def test_json_bad_schema_is_rejected(self):
        variants = []
        for key, value in [("version", 2), ("origin", None), ("routes", {})]:
            source = editor_project()
            source[key] = value
            variants.append(source)
        source = editor_project()
        source["routes"][0]["points"] = "not points"
        variants.append(source)
        source = editor_project()
        source["routes"][0]["closed"] = "false"
        variants.append(source)
        for index, source in enumerate(variants):
            with self.subTest(index=index), self.assertRaises(ValueError):
                CORE.load_routes(self.write_json(source))

    def test_json_duplicate_route_ids_are_rejected(self):
        source = editor_project()
        source["routes"][2]["id"] = "b"
        with self.assertRaises(ValueError):
            CORE.load_routes(self.write_json(source))

    def test_json_nonfinite_out_of_range_and_missing_coordinates_are_rejected(self):
        bad_points = [
            {"lat": math.nan, "lng": 127.0}, {"lat": 37.0, "lng": math.inf},
            {"lat": 90.0001, "lng": 127.0}, {"lat": 37.0, "lng": -180.001},
            {"lat": 37.0},
        ]
        for point in bad_points:
            source = editor_project()
            source["routes"][0]["points"][1] = point
            with self.subTest(point=point), self.assertRaises(ValueError):
                CORE.load_routes(self.write_json(source))

    def test_bad_load_leaves_input_file_and_previously_loaded_project_unchanged(self):
        project = CORE.load_routes(self.write_json(editor_project(), "good.json"))
        before = copy.deepcopy(project)
        bad = self.directory / "broken.json"
        bad.write_text('{"version": 1, bad json', encoding="utf-8")
        bad_bytes = bad.read_bytes()
        with self.assertRaises(ValueError):
            CORE.load_routes(bad)
        self.assertEqual(before, project)
        self.assertEqual(bad_bytes, bad.read_bytes())

    def test_csv_groups_first_appearance_and_keeps_per_route_row_order(self):
        rows = [
            csv_row(),
            csv_row("a", "경로 A", "true", 0, 37.2888, 127.1073),
            csv_row(index=1, lat=37.2889, lon=127.1074),
            csv_row("a", "경로 A", "true", 1, 37.289, 127.1075),
        ]
        path = self.write_csv(rows, bom=True)
        original = path.read_bytes()
        project = CORE.load_routes(path)
        self.assertEqual(ORIGIN, project.origin)
        self.assertEqual(("b", "a"), tuple(route.id for route in project.routes))
        self.assertEqual("경로, B", project.routes[0].name)
        self.assertFalse(project.routes[0].closed)
        self.assertTrue(project.routes[1].closed)
        self.assertEqual((ORIGIN, (37.2889, 127.1074)), project.routes[0].points)
        self.assertEqual(((37.2888, 127.1073), (37.289, 127.1075)), project.routes[1].points)
        self.assertEqual(original, path.read_bytes())

    def test_csv_declared_origin_and_lat_lon_override_untrusted_local_columns(self):
        declared = (37.0, 127.0)
        project = CORE.load_routes(self.write_csv([csv_row()]), csv_origin=declared)
        self.assertEqual(declared, project.origin)
        self.assertEqual((ORIGIN,), project.routes[0].points)
        x, y = CORE.project_point(*project.routes[0].points[0], project.origin)
        self.assertTrue(math.isfinite(x) and math.isfinite(y))
        self.assertGreater(abs(x), 1000)
        self.assertGreater(abs(y), 1000)

    def test_csv_requires_every_contract_column(self):
        for missing in CSV_FIELDS[:6]:
            fields = [field for field in CSV_FIELDS if field != missing]
            with self.subTest(missing=missing), self.assertRaises(ValueError):
                CORE.load_routes(self.write_csv([csv_row()], fields=fields))

    def test_csv_rejects_gaps_duplicates_reverse_order_and_noninteger_indexes(self):
        for indexes in [(0, 2), (0, 0), (1, 0), (0, "1.5"), (-1, 0)]:
            rows = [csv_row(index=i) for i in indexes]
            with self.subTest(indexes=indexes), self.assertRaises(ValueError):
                CORE.load_routes(self.write_csv(rows))

    def test_csv_requires_consistent_route_metadata(self):
        for changes in [{"route_name": "changed"}, {"closed": "true"}, {"closed": "maybe"}]:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                CORE.load_routes(self.write_csv([csv_row(), csv_row(index=1, **changes)]))

    def test_csv_rejects_nonfinite_out_of_range_and_missing_coordinates(self):
        for changes in [
            {"latitude": "NaN"}, {"longitude": "inf"}, {"latitude": "91"},
            {"longitude": "181"}, {"latitude": ""}, {"longitude": "not a number"},
        ]:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                CORE.load_routes(self.write_csv([csv_row(**changes)]))


class NavPvtStatusTest(unittest.TestCase):
    def test_unit_conversion_retains_receiver_estimate_and_satellite_count(self):
        sample = CORE.navpvt_sample(navpvt(hAcc=123456, numSV=29))
        self.assertAlmostEqual(ORIGIN[0], sample.lat, places=7)
        self.assertAlmostEqual(ORIGIN[1], sample.lon, places=7)
        self.assertAlmostEqual(123.456, sample.hacc_m)
        self.assertEqual(29, sample.satellites)
        self.assertTrue(sample.valid, "hAcc is displayed as an estimate, not used as a gate")

    def test_fix_carrier_and_differential_status_precedence(self):
        cases = [
            (2, 1, "2D_FIX"),
            (2, 1 | 2, "2D_FIX"),
            (2, 1 | (1 << 6), "2D_FIX"),
            (2, 1 | (2 << 6), "2D_FIX"),
            (2, 1 | (3 << 6), "2D_FIX"),
            (3, 1 | (2 << 6), "RTK_FIXED"),
            (3, 1 | 2 | (2 << 6), "RTK_FIXED"),
            (3, 1 | (1 << 6), "RTK_FLOAT"),
            (3, 1 | 2 | (1 << 6), "RTK_FLOAT"),
            (3, 1 | 2, "DGNSS"),
            (3, 1, "3D_FIX"),
            (4, 1, "GNSS_DR"),
            (4, 1 | (2 << 6), "RTK_FIXED"),
            (3, 1 | (3 << 6), "UNKNOWN"),
        ]
        for fix_type, flags, expected in cases:
            with self.subTest(fix_type=fix_type, flags=flags):
                sample = CORE.navpvt_sample(navpvt(fixType=fix_type, flags=flags))
                self.assertTrue(sample.valid)
                self.assertEqual(expected, sample.status)

    def test_fix_ok_and_supported_position_fix_are_both_required(self):
        cases = [(2, 2 << 6), (3, 2 << 6), (4, 2), (0, 129), (1, 129), (5, 129)]
        for fix_type, flags in cases:
            with self.subTest(fix_type=fix_type, flags=flags):
                sample = CORE.navpvt_sample(navpvt(fixType=fix_type, flags=flags))
                self.assertFalse(sample.valid)
                self.assertEqual("NO_FIX", sample.status)

    def test_bad_coordinates_return_invalid_samples_instead_of_crashing_replay(self):
        for changes in [
            {"lat": math.nan}, {"lon": math.inf}, {"lat": 900000001},
            {"lon": -1800000001}, {"lat": None},
        ]:
            with self.subTest(changes=changes):
                raw = navpvt()
                vars(raw).update(changes)
                sample = CORE.navpvt_sample(raw)
                self.assertFalse(sample.valid)
                self.assertEqual("NO_FIX", sample.status)

    def test_missing_or_invalid_horizontal_accuracy_does_not_hide_position(self):
        raws = [navpvt(hAcc=None), navpvt(hAcc=math.nan), navpvt(hAcc=-1)]
        absent = navpvt()
        del absent.hAcc
        raws.append(absent)
        for raw in raws:
            with self.subTest(raw=raw):
                sample = CORE.navpvt_sample(raw)
                self.assertTrue(sample.valid)
                self.assertIsNone(sample.hacc_m)


class GpsTrackTest(unittest.TestCase):
    def test_2d_fix_keeps_diagnostic_position_and_track_continuity(self):
        track = CORE.GpsTrack(ORIGIN)
        track.add(navpvt(), 1.0)
        sample = track.add(navpvt(
            lat=37.288831, lon=127.1073336, fixType=2, flags=129,
        ), 1.1)
        self.assertTrue(sample.valid)
        self.assertEqual("2D_FIX", sample.status)
        self.assertIs(track.current, sample)
        self.assertEqual(2, len(track.points))
        point = track.points[-1]
        self.assertEqual("2D_FIX", point.status)
        self.assertAlmostEqual(8.867397426498712, point.x, places=5)
        self.assertAlmostEqual(11.098306151194366, point.y, places=5)
        self.assertEqual(track.points[0].segment, point.segment)

    def test_track_projects_receiver_coordinates_in_the_declared_route_frame(self):
        track = CORE.GpsTrack(ORIGIN)
        sample = track.add(navpvt(lat=37.288831, lon=127.1073336), 10.0)
        self.assertIs(track.current, sample)
        self.assertIsInstance(track.points, deque)
        self.assertEqual(1, len(track.points))
        point = track.points[0]
        self.assertEqual(10.0, point.stamp)
        self.assertAlmostEqual(8.867397426498712, point.x, places=5)
        self.assertAlmostEqual(11.098306151194366, point.y, places=5)
        self.assertEqual(sample.status, point.status)

    def test_quality_changes_keep_position_and_continuity(self):
        track = CORE.GpsTrack(ORIGIN)
        statuses = []
        for index, flags in enumerate([129, 65, 3, 1, 193]):
            statuses.append(track.add(navpvt(lon=ORIGIN[1] + index * 0.00001, flags=flags), index * 0.1).status)
        self.assertEqual(["RTK_FIXED", "RTK_FLOAT", "DGNSS", "3D_FIX", "UNKNOWN"], statuses)
        self.assertEqual(5, len(track.points))
        self.assertEqual(1, len({point.segment for point in track.points}))

    def test_invalid_sample_updates_status_without_appending_and_breaks_connection(self):
        track = CORE.GpsTrack(ORIGIN)
        track.add(navpvt(), 1.0)
        first_segment = track.points[-1].segment
        invalid = track.add(navpvt(flags=0), 1.1)
        self.assertFalse(invalid.valid)
        self.assertIs(invalid, track.current)
        self.assertEqual(1, len(track.points))
        track.add(navpvt(lon=ORIGIN[1] + 0.00001), 1.2)
        self.assertNotEqual(first_segment, track.points[-1].segment)

    def test_nonfinite_sample_never_adds_a_track_point(self):
        track = CORE.GpsTrack(ORIGIN)
        track.add(navpvt(), 1.0)
        raw = navpvt()
        raw.lat = math.nan
        track.add(raw, 1.1)
        self.assertEqual(1, len(track.points))
        self.assertFalse(track.current.valid)

    def test_time_gap_starts_a_new_segment(self):
        track = CORE.GpsTrack(ORIGIN, gap_seconds=2.0)
        track.add(navpvt(), 1.0)
        track.add(navpvt(lon=ORIGIN[1] + 0.00001), 1.1)
        track.add(navpvt(lon=ORIGIN[1] + 0.00002), 3.5)
        self.assertEqual(track.points[0].segment, track.points[1].segment)
        self.assertNotEqual(track.points[1].segment, track.points[2].segment)

    def test_position_jump_starts_a_new_segment_without_discarding_valid_sample(self):
        track = CORE.GpsTrack(ORIGIN, max_step_m=50.0)
        track.add(navpvt(), 1.0)
        sample = track.add(navpvt(lon=ORIGIN[1] + 0.002), 1.1)
        self.assertTrue(sample.valid)
        self.assertEqual(2, len(track.points))
        self.assertNotEqual(track.points[0].segment, track.points[1].segment)

    def test_backward_clock_clears_history_current_and_reports_reset(self):
        track = CORE.GpsTrack(ORIGIN)
        self.assertFalse(track.clock(10.0))
        track.add(navpvt(), 10.0)
        self.assertFalse(track.clock(11.0))
        self.assertEqual(1, len(track.points))
        self.assertTrue(track.clock(2.0))
        self.assertEqual(0, len(track.points))
        self.assertIsNone(track.current)
        self.assertFalse(track.clock(2.0))
        track.add(navpvt(lon=ORIGIN[1] + 0.00001), 2.1)
        self.assertEqual(1, len(track.points))
        self.assertEqual(2.1, track.points[0].stamp)

    def test_history_is_bounded_and_keeps_latest_samples(self):
        track = CORE.GpsTrack(ORIGIN, max_points=3)
        for index in range(8):
            track.add(navpvt(lon=ORIGIN[1] + index * 0.00001), index * 0.1)
        self.assertEqual(3, len(track.points))
        self.assertEqual([0.5, 0.6000000000000001, 0.7000000000000001], [p.stamp for p in track.points])
        self.assertAlmostEqual(ORIGIN[1] + 0.00007, track.current.lon, places=7)


if __name__ == "__main__":
    unittest.main()
