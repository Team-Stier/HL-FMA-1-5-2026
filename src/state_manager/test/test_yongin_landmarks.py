import csv
import json
import math
from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE / "src"))

from stier_state_manager.geometry import Route, project  # noqa: E402
from stier_state_manager.mission import hill_target  # noqa: E402


class YonginLandmarkIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with (PACKAGE / "config" / "missions.json").open(encoding="utf-8") as stream:
            cls.config = json.load(stream)
        cls.rddf = REPOSITORY / "src" / "localization" / "rddf"
        with (cls.rddf / "yongin_mission_landmarks.json").open(encoding="utf-8") as stream:
            cls.source = json.load(stream)

    def route(self, name):
        with (self.rddf / ("yongin_" + name + ".csv")).open(
                encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream))
        return Route(name, [(float(row["east_m"]), float(row["north_m"]),
                             float(row["path_yaw_rad"])) for row in rows])

    def test_confirmed_source_and_runtime_config_use_the_same_origin(self):
        self.assertEqual(self.source["origin"], self.config["map_origin"])

    def test_confirmed_markers_project_back_to_the_named_global_rddf(self):
        for route_name, entries in self.source["landmarks"].items():
            route = self.route(route_name)
            for key, record in entries.items():
                if not isinstance(record, dict):
                    continue
                with self.subTest(route=route_name, landmark=key):
                    configured = self.config["landmarks"][route_name][key]
                    self.assertAlmostEqual(configured, record["projected_s_m"], places=8)
                    source_path = self.rddf / record["source_file"]
                    with source_path.open(encoding="utf-8-sig", newline="") as stream:
                        source_rows = list(csv.DictReader(stream))
                    source_row = source_rows[record["source_index"]]
                    self.assertAlmostEqual(float(source_row["latitude"]), record["latitude"], places=9)
                    self.assertAlmostEqual(float(source_row["longitude"]), record["longitude"], places=9)
                    x, y, _ = route.pose_at(configured)
                    self.assertLessEqual(math.hypot(x - record["east_m"], y - record["north_m"]), .002)
                    self.assertLessEqual(record["projection_error_m"], .002)

    def test_complete_hill_segments_stay_aligned_with_global_rddf(self):
        for route_name in ("1_left", "1_right"):
            route = self.route(route_name)
            record = self.source["landmarks"][route_name]
            source_path = self.rddf / record["hill_zone_start_s"]["source_file"]
            with source_path.open(encoding="utf-8-sig", newline="") as stream:
                source_rows = list(csv.DictReader(stream))
            errors = [project(route, float(row["east_m"]), float(row["north_m"]))["distance"]
                      for row in source_rows]
            self.assertEqual(len(source_rows), record["source_segment_points"])
            self.assertAlmostEqual(max(errors), record["segment_max_projection_error_m"], places=8)
            self.assertLess(max(errors), .04)

    def test_hill_target_is_the_arc_length_midpoint_of_white_lines(self):
        for route_name in ("1_left", "1_right"):
            values = self.config["landmarks"][route_name]
            _, target, _ = hill_target(values)
            expected = self.source["landmarks"][route_name]["derived_hill_stop_s_m"]
            self.assertAlmostEqual(target, expected, places=8)

    def test_traffic_sections_require_only_the_measured_stop_line(self):
        self.assertEqual(self.source["pending"], {})
        for route_name in ("2", "4", "7"):
            self.assertNotIn("intersection_exit_s", self.config["landmarks"][route_name])


if __name__ == "__main__":
    unittest.main()
