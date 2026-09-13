#!/usr/bin/env python3
"""ROS master 없이 실행하는 Localization 수학 단위 테스트."""

import importlib.machinery
import importlib.util
import math
from pathlib import Path
import unittest

from geometry_msgs.msg import Quaternion


PACKAGE_DIR = Path(__file__).resolve().parents[1]
NODE_PATH = PACKAGE_DIR / "scripts" / "localization_node"
LOADER = importlib.machinery.SourceFileLoader("localization_node_core", str(NODE_PATH))
SPEC = importlib.util.spec_from_loader(LOADER.name, LOADER)
LOCALIZATION = importlib.util.module_from_spec(SPEC)
LOADER.exec_module(LOCALIZATION)

ODOMETRY_PATH = PACKAGE_DIR / "scripts" / "wheel_imu_gps_odometry_node"
ODOMETRY_LOADER = importlib.machinery.SourceFileLoader(
    "wheel_imu_gps_odometry_core", str(ODOMETRY_PATH)
)
ODOMETRY_SPEC = importlib.util.spec_from_loader(
    ODOMETRY_LOADER.name, ODOMETRY_LOADER
)
ODOMETRY = importlib.util.module_from_spec(ODOMETRY_SPEC)
ODOMETRY_LOADER.exec_module(ODOMETRY)


class LocalizationCoreTest(unittest.TestCase):
    def test_datum_projects_to_exact_configured_map_point(self):
        datum = LOCALIZATION.Datum(37.2, 126.8, 45.0, 12.0, -3.0, 1.0)
        point = LOCALIZATION.project_to_map(37.2, 126.8, 45.0, datum, 0.0)
        self.assertEqual((12.0, -3.0, 1.0), point)

    def test_one_meter_east_projects_to_one_meter(self):
        latitude = 37.2
        datum = LOCALIZATION.Datum(latitude, 126.8, 45.0, 0.0, 0.0, 0.0)
        latitude_rad = math.radians(latitude)
        denominator = math.sqrt(
            1.0
            - LOCALIZATION.WGS84_ECCENTRICITY_SQUARED
            * math.sin(latitude_rad) ** 2
        )
        radius = LOCALIZATION.WGS84_SEMI_MAJOR_AXIS_M / denominator
        longitude_delta = math.degrees(1.0 / (radius * math.cos(latitude_rad)))
        x, y, _ = LOCALIZATION.project_to_map(
            latitude, datum.longitude_deg + longitude_delta, 45.0, datum, 0.0
        )
        self.assertAlmostEqual(1.0, x, places=6)
        self.assertAlmostEqual(0.0, y, places=9)

    def test_projection_and_heading_share_map_yaw_rotation(self):
        datum = LOCALIZATION.Datum(37.2, 126.8, 45.0, 0.0, 0.0, 0.0)
        latitude_rad = math.radians(datum.latitude_deg)
        denominator = math.sqrt(
            1.0
            - LOCALIZATION.WGS84_ECCENTRICITY_SQUARED
            * math.sin(latitude_rad) ** 2
        )
        radius = LOCALIZATION.WGS84_SEMI_MAJOR_AXIS_M / denominator
        longitude_delta = math.degrees(1.0 / (radius * math.cos(latitude_rad)))
        x, y, _ = LOCALIZATION.project_to_map(
            datum.latitude_deg,
            datum.longitude_deg + longitude_delta,
            datum.altitude_m,
            datum,
            math.pi / 2.0,
        )
        self.assertAlmostEqual(0.0, x, places=6)
        self.assertAlmostEqual(1.0, y, places=6)
        orientation = LOCALIZATION.yaw_quaternion(0.2)
        output_yaw = LOCALIZATION.compose_output_yaw(
            orientation, 0.3, math.pi / 2.0
        )
        self.assertAlmostEqual(0.2 + 0.3 + math.pi / 2.0, output_yaw)

    def test_planar_lever_arm_rotation(self):
        x, y = LOCALIZATION.rotate_planar_vector(1.0, 0.0, math.pi / 2.0)
        self.assertAlmostEqual(0.0, x, places=9)
        self.assertAlmostEqual(1.0, y, places=9)

    def test_quaternion_normalization_and_yaw(self):
        quaternion = Quaternion(x=0.0, y=0.0, z=0.0, w=1.001)
        normalized = LOCALIZATION.normalize_quaternion(quaternion, 0.01)
        self.assertAlmostEqual(1.0, normalized.w)
        self.assertAlmostEqual(0.0, LOCALIZATION.quaternion_yaw(normalized))

    def test_bad_quaternion_is_rejected(self):
        quaternion = Quaternion(x=0.0, y=0.0, z=0.0, w=0.5)
        with self.assertRaises(ValueError):
            LOCALIZATION.normalize_quaternion(quaternion, 0.05)

    def test_angle_wrap(self):
        self.assertAlmostEqual(-math.pi + 0.2, LOCALIZATION.normalize_angle(math.pi + 0.2))

    def test_encoder_rollover_delta(self):
        self.assertEqual(3, ODOMETRY.shortest_encoder_delta(1, 14, 16))
        self.assertEqual(-3, ODOMETRY.shortest_encoder_delta(14, 1, 16))
        self.assertEqual(-4, ODOMETRY.shortest_encoder_delta(6, 10, 0))

    def test_window_delta_uses_arduino_value_without_second_difference(self):
        self.assertEqual(
            10, ODOMETRY.encoder_sample_delta(10, 200, "window_delta", 0)
        )
        self.assertEqual(
            -7, ODOMETRY.encoder_sample_delta(-7, 10, "window_delta", 0)
        )

    def test_cumulative_count_keeps_legacy_difference(self):
        self.assertEqual(
            10, ODOMETRY.encoder_sample_delta(210, 200, "cumulative_count", 0)
        )

    def test_map_to_odom_composition(self):
        sample = ODOMETRY.MotionSample(
            None, 2.0, 0.0, math.pi / 2.0, 0.0, 0.0, 2.0
        )
        x, y, yaw = ODOMETRY.compose_planar_pose(
            (10.0, -1.0, math.pi / 2.0), sample
        )
        self.assertAlmostEqual(10.0, x)
        self.assertAlmostEqual(1.0, y)
        self.assertAlmostEqual(math.pi, abs(yaw))

    def test_vehicle_outline_is_closed_and_has_requested_dimensions(self):
        points = ODOMETRY.vehicle_outline_points(1.35, 0.85)
        self.assertEqual(points[0], points[-1])
        self.assertEqual(5, len(points))
        self.assertAlmostEqual(1.35, max(x for x, _ in points) - min(x for x, _ in points))
        self.assertAlmostEqual(0.85, max(y for _, y in points) - min(y for _, y in points))


if __name__ == "__main__":
    unittest.main()
