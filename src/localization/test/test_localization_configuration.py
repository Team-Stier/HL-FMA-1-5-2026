#!/usr/bin/env python3
"""저장소 README와 Localization/Sensor launch 계약 회귀 테스트."""

import os
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
REPOSITORY_DIR = PACKAGE_DIR.parents[1]


class LocalizationConfigurationTest(unittest.TestCase):
    def test_public_topics_and_frames_match_root_readme(self):
        with (PACKAGE_DIR / "config" / "localization.yaml").open(
            encoding="utf-8"
        ) as stream:
            config = yaml.safe_load(stream)

        expected_topics = {
            "gps_fix": "/ublox_position_receiver/fix",
            "gps_status": "/gps/status",
            "imu": "/imu/data",
            "encoder": "/erp42_serial/feedback",
            "current_pose": "/current_pos",
            "local_odometry": "/localization/local/odometry",
            "global_odometry": "/localization/global/odometry",
            "vehicle_marker": "/localization/vehicle_marker",
        }
        for name, topic in expected_topics.items():
            self.assertEqual(topic, config["topics"][name])
        self.assertEqual("gps_link", config["frames"]["gps"])
        self.assertEqual("imu_link", config["frames"]["imu"])
        self.assertEqual("map", config["frames"]["output"])
        self.assertEqual("odom", config["frames"]["odom"])
        self.assertEqual("base_link", config["frames"]["base"])
        self.assertEqual("manual_datum", config["reference"]["mode"])
        self.assertFalse(config["reference"]["measured"])
        self.assertFalse(config["operation"]["allow_unmeasured_test_mode"])
        self.assertTrue(config["output"]["planar_mode"])
        self.assertGreater(
            config["quality"]["max_imu_orientation_variance_rad2"], 0.0
        )
        self.assertEqual([1], config["quality"]["allowed_spoofing_states"])
        self.assertEqual([3, 4], config["quality"]["allowed_fix_types"])
        self.assertFalse(config["quality"]["allow_unknown_imu_orientation_covariance"])
        self.assertTrue(config["motion"]["encoder_calibrated"])
        self.assertEqual(45, config["motion"]["encoder_counts_per_revolution"])
        self.assertAlmostEqual(
            config["motion"]["encoder_wheel_circumference_m"] / 45.0,
            config["motion"]["encoder_meters_per_tick"],
        )
        self.assertEqual("window_delta", config["motion"]["encoder_value_mode"])
        self.assertEqual(1.35, config["visualization"]["vehicle_length_m"])
        self.assertEqual(0.85, config["visualization"]["vehicle_width_m"])
        production_rviz = (PACKAGE_DIR / "rviz" / "localization.rviz").read_text(
            encoding="utf-8"
        )
        self.assertIn("Fixed Frame: odom", production_rviz)

        with (PACKAGE_DIR / "test" / "localization_test.yaml").open(
            encoding="utf-8"
        ) as stream:
            test_config = yaml.safe_load(stream)
        self.assertEqual("first_fix", test_config["reference"]["mode"])
        self.assertFalse(test_config["reference"]["measured"])
        self.assertTrue(test_config["operation"]["allow_unmeasured_test_mode"])
        self.assertTrue(test_config["motion"]["encoder_calibrated"])
        self.assertEqual(
            "cumulative_count", test_config["motion"]["encoder_value_mode"]
        )
        for topic in test_config["topics"].values():
            self.assertTrue(topic.startswith("/localization/synthetic/"), topic)

        with (PACKAGE_DIR / "test" / "localization_live_test.yaml").open(
            encoding="utf-8"
        ) as stream:
            live_config = yaml.safe_load(stream)
        self.assertEqual("/imu/data", live_config["topics"]["imu"])
        self.assertEqual(
            "/erp42_serial/feedback", live_config["topics"]["encoder"]
        )
        self.assertTrue(
            live_config["topics"]["current_pose"].startswith("/localization/live/")
        )
        self.assertTrue(live_config["motion"]["encoder_calibrated"])
        self.assertEqual(
            45, live_config["motion"]["encoder_counts_per_revolution"]
        )
        self.assertAlmostEqual(
            live_config["motion"]["encoder_wheel_circumference_m"] / 45.0,
            live_config["motion"]["encoder_meters_per_tick"],
        )
        self.assertEqual(
            "window_delta", live_config["motion"]["encoder_value_mode"]
        )
        self.assertEqual(
            "/localization/live/vehicle_marker",
            live_config["topics"]["vehicle_marker"],
        )
        self.assertEqual(1.35, live_config["visualization"]["vehicle_length_m"])
        self.assertEqual(0.85, live_config["visualization"]["vehicle_width_m"])
        live_rviz = (
            PACKAGE_DIR / "test" / "localization_live_sensor_debug.rviz"
        ).read_text(encoding="utf-8")
        self.assertIn("Fixed Frame: odom_live_test", live_rviz)
        self.assertFalse(
            live_config["quality"]["allow_unknown_imu_orientation_covariance"]
        )

        root_readme = (REPOSITORY_DIR / "README.md").read_text(encoding="utf-8")
        for contract in (
            "/ublox_position_receiver/fix",
            "/gps/status",
            "/imu/data",
            "/current_pos",
            "./src/localization/launch.sh",
        ):
            self.assertIn(contract, root_readme)

    def test_launch_loads_one_configuration_into_three_scoped_nodes(self):
        launch = ET.parse(PACKAGE_DIR / "launch" / "localization.launch").getroot()
        nodes = [node for node in launch.findall("node") if node.get("pkg") == "localization"]
        self.assertEqual(
            {
                "localization_node",
                "wheel_imu_gps_odometry_node",
                "localization_sensor_status_node",
            },
            {node.get("type") for node in nodes},
        )
        for node in nodes:
            rosparams = node.findall("rosparam")
            self.assertEqual(1, len(rosparams))
            self.assertEqual("$(arg config)", rosparams[0].get("file"))

    def test_run_script_uses_localization_launcher(self):
        run_script = (REPOSITORY_DIR / "run.sh").read_text(encoding="utf-8")
        localization_launcher = (
            'start_package_launcher localization '
            '"${STIER_WORKSPACE_ROOT}/src/localization/launch.sh"'
        )
        self.assertIn(
            localization_launcher,
            run_script,
        )
        self.assertTrue(os.access(PACKAGE_DIR / "launch.sh", os.X_OK))
        self.assertTrue(os.access(PACKAGE_DIR / "scripts" / "localization_node", os.X_OK))
        self.assertTrue(
            os.access(PACKAGE_DIR / "scripts" / "wheel_imu_gps_odometry_node", os.X_OK)
        )
        self.assertTrue(
            os.access(PACKAGE_DIR / "scripts" / "localization_sensor_status_node", os.X_OK)
        )
        command_script = (
            PACKAGE_DIR / "test" / "localization_command.sh"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "arduino_by_id_candidates=(/dev/serial/by-id/*Arduino*)",
            command_script,
        )
        self.assertLess(
            command_script.index("arduino_by_id_candidates="),
            command_script.index("encoder_candidates=()"),
        )

    def test_synthetic_and_live_debug_are_separated(self):
        synthetic = (PACKAGE_DIR / "test" / "synthetic_localization_debug.py").read_text(
            encoding="utf-8"
        )
        for real_topic in (
            '"/ublox_position_receiver/fix"',
            '"/gps/status"',
            '"/imu/data"',
            '"/current_pos"',
        ):
            self.assertNotIn(real_topic, synthetic)
        live_launch = ET.parse(
            PACKAGE_DIR / "test" / "localization_live_sensor_debug.launch"
        ).getroot()
        self.assertEqual([], [node for node in live_launch.findall("node") if node.get("type") == "synthetic_localization_debug.py"])
        live_args = {argument.get("name"): argument.get("default") for argument in live_launch.findall("arg")}
        self.assertEqual("true", live_args["encoder_calibrated"])
        self.assertAlmostEqual(
            0.84823 / 45.0, float(live_args["encoder_meters_per_tick"])
        )

    def test_gps_launch_applies_readme_frame_arguments(self):
        launch_path = (
            REPOSITORY_DIR
            / "src"
            / "sensor_drivers"
            / "gps"
            / "ublox_utils"
            / "launch"
            / "ublox.launch"
        )
        launch = ET.parse(launch_path).getroot()
        nodes = {node.get("name"): node for node in launch.iter("node")}
        position_params = {
            parameter.get("name"): parameter.get("value")
            for parameter in nodes["ublox_position_receiver"].findall("param")
        }
        baseline_params = {
            parameter.get("name"): parameter.get("value")
            for parameter in nodes["ublox_moving_baseline_receiver"].findall("param")
        }
        self.assertEqual("$(arg frame_id_position_receiver)", position_params["frame_id"])
        self.assertEqual(
            "$(arg frame_id_moving_baseline_receiver)", baseline_params["frame_id"]
        )

        bringup_path = (
            REPOSITORY_DIR
            / "src"
            / "sensor_drivers"
            / "gps"
            / "gps_bringup"
            / "launch"
            / "gps.launch"
        )
        bringup = ET.parse(bringup_path).getroot()
        status_node = next(
            node for node in bringup.findall("node") if node.get("name") == "gps_status"
        )
        status_params = {
            parameter.get("name"): parameter.get("value")
            for parameter in status_node.findall("param")
        }
        self.assertEqual("$(arg nav_status_timeout)", status_params["nav_status_timeout"])
        self.assertEqual(
            "$(arg max_nav_status_itow_skew_ms)",
            status_params["max_nav_status_itow_skew_ms"],
        )

    def test_imu_driver_does_not_claim_world_transform(self):
        launch_path = (
            REPOSITORY_DIR
            / "src"
            / "sensor_drivers"
            / "imu"
            / "imu_bringup"
            / "launch"
            / "xsens_mti.launch"
        )
        launch = ET.parse(launch_path).getroot()
        node = launch.find("node")
        parameters = {
            parameter.get("name"): parameter.get("value")
            for parameter in node.findall("param")
        }
        self.assertEqual("false", parameters["pub_transform"])
        self.assertEqual("$(arg frame_id)", parameters["frame_id"])

    def test_production_files_do_not_use_previous_repository_topics(self):
        production_files = (
            PACKAGE_DIR / "scripts" / "localization_node",
            PACKAGE_DIR / "config" / "localization.yaml",
            PACKAGE_DIR / "launch" / "localization.launch",
        )
        for path in production_files:
            self.assertNotIn("/molit/", path.read_text(encoding="utf-8"), str(path))


if __name__ == "__main__":
    unittest.main()
