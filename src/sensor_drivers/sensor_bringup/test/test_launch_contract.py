#!/usr/bin/env python3

import pathlib
import unittest
import xml.etree.ElementTree as ET


PACKAGE = pathlib.Path(__file__).resolve().parents[1]


class SensorBringupContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = ET.parse(PACKAGE / "launch" / "sensors.launch").getroot()
        cls.arguments = {
            element.attrib["name"]: element.attrib.get("default")
            for element in cls.root.findall("arg")
        }

    def test_arduino_is_explicit_opt_in(self):
        self.assertEqual("false", self.arguments["enable_arduino"])
        include = next(
            element for element in self.root.findall("include")
            if "vehicle_interface_bringup" in element.attrib["file"]
        )
        self.assertEqual("$(arg enable_arduino)", include.attrib["if"])

    def test_lidar_uses_system_contract(self):
        self.assertEqual(
            "/molit/sensors/lidar/scan", self.arguments["lidar_scan_topic"]
        )
        self.assertEqual("laser_link", self.arguments["lidar_frame_id"])
        self.assertEqual("false", self.arguments["publish_lidar_static_tf"])

    def test_localization_driver_topics_are_explicit(self):
        self.assertEqual(
            "/mando_localization/internal/driver/imu",
            self.arguments["imu_driver_topic"],
        )
        self.assertEqual(
            "/mando_localization/internal/driver/gps_fix",
            self.arguments["gps_fix_topic"],
        )
        self.assertEqual(
            "/mando_localization/internal/driver/gps_navpvt",
            self.arguments["gps_navpvt_topic"],
        )
        gps_include = next(
            element for element in self.root.findall("include")
            if "gps_bringup" in element.attrib["file"]
        )
        forwarded = {
            element.attrib["name"]: element.attrib["value"]
            for element in gps_include.findall("arg")
        }
        self.assertEqual("$(arg gps_fix_topic)", forwarded["fix_topic"])
        self.assertEqual("$(arg gps_navpvt_topic)", forwarded["navpvt_topic"])


if __name__ == "__main__":
    unittest.main()
