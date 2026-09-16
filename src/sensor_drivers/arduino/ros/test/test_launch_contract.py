#!/usr/bin/env python3

import pathlib
import unittest
import xml.etree.ElementTree as ET


PACKAGE = pathlib.Path(__file__).resolve().parents[1]


class ArduinoBringupContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = ET.parse(PACKAGE / "launch" / "arduino.launch").getroot()
        cls.arguments = {
            element.attrib["name"]: element.attrib.get("default")
            for element in cls.root.findall("arg")
        }

    def test_yaml_is_default_and_cli_overrides_are_optional(self):
        self.assertEqual(
            "$(find vehicle_interface_bringup)/config/serial.yaml",
            self.arguments["config"],
        )
        self.assertEqual("", self.arguments["port"])
        self.assertEqual("", self.arguments["baud"])

    def test_serial_node_loads_config_before_overrides(self):
        node = self.root.find("node")
        self.assertIsNotNone(node)
        rosparam = node.find("rosparam")
        self.assertEqual("load", rosparam.attrib["command"])
        self.assertEqual("$(arg config)", rosparam.attrib["file"])

        parameters = {
            element.attrib["name"]: element
            for element in node.findall("param")
        }
        self.assertEqual("$(arg port)", parameters["port"].attrib["value"])
        self.assertIn("arg('port') != ''", parameters["port"].attrib["if"])
        self.assertEqual("$(arg baud)", parameters["baud"].attrib["value"])
        self.assertEqual("int", parameters["baud"].attrib["type"])

    def test_default_config_contains_serial_values(self):
        config = (PACKAGE / "config" / "serial.yaml").read_text()
        self.assertIn("port: /dev/ttyACM0", config)
        self.assertIn("baud: 57600", config)


if __name__ == "__main__":
    unittest.main()
