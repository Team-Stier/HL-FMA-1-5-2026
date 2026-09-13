#!/usr/bin/env python3

import importlib.util
from importlib.machinery import SourceFileLoader
import pathlib
import unittest


SCRIPT = pathlib.Path(__file__).parents[1] / "scripts" / "imu_covariance_override_node"
LOADER = SourceFileLoader("imu_covariance_override_node", str(SCRIPT))
SPEC = importlib.util.spec_from_loader(LOADER.name, LOADER)
MODULE = importlib.util.module_from_spec(SPEC)
LOADER.exec_module(MODULE)


class ImuCovarianceOverrideTest(unittest.TestCase):
    def test_stddev_is_squared_into_variance(self):
        self.assertSequenceEqual(
            (0.01, 0.04, 0.09),
            tuple(
                round(value, 12)
                for value in MODULE.variances_from_stddev(
                    [0.1, 0.2, 0.3], "test"
                )
            ),
        )

    def test_all_zero_covariance_gets_measured_diagonal(self):
        output, replaced = MODULE.replace_all_zero_covariance(
            [0.0] * 9, (1.0, 2.0, 3.0)
        )
        self.assertTrue(replaced)
        self.assertEqual([1.0, 2.0, 3.0], [output[0], output[4], output[8]])

    def test_existing_covariance_is_preserved(self):
        original = [0.0] * 9
        original[0] = 0.5
        output, replaced = MODULE.replace_all_zero_covariance(
            original, (1.0, 2.0, 3.0)
        )
        self.assertFalse(replaced)
        self.assertEqual(original, output)


if __name__ == "__main__":
    unittest.main()
