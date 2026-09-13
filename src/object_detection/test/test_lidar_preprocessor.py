import math
from pathlib import Path
import runpy
import unittest

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import LaserScan

scan_to_cloud = runpy.run_path(str(Path(__file__).resolve().parents[1] /
                                  'scripts/lidar_preprocessor_node'))['scan_to_cloud']


class RotationTest(unittest.TestCase):
    def scan(self, ranges, angle_min=0., angle_increment=math.pi / 2):
        scan = LaserScan()
        scan.header.frame_id = 'laser'
        scan.header.stamp = rospy.Time(123, 456)
        scan.angle_min, scan.angle_increment = angle_min, angle_increment
        scan.range_min, scan.range_max = .1, 30.
        scan.ranges = ranges
        return scan

    def test_cardinal_directions_roll_not_yaw_and_header_preserved(self):
        scan = self.scan([1., 1., 1., 1.])
        cloud = scan_to_cloud(scan, 'lidar_preprocessed')
        actual = list(point_cloud2.read_points(cloud))
        expected = [(1, 0, 0), (0, -1, 0), (-1, 0, 0), (0, 1, 0)]
        for point, target in zip(actual, expected):
            for value, want in zip(point, target):
                self.assertAlmostEqual(value, want, places=6)
        self.assertEqual(cloud.width, 4)
        self.assertEqual(cloud.header.stamp, scan.header.stamp)
        self.assertEqual(cloud.header.frame_id, 'lidar_preprocessed')

    def test_invalid_ranges_removed_without_shifting_intensities(self):
        scan = self.scan([float('nan'), float('inf'), .01, 31., .1, 30.])
        scan.intensities = [10., 20., 30., 40., 50., 60.]
        cloud = scan_to_cloud(scan, 'lidar_preprocessed')
        self.assertEqual(cloud.width, 2)
        self.assertEqual([p[3] for p in point_cloud2.read_points(cloud)], [50., 60.])

    def test_clockwise_input_and_partial_scan(self):
        scan = self.scan([2., 3.], math.pi / 2, -math.pi / 2)
        points = list(point_cloud2.read_points(scan_to_cloud(scan, 'lidar_preprocessed')))
        self.assertAlmostEqual(points[0][1], -2.)
        self.assertAlmostEqual(points[1][0], 3.)

    def test_empty_scan_and_missing_intensities(self):
        self.assertEqual(scan_to_cloud(self.scan([]), 'lidar_preprocessed').width, 0)
        scan = self.scan([1., 2.])
        scan.intensities = [7.]
        self.assertEqual([f.name for f in scan_to_cloud(scan, 'lidar_preprocessed').fields], ['x', 'y', 'z'])

    def test_bad_geometry_and_same_frame_rejected(self):
        scan = self.scan([1., 2.])
        with self.assertRaises(ValueError):
            scan_to_cloud(scan, 'laser')
        scan.angle_increment = float('nan')
        with self.assertRaises(ValueError):
            scan_to_cloud(scan, 'lidar_preprocessed')


if __name__ == '__main__':
    unittest.main()
