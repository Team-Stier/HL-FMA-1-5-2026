#!/usr/bin/env python3
"""Expand launch parameters only: never starts ROS nodes or opens devices."""
import os
from pathlib import Path
import subprocess
import unittest

import yaml


class LaunchContract(unittest.TestCase):
    def test_course_lookahead_and_parking_overrides(self):
        repository = Path(__file__).resolve().parents[3]
        environment = dict(os.environ)
        environment['ROS_PACKAGE_PATH'] = str(repository / 'src') + ':' + environment.get('ROS_PACKAGE_PATH', '')
        for course, directory in ((None, 'rddf'), ('hongik_test', 'test_data/hongik_rddf'),
                                  ('hongik_s_test', 'test_data/hongik_s_rddf')):
            for lookahead in (1, 2):
                with self.subTest(course=course, lookahead=lookahead):
                    # The full-workspace overlay can already contain this same
                    # source root; avoid ambiguous duplicate package lookup.
                    arguments = ['roslaunch', '--dump-params',
                                 str(repository / 'src/stier_bringup/launch/full_vehicle.launch'),
                                 'lookahead_m:='+str(lookahead), 't_parking_side:=right',
                                 'parallel_parking_side:=left']
                    if course:
                        arguments.append(course+':=true')
                    params = yaml.safe_load(subprocess.check_output(arguments, env=environment, text=True))
                    for name in ('lookahead_min_m', 'lookahead_max_m'):
                        self.assertEqual(params['/control_node/'+name], lookahead)
                    self.assertEqual(params['/state_manager/t_parking_side'], 'right')
                    self.assertEqual(params['/state_manager/parallel_parking_side'], 'left')
                    self.assertEqual(params['/rddf_tracker/initialization/rddf_directory'], directory)
                    expected = str(repository / 'src/localization' / directory)
                    self.assertEqual(params['/rddf_route_provider/rddf_directory'], expected)
                    self.assertEqual(params['/rddf_roi_detector/rddf_directory'], expected)
                    self.assertEqual(params['/rddf_roi_detector/max_cluster_extent_m'], 1.5)
                    self.assertFalse(params['/path_planner/calibration_required'])
                    self.assertEqual(params['/vehicle/wheelbase_m'], .75)


if __name__ == '__main__':
    unittest.main()
