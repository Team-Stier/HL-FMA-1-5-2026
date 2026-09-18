#!/usr/bin/env python3
"""Expand launch parameters only: never starts ROS nodes or opens devices."""
import os
import json
from pathlib import Path
import subprocess
import shlex
import unittest

import yaml


class LaunchContract(unittest.TestCase):
    def test_measured_s_preset_shares_portable_map_with_every_consumer(self):
        repository = Path(__file__).resolve().parents[3]
        relative = 'test_data/hongik_s_live_20260918'
        directory = repository / 'src/localization' / relative
        wrapper = (repository / 'run_school_s_current.sh').read_text().replace('\\\n', ' ')
        wrapper = wrapper.replace('${STIER_S_COURSE_MAP}', str(directory))
        preset = [word for word in shlex.split(wrapper) if ':=' in word]
        environment = dict(os.environ)
        environment['ROS_PACKAGE_PATH'] = str(repository / 'src') + ':' + environment.get('ROS_PACKAGE_PATH', '')
        params = yaml.safe_load(subprocess.check_output([
            'roslaunch', '--dump-params',
            str(repository / 'src/stier_bringup/launch/full_vehicle.launch')
        ] + preset, env=environment, text=True))
        for node in ('rddf_tracker', 'rddf_initializer'):
            self.assertEqual(params['/' + node + '/initialization/rddf_directory'], relative)
        for node in ('rddf_route_provider', 'rddf_roi_detector'):
            self.assertEqual(params['/' + node + '/rddf_directory'], str(directory))
        self.assertEqual(params['/control_node/target_speed_kph'], 5)
        self.assertEqual(params['/control_node/maximum_speed_kph'], 15)
        self.assertEqual(params['/rddf_roi_detector/max_cluster_extent_m'], 1.5)
        self.assertFalse(any(key.startswith('/traffic_light_node/') for key in params))
        self.assertNotIn('/home/choiminho/', (directory / 'initialization.yaml').read_text())
        self.assertNotIn('/home/choiminho/', (directory / 'viewer.yaml').read_text())

    def test_school_full_preset_uses_one_consistent_course_and_keeps_overrides(self):
        repository = Path(__file__).resolve().parents[3]
        # Check the wrapper's actual preset without sourcing a local devel/;
        # CI builds the packages in a separate workspace.
        wrapper = (repository / 'run_school_full.sh').read_text().replace('\\\n', ' ')
        preset = [word for word in shlex.split(wrapper) if ':=' in word]
        environment = dict(os.environ)
        environment['ROS_PACKAGE_PATH'] = str(repository / 'src') + ':' + environment.get('ROS_PACKAGE_PATH', '')
        command = ['roslaunch', '--dump-params',
                   str(repository / 'src/stier_bringup/launch/full_vehicle.launch')] + preset
        defaults = yaml.safe_load(subprocess.check_output(command, env=environment, text=True))
        directory = str(repository / 'src/localization/test_data/hongik_rddf')
        self.assertEqual(defaults['/state_manager/config_file'],
                         str(repository / 'src/state_manager/config/missions_hongik.json'))
        self.assertEqual(defaults['/rddf_route_provider/rddf_directory'], directory)
        self.assertEqual(defaults['/rddf_roi_detector/rddf_directory'], directory)
        self.assertEqual(defaults['/rddf_tracker/initialization/rddf_directory'],
                         'test_data/hongik_rddf')
        self.assertEqual(defaults['/control_node/target_speed_kph'], 5)
        self.assertEqual(defaults['/control_node/maximum_speed_kph'], 15)
        self.assertTrue(defaults['/control_node/adaptive_speed_enabled'])
        self.assertEqual(defaults['/arduino_serial/port'],
                         '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
        self.assertEqual(defaults['/arduino_serial/baud'], 57600)
        self.assertFalse(any(key.startswith('/usb_cam/') for key in defaults))
        self.assertFalse(any(key.startswith('/traffic_light_node/') for key in defaults))
        changed = yaml.safe_load(subprocess.check_output(command + [
            'target_speed_kph:=15', 'lookahead_m:=2',
            't_parking_side:=right', 'parallel_parking_side:=left'], env=environment, text=True))
        self.assertEqual(changed['/control_node/target_speed_kph'], 15)
        self.assertEqual(changed['/control_node/maximum_speed_kph'], 15)
        self.assertEqual(changed['/control_node/lookahead_max_m'], 2)
        self.assertEqual(changed['/state_manager/t_parking_side'], 'right')
        self.assertEqual(changed['/state_manager/parallel_parking_side'], 'left')

    def test_speed_defaults_and_launch_overrides(self):
        repository = Path(__file__).resolve().parents[3]
        environment = dict(os.environ)
        environment['ROS_PACKAGE_PATH'] = str(repository / 'src') + ':' + environment.get('ROS_PACKAGE_PATH', '')
        command = ['roslaunch', '--dump-params',
                   str(repository / 'src/stier_bringup/launch/full_vehicle.launch')]
        defaults = yaml.safe_load(subprocess.check_output(command, env=environment, text=True))
        self.assertEqual(defaults['/control_node/target_speed_kph'], 15)
        self.assertEqual(defaults['/control_node/maximum_speed_kph'], 15)
        self.assertTrue(defaults['/control_node/adaptive_speed_enabled'])
        self.assertEqual(defaults['/control_node/speed_profile/preview_distance_m'], 40.)
        self.assertEqual(defaults['/control_node/lookahead_min_m'], 2.)
        self.assertEqual(defaults['/control_node/lookahead_max_m'], 4.)
        arguments = ['target_speed_kph:=8', 'adaptive_speed_enabled:=false',
                     'normal_speed_kph:=8', 'hill_speed_kph:=3', 'static_speed_kph:=4',
                     'intersection_speed_kph:=4', 'parking_speed_kph:=2']
        changed = yaml.safe_load(subprocess.check_output(command + arguments, env=environment, text=True))
        self.assertEqual(changed['/control_node/target_speed_kph'], 8)
        self.assertEqual(changed['/control_node/maximum_speed_kph'], 15)
        self.assertFalse(changed['/control_node/adaptive_speed_enabled'])
        for kind, speed in (('normal', 8), ('hill', 3), ('static', 4), ('intersection', 4), ('parking', 2)):
            self.assertEqual(float(changed['/state_manager/'+kind+'_speed_kph']), speed)
        for name in ('missions.json', 'missions_hongik.json'):
            config = json.loads((repository / 'src/state_manager/config' / name).read_text())
            self.assertAlmostEqual(config['speeds']['normal'] * 3.6, 15.)
            self.assertEqual(config['path_lookahead_m'], 40.)
            self.assertEqual(config['rules']['hill_hold_s'], 3.5)

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
