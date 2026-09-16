import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from stier_state_manager.geometry import Route
from stier_state_manager.visualization import marker_specs


class VisualizationTests(unittest.TestCase):
    def setUp(self):
        self.routes = {'7': Route('7', [(0, 0, 0), (10, 0, 0)])}
        self.decision = {'route': '7', 'section': 7, 'mission': 'TRAFFIC_LIGHT',
                         'phase': 'WAIT_SIGNAL', 'progress': .4, 'distance_m': 4,
                         'stop_requested': True, 'reason': 'WAIT_LEFT_ARROW',
                         'safety': {'stop': False, 'reason': 'CLEAR'}}

    def test_route_progress_and_state_are_available_without_live_odom(self):
        specs = marker_specs(self.routes, self.decision)
        progress = next(m for m in specs if m['ns'] == 'route_progress')
        self.assertEqual(progress['position'][:2], (4, 0))
        label = next(m for m in specs if m['ns'] == 'mission_status')
        self.assertIn('40.0%', label['text'])
        self.assertIn('WAIT_LEFT_ARROW', label['text'])
        self.assertIn('STOP', label['text'])
        self.assertTrue(any(m['ns'] == 'active_route' for m in specs))

    def test_landmarks_project_from_measured_route_distance(self):
        config = {'landmarks': {'7': {'stop_line_s': 6, 'parking_yaw_rad': 1, 'bad_s': 12}}}
        specs = marker_specs(self.routes, self.decision, config=config)
        landmarks = [m for m in specs if m['ns'] == 'landmarks']
        self.assertEqual(len(landmarks), 1)
        self.assertEqual(landmarks[0]['position'][:2], (6, 0))

    def test_nonfinite_odom_never_creates_invalid_arrow(self):
        specs = marker_specs(self.routes, self.decision, {'x': math.nan, 'y': 0, 'yaw': 0})
        self.assertFalse(any(m['ns'] == 'localization' for m in specs))

    def test_signal_wall_opens_on_permission_and_hill_target_is_derived(self):
        self.decision.update(valid=True, virtual_stop={'valid': True, 'active': True,
                             'pose': (6, 0, 0), 'required_signal': 'LEFT_ARROW'})
        specs = marker_specs(self.routes, self.decision)
        wall = next(m for m in specs if m['ns'] == 'traffic_wall')
        self.assertEqual(wall['position'], (6, 0, .75))
        self.decision['virtual_stop']['active'] = False
        self.assertFalse(any(m['ns'] == 'traffic_wall' for m in marker_specs(self.routes, self.decision)))
        self.routes['1_left'] = Route('1_left', [(0,0,0), (10,0,0)])
        config = {'landmarks': {'1_left': {'hill_zone_start_s': 3, 'hill_zone_end_s': 8}}}
        marks = marker_specs(self.routes, self.decision, config=config)
        target = next(m for m in marks if m['ns'] == 'landmarks' and m['key'] == '1_lefthill_stop_s')
        self.assertEqual(target['position'][:2], (5.5, 0))

    def test_confirmed_stop_and_hill_lines_have_distinct_rviz_geometry(self):
        self.routes['1_left'] = Route('1_left', [(0, 0, 0), (10, 0, 0)])
        config = {'landmarks': {
            '7': {'stop_line_s': 6},
            '1_left': {'hill_zone_start_s': 3, 'hill_zone_end_s': 8},
        }}
        specs = marker_specs(self.routes, self.decision, config=config)
        stop = next(m for m in specs if m['key'] == '7stop_line_s')
        hill_start = next(m for m in specs if m['key'] == '1_lefthill_zone_start_s')
        hill_target = next(m for m in specs if m['key'] == '1_lefthill_stop_s')
        self.assertEqual((stop['kind'], stop['color']), ('CUBE', (1, .1, .1, .95)))
        self.assertEqual((hill_start['kind'], hill_start['color']), ('CUBE', (.95, .95, .95, 1)))
        self.assertEqual((hill_target['kind'], hill_target['color']), ('SPHERE', (1, .55, .05, 1)))


if __name__ == '__main__':
    unittest.main()
