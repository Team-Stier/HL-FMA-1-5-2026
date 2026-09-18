"""Exercise overlapping reverse legs with the real RDDF gear profiles."""
import json
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from rddf_initialization_core import RddfRouteMap
from rddf_tracking_core import RddfTracker


class ParkingLegTrackingTest(unittest.TestCase):
    def test_all_recorded_parallel_legs_use_the_selected_station_interval(self):
        config = json.loads((ROOT.parent / 'state_manager/config/missions.json').read_text())
        for directory in ('rddf', 'test_data/hongik_rddf', 'test_data/hongik_s_rddf'):
            routes = RddfRouteMap(ROOT / directory)
            for name, profile in config['parallel_parking_profiles'].items():
                points = routes.routes[name]
                stations = [0.]
                for a, b in zip(points, points[1:]):
                    stations.append(stations[-1] + math.hypot(*(b-a)))
                legs = [(0., profile['initial_direction'])] + [
                    (change['s'], change['direction']) for change in profile['changes']]
                tracker = RddfTracker(routes)
                tracker.active_source = name
                for index, (start, direction) in enumerate(legs):
                    end = legs[index+1][0] if index+1 < len(legs) else stations[-1]
                    tracker.update_parking_leg(name, index, start, end, direction)
                    for fraction in (.01, .5, .99):
                        s = start + fraction * (end-start)
                        segment = next(i for i in range(len(stations)-1)
                                       if stations[i] <= s < stations[i+1])
                        a, b = points[segment], points[segment+1]
                        t = (s-stations[segment])/(stations[segment+1]-stations[segment])
                        x, y = a + t*(b-a)
                        yaw = math.atan2(direction*(b[1]-a[1]), direction*(b[0]-a[0]))
                        tracker.update_pose(x, y, 10., 10., 'map', yaw)
                        tracker.update_valid(True, 10.)
                        result = tracker.evaluate(10.)
                        with self.subTest(directory=directory, route=name, leg=index, s=s):
                            self.assertTrue(result['accepted'], result)
                            if name.startswith('11') and index == len(legs)-1 and end-s < .5:
                                self.assertEqual(result['source_route'], '12')
                                continue
                            self.assertEqual(result['source_route'], name)
                            found_s = stations[result['index']] + result['fraction'] * (
                                stations[result['index']+1]-stations[result['index']])
                            self.assertAlmostEqual(found_s, s, places=5)
                            self.assertAlmostEqual(math.atan2(math.sin(result['yaw']-yaw),
                                                             math.cos(result['yaw']-yaw)), 0., places=5)

    def test_clear_leg_on_ordinary_route_but_keep_through_input_wait(self):
        tracker = RddfTracker(RddfRouteMap(ROOT / 'rddf'))
        tracker.update_parking_leg('10_parallel-left-in', 1, 9.3, 10.5, -1)
        saved = tracker.parking_leg
        tracker.update_parking_leg('10_parallel-left-in', -1, -1., -1., 1)
        self.assertEqual(tracker.parking_leg, saved)
        tracker.update_parking_leg('12', -1, -1., -1., 1)
        self.assertIsNone(tracker.parking_leg)


if __name__ == '__main__':
    unittest.main()
