import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from stier_state_manager.geometry import Route
from stier_state_manager.runtime import MissionRuntime


class TParkingSelectionTests(unittest.TestCase):
    def runtime(self, default='right', angle=0.0):
        def point(x, y):
            return (10 + math.cos(angle)*x - math.sin(angle)*y,
                    20 + math.sin(angle)*x + math.cos(angle)*y, angle)
        routes = {'4': Route('4', [point(0, 0), point(74, 0)]),
                  '5_T-left-in': Route('5_T-left-in', [point(0, 0), point(10, 0)]),
                  '5_T-right-in': Route('5_T-right-in', [point(0, 3), point(10, 3)])}
        runtime = MissionRuntime(routes, {
            'start_route': '4', 'parking_branches': {'t': default, 'parallel': 'right'},
            't_parking_detection': {'entry_s': {'left': 5, 'right': 5},
                                    'start_s': 58, 'end_s': 64,
                                    'length_m': 1, 'width_m': .6,
                                    'blocked_min_points': 5, 'clear_max_points': 2,
                                    'consecutive_frames': 5}})
        runtime.progress_s = 60
        return runtime, point

    def observe(self, runtime, points, valid=True):
        runtime.observe_t_parking({'valid': valid, 'clusters': [points]})

    def test_success_overrides_default_and_stays_selected(self):
        for default, blocked_y, expected in [('right', 3, 'left'), ('left', 0, 'right')]:
            runtime, point = self.runtime(default)
            blocked = [point(5, blocked_y)[:2]] * 5
            for _ in range(4):
                self.observe(runtime, blocked)
            self.assertEqual(runtime.engine.branches['t'], default)
            self.observe(runtime, blocked)
            self.assertEqual(runtime.engine.branches['t'], expected)
            for _ in range(10):
                self.observe(runtime, [point(5, 3-blocked_y)[:2]] * 5)
            self.assertEqual(runtime.engine.branches['t'], expected)
            self.assertEqual(runtime.engine.branches['parallel'], 'right')

    def test_failed_or_missing_detection_keeps_default(self):
        for default in ['left', 'right']:
            runtime, point = self.runtime(default)
            for _ in range(10):
                self.observe(runtime, [])
                self.observe(runtime, [point(5, 0)[:2], point(5, 3)[:2]] * 5)
                self.observe(runtime, [point(5, 0)[:2]] * 5, valid=False)
            runtime.progress_s = 74
            self.observe(runtime, [point(5, 0)[:2]] * 5)
            self.assertEqual(runtime.engine.branches['t'], default)
            self.assertIsNone(runtime.t_parking_selector.selected)

    def test_interrupted_candidates_do_not_accumulate(self):
        runtime, point = self.runtime()
        for _ in range(10):
            for _ in range(4):
                self.observe(runtime, [point(5, 3)[:2]] * 5)
            self.observe(runtime, [])
        self.assertEqual(runtime.engine.branches['t'], 'right')

    def test_rotated_narrow_roi_excludes_side_boundary(self):
        runtime, point = self.runtime(angle=.7)
        for _ in range(5):
            self.observe(runtime, [point(5, 3)[:2]] * 5 + [point(5, .4)[:2]] * 20)
        self.assertEqual(runtime.engine.branches['t'], 'left')

    def test_outside_window_and_after_entry_do_not_change_default(self):
        runtime, point = self.runtime()
        for distance in [57, 65]:
            runtime.progress_s = distance
            for _ in range(5):
                self.observe(runtime, [point(5, 3)[:2]] * 5)
        runtime.active_route = runtime.routes['5_T-right-in']
        runtime.progress_s = 60
        for _ in range(5):
            self.observe(runtime, [point(5, 3)[:2]] * 5)
        self.assertEqual(runtime.engine.branches['t'], 'right')


if __name__ == '__main__':
    unittest.main()
