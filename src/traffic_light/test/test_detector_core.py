#!/usr/bin/env python3
import unittest

from traffic_light_core import Detection, decide_frame


class DetectorDecisionTest(unittest.TestCase):
    def test_highest_confidence_traffic_signal_wins(self):
        decision = decide_frame([
            Detection('red', 0.72, 10, 10, 30, 40),
            Detection('green', 0.91, 50, 10, 70, 40),
            Detection('speed_20', 0.99, 80, 10, 100, 40),
        ], image_width=120)
        self.assertEqual('GREEN', decision.signal)
        self.assertAlmostEqual(0.91, decision.signal_confidence)

    def test_finish_signs_are_assigned_by_image_side(self):
        decision = decide_frame([
            Detection('down_arrow', 0.94, 10, 5, 30, 30),
            Detection('x_sign', 0.88, 90, 5, 110, 30),
            Detection('speed_20', 0.99, 50, 5, 70, 30),
        ], image_width=120)
        self.assertEqual('UNKNOWN', decision.signal)
        self.assertEqual(0.0, decision.signal_confidence)
        self.assertEqual('DOWN', decision.lane_left)
        self.assertEqual('X', decision.lane_right)
        self.assertAlmostEqual(0.88, decision.lane_confidence)

    def test_missing_detection_is_never_permission(self):
        decision = decide_frame([], image_width=640)
        self.assertEqual('UNKNOWN', decision.signal)
        self.assertEqual(0.0, decision.signal_confidence)
        self.assertEqual('UNKNOWN', decision.lane_left)
        self.assertEqual('UNKNOWN', decision.lane_right)
        self.assertEqual(0.0, decision.lane_confidence)


if __name__ == '__main__':
    unittest.main()
