#!/usr/bin/env python3
"""테스트 전용 Local/Global RViz 데이터가 같은 0,0에서 시작하는지 검증한다."""

import time
import unittest

import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Path


class LocalizationDebugPipelineTest(unittest.TestCase):
    @staticmethod
    def _wait_for_path(topic, minimum_poses, minimum_last_x=None, timeout=8.0):
        deadline = time.monotonic() + timeout
        latest = None
        while time.monotonic() < deadline and not rospy.is_shutdown():
            remaining = max(0.1, deadline - time.monotonic())
            try:
                latest = rospy.wait_for_message(
                    topic, Path, timeout=min(0.5, remaining)
                )
            except rospy.ROSException:
                continue
            enough_poses = len(latest.poses) >= minimum_poses
            enough_distance = (
                minimum_last_x is None
                or (
                    latest.poses
                    and latest.poses[-1].pose.position.x > minimum_last_x
                )
            )
            if enough_poses and enough_distance:
                return latest
        raise AssertionError(
            "{} did not reach {} poses and x>{} (latest poses={}, x={})".format(
                topic,
                minimum_poses,
                minimum_last_x,
                0 if latest is None else len(latest.poses),
                "n/a"
                if latest is None or not latest.poses
                else latest.poses[-1].pose.position.x,
            )
        )

    def test_local_and_global_paths_start_at_exact_zero(self):
        local_path = self._wait_for_path("/localization/synthetic/local_path", 25)
        global_path = self._wait_for_path(
            "/localization/synthetic/global_path", 6, minimum_last_x=0.8
        )

        self.assertEqual("odom", local_path.header.frame_id)
        self.assertEqual("map", global_path.header.frame_id)
        for path in (local_path, global_path):
            self.assertEqual(0.0, path.poses[0].pose.position.x)
            self.assertEqual(0.0, path.poses[0].pose.position.y)
            self.assertEqual(0.0, path.poses[0].pose.position.z)

        self.assertGreater(local_path.poses[-1].pose.position.x, 1.0)
        self.assertGreater(global_path.poses[-1].pose.position.x, 0.8)
        self.assertAlmostEqual(0.0, global_path.poses[-1].pose.position.y, places=5)
        self.assertGreater(
            local_path.poses[-1].pose.position.x,
            global_path.poses[-1].pose.position.x,
        )

        diagnostics = rospy.wait_for_message(
            "/localization/synthetic/diagnostics", DiagnosticArray, timeout=3.0
        )
        statuses = {status.name: status for status in diagnostics.status}
        expected = {
            "localization/gps_fix",
            "localization/gps_quality",
            "localization/imu",
            "localization/encoder",
            "localization/gps_pose",
            "localization/local_odom",
            "localization/global_odom",
        }
        self.assertEqual(expected, set(statuses))
        self.assertTrue(
            all(status.level == DiagnosticStatus.OK for status in statuses.values()),
            {name: status.message for name, status in statuses.items()},
        )


if __name__ == "__main__":
    rospy.init_node("test_localization_debug_pipeline")
    rostest.rosrun(
        "localization",
        "test_localization_debug_pipeline",
        LocalizationDebugPipelineTest,
    )
