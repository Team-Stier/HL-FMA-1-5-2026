#!/usr/bin/env python3
"""합성 메시지로 품질 gate, 실행 원점, 회복 동작을 검증하는 rostest."""

import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from sensor_interfaces.msg import GpsStatus
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus


class LocalizationPipelineTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._poses = []
        self._pose_subscriber = rospy.Subscriber(
            "/localization/synthetic/current_pos",
            PoseStamped,
            self._pose_callback,
            queue_size=20,
        )
        self._fix_publisher = rospy.Publisher(
            "/localization/synthetic/input/gps_fix", NavSatFix, queue_size=20
        )
        self._status_publisher = rospy.Publisher(
            "/localization/synthetic/input/gps_status", GpsStatus, queue_size=20
        )
        self._imu_publisher = rospy.Publisher(
            "/localization/synthetic/input/imu", Imu, queue_size=20
        )
        self._wait_for_connections()

    def _pose_callback(self, message):
        with self._lock:
            self._poses.append(message)

    def _pose_count(self):
        with self._lock:
            return len(self._poses)

    def _pose_at(self, index):
        with self._lock:
            return self._poses[index]

    def _wait_for_connections(self):
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if (
                self._fix_publisher.get_num_connections() > 0
                and self._status_publisher.get_num_connections() > 0
                and self._imu_publisher.get_num_connections() > 0
                and self._pose_subscriber.get_num_connections() > 0
            ):
                return
            rospy.sleep(0.05)
        self.fail("localization subscribers did not connect")

    def _wait_for_pose_count(self, expected, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self._pose_count() >= expected:
                return True
            rospy.sleep(0.02)
        return self._pose_count() >= expected

    @staticmethod
    def _status(
        stamp,
        spoofing=GpsStatus.SPOOF_NONE,
        fix_type=3,
        solution=GpsStatus.SOLUTION_RTK_FIXED,
    ):
        message = GpsStatus()
        message.header.stamp = stamp
        message.header.frame_id = "gps_link"
        message.solution = solution
        message.status_text = "SYNTHETIC_TEST"
        message.fix_ok = True
        message.fix_type = fix_type
        message.differential_solution = True
        message.carrier_solution = GpsStatus.CARRIER_FIXED
        message.nav_status_available = True
        message.spoofing_state = spoofing
        message.satellites_used = 15
        message.horizontal_accuracy_m = 0.2
        message.vertical_accuracy_m = 0.4
        return message

    @staticmethod
    def _imu(stamp, yaw=0.4, orientation_variance=0.01):
        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = "imu_link"
        message.orientation.z = math.sin(0.5 * yaw)
        message.orientation.w = math.cos(0.5 * yaw)
        message.orientation_covariance[0] = orientation_variance
        message.orientation_covariance[4] = orientation_variance
        message.orientation_covariance[8] = orientation_variance
        return message

    @staticmethod
    def _fix(stamp, longitude):
        message = NavSatFix()
        message.header.stamp = stamp
        message.header.frame_id = "gps_link"
        message.status.status = NavSatStatus.STATUS_GBAS_FIX
        message.status.service = NavSatStatus.SERVICE_GPS
        message.latitude = 37.2
        message.longitude = longitude
        message.altitude = 45.0
        message.position_covariance[0] = 0.04
        message.position_covariance[4] = 0.04
        message.position_covariance[8] = 0.16
        message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        return message

    def _publish_sample(
        self,
        longitude,
        spoofing=GpsStatus.SPOOF_NONE,
        stamp=None,
        fix_type=3,
        solution=GpsStatus.SOLUTION_RTK_FIXED,
        imu_orientation_variance=0.01,
    ):
        if stamp is None:
            stamp = rospy.Time.now()
        self._status_publisher.publish(
            self._status(stamp, spoofing, fix_type, solution)
        )
        self._imu_publisher.publish(
            self._imu(stamp, orientation_variance=imu_orientation_variance)
        )
        rospy.sleep(0.04)
        self._fix_publisher.publish(self._fix(stamp, longitude))
        rospy.sleep(0.10)
        return stamp

    def test_quality_gated_zero_origin_and_duplicate_recovery(self):
        origin_longitude = 126.8
        for _ in range(2):
            self._publish_sample(origin_longitude)
        self.assertEqual(0, self._pose_count())

        self._publish_sample(origin_longitude)
        self.assertTrue(self._wait_for_pose_count(1))
        origin = self._pose_at(0)
        self.assertEqual("map", origin.header.frame_id)
        self.assertEqual(0.0, origin.pose.position.x)
        self.assertEqual(0.0, origin.pose.position.y)
        self.assertEqual(0.0, origin.pose.position.z)
        self.assertAlmostEqual(math.sin(0.2), origin.pose.orientation.z, places=6)
        self.assertAlmostEqual(math.cos(0.2), origin.pose.orientation.w, places=6)

        longitude_per_meter = 1.0 / (111320.0 * math.cos(math.radians(37.2)))
        self._publish_sample(origin_longitude + longitude_per_meter)
        self.assertTrue(self._wait_for_pose_count(2))
        east = self._pose_at(1)
        self.assertGreater(east.pose.position.x, 0.8)
        self.assertLess(east.pose.position.x, 1.2)

        before_rejection = self._pose_count()
        self._publish_sample(
            origin_longitude + 2.0 * longitude_per_meter,
            imu_orientation_variance=0.0,
        )
        self.assertEqual(before_rejection, self._pose_count())

        self._publish_sample(
            origin_longitude + 2.0 * longitude_per_meter,
            fix_type=2,
            solution=GpsStatus.SOLUTION_RTK_FIXED,
        )
        self.assertEqual(before_rejection, self._pose_count())

        self._publish_sample(
            origin_longitude + 2.0 * longitude_per_meter,
            spoofing=4,
        )
        self.assertEqual(before_rejection, self._pose_count())

        self._publish_sample(
            origin_longitude + 2.0 * longitude_per_meter,
            spoofing=GpsStatus.SPOOF_UNKNOWN,
        )
        self.assertEqual(before_rejection, self._pose_count())

        duplicate_stamp = self._publish_sample(
            origin_longitude + 2.0 * longitude_per_meter
        )
        self._publish_sample(
            origin_longitude + 2.0 * longitude_per_meter,
            stamp=duplicate_stamp,
        )
        self._publish_sample(origin_longitude + 2.1 * longitude_per_meter)
        self._publish_sample(origin_longitude + 2.2 * longitude_per_meter)
        self.assertEqual(before_rejection, self._pose_count())

        self._publish_sample(origin_longitude + 2.3 * longitude_per_meter)
        self.assertTrue(self._wait_for_pose_count(before_rejection + 1))

        before_far_jump = self._pose_count()
        for offset_m in (100.0, 100.1, 100.2):
            self._publish_sample(
                origin_longitude + offset_m * longitude_per_meter
            )
        self.assertEqual(before_far_jump, self._pose_count())


if __name__ == "__main__":
    rospy.init_node("test_localization_pipeline")
    rostest.rosrun(
        "localization", "test_localization_pipeline", LocalizationPipelineTest
    )
