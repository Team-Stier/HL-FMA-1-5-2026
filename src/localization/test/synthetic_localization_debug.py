#!/usr/bin/env python3
"""Publish isolated synthetic GPS, IMU, and encoder inputs for regression tests."""

import math

import rospy
from erp42_msgs.msg import SerialFeedBack
from geometry_msgs.msg import Quaternion
from sensor_interfaces.msg import GpsStatus
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus


WGS84_SEMI_MAJOR_AXIS_M = 6378137.0
WGS84_ECCENTRICITY_SQUARED = 6.69437999014e-3


def yaw_quaternion(yaw):
    result = Quaternion()
    result.z = math.sin(0.5 * yaw)
    result.w = math.cos(0.5 * yaw)
    return result


class SyntheticLocalizationInputs:
    def __init__(self):
        self._latitude = float(rospy.get_param("~latitude_deg", 37.239000000))
        self._longitude = float(rospy.get_param("~longitude_deg", 126.773000000))
        self._altitude = float(rospy.get_param("~altitude_m", 50.0))
        self._speed = float(rospy.get_param("~speed_mps", 1.0))
        self._encoder_meters_per_tick = float(
            rospy.get_param("~encoder_meters_per_tick", 0.01)
        )
        self._encoder_scale = float(rospy.get_param("~encoder_scale", 1.02))
        if not all(
            math.isfinite(value)
            for value in (
                self._latitude,
                self._longitude,
                self._altitude,
                self._speed,
                self._encoder_meters_per_tick,
                self._encoder_scale,
            )
        ):
            raise ValueError("synthetic parameters must be finite")
        if (
            self._speed <= 0.0
            or self._encoder_meters_per_tick <= 0.0
            or self._encoder_scale <= 0.0
        ):
            raise ValueError("synthetic motion parameters must be positive")

        latitude_rad = math.radians(self._latitude)
        denominator = math.sqrt(
            1.0
            - WGS84_ECCENTRICITY_SQUARED * math.sin(latitude_rad) ** 2
        )
        radius = WGS84_SEMI_MAJOR_AXIS_M / denominator
        self._longitude_deg_per_meter = math.degrees(
            1.0 / (radius * math.cos(latitude_rad))
        )

        self._fix_publisher = rospy.Publisher(
            "/localization/synthetic/input/gps_fix", NavSatFix, queue_size=20
        )
        self._status_publisher = rospy.Publisher(
            "/localization/synthetic/input/gps_status", GpsStatus, queue_size=30
        )
        self._imu_publisher = rospy.Publisher(
            "/localization/synthetic/input/imu", Imu, queue_size=100
        )
        self._encoder_publisher = rospy.Publisher(
            "/localization/synthetic/input/encoder",
            SerialFeedBack,
            queue_size=100,
        )
        self._start_time = rospy.Time.now()
        self._alive = 0
        self._state_timer = rospy.Timer(rospy.Duration(0.05), self._publish_state)
        self._fix_timer = rospy.Timer(rospy.Duration(0.20), self._publish_fix)
        rospy.on_shutdown(self._shutdown)
        rospy.logwarn(
            "SYNTHETIC TEST ONLY: 격리된 /localization/synthetic/input/* 토픽을 "
            "발행합니다"
        )

    def _distance(self, stamp):
        return max(0.0, (stamp - self._start_time).to_sec()) * self._speed

    def _publish_state(self, _event):
        stamp = rospy.Time.now()

        status = GpsStatus()
        status.header.stamp = stamp
        status.header.frame_id = "gps_link"
        status.solution = GpsStatus.SOLUTION_RTK_FIXED
        status.status_text = "SYNTHETIC_TEST_RTK_FIXED"
        status.fix_ok = True
        status.fix_type = 3
        status.differential_solution = True
        status.carrier_solution = GpsStatus.CARRIER_FIXED
        status.nav_status_available = True
        status.spoofing_state = GpsStatus.SPOOF_NONE
        status.satellites_used = 15
        status.horizontal_accuracy_m = 0.20
        status.vertical_accuracy_m = 0.40
        self._safe_publish(self._status_publisher, status)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        imu.orientation = yaw_quaternion(0.0)
        imu.orientation_covariance[0] = 0.01
        imu.orientation_covariance[4] = 0.01
        imu.orientation_covariance[8] = 0.02
        self._safe_publish(self._imu_publisher, imu)

        encoder = SerialFeedBack()
        encoder.MorA = 1
        encoder.EStop = 0
        encoder.Gear = 0
        encoder.speed = self._speed * 3.6
        encoder.steer = 0.0
        encoder.brake = 0
        encoder.encoder = int(
            round(
                self._distance(stamp)
                * self._encoder_scale
                / self._encoder_meters_per_tick
            )
        )
        encoder.alive = self._alive % 256
        self._alive += 1
        self._safe_publish(self._encoder_publisher, encoder)

    def _publish_fix(self, _event):
        stamp = rospy.Time.now()
        distance = self._distance(stamp)
        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = "gps_link"
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = self._latitude
        fix.longitude = self._longitude + distance * self._longitude_deg_per_meter
        fix.altitude = self._altitude
        fix.position_covariance[0] = 0.04
        fix.position_covariance[4] = 0.04
        fix.position_covariance[8] = 0.16
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._safe_publish(self._fix_publisher, fix)

    @staticmethod
    def _safe_publish(publisher, message):
        if rospy.is_shutdown():
            return
        try:
            publisher.publish(message)
        except rospy.ROSException:
            return

    def _shutdown(self):
        self._state_timer.shutdown()
        self._fix_timer.shutdown()


if __name__ == "__main__":
    rospy.init_node("synthetic_localization_debug")
    try:
        SyntheticLocalizationInputs()
        rospy.spin()
    except (TypeError, ValueError, rospy.ROSException) as error:
        rospy.logfatal("합성 localization 입력 설정 실패: %s", error)
        raise
