#!/usr/bin/env python3

import threading

import rospy
from sensor_interfaces.msg import GpsStatus
from ublox_msgs.msg import NavPVT, NavSTATUS


GPS_WEEK_MILLISECONDS = 604800000


class GpsStatusNode:
    def __init__(self):
        self._frame_id = rospy.get_param("~frame_id", "gps_link")
        self._nav_status_timeout = rospy.Duration(
            rospy.get_param("~nav_status_timeout", 0.5)
        )
        self._max_nav_status_itow_skew_ms = int(
            rospy.get_param("~max_nav_status_itow_skew_ms", 500)
        )
        if self._nav_status_timeout.to_sec() <= 0.0:
            raise ValueError("nav_status_timeout must be positive")
        if self._max_nav_status_itow_skew_ms < 0:
            raise ValueError("max_nav_status_itow_skew_ms must be nonnegative")
        self._latest_nav_status = None
        self._latest_nav_status_time = rospy.Time(0)
        self._lock = threading.Lock()

        navpvt_topic = rospy.get_param(
            "~navpvt_topic", "/ublox_position_receiver/navpvt"
        )
        navstatus_topic = rospy.get_param(
            "~navstatus_topic", "/ublox_position_receiver/navstatus"
        )
        status_topic = rospy.get_param("~status_topic", "/gps/status")

        self._publisher = rospy.Publisher(
            status_topic, GpsStatus, queue_size=10
        )
        self._nav_status_subscriber = rospy.Subscriber(
            navstatus_topic, NavSTATUS, self._nav_status_callback, queue_size=10
        )
        self._nav_pvt_subscriber = rospy.Subscriber(
            navpvt_topic, NavPVT, self._nav_pvt_callback, queue_size=10
        )

        rospy.loginfo(
            "GPS status: %s + %s -> %s",
            navpvt_topic,
            navstatus_topic,
            status_topic,
        )

    def _nav_status_callback(self, message):
        with self._lock:
            self._latest_nav_status = message
            self._latest_nav_status_time = rospy.Time.now()

    def _nav_pvt_callback(self, message):
        now = rospy.Time.now()
        output = GpsStatus()
        output.header.stamp = now
        output.header.frame_id = self._frame_id

        output.fix_type = message.fixType
        output.fix_ok = bool(message.flags & NavPVT.FLAGS_GNSS_FIX_OK)
        output.differential_solution = bool(
            message.flags & NavPVT.FLAGS_DIFF_SOLN
        )

        output.carrier_solution = self._carrier_solution(message.flags)
        output.solution, output.status_text = self._solution_status(
            output.fix_ok,
            output.fix_type,
            output.differential_solution,
            output.carrier_solution,
        )

        output.satellites_used = message.numSV
        output.horizontal_accuracy_m = message.hAcc * 1e-3
        output.vertical_accuracy_m = message.vAcc * 1e-3
        output.position_dop = message.pDOP * 1e-2
        output.speed_accuracy_mps = message.sAcc * 1e-3
        output.heading_accuracy_deg = message.headAcc * 1e-5

        nav_status = self._current_nav_status(now, message.iTOW)
        output.nav_status_available = nav_status is not None
        if nav_status is None:
            output.spoofing_state = GpsStatus.SPOOF_UNKNOWN
        else:
            output.spoofing_state = self._spoofing_state(nav_status.flags2)
            output.time_to_first_fix_ms = nav_status.ttff

        self._publisher.publish(output)

    def _current_nav_status(self, now, nav_pvt_itow):
        with self._lock:
            nav_status = self._latest_nav_status
            received_at = self._latest_nav_status_time

        if nav_status is None:
            return None
        if now < received_at:
            return None
        if now - received_at > self._nav_status_timeout:
            return None
        if (
            self._itow_distance_ms(nav_status.iTOW, nav_pvt_itow)
            > self._max_nav_status_itow_skew_ms
        ):
            return None
        return nav_status

    @staticmethod
    def _itow_distance_ms(first_itow, second_itow):
        first = int(first_itow) % GPS_WEEK_MILLISECONDS
        second = int(second_itow) % GPS_WEEK_MILLISECONDS
        difference = abs(first - second)
        return min(difference, GPS_WEEK_MILLISECONDS - difference)

    @staticmethod
    def _carrier_solution(flags):
        carrier_bits = flags & NavPVT.FLAGS_CARRIER_PHASE_MASK
        if carrier_bits == NavPVT.CARRIER_PHASE_FLOAT:
            return GpsStatus.CARRIER_FLOAT
        if carrier_bits == NavPVT.CARRIER_PHASE_FIXED:
            return GpsStatus.CARRIER_FIXED
        return GpsStatus.CARRIER_NONE

    @staticmethod
    def _spoofing_state(flags2):
        spoof_bits = flags2 & NavSTATUS.FLAGS2_SPOOF_DET_STATE_MASK
        if spoof_bits == NavSTATUS.SPOOF_DET_STATE_NONE:
            return GpsStatus.SPOOF_NONE
        if spoof_bits == NavSTATUS.SPOOF_DET_STATE_SPOOFING:
            return GpsStatus.SPOOF_INDICATED
        if spoof_bits == NavSTATUS.SPOOF_DET_STATE_MULTIPLE:
            return GpsStatus.SPOOF_MULTIPLE
        return GpsStatus.SPOOF_UNKNOWN

    @staticmethod
    def _solution_status(fix_ok, fix_type, differential, carrier):
        if not fix_ok or fix_type == NavPVT.FIX_TYPE_NO_FIX:
            return GpsStatus.SOLUTION_NO_FIX, "NO_FIX"
        if fix_type == NavPVT.FIX_TYPE_DEAD_RECKONING_ONLY:
            return GpsStatus.SOLUTION_DEAD_RECKONING, "DEAD_RECKONING"
        if fix_type == NavPVT.FIX_TYPE_TIME_ONLY:
            return GpsStatus.SOLUTION_TIME_ONLY, "TIME_ONLY"
        if carrier == GpsStatus.CARRIER_FIXED:
            return GpsStatus.SOLUTION_RTK_FIXED, "RTK_FIXED"
        if carrier == GpsStatus.CARRIER_FLOAT:
            return GpsStatus.SOLUTION_RTK_FLOAT, "RTK_FLOAT"
        if fix_type == NavPVT.FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED:
            return (
                GpsStatus.SOLUTION_GNSS_DEAD_RECKONING,
                "GNSS_DEAD_RECKONING",
            )
        if differential:
            return GpsStatus.SOLUTION_DGNSS, "DGNSS"
        if fix_type == NavPVT.FIX_TYPE_2D:
            return GpsStatus.SOLUTION_2D, "2D_FIX"
        return GpsStatus.SOLUTION_3D, "3D_FIX"


if __name__ == "__main__":
    rospy.init_node("gps_status")
    GpsStatusNode()
    rospy.spin()
