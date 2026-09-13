#!/usr/bin/env python3
"""Visualize a recorded NavSatFix as a first-fix-relative GPS-only path."""

import copy
import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_interfaces.msg import GpsStatus
from sensor_msgs.msg import NavSatFix, NavSatStatus
from visualization_msgs.msg import Marker, MarkerArray


WGS84_A_M = 6378137.0
WGS84_E2 = 6.69437999014e-3


class GpsBagRvizReplay:
    def __init__(self):
        self._datum = None
        self._status = None
        self._path = Path()
        self._path.header.frame_id = "map"
        self._path_publisher = rospy.Publisher(
            "/localization/replay/gps_path", Path, queue_size=1, latch=True
        )
        self._marker_publisher = rospy.Publisher(
            "/localization/replay/gps_markers", MarkerArray, queue_size=5
        )
        self._status_subscriber = rospy.Subscriber(
            "/gps/status", GpsStatus, self._status_callback, queue_size=20
        )
        self._fix_subscriber = rospy.Subscriber(
            "/ublox_position_receiver/fix",
            NavSatFix,
            self._fix_callback,
            queue_size=20,
        )
        rospy.logwarn(
            "GPS POSITION ONLY: IMU가 없는 bag이므로 방향은 표시하지 않습니다"
        )

    def _status_callback(self, message):
        self._status = copy.deepcopy(message)

    @staticmethod
    def _valid_fix(message):
        return (
            message.status.status >= NavSatStatus.STATUS_FIX
            and math.isfinite(message.latitude)
            and math.isfinite(message.longitude)
            and math.isfinite(message.altitude)
            and -90.0 <= message.latitude <= 90.0
            and -180.0 <= message.longitude <= 180.0
        )

    def _project(self, latitude, longitude):
        datum_latitude, datum_longitude = self._datum
        latitude_rad = math.radians(datum_latitude)
        sine = math.sin(latitude_rad)
        denominator = math.sqrt(1.0 - WGS84_E2 * sine * sine)
        prime_vertical = WGS84_A_M / denominator
        meridian = WGS84_A_M * (1.0 - WGS84_E2) / denominator ** 3
        east = (
            prime_vertical
            * math.cos(latitude_rad)
            * math.radians(longitude - datum_longitude)
        )
        north = meridian * math.radians(latitude - datum_latitude)
        return east, north

    def _fix_callback(self, message):
        if not self._valid_fix(message):
            return
        if self._status is not None and not self._status.fix_ok:
            return
        if self._datum is None:
            self._datum = (message.latitude, message.longitude)
            rospy.loginfo(
                "Replay 첫 GPS를 (0,0)으로 설정: %.9f, %.9f",
                message.latitude,
                message.longitude,
            )
        east, north = self._project(message.latitude, message.longitude)
        pose = PoseStamped()
        pose.header = copy.deepcopy(message.header)
        pose.header.frame_id = "map"
        pose.pose.position.x = east
        pose.pose.position.y = north
        pose.pose.orientation.w = 1.0
        self._path.header.stamp = pose.header.stamp
        self._path.poses.append(pose)
        self._path_publisher.publish(self._path)
        self._marker_publisher.publish(self._markers(message, east, north))

    def _markers(self, fix, east, north):
        output = MarkerArray()

        point = Marker()
        point.header.stamp = fix.header.stamp
        point.header.frame_id = "map"
        point.ns = "gps_replay"
        point.id = 0
        point.type = Marker.SPHERE
        point.action = Marker.ADD
        point.pose.position.x = east
        point.pose.position.y = north
        point.pose.position.z = 0.15
        point.pose.orientation.w = 1.0
        point.scale.x = 0.35
        point.scale.y = 0.35
        point.scale.z = 0.35
        point.color.r = 1.0
        point.color.g = 0.15
        point.color.b = 0.1
        point.color.a = 1.0
        output.markers.append(point)

        status = self._status
        label = "GPS POSITION ONLY"
        if status is not None:
            label += "\n{}  sats={}  hAcc={:.2f}m".format(
                status.status_text,
                status.satellites_used,
                status.horizontal_accuracy_m,
            )
        label += "\nENU x={:.2f}m y={:.2f}m".format(east, north)

        text = Marker()
        text.header = copy.deepcopy(point.header)
        text.ns = "gps_replay"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = east
        text.pose.position.y = north
        text.pose.position.z = 1.0
        text.pose.orientation.w = 1.0
        text.scale.z = 0.4
        text.color.r = 1.0
        text.color.g = 0.9
        text.color.b = 0.2
        text.color.a = 1.0
        text.text = label
        output.markers.append(text)
        return output


if __name__ == "__main__":
    rospy.init_node("gps_bag_rviz_replay")
    GpsBagRvizReplay()
    rospy.spin()
