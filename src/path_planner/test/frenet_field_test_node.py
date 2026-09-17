#!/usr/bin/env python3

"""Vehicle-frame adapter for a no-localization Frenet field test."""

import threading

import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from planning_interfaces.msg import (
    MissionState,
    PathStatus,
    PlannedPath,
    Route,
    RouteMap,
)
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import SetBool, SetBoolResponse
from visualization_msgs.msg import Marker, MarkerArray


FRAME_ID = "base_link"
ROUTE_NAME = "frenet_field_test"


class FrenetFieldTestNode:
    def __init__(self):
        self.lock = threading.RLock()
        self.auto_start = bool(rospy.get_param("~auto_start", True))
        self.armed = self.auto_start
        self.planner_ready = False
        self.planner_reason = "WAITING_FOR_PATH_PLANNER"
        self.planner_status_time = rospy.Time(0)
        self.control_state = "WAITING_FOR_CONTROL"
        self.speed_limit_mps = float(rospy.get_param("~speed_limit_mps", 1.40))
        self.road_left_m = float(rospy.get_param("~road_left_bound_m", 3.0))
        self.road_right_m = float(rospy.get_param("~road_right_bound_m", 3.0))
        self.vehicle_length_m = float(rospy.get_param("~vehicle_length_m", 1.35))
        self.vehicle_width_m = float(rospy.get_param("~vehicle_width_m", 0.85))
        self.wheelbase_m = float(rospy.get_param("~wheelbase_m", 0.75))
        self.rear_axle_to_center_m = float(
            rospy.get_param("~rear_axle_to_center_m", 0.38)
        )

        self.route_pub = rospy.Publisher("/route/map", RouteMap, queue_size=1, latch=True)
        self.mission_pub = rospy.Publisher("/mission/state", MissionState, queue_size=1)
        self.odometry_pub = rospy.Publisher(
            "/frenet_test/odometry", Odometry, queue_size=1
        )
        self.final_path_pub = rospy.Publisher("/path/final", Path, queue_size=1)
        self.reference_pub = rospy.Publisher(
            "/frenet_test/reference_path", Path, queue_size=1, latch=True
        )
        self.planned_path_pub = rospy.Publisher(
            "/frenet_test/planned_path", Path, queue_size=1, latch=True
        )
        self.marker_pub = rospy.Publisher(
            "/frenet_test/markers", MarkerArray, queue_size=1
        )

        rospy.Subscriber("/path/local", PlannedPath, self._path_callback, queue_size=1)
        rospy.Subscriber(
            "/path_planner/status", PathStatus, self._status_callback, queue_size=1
        )
        rospy.Subscriber("/control/state", String, self._control_callback, queue_size=1)
        self.run_service = rospy.Service("/frenet_test/run", SetBool, self._set_run)

        self.reference_path = self._make_reference_path()
        self._publish_route()
        self.reference_pub.publish(self.reference_path)
        self.timer = rospy.Timer(rospy.Duration(0.05), self._timer_callback)
        if self.auto_start:
            rospy.logwarn(
                "Frenet field test auto-start is enabled. The vehicle can move at "
                "5 km/h when the planner is ready and the controller enters ROS mode."
            )
        else:
            rospy.logwarn(
                "Frenet field test is DISARMED. No localization is used; all planning "
                "runs in base_link. Call /frenet_test/run with data=true after RViz checks."
            )

    @staticmethod
    def _pose(x, y, stamp):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = FRAME_ID
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    def _make_reference_path(self):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = FRAME_ID
        x = -2.0
        while x <= 20.0 + 1.0e-9:
            path.poses.append(self._pose(x, 0.0, path.header.stamp))
            x += 0.25
        return path

    def _publish_route(self):
        message = RouteMap()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = FRAME_ID
        route = Route()
        route.name = ROUTE_NAME
        route.section = 0
        route.direction = 1
        route.path = self.reference_path
        message.routes = [route]
        self.route_pub.publish(message)

    def _path_callback(self, message):
        if (
            message.decision_id != 1
            or message.route_name != ROUTE_NAME
            or message.direction != 1
            or message.path.header.frame_id != FRAME_ID
        ):
            return
        self.planned_path_pub.publish(message.path)
        self.final_path_pub.publish(message.path)

    def _status_callback(self, message):
        with self.lock:
            self.planner_ready = message.ready
            self.planner_reason = message.reason
            self.planner_status_time = rospy.Time.now()
        if not message.ready:
            empty = Path()
            empty.header.stamp = rospy.Time.now()
            empty.header.frame_id = FRAME_ID
            self.final_path_pub.publish(empty)
            self.planned_path_pub.publish(empty)

    def _control_callback(self, message):
        with self.lock:
            self.control_state = message.data

    def _set_run(self, request):
        with self.lock:
            if not request.data:
                self.armed = False
                return SetBoolResponse(success=True, message="DISARMED: brake requested")
            status_age = (rospy.Time.now() - self.planner_status_time).to_sec()
            if not self.planner_ready or status_age > 0.5:
                return SetBoolResponse(
                    success=False,
                    message="planner not ready: {}".format(self.planner_reason),
                )
            self.armed = True
            return SetBoolResponse(success=True, message="ARMED: 5 km/h field test")

    @staticmethod
    def _odometry(stamp):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = FRAME_ID
        message.child_frame_id = FRAME_ID
        message.pose.pose.orientation.w = 1.0
        return message

    def _mission(self, stamp):
        with self.lock:
            armed = self.armed
        message = MissionState()
        message.header.stamp = stamp
        message.header.frame_id = FRAME_ID
        message.decision_id = 1
        message.route_name = ROUTE_NAME
        message.section = 0
        message.mission = "FRENET_FIELD_TEST"
        message.phase = "RUN" if armed else "DISARMED"
        message.path_mode = "LOCAL"
        message.direction = 1
        message.progress = 2.0 / 22.0
        message.distance_m = 2.0
        message.speed_limit_mps = self.speed_limit_mps
        message.remaining_stop_m = -1.0
        message.stop_requested = not armed
        message.emergency_stop_requested = False
        message.valid = True
        message.finished = False
        message.reason = "FIELD_TEST_ARMED" if armed else "FIELD_TEST_DISARMED"
        message.parking_leg_index = -1
        return message

    @staticmethod
    def _line(stamp, marker_id, namespace, y, color):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = FRAME_ID
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.06
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = [Point(x=-2.0, y=y), Point(x=20.0, y=y)]
        return marker

    def _markers(self, stamp):
        output = MarkerArray()
        output.markers.append(
            self._line(stamp, 0, "left_test_bound", self.road_left_m, (0.2, 0.7, 1.0, 0.9))
        )
        output.markers.append(
            self._line(
                stamp,
                0,
                "right_test_bound",
                -self.road_right_m,
                (0.2, 0.7, 1.0, 0.9),
            )
        )
        half_length = self.vehicle_length_m * 0.5
        half_width = self.vehicle_width_m * 0.5
        rear = self.rear_axle_to_center_m - half_length
        front = self.rear_axle_to_center_m + half_length
        body = Marker()
        body.header.stamp = stamp
        body.header.frame_id = FRAME_ID
        body.ns = "field_test_vehicle_body"
        body.id = 0
        body.type = Marker.LINE_STRIP
        body.action = Marker.ADD
        body.pose.orientation.w = 1.0
        body.scale.x = 0.045
        body.color = ColorRGBA(0.85, 0.88, 0.92, 1.0)
        body.points = [
            Point(rear, -half_width, 0.08),
            Point(front, -half_width, 0.08),
            Point(front, half_width, 0.08),
            Point(rear, half_width, 0.08),
            Point(rear, -half_width, 0.08),
        ]
        output.markers.append(body)

        rear_axle = self._line(
            stamp, 0, "field_test_rear_axle", 0.0, (0.2, 0.85, 1.0, 1.0)
        )
        rear_axle.points = [
            Point(0.0, -half_width, 0.10),
            Point(0.0, half_width, 0.10),
        ]
        rear_axle.scale.x = 0.07
        output.markers.append(rear_axle)

        front_axle = self._line(
            stamp, 0, "field_test_front_axle", 0.0, (0.25, 1.0, 0.35, 1.0)
        )
        front_axle.points = [
            Point(self.wheelbase_m, -half_width, 0.10),
            Point(self.wheelbase_m, half_width, 0.10),
        ]
        front_axle.scale.x = 0.07
        output.markers.append(front_axle)
        with self.lock:
            armed = self.armed
            planner_reason = self.planner_reason
            control_state = self.control_state
        text = Marker()
        text.header.stamp = stamp
        text.header.frame_id = FRAME_ID
        text.ns = "field_test_status"
        text.id = 0
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = 4.0
        text.pose.position.y = 3.7
        text.pose.position.z = 0.4
        text.pose.orientation.w = 1.0
        text.scale.z = 0.38
        text.color.r = 0.2 if armed else 1.0
        text.color.g = 1.0 if armed else 0.45
        text.color.b = 0.2
        text.color.a = 1.0
        text.text = "{} | planner={} | control={}".format(
            "ARMED" if armed else "DISARMED", planner_reason, control_state
        )
        output.markers.append(text)
        return output

    def _timer_callback(self, _event):
        if rospy.is_shutdown():
            return
        stamp = rospy.Time.now()
        try:
            self.odometry_pub.publish(self._odometry(stamp))
            self.mission_pub.publish(self._mission(stamp))
            self.marker_pub.publish(self._markers(stamp))
        except rospy.ROSException:
            # roslaunch closes publishers while the timer thread is winding down.
            if not rospy.is_shutdown():
                raise


if __name__ == "__main__":
    rospy.init_node("frenet_field_test")
    FrenetFieldTestNode()
    rospy.spin()
