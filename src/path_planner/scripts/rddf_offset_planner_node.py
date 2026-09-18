#!/usr/bin/env python3
"""Build a simple static-obstacle path by laterally offsetting the RDDF."""

import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from planning_interfaces.msg import MissionState, PlannedPath
from visualization_msgs.msg import Marker, MarkerArray


def yaw_of(pose):
    q = pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def nearest_obstacle_side(path, clusters, corridor_m, lookahead_m):
    if len(path) < 2:
        return 0
    stations = [0.0]
    for a, b in zip(path, path[1:]):
        stations.append(stations[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    nearest = None
    for marker in clusters:
        if (marker.ns != 'dbscan_clusters' or marker.action != Marker.ADD
                or marker.type != Marker.POINTS or not marker.points):
            continue
        ox = sum(point.x for point in marker.points) / len(marker.points)
        oy = sum(point.y for point in marker.points) / len(marker.points)
        for index, (a, b) in enumerate(zip(path, path[1:])):
            vx, vy = b[0] - a[0], b[1] - a[1]
            length2 = vx * vx + vy * vy
            if length2 <= 1e-9:
                continue
            t = max(0.0, min(1.0, ((ox-a[0])*vx + (oy-a[1])*vy) / length2))
            px, py = a[0] + t*vx, a[1] + t*vy
            lateral = (vx*(oy-py) - vy*(ox-px)) / math.sqrt(length2)
            station = stations[index] + t * math.sqrt(length2)
            distance = math.hypot(ox-px, oy-py)
            if distance <= corridor_m and station <= lookahead_m:
                candidate = (station, lateral)
                if nearest is None or candidate[0] < nearest[0]:
                    nearest = candidate
    if nearest is None:
        return 0
    return 1 if nearest[1] >= 0.0 else -1


def offset_path(path, offset_m):
    shifted = []
    for index, point in enumerate(path):
        # The RDDF's first point is a projection of the vehicle onto the path
        # and can be only a few centimetres from the next point.  Use the first
        # complete segment for both leading points so their offset normals
        # cannot invert that short prefix.
        if index == 0 and len(path) > 2:
            point_for_yaw, other = path[1], path[2]
            yaw = math.atan2(other[1] - point_for_yaw[1],
                             other[0] - point_for_yaw[0])
        elif index + 1 < len(path):
            other = path[index + 1]
            yaw = math.atan2(other[1] - point[1], other[0] - point[0])
        else:
            other = path[index - 1]
            yaw = math.atan2(point[1] - other[1], point[0] - other[0])
        shifted.append((point[0] - math.sin(yaw)*offset_m,
                        point[1] + math.cos(yaw)*offset_m, yaw))
    return shifted


class RddfOffsetPlanner:
    def __init__(self):
        self.lock = threading.RLock()
        self.mission = None
        self.rddf = None
        self.clusters = []
        self.offset_m = float(rospy.get_param('~offset_m', 1.0))
        self.corridor_m = float(rospy.get_param('~obstacle_corridor_m', 1.0))
        self.lookahead_m = float(rospy.get_param('~obstacle_lookahead_m', 8.0))
        self.path_pub = rospy.Publisher('/path/local', PlannedPath, queue_size=1)
        self.visualization_pub = rospy.Publisher(
            '/path/local_visualization', Path, queue_size=1)
        rospy.Subscriber('/mission/state', MissionState, self.on_mission, queue_size=1)
        rospy.Subscriber('/path/rddf', PlannedPath, self.on_rddf, queue_size=1)
        rospy.Subscriber('/dbscan_clusters', MarkerArray, self.on_clusters, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish, reset=True)

    def on_mission(self, message):
        with self.lock:
            self.mission = message

    def on_rddf(self, message):
        with self.lock:
            self.rddf = message

    def on_clusters(self, message):
        with self.lock:
            self.clusters = list(message.markers)

    def publish(self, _event):
        with self.lock:
            mission, source = self.mission, self.rddf
            if mission is None or source is None:
                return
            if mission.section != 3 or mission.path_mode != 'LOCAL':
                return
            if (source.decision_id != mission.decision_id
                    or source.route_name != mission.route_name):
                return
            base = [(pose.pose.position.x, pose.pose.position.y, yaw_of(pose.pose))
                    for pose in source.path.poses]
            if len(base) < 2:
                return
            obstacle_side = nearest_obstacle_side(
                base, self.clusters, self.corridor_m, self.lookahead_m)
            # Positive lateral offset is left of the RDDF.
            offset = -self.offset_m if obstacle_side > 0 else (
                self.offset_m if obstacle_side < 0 else 0.0)
            points = offset_path(base, offset) if offset else base
            now = rospy.Time.now()
            output = PlannedPath()
            output.header.stamp, output.header.frame_id = now, 'map'
            output.decision_id = mission.decision_id
            output.route_name = mission.route_name
            output.direction = mission.direction
            output.path.header = output.header
            for x, y, yaw in points:
                pose = PoseStamped()
                pose.header = output.header
                pose.pose.position.x, pose.pose.position.y = x, y
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                output.path.poses.append(pose)
            self.path_pub.publish(output)
            self.visualization_pub.publish(output.path)


if __name__ == '__main__':
    rospy.init_node('rddf_offset_planner')
    RddfOffsetPlanner()
    rospy.spin()
