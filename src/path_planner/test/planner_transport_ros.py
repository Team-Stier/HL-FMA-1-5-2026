#!/usr/bin/env python3
"""Exercise path_planner_node invalidation and short-tail handoff over ROS."""
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from planning_interfaces.msg import MissionState, PlannedPath, PathStatus, Route, RouteMap
from visualization_msgs.msg import Marker, MarkerArray


class PlannerTransport(unittest.TestCase):
    def test_clear_blocked_clear_and_short_tail(self):
        seen = {}
        subs = [rospy.Subscriber('/planner_contract/'+key, kind,
                lambda value, k=key: seen.update({k: value}), queue_size=10)
                for key, kind in (('path', PlannedPath), ('status', PathStatus))]
        pubs = {key: rospy.Publisher('/planner_contract/'+key, kind, queue_size=1, latch=key=='map')
                for key, kind in (('map', RouteMap), ('mission', MissionState),
                                  ('odom', Odometry), ('clusters', MarkerArray))}
        route_map = RouteMap()
        route_map.header.frame_id = 'map'
        route = Route()
        route.name, route.section, route.direction = '3_s-static-obstacle', 3, 1
        route.path.header.frame_id = 'map'
        for i in range(121):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.orientation.w = i*.1, 1.
            route.path.poses.append(pose)
        route_map.routes = [route]
        pubs['map'].publish(route_map)

        def exercise(x, blocked, expected_ready):
            start = rospy.Time.now()
            deadline = time.monotonic() + 5.
            while time.monotonic() < deadline:
                stamp = rospy.Time.now()
                mission = MissionState()
                mission.header.stamp, mission.header.frame_id = stamp, 'map'
                mission.route_name, mission.path_mode = route.name, 'LOCAL'
                mission.decision_id, mission.direction, mission.valid = 1, 1, True
                mission.distance_m = x
                odom = Odometry()
                odom.header, odom.child_frame_id = mission.header, 'base_link'
                odom.pose.pose.orientation.w = 1.
                odom.pose.pose.position.x = x
                heartbeat = Marker()
                heartbeat.header = mission.header
                heartbeat.action = Marker.DELETEALL
                markers = MarkerArray(markers=[heartbeat])
                if blocked:
                    box = Marker()
                    box.header, box.ns = mission.header, 'dbscan_clusters'
                    box.action, box.type = Marker.ADD, Marker.POINTS
                    box.pose.orientation.w = 1.
                    box.points = [Point(3.,-2.4,0.), Point(3.4,2.4,0.)]
                    markers.markers.append(box)
                for key, value in (('mission',mission), ('odom',odom), ('clusters',markers)):
                    pubs[key].publish(value)
                time.sleep(.025)
                path, status = seen.get('path'), seen.get('status')
                if (path and status and path.header.stamp > start and status.header.stamp > start
                        and path.header.stamp == status.header.stamp
                        and status.ready == expected_ready
                        and bool(path.path.poses) == expected_ready):
                    return path
            self.fail('Planner contract timed out: '+str(seen))
        try:
            self.assertGreater(len(exercise(0., False, True).path.poses), 2)
            exercise(0., True, False)
            exercise(0., False, True)
            tail = exercise(11.25, False, True)
            self.assertAlmostEqual(tail.path.poses[-1].pose.position.x, 12.)
            # Warning-only CPU budgets must not disable genuine freshness
            # invalidation when the input publishers stop updating.
            deadline = time.monotonic()+3.
            while time.monotonic() < deadline:
                path, status = seen.get('path'), seen.get('status')
                if (status and not status.ready and status.reason == 'MISSION_STATE_STALE'
                        and path and path.header.stamp == status.header.stamp
                        and not path.path.poses):
                    break
                time.sleep(.025)
            else:
                self.fail('Stale inputs did not invalidate the path: '+str(seen))
        finally:
            for endpoint in list(pubs.values()) + subs:
                endpoint.unregister()


if __name__ == '__main__':
    rospy.init_node('planner_transport_test')
    rostest.rosrun('path_planner', 'planner_transport_'+rospy.get_param('~test_case', 'normal'),
                  PlannerTransport)
