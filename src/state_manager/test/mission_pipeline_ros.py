#!/usr/bin/env python3
"""ROS Noetic State Manager/Selector transport smoke test."""
import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from mando_localization.msg import RddfMatch
from nav_msgs.msg import Odometry
from planning_interfaces.msg import MissionState, PathStatus, PlannedPath, Route, RouteMap, SafetyStatus, TrafficConstraint
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray
from stier_state_manager.mission import PARKING_ROUTES, ROUTES


class PipelineSmoke(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.seen = {}
        self.subscribers = []
        topics = {'state': ('/mission/state', MissionState),
                  'inspection': ('/mission/inspection_markers', MarkerArray),
                  'safety': ('/mission/safety', SafetyStatus),
                  'constraint': ('/mission/traffic_constraint', TrafficConstraint),
                  'markers': ('/mission/markers', MarkerArray),
                  'selection': ('/path/selector_status', PathStatus)}
        for key, (topic, kind) in topics.items():
            self.subscribers.append(rospy.Subscriber(topic, kind, self.remember,
                                                     callback_args=key, queue_size=10))
        publishers = {'map': ('/route/map', RouteMap),
                      'observed': ('/molit/localization/rddf/current', RddfMatch),
                      'odom': ('/molit/localization/odometry', Odometry),
                      'scan': ('/molit/sensors/lidar/scan', LaserScan),
                      'valid': ('/molit/localization/valid', Bool),
                      'localization_state': ('/molit/localization/state', String)}
        self.publishers = {key: rospy.Publisher(topic, kind, queue_size=10, latch=key == 'map')
                           for key, (topic, kind) in publishers.items()}

    def tearDown(self):
        for item in self.subscribers:
            item.unregister()
        for item in self.publishers.values():
            item.unregister()

    def remember(self, message, key):
        with self.lock:
            self.seen[key] = message

    def wait_for(self, predicate, pump, timeout=12):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            pump()
            with self.lock:
                if predicate(dict(self.seen)):
                    return
            time.sleep(.03)
        with self.lock:
            summary = {key: str(value)[:300] for key, value in self.seen.items()}
        self.fail('ROS smoke condition timed out: ' + str(summary))

    @staticmethod
    def route_map():
        message = RouteMap()
        message.header.frame_id = 'map'
        message.header.stamp = rospy.Time.now()
        message.origin_latitude, message.origin_longitude = 37.288731, 127.1072336
        names = set(ROUTES.values()) | {'1_left', '1_right', '13_left', '13_right'}
        names.update(name for sides in PARKING_ROUTES.values() for pair in sides.values() for name in pair)
        for name in sorted(names):
            route = Route()
            route.name = name
            route.section = int(name.split('_')[0].split('-')[0])
            route.direction = 1
            route.path.header = message.header
            for x in range(31):
                pose = PoseStamped()
                pose.header = message.header
                pose.pose.position.x = float(x)
                pose.pose.orientation.w = 1.0
                route.path.poses.append(pose)
            message.routes.append(route)
        return message

    def publish_default_inputs(self):
        stamp = rospy.Time.now() - rospy.Duration(.02)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'map', 'base_link'
        odom.pose.pose.orientation.w = 1.0
        odom.pose.covariance[0] = odom.pose.covariance[7] = odom.pose.covariance[35] = .01
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, 'map'
        scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, math.pi / 180
        scan.range_min, scan.range_max = .01, 30.0
        scan.ranges = [math.inf] * 361
        scan.scan_time, scan.time_increment = .01, .01 / 360
        self.publishers['odom'].publish(odom)
        self.publishers['scan'].publish(scan)
        self.publishers['valid'].publish(Bool(data=True))
        self.publishers['localization_state'].publish(String(data='TRACKING'))
        observed = RddfMatch()
        observed.header.stamp, observed.header.frame_id = stamp, 'map'
        observed.pose_stamp = stamp
        observed.matched = observed.has_nearest = True
        observed.route_name = observed.source_route_name = '8_dynamic-obstacle'
        observed.segment_index = 0
        observed.nearest.source_route_name = observed.source_route_name
        observed.nearest.segment_index = observed.segment_index
        observed.nearest.segment_fraction = 0.0
        observed.reason = 'MATCHED'
        self.publishers['observed'].publish(observed)

    def test_state_and_selector_transport(self):
        self.publishers['map'].publish(self.route_map())
        self.wait_for(lambda s: all(k in s for k in ('state', 'safety', 'markers', 'selection', 'constraint'))
                      and s['state'].route_name == '8_dynamic-obstacle' and s['selection'].ready,
                      self.publish_default_inputs)
        with self.lock:
            seen = dict(self.seen)
        # The global calibration flags no longer invalidate an unrelated
        # route. Section 8 still requests stop because its cluster-based
        # E-Stop specifically requires measured vehicle geometry.
        self.assertTrue(seen['state'].valid)
        self.assertTrue(seen['state'].stop_requested)
        self.assertEqual(seen['state'].reason, 'DYNAMIC_OBSTACLE_VEHICLE_CALIBRATION_REQUIRED')
        self.assertTrue(seen['markers'].markers)
        self.assertTrue(seen['safety'].stop)
        self.assertTrue(seen['constraint'].valid)
        self.assertFalse(seen['constraint'].active)
        self.assertEqual(seen['constraint'].route_name, '8_dynamic-obstacle')

        def inspected(s):
            text = '\n'.join(m.text for m in s['inspection'].markers) if 'inspection' in s else ''
            return ('Observed S08: 8_dynamic-obstacle' in text and 'Manager: S08' in text
                    and 'Traffic signal: NO_DATA' in text)
        self.wait_for(inspected, self.publish_default_inputs)
        self.wait_for(lambda s: 'inspection' in s and not any(
            m.ns == 'observed_rddf' for m in s['inspection'].markers), lambda: None)

if __name__ == '__main__':
    rospy.init_node('mission_pipeline_smoke_test')
    rostest.rosrun('state_manager', 'mission_pipeline', PipelineSmoke)
