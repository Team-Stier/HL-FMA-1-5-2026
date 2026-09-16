#!/usr/bin/env python3
"""ROS Noetic transport smoke test; fixtures are synthetic, output is preview.

The production-default stack must stay stopped without field calibration. An
isolated selector/gate pair then proves real ROS messages can authorize preview
commands, and loss of required input stops them. No physical driver is launched.
"""
import json
import math
import threading
import time
import unittest

import rosgraph
import rospy
import rostest
from erp42_msgs.msg import DriveCmd
from geometry_msgs.msg import PoseStamped
from mando_localization.msg import RddfMatch
from nav_msgs.msg import Odometry
from planning_interfaces.msg import MissionState, PathStatus, PlannedPath, Route, RouteMap, SafetyStatus, TrafficConstraint
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray
from selector.core import Candidate, Pose, path_fingerprint
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
                  'selection': ('/path/selector_status', PathStatus),
                  'preview': ('/vehicle_safety/preview_drive', DriveCmd),
                  'probe_status': ('/smoke/status', String),
                  'probe_preview': ('/smoke/preview_drive', DriveCmd)}
        for key, (topic, kind) in topics.items():
            self.subscribers.append(rospy.Subscriber(topic, kind, self.remember,
                                                     callback_args=key, queue_size=10))
        publishers = {'map': ('/route/map', RouteMap),
                      'observed': ('/molit/localization/rddf/current', RddfMatch),
                      'odom': ('/molit/localization/odometry', Odometry),
                      'scan': ('/molit/sensors/lidar/scan', LaserScan),
                      'valid': ('/molit/localization/valid', Bool),
                      'probe_state': ('/smoke/mission/state', MissionState),
                      'probe_safety': ('/smoke/mission/safety', SafetyStatus),
                      'probe_path': ('/smoke/path/rddf', PlannedPath),
                      'probe_valid': ('/smoke/localization/valid', Bool),
                      'probe_raw': ('/smoke/raw_drive', DriveCmd)}
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
        observed = RddfMatch()
        observed.header.stamp, observed.header.frame_id = stamp, 'map'
        observed.pose_stamp = stamp
        observed.matched = observed.has_nearest = True
        observed.route_name = observed.source_route_name = '7'
        observed.reason = 'MATCHED'
        self.publishers['observed'].publish(observed)

    def publish_probe(self, safety=True, raw=True, stop=False):
        now = rospy.Time.now()
        state = MissionState()
        state.header.stamp, state.header.frame_id = now, 'map'
        state.decision_id, state.route_name, state.section = 71, '2', 2
        state.path_mode, state.direction, state.valid = 'RDDF', 1, True
        state.speed_limit_mps = 1.0
        path = PlannedPath()
        path.header = state.header
        path.decision_id, path.route_name, path.direction = 71, '2', 1
        path.path.header = state.header
        for x in (0.0, 1.0, 2.0):
            pose = PoseStamped()
            pose.header = state.header
            pose.pose.position.x = x
            pose.pose.orientation.w = 1.0
            path.path.poses.append(pose)
        candidate = Candidate(now.to_sec(), now.to_sec(), 71, '2', 1, 'map', 'map', now.to_sec(),
                              tuple(Pose('map', (x, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
                                    for x in (0.0, 1.0, 2.0)))
        check = SafetyStatus()
        check.header = state.header
        check.stop, check.sensor_valid = stop, True
        check.path_fingerprint = path_fingerprint(candidate)
        self.publishers['probe_state'].publish(state)
        self.publishers['probe_path'].publish(path)
        self.publishers['probe_valid'].publish(Bool(data=True))
        if safety:
            self.publishers['probe_safety'].publish(check)
        if raw:
            command = DriveCmd()
            command.KPH, command.Deg, command.brake = 10, 0, 0
            self.publishers['probe_raw'].publish(command)

    @staticmethod
    def status(seen):
        return json.loads(seen['probe_status'].data) if 'probe_status' in seen else {}

    def test_transport_fail_closed_and_preview_recovery(self):
        self.publishers['map'].publish(self.route_map())
        self.wait_for(lambda s: all(k in s for k in ('state', 'safety', 'markers', 'selection', 'preview', 'constraint'))
                      and s['state'].route_name == '1_right' and s['selection'].ready,
                      self.publish_default_inputs)
        with self.lock:
            seen = dict(self.seen)
        self.assertFalse(seen['state'].valid)
        self.assertTrue(seen['state'].stop_requested)
        self.assertNotIn('EXCEPTION', seen['state'].reason)
        self.assertTrue(seen['markers'].markers)
        self.assertTrue(seen['safety'].stop)
        self.assertFalse(seen['constraint'].valid)
        self.assertTrue(seen['constraint'].active)
        self.assertEqual(seen['constraint'].route_name, '1_right')
        self.assertEqual((seen['preview'].KPH, seen['preview'].brake), (0, 1))

        def inspected(s):
            text = '\n'.join(m.text for m in s['inspection'].markers) if 'inspection' in s else ''
            return ('Observed S07: 7' in text and 'Manager: S01' in text
                    and 'Traffic signal: NO_DATA' in text)
        self.wait_for(inspected, self.publish_default_inputs)
        self.wait_for(lambda s: 'inspection' in s and not any(
            m.ns == 'observed_rddf' for m in s['inspection'].markers), lambda: None)

        self.wait_for(lambda s: self.status(s).get('allowed') is True
                      and 'probe_preview' in s and s['probe_preview'].KPH == 3
                      and s['probe_preview'].brake == 0, self.publish_probe)
        self.wait_for(lambda s: self.status(s).get('reason') == 'SAFETY_MISSING_STALE_OR_FUTURE'
                      and s['probe_preview'].KPH == 0 and s['probe_preview'].brake == 1,
                      lambda: self.publish_probe(safety=False))
        self.wait_for(lambda s: self.status(s).get('reason') in
                      ('FRESH_RAW_COMMAND_REQUIRED', 'RAW_COMMAND_MISSING_OR_STALE'),
                      lambda: self.publish_probe(raw=False))
        self.wait_for(lambda s: self.status(s).get('allowed') is True, self.publish_probe)
        publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        self.assertNotIn('/erp42_serial/drive', {topic for topic, _ in publishers},
                         'Preview test must never create a vehicle command publisher')


if __name__ == '__main__':
    rospy.init_node('mission_pipeline_smoke_test')
    rostest.rosrun('state_manager', 'mission_pipeline', PipelineSmoke)
