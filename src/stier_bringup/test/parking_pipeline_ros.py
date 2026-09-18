#!/usr/bin/env python3
"""Recorded RDDF transport/gear replay, not a tire/vehicle-dynamics simulation."""
import json
import math
from pathlib import Path
import sys
import threading
import time
import unittest

import rospkg
import rospy
import rostest
from erp42_msgs.msg import DriveCmd, SerialFeedBack
from mando_localization.msg import RddfMatch
from nav_msgs.msg import Odometry
from planning_interfaces.msg import MissionState, PathStatus, SignalObservation
from std_msgs.msg import Bool, String

sys.path.insert(0, str(Path(rospkg.RosPack().get_path('mando_localization')) / 'scripts'))
from rddf_route_provider import load_catalogue
from stier_state_manager.geometry import Route
from stier_state_manager.mission import PARKING_ROUTES


class VehiclePipelineFixture(unittest.TestCase):
    def setUp(self):
        self.lock = threading.RLock()
        self.seen = {}
        self.trace = []
        self.kind, self.side = rospy.get_param('~kind', 't'), rospy.get_param('~side', 'left')
        _, catalogue = load_catalogue(rospy.get_param('~rddf_directory'))
        self.routes = {name: Route(name, points, direction) for name, direction, points in catalogue}
        self.config = json.loads(Path(rospy.get_param('~mission_config')).read_text())
        self.before = rospy.get_param('~before', '4' if self.kind == 't' else '9')
        self.after = rospy.get_param('~after', '7' if self.kind == 't' else '12')
        station = rospy.get_param('~start_station_m', self.routes[self.before].length-
                                  rospy.get_param('~start_remaining_m', .05))
        self.pose = self.body_pose(self.before, station, 1)
        self.speed = 0.
        self.alive = 0
        self.subscribers = [rospy.Subscriber(topic, message,
            lambda msg, k=key: self.remember(k, msg), queue_size=10)
            for key, topic, message in (
                ('mission', '/mission/state', MissionState),
                ('selection', '/path/selector_status', PathStatus),
                ('match', '/molit/localization/rddf/current', RddfMatch),
                ('drive', '/parking_contract/drive', DriveCmd),
                ('control', '/control/state', String))]
        self.pub = {key: rospy.Publisher(topic, message, queue_size=1) for key, topic, message in (
            ('odom', '/molit/localization/odometry', Odometry),
            ('global', '/molit/localization/global/odometry', Odometry),
            ('valid', '/molit/localization/valid', Bool),
            ('state', '/molit/localization/state', String),
            ('initialization', '/mando_localization/internal/initialization/status', String),
            ('feedback', '/erp42_serial/feedback', SerialFeedBack),
            ('signal', '/perception/traffic_signal', SignalObservation))}
        self.timer = rospy.Timer(rospy.Duration(.025), self.pump)

    def tearDown(self):
        self.timer.shutdown()
        for endpoint in list(self.pub.values()) + self.subscribers:
            endpoint.unregister()

    def remember(self, key, message):
        with self.lock:
            self.seen[key] = message
            if key == 'mission':
                state = (message.route_name, message.parking_leg_index, message.direction,
                         message.stop_requested, message.reason)
                if not self.trace or self.trace[-1] != state:
                    self.trace.append(state)

    def body_pose(self, name, station, direction, lower=0., upper=None):
        route = self.routes[name]
        upper = route.length if upper is None else upper
        x, y, _ = route.pose_at(station)
        a, b = route.pose_at(max(lower, station-.01)), route.pose_at(min(upper, station+.01))
        yaw = math.atan2(direction*(b[1]-a[1]), direction*(b[0]-a[0]))
        return x, y, yaw

    def pump(self, _event):
        with self.lock:
            x, y, yaw = self.pose
            speed = self.speed
        stamp = rospy.Time.now()-rospy.Duration(.01)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'map', 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        odom.twist.twist.linear.x = speed
        odom.pose.covariance[0] = odom.pose.covariance[7] = odom.pose.covariance[35] = .01
        self.pub['odom'].publish(odom)
        self.pub['global'].publish(odom)
        self.pub['valid'].publish(Bool(True))
        self.pub['state'].publish(String('TRACKING'))
        self.pub['initialization'].publish(String(json.dumps({'ready': True, 'route': self.before})))
        feedback = SerialFeedBack()
        feedback.MorA, feedback.speed = 1, speed
        feedback.Gear = 2 if speed < 0 else 0 if speed > 0 else 1
        feedback.alive = self.alive
        self.alive = (self.alive+1) % 256
        self.pub['feedback'].publish(feedback)
        signal = SignalObservation()
        signal.header.stamp, signal.header.frame_id = stamp, 'map'
        with self.lock:
            mission = self.seen.get('mission')
            signal.route_name = mission.route_name if mission and mission.route_name else self.before
        signal.value, signal.confidence = 'GREEN', 1.
        self.pub['signal'].publish(signal)

    def wait_for(self, predicate, description, timeout=10.):
        deadline, consecutive, last = time.monotonic()+timeout, 0, None
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                seen = dict(self.seen)
            command = seen.get('drive')
            if command is not last:
                consecutive = consecutive+1 if predicate(seen) else 0
                last = command
            if consecutive >= 4:
                return seen
            time.sleep(.02)
        self.fail(description+'; trace='+repr(self.trace[-20:])+'; last='+
                  repr({k: str(v)[:700] for k, v in self.seen.items()}))

    def await_drive(self, name, direction, leg=-1):
        def accepted(seen):
            m, d, p = seen.get('mission'), seen.get('drive'), seen.get('selection')
            return (m is not None and d is not None and p is not None and m.valid
                    and m.route_name == name and m.direction == direction
                    and m.parking_leg_index == leg and not m.stop_requested
                    and p.ready and p.decision_id == m.decision_id
                    and d.KPH > 0 and not d.brake and not d.EStop
                    and d.Gear == (0 if direction > 0 else 2))
        return self.wait_for(accepted, 'expected motion '+str((name, direction, leg)))


class ParkingPipeline(VehiclePipelineFixture):
    def test_complete_parking_pair(self):
        pair = PARKING_ROUTES[self.kind][self.side]
        for name in pair:
            route = self.routes[name]
            if self.kind == 't':
                starts = [(0., -1 if route.section == 5 else 1)]
            else:
                profile = self.config['parallel_parking_profiles'][name]
                starts = [(0., profile['initial_direction'])] + [(c['s'], c['direction']) for c in profile['changes']]
            for index, (lower, direction) in enumerate(starts):
                upper = starts[index+1][0] if index+1 < len(starts) else route.length
                leg = index if self.kind == 'parallel' else -1
                # Do not move the pose to force a handoff. New leg/route must
                # be approved while still at the preceding stop point.
                self.await_drive(name, direction, leg)
                stop_tolerance = (.8 if (self.kind == 't' or
                    route.section == 11 and index == len(starts)-1) else .2)
                usable_end = upper-stop_tolerance-.15
                for station in (lower+min(.1, (upper-lower)/4), (lower+usable_end)/2, usable_end):
                    if not lower < station < usable_end+.01:
                        continue
                    with self.lock:
                        self.pose = self.body_pose(name, station, direction, lower, upper)
                        self.speed = direction*.28
                    self.await_drive(name, direction, leg)
                with self.lock:
                    self.pose = self.body_pose(name, upper-.05, direction, lower, upper)
                    self.speed = 0.
        self.wait_for(lambda s: s.get('mission') is not None and
            s['mission'].route_name == self.after and s['mission'].valid,
            'parking exit must hand off to '+self.after)
        self.await_drive(self.after, 1)
        self.assertTrue(any(item[2] == -1 and not item[3] for item in self.trace))
        self.assertTrue(any(item[2] == 1 and not item[3] for item in self.trace))
        print('Parking transport completed:', self.kind, self.side, self.trace)


if __name__ == '__main__':
    rospy.init_node('parking_pipeline_test')
    rostest.rosrun('stier_bringup', 'parking_pipeline_'+rospy.get_param('~course')+'_'+rospy.get_param('~kind')+
                  '_'+rospy.get_param('~side'), ParkingPipeline)
