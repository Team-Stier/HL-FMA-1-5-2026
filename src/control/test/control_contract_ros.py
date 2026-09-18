#!/usr/bin/env python3
"""Real control_node, synthetic messages, never connected to an Arduino."""
import math
import time
import unittest

import rospy
import rostest
from erp42_msgs.msg import DriveCmd, SerialFeedBack
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from planning_interfaces.msg import MissionState
from std_msgs.msg import String


class ControlContract(unittest.TestCase):
    def test_adaptive_speed_ceiling_curves_and_stop_approach(self):
        latest = {}
        subscriber = rospy.Subscriber('/control_contract/drive', DriveCmd,
                                      lambda msg: latest.update(drive=msg), queue_size=10)
        publishers = {name: rospy.Publisher('/control_contract/'+name, kind, queue_size=1)
                      for name, kind in (('path', Path), ('mission', MissionState),
                                         ('odom', Odometry), ('feedback', SerialFeedBack))}

        def exercise(points, ceiling, remaining, expected, stop=False):
            accepted, last = 0, None
            deadline = time.monotonic() + 5.
            while time.monotonic() < deadline:
                mission = MissionState()
                mission.header.stamp, mission.header.frame_id = rospy.Time.now(), 'map'
                mission.valid, mission.direction = True, 1
                mission.route_name, mission.decision_id = 'speed_fixture', 2
                mission.speed_limit_mps, mission.remaining_stop_m = ceiling, remaining
                mission.stop_requested = stop
                path = Path()
                path.header = mission.header
                for x, y in points:
                    pose = PoseStamped()
                    pose.header = path.header
                    pose.pose.position.x, pose.pose.position.y = x, y
                    pose.pose.orientation.w = 1.
                    path.poses.append(pose)
                odom = Odometry()
                odom.header, odom.child_frame_id = mission.header, 'base_link'
                odom.pose.pose.orientation.w = 1.
                feedback = SerialFeedBack()
                feedback.MorA, feedback.speed, feedback.Gear = 1, 2., DriveCmd.GEAR_FORWARD
                feedback.steer = 512  # Hardware ADC, not 512 degrees or radians.
                for key, msg in (('mission', mission), ('path', path), ('odom', odom), ('feedback', feedback)):
                    publishers[key].publish(msg)
                time.sleep(.03)
                command = latest.get('drive')
                if command is None or command is last:
                    continue
                last = command
                self.assertLessEqual(command.KPH, 15)
                correct = (command.KPH == 0 and command.brake != 0) if stop else (
                    expected[0] <= command.KPH <= expected[1] and command.brake == 0
                    and command.Gear == DriveCmd.GEAR_FORWARD and command.EStop == 0)
                accepted = accepted + 1 if correct else 0
                if accepted >= 10:
                    return
            self.fail('Expected adaptive speed '+str(expected)+', got '+str(latest))

        try:
            straight = [(i*.25, 0.) for i in range(181)]
            curve = [(4.*math.sin(i*.025), 4.*(1.-math.cos(i*.025))) for i in range(61)]
            exercise(straight, 15./3.6, -1., (15, 15))  # float32 mission speed must not round to 14.
            exercise(straight, 30./3.6, -1., (15, 15))  # External cap cannot lift the ceiling.
            exercise(curve, 15./3.6, -1., (1, 7))
            exercise(straight, 15./3.6, 2., (2, 3))
            exercise(straight, 15./3.6, .1, (1, 1))  # No rounding-to-neutral deadlock.
            exercise(straight, .5, -1., (1, 1))
            exercise(straight, 15./3.6, -1., (0, 0), stop=True)
            exercise(straight, 15./3.6, -1., (15, 15))
        finally:
            subscriber.unregister()
            for endpoint in publishers.values():
                endpoint.unregister()

    def test_signed_reverse_hold_and_forward(self):
        latest = {}
        subscribers = [rospy.Subscriber('/control_contract/'+name, kind,
                       lambda msg, key=name: latest.update({key: msg}), queue_size=10)
                       for name, kind in (('drive', DriveCmd), ('state', String),
                                          ('target', PointStamped))]
        publishers = {name: rospy.Publisher('/control_contract/'+name, kind, queue_size=1)
                      for name, kind in (('path', Path), ('mission', MissionState),
                                         ('odom', Odometry), ('feedback', SerialFeedBack))}

        def exercise(direction, speed, stop=False, emergency=False,
                     feedback_estop=False, ros_mode=True):
            accepted = 0
            last_command = None
            deadline = time.monotonic() + 5.
            while time.monotonic() < deadline:
                stamp = rospy.Time.now()
                mission = MissionState()
                mission.header.stamp, mission.header.frame_id = stamp, 'map'
                mission.valid = True
                mission.remaining_stop_m = -1.
                mission.direction, mission.speed_limit_mps = direction, .5
                mission.stop_requested, mission.emergency_stop_requested = stop, emergency
                mission.route_name, mission.decision_id = 'parking_fixture', 1
                path = Path()
                path.header = mission.header
                for i in range(21):
                    p = PoseStamped()
                    p.header = path.header
                    p.pose.position.x = direction * .2 * i
                    p.pose.position.y = .05 * i
                    p.pose.orientation.w = 1.
                    path.poses.append(p)
                odom = Odometry()
                odom.header, odom.child_frame_id = mission.header, 'base_link'
                odom.pose.pose.orientation.w = 1.
                feedback = SerialFeedBack()
                feedback.MorA, feedback.speed = int(ros_mode), speed
                feedback.EStop = int(feedback_estop)
                feedback.Gear = DriveCmd.GEAR_REVERSE if direction < 0 else DriveCmd.GEAR_FORWARD
                for key, value in (('path', path), ('mission', mission),
                                   ('odom', odom), ('feedback', feedback)):
                    publishers[key].publish(value)
                time.sleep(.03)
                command = latest.get('drive')
                if command is None or command is last_command:
                    continue
                last_command = command
                expected_stop = (stop or emergency or feedback_estop or not ros_mode
                                 or not math.isfinite(speed))
                correct = (command.KPH == 0 and command.brake != 0
                           and command.Gear == DriveCmd.GEAR_NEUTRAL
                           and bool(command.EStop) == emergency) if expected_stop else (
                           command.KPH == 1 and command.brake == 0 and command.EStop == 0
                           and command.Gear == feedback.Gear and command.Deg < 0
                           and 'target' in latest and latest['target'].point.x * direction > 0)
                accepted = accepted + 1 if correct else 0
                if accepted >= 10:
                    return
            self.fail('No sustained expected command: '+str(latest))

        try:
            exercise(1, .28)
            exercise(-1, 0., stop=True)
            exercise(-1, -.28)  # signed feedback formerly stopped every cycle
            exercise(-1, .28)   # magnitude feedback remains accepted by PP
            exercise(-1, 0., emergency=True)
            exercise(1, 0., feedback_estop=True)  # brake, never echo EStop into a latch
            exercise(1, .28)    # release normal E-Stop request
            exercise(1, 0., ros_mode=False)
            exercise(1, .28)
            exercise(1, float('nan'))
        finally:
            for endpoint in list(publishers.values()) + subscribers:
                endpoint.unregister()


if __name__ == '__main__':
    rospy.init_node('control_contract_test')
    rostest.rosrun('control', 'control_contract', ControlContract)
