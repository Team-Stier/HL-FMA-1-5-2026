#!/usr/bin/env python3
"""Actual RDDF -> Frenet -> RDDF transport, with synthetic pose/cluster replay."""
import math
from pathlib import Path
import sys
import time

import rospkg
import rospy
import rostest
from geometry_msgs.msg import Point
from nav_msgs.msg import Path as NavPath
from planning_interfaces.msg import PlannedPath, PathStatus
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, str(Path(rospkg.RosPack().get_path('stier_bringup')) / 'test'))
from parking_pipeline_ros import VehiclePipelineFixture
from stier_state_manager.geometry import project


class FrenetPipeline(VehiclePipelineFixture):
    def setUp(self):
        super().setUp()
        self.cone = None
        self.pub['clusters'] = rospy.Publisher('/dbscan_clusters', MarkerArray, queue_size=1)
        self.subscribers.extend(rospy.Subscriber(topic, kind,
            lambda message, k=key: self.remember(k, message), queue_size=10)
            for key, topic, kind in (
                ('local', '/path/local', PlannedPath),
                ('final', '/path/final', NavPath),
                ('planner', '/path_planner/status', PathStatus)))
        self.cluster_timer = rospy.Timer(rospy.Duration(.025), self.publish_clusters)

    def tearDown(self):
        self.cluster_timer.shutdown()
        super().tearDown()

    def publish_clusters(self, _event):
        stamp = rospy.Time.now()-rospy.Duration(.01)
        heartbeat = Marker()
        heartbeat.header.stamp, heartbeat.header.frame_id = stamp, 'map'
        heartbeat.action = Marker.DELETEALL
        result = MarkerArray(markers=[heartbeat])
        with self.lock:
            cone = self.cone
        if cone:
            x, y = cone
            box = Marker()
            box.header, box.ns = heartbeat.header, 'dbscan_clusters'
            box.action, box.type, box.pose.orientation.w = Marker.ADD, Marker.POINTS, 1.
            box.points = [Point(x-.18, y-.18, 0.), Point(x+.18, y+.18, 0.)]
            result.markers.append(box)
        self.pub['clusters'].publish(result)

    def replay(self, name, start, end):
        # Smooth timestamped position replay for route switching, not simulated
        # tracking. Closed-loop tracking is tested by planner_control_regression.
        steps = max(1, int(math.ceil((end-start)/.10)))
        for i in range(steps+1):
            with self.lock:
                self.pose = self.body_pose(name, start+(end-start)*i/steps, 1)
                self.speed = .8
            time.sleep(.05)

    def test_enter_avoid_clear_and_exit(self):
        name = '3_s-static-obstacle'
        route = self.routes[name]
        seen = self.await_drive('2', 1)
        self.assertEqual(seen['selection'].source, 'RDDF')
        self.replay('2', self.routes['2'].length-3., self.routes['2'].length-.05)
        seen = self.await_drive(name, 1)
        self.assertEqual(seen['selection'].source, 'LOCAL')
        self.assertEqual(seen['mission'].path_mode, 'LOCAL')
        # A cone at +6 m must produce an actually offset local/final path,
        # not a ready status with the unchanged reference polyline.
        changed_at = rospy.Time.now()
        with self.lock:
            self.cone = route.pose_at(6.)[:2]

        def offset_path(seen):
            local, final = seen.get('local'), seen.get('final')
            status, selection = seen.get('planner'), seen.get('selection')
            drive = seen.get('drive')
            if not (local and final and status and selection and drive
                    and local.header.stamp > changed_at and status.ready and selection.ready
                    and local.decision_id == selection.decision_id and drive.KPH > 0
                    and not drive.brake and not drive.EStop):
                return False
            return all(max((project(route, p.pose.position.x, p.pose.position.y)['distance']
                            for p in path.poses), default=0.) > .3
                       for path in (local.path, final))
        self.wait_for(offset_path, 'cone must reach actual final avoidance path')
        with self.lock:
            self.cone = None
        changed_at = rospy.Time.now()
        self.wait_for(lambda s: s.get('planner') is not None and s['planner'].ready
            and s['planner'].header.stamp > changed_at and s['planner'].reason == 'RDDF_CLEAR',
            'removing cone must restore clear reference')
        # Pause before both the mission's 0.8 m completion and the tracker's
        # 0.5 m endpoint handoff. The former 1 m planner cutoff fails at 0.9 m;
        # at 0.75 m, however, an already accepted next route is valid behavior.
        self.replay(name, 0., route.length-.90)
        self.await_drive(name, 1)
        self.replay(name, route.length-.90, route.length-.05)
        seen = self.await_drive('4', 1)
        self.assertEqual(seen['selection'].source, 'RDDF')
        self.assertEqual(seen['mission'].path_mode, 'RDDF')
        print('RDDF/Frenet/RDDF completed:', rospy.get_param('~course'), self.trace)


if __name__ == '__main__':
    rospy.init_node('frenet_pipeline_test')
    rostest.rosrun('stier_bringup', 'frenet_pipeline_'+rospy.get_param('~course')+
                  '_ld'+str(rospy.get_param('~lookahead')), FrenetPipeline)
