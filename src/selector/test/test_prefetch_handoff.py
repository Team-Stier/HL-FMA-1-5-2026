"""Exercise real selector callbacks without a ROS master or vehicle output."""
import importlib.machinery
import importlib.util
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import patch

import rospy
from geometry_msgs.msg import PoseStamped
from planning_interfaces.msg import MissionState, PlannedPath

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
loader = importlib.machinery.SourceFileLoader('prefetch_selector_node', str(ROOT / 'scripts/selector_node'))
spec = importlib.util.spec_from_loader(loader.name, loader)
node_module = importlib.util.module_from_spec(spec)
loader.exec_module(node_module)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class PrefetchTests(unittest.TestCase):
    def setUp(self):
        self.node = node_module.SelectorNode.__new__(node_module.SelectorNode)
        self.node.lock = threading.RLock()
        self.node.core = node_module.SelectorCore()
        self.node.state = None
        self.node.last_time = None
        self.node.candidates, self.node.messages, self.node.rddf_cache = {}, {}, {}
        self.node.path_pub = Publisher()
        self.node.status_pub = Publisher()
        self.node.prefetch_status_pub = Publisher()
        self.clock = patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(10.))
        self.clock.start()
        self.addCleanup(self.clock.stop)

    def path(self, route, decision, origin=0.):
        msg = PlannedPath()
        msg.header.stamp, msg.header.frame_id = rospy.Time.from_sec(10.), 'map'
        msg.path.header = msg.header
        msg.route_name, msg.decision_id, msg.direction = route, decision, 1
        for x in (origin, origin+1.):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.orientation.w = 1.
            msg.path.poses.append(pose)
        return msg

    def state(self, route, decision):
        msg = MissionState()
        msg.header.stamp = rospy.Time.from_sec(10.)
        msg.route_name, msg.decision_id = route, decision
        msg.path_mode, msg.direction, msg.valid = 'RDDF', 1, True
        self.node.on_state(msg)

    def test_prepared_path_is_selected_without_empty_output_in_either_callback_order(self):
        for path_first in (True, False):
            self.state('1_left', 1)
            self.node.on_path(self.path('1_left', 1), 'RDDF')
            prepared = self.path('2', 2, 1.)
            self.node.on_path(prepared, 'PREFETCH')
            self.assertTrue(self.node.prefetch_status_pub.messages[-1].ready)
            self.node.tick(None)
            self.assertEqual(self.node.path_pub.messages[-1].poses[0].pose.position.x, 0.)
            if path_first:
                self.node.on_path(prepared, 'RDDF')
                self.node.tick(None)
                self.assertTrue(self.node.status_pub.messages[-1].ready)
            self.state('2', 2)
            self.node.tick(None)
            self.assertTrue(self.node.status_pub.messages[-1].ready)
            self.assertEqual(self.node.path_pub.messages[-1].poses[0].pose.position.x, 1.)

    def test_invalid_prepared_geometry_is_not_approved(self):
        prepared = self.path('2', 2)
        prepared.path.poses.clear()
        self.node.on_path(prepared, 'PREFETCH')
        self.assertFalse(self.node.prefetch_status_pub.messages[-1].ready)
        self.state('2', 2)
        self.node.tick(None)
        self.assertFalse(self.node.path_pub.messages[-1].poses)


if __name__ == '__main__':
    unittest.main()
