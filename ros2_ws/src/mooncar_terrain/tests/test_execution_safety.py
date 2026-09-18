from pathlib import Path
import math
import sys
import threading
import time
import unittest
from unittest.mock import Mock
from geometry_msgs.msg import PoseStamped

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from navigation_gateway import NavigationGateway, ReplanNeeded, motion_segments


def pose(x, y, yaw):
    result = PoseStamped()
    result.pose.position.x, result.pose.position.y = float(x), float(y)
    result.pose.orientation.z, result.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return result


class ExecutionSafetyTests(unittest.TestCase):
    def test_primitives_preserve_turns_and_reject_curves(self):
        route = [pose(0, 0, 0), pose(1, 0, 0), pose(2, 0, 0), pose(2, 0, math.pi/2), pose(2, 1, math.pi/2)]
        parts = motion_segments(route)
        self.assertEqual([kind for kind, _ in parts], ['drive', 'turn', 'drive'])
        self.assertEqual(len(parts[0][1]), 3)
        with self.assertRaises(ValueError):
            motion_segments([pose(0, 0, 0), pose(1, 1, math.pi/4)])

    def test_expired_browser_lease_withholds_motion_even_with_valid_pose(self):
        node = Mock(lock=threading.RLock(), active=True, phase='executing', fault=False,
                    last_heartbeat=time.monotonic(), stop_event=threading.Event(),
                    last_trace=time.monotonic(), runtime_checker=None)
        NavigationGateway.tick(node)
        self.assertTrue(node.permit_pub.publish.call_args.args[0].data)
        node.last_heartbeat = time.monotonic() - 3.1
        NavigationGateway.tick(node)
        self.assertTrue(node.stop_event.is_set())
        self.assertFalse(node.permit_pub.publish.call_args.args[0].data)

    def test_replan_confirms_cancellation_before_new_route(self):
        events = []
        node = Mock(stop_event=threading.Event(), replans=0, active=True, fault=False)
        node.wait.return_value = Mock(values=[Mock(bool_value=True)])
        node.pose.return_value = dict(x=1., y=0.)
        def route(*args):
            events.append('route')
            if events.count('route') == 1:
                raise ReplanNeeded('new obstacle')
        node.execute_route.side_effect = route
        node.cancel_action.side_effect = lambda: events.append('cancel-confirmed')
        NavigationGateway.run_mission(node, [1., 0.], 0., 'full')
        self.assertEqual(events, ['route', 'cancel-confirmed', 'route', 'cancel-confirmed'])
        self.assertEqual(node.replans, 1)
        self.assertEqual(node.phase, 'succeeded')
        self.assertFalse(node.active)
