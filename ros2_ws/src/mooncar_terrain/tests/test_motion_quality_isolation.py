"""Real ROS transport and action-boundary checks for independent quality input."""
import json
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import Mock

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'scripts'))
import rclpy
from rclpy.clock import ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from experiment_motion import ExperimentMotion
from localization_quality import LaserQualitySource


def ready_source(stamp=10.):
    source=LaserQualitySource('motion-isolation')
    source.observe(stamp-.4,dict(accepted=True,reason='initialized'))
    for offset in (.3,.2,.1):
        source.observe(stamp-offset,dict(accepted=True,reason='accepted'))
    return source


def message(source,stamp):
    return String(data=json.dumps(source.snapshot(stamp)))


def wait_until(predicate,timeout=3.):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if predicate():return True
        time.sleep(.005)
    return bool(predicate())


class MotionQualityIsolationTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=['--ros-args','-p','use_sim_time:=true'],domain_id=94)
        self.node=ExperimentMotion(require_laser_localization=True)
        self.node.get_clock().set_ros_time_override(rclpy.time.Time(seconds=10.,clock_type=ClockType.ROS_TIME))

    def tearDown(self):
        self.node.destroy_node();rclpy.shutdown()

    def test_clock_and_quality_advance_while_data_group_and_main_lock_are_blocked(self):
        node=self.node;publisher=Node('quality_independent_executor_test')
        entered=threading.Event();release=threading.Event()
        subscription=node.create_subscription(String,'/blocked_quality_diagnostic',
            lambda msg:(entered.set(),release.wait(5.)),1,callback_group=node.data_group)
        data=publisher.create_publisher(String,'/blocked_quality_diagnostic',1)
        status=publisher.create_publisher(String,'/mooncar/laser_odometry/status',10)
        clock=publisher.create_publisher(Clock,'/clock',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        executor=MultiThreadedExecutor(num_threads=2);executor.add_node(node)
        thread=threading.Thread(target=executor.spin);thread.start()
        try:
            self.assertTrue(wait_until(lambda:data.get_subscription_count()>=1 and status.get_subscription_count()>=1))
            data.publish(String(data='block'))
            self.assertTrue(entered.wait(2.))
            source=ready_source(12.)
            # The data callback occupies one motion worker; the ordinary tick
            # occupies the other while waiting on this held main lock.
            with node.lock:
                for index in range(4):
                    stamp=12.+index*.1
                    tick=Clock();tick.clock.sec=int(stamp);tick.clock.nanosec=round((stamp-int(stamp))*1e9)
                    self.assertTrue(wait_until(lambda:(clock.publish(tick) or node.get_clock().now().nanoseconds==round(stamp*1e9)),1.))
                    if index:source.observe(stamp-.1,dict(accepted=True,reason='accepted'))
                    payload=message(source,stamp)
                    status.publish(payload)
                    expected=json.loads(payload.data)['sequence']
                    self.assertTrue(wait_until(lambda:node.localization_state()['sequence']==expected,1.))
                    self.assertTrue(node.localization_state()['healthy'])
                deadline=time.monotonic()+.55
                while time.monotonic()<deadline:
                    status.publish(message(source,12.3));time.sleep(.02)
                self.assertTrue(node.localization_state()['healthy'])
                self.assertEqual(node.localization_state()['fault_counter'],0)
                self.assertEqual(node.get_clock().now().nanoseconds,12_300_000_000)
                self.assertFalse(release.is_set())
                self.assertEqual(sum(s.topic_name=='/mooncar/laser_odometry/status' for s in node.subscriptions),0)
                self.assertEqual(sum(s.topic_name=='/mooncar/laser_odometry/status' for s in node.clock_node.subscriptions),1)
        finally:
            release.set();executor.shutdown();thread.join(3.)
            node.destroy_subscription(subscription);publisher.destroy_node()

    def test_recovery_before_tick_still_cancels_old_action_and_allows_explicit_new_action(self):
        node=self.node;source=ready_source()
        node.on_localization_quality(message(source,10.))
        with node.lock:node.activate_localization_action()
        node.phase='executing';node.goal_request_kind='spin';node.goal_handle=Mock();node.permit=Mock()
        action_fault=node.localization_state()['action_fault_counter']
        source.observe(10.,dict(accepted=False,reason='too_few_line_correspondences'))
        node.on_localization_quality(message(source,10.))
        self.assertEqual(node.phase,'executing')  # Callback cannot wait for main work.
        for stamp in (10.1,10.2,10.3):source.observe(stamp,dict(accepted=True,reason='accepted'))
        node.get_clock().set_ros_time_override(rclpy.time.Time(seconds=10.3,clock_type=ClockType.ROS_TIME))
        node.on_localization_quality(message(source,10.3))
        self.assertTrue(node.localization_state()['healthy'])
        self.assertGreater(node.localization_state()['fault_counter'],action_fault)
        handle=node.goal_handle;node.tick()
        self.assertEqual(node.phase,'cancelling');handle.cancel_goal_async.assert_called_once()
        self.assertFalse(node.permit.publish.call_args.args[0].data)
        node.active=False;node.phase='idle';node.state=Mock(return_value=dict(ready=True))
        node.pose=Mock(return_value=dict(x=0.,y=0.,z=0.,yaw=0.))
        node.pub_camera_pan=Mock();node.pub_camera_tilt=Mock()
        node.pan_tilt(0.,0.)
        self.assertTrue(node.active)
        self.assertEqual(node.localization_state()['fault_counter'],node.localization_state()['action_fault_counter'])
        node.tick();self.assertTrue(node.active);self.assertEqual(node.phase,'executing')

    def test_every_action_rechecks_quality_at_atomic_activation_boundary(self):
        node=self.node
        for kind in ('start','turn','gimbal'):
            with self.subTest(kind=kind):
                with node.localization_lock:
                    node.localization_quality.reset();node.localization_was_healthy=False
                source=ready_source();node.on_localization_quality(message(source,10.))
                node.active=False;node.phase='idle';node.start_checked=False
                node.state=Mock(return_value=dict(ready=True));node.check_goal=Mock()
                node.spin_client.server_is_ready=Mock(return_value=True);node.send_goal=Mock()
                def pose_then_failure():
                    source.observe(10.,dict(accepted=False,reason='too_few_line_correspondences'))
                    node.on_localization_quality(message(source,10.))
                    return dict(x=0.,y=0.,z=0.,yaw=0.)
                node.pose=pose_then_failure
                task=dict(id='test',region=[-5.,-5.,5.,5.],start=[0.,0.])
                with self.assertRaisesRegex(ValueError,'激光定位质量失效'):
                    if kind=='start':node.start(dict(experiment='test',goal=[1.,0.]),task)
                    elif kind=='turn':node.turn(.2,task)
                    else:node.pan_tilt(0.,0.)
                self.assertFalse(node.active);node.send_goal.assert_not_called()

    def test_missed_heartbeat_interval_is_latched_before_new_status_replaces_receipt(self):
        node=self.node;source=ready_source();node.on_localization_quality(message(source,10.))
        with node.lock:node.activate_localization_action()
        node.phase='executing';node.goal_request_kind='spin';node.goal_handle=Mock();node.permit=Mock()
        with node.localization_lock:node.localization_quality.received_wall=time.monotonic()-.501
        # A fresh producer with the same ready epoch must not erase the local
        # lost-status interval while the ordinary motion tick was blocked.
        for stamp in (10.,10.1,10.2):
            source.observe(stamp,dict(accepted=True,reason='accepted'))
            node.get_clock().set_ros_time_override(rclpy.time.Time(seconds=stamp,clock_type=ClockType.ROS_TIME))
            node.on_localization_quality(message(source,stamp))
        self.assertTrue(node.localization_state()['healthy'])
        self.assertGreater(node.localization_state()['fault_counter'],node.localization_state()['action_fault_counter'])
        node.tick();self.assertEqual(node.phase,'cancelling')

    def test_fresh_incoming_measurement_replaces_old_sample_before_sim_age_check(self):
        node=self.node;source=ready_source()
        for now,snapshot in ((10.,10.),(10.1,10.1)):
            node.get_clock().set_ros_time_override(rclpy.time.Time(seconds=now,clock_type=ClockType.ROS_TIME))
            node.on_localization_quality(message(source,snapshot))
            self.assertTrue(node.localization_state()['healthy'])
        with node.lock:node.activate_localization_action()
        # Producer scan and heartbeat phases can straddle the old sample's
        # expiry. This newly received fit is only 0.11 s old and must be checked
        # instead of first manufacturing a failure from the previous 0.31 s fit.
        source.observe(10.1,dict(accepted=True,reason='accepted'))
        node.get_clock().set_ros_time_override(rclpy.time.Time(seconds=10.21,clock_type=ClockType.ROS_TIME))
        node.on_localization_quality(message(source,10.2))
        health=node.localization_state()
        self.assertTrue(health['healthy'],health)
        self.assertAlmostEqual(health['observation_age_sim_seconds'],.11)
        self.assertEqual(health['fault_counter'],0)


if __name__=='__main__':unittest.main()
