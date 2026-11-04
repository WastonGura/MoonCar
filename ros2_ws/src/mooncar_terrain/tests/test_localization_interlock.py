"""ROS integration checks for motion cancellation and timestamped capture gating."""
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'scripts'))
sys.path.insert(0,str(ROOT/'ros2_ws/src/mooncar_terrain/scripts'))
from localization_quality import LaserQualitySource, LocalizationQuality
import rclpy
from rclpy.clock import ClockType
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from experiment_motion import ExperimentMotion
from map_recorder import MapRecorder


def source_ready():
    source=LaserQualitySource('capture-fixture')
    source.observe(9.6,dict(accepted=True,reason='initialized'))
    for stamp in (9.7,9.8,9.9):source.observe(stamp,dict(accepted=True,reason='accepted'))
    return source


def message(source,now=10.):
    return String(data=json.dumps(source.snapshot(now)))


class MotionLocalizationTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=['--ros-args','-p','use_sim_time:=true'],domain_id=97)
        self.node=ExperimentMotion(require_laser_localization=True)
        self.node.get_clock().set_ros_time_override(rclpy.time.Time(seconds=10.,clock_type=ClockType.ROS_TIME))

    def tearDown(self):
        self.node.destroy_node();rclpy.shutdown()

    def test_required_quality_blocks_pose_before_any_tf_access(self):
        self.node.buffer=Mock()
        with self.assertRaisesRegex(ValueError,'激光定位质量失效'):self.node.pose()
        self.node.buffer.lookup_transform.assert_not_called()
        self.assertFalse(self.node.state()['ready'])

    def test_match_failure_cancels_action_and_recovery_never_resumes_it(self):
        node=self.node;source=source_ready()
        node.on_localization_quality(message(source))
        self.assertTrue(node.localization_state()['healthy'])
        node.active=True;node.phase='executing';node.goal_request_kind='spin'
        node.goal_handle=Mock();node.permit=Mock()
        source.observe(10.,dict(accepted=False,reason='too_few_line_correspondences'))
        node.on_localization_quality(message(source))
        self.assertEqual(node.phase,'executing')  # Helper only records the outage.
        node.tick()
        self.assertEqual(node.phase,'cancelling')
        node.goal_handle.cancel_goal_async.assert_called_once()
        self.assertFalse(node.permit.publish.call_args.args[0].data)
        for stamp in (10.1,10.2,10.3):source.observe(stamp,dict(accepted=True,reason='accepted'))
        node.get_clock().set_ros_time_override(rclpy.time.Time(seconds=10.3,clock_type=ClockType.ROS_TIME))
        node.on_localization_quality(message(source,10.3));node.tick()
        self.assertTrue(node.localization_state()['healthy'])
        self.assertEqual(node.phase,'cancelling')
        self.assertFalse(node.permit.publish.call_args.args[0].data)

    def test_missing_status_tick_revokes_permit(self):
        node=self.node;source=source_ready();node.on_localization_quality(message(source))
        node.active=True;node.phase='executing';node.goal_request_kind='spin'
        node.goal_handle=Mock();node.permit=Mock()
        node.localization_quality.received_wall=time.monotonic()-.501
        node.tick()
        self.assertEqual(node.phase,'cancelling')
        self.assertFalse(node.permit.publish.call_args.args[0].data)

    def test_old_callback_generation_cannot_seed_reset_session(self):
        node=self.node;source=source_ready();old=node.localization_generation
        node.on_localization_quality(message(source),old)
        node.reset()
        node.on_localization_quality(message(source),old)
        self.assertIsNone(node.localization_quality.session_id)
        replacement=source_ready();replacement.session_id='replacement'
        node.on_localization_quality(message(replacement),node.localization_generation)
        self.assertEqual(node.localization_state()['reason'],'awaiting_new_clock_epoch')
        self.assertIsNone(node.localization_quality.session_id)
        replacement.observe(10.,dict(accepted=True,reason='accepted'))
        node.get_clock().set_ros_time_override(rclpy.time.Time(seconds=10.1,clock_type=ClockType.ROS_TIME))
        node.on_localization_quality(message(replacement,10.1),node.localization_generation)
        self.assertEqual(node.localization_quality.session_id,'replacement')
        self.assertTrue(node.localization_state()['healthy'])


class RecorderLocalizationTests(unittest.TestCase):
    def state(self,stamp=10.,required=True):
        cloud=PointCloud2();cloud.header.stamp.sec=int(stamp)
        cloud.header.stamp.nanosec=round((stamp-int(stamp))*1e9)
        state=SimpleNamespace(pending=(cloud,time.monotonic()),imu=[],buffer=Mock(),terrain=Mock(),
                              uplink=Mock(),status=Mock(),get_clock=Mock(),
                              localization_quality=LocalizationQuality(required=required))
        state.get_clock.return_value.now.return_value.nanoseconds=10_000_000_000
        return state

    def test_unknown_quality_drops_frame_before_tf_fusion_or_uplink(self):
        state=self.state();MapRecorder.process_pending(state)
        self.assertIsNone(state.pending)
        state.buffer.can_transform.assert_not_called()
        state.terrain.add_frame.assert_not_called();state.uplink.enqueue.assert_not_called()

    def test_healthy_epoch_allows_timestamp_to_reach_existing_sensor_checks(self):
        state=self.state();source=source_ready()
        state.localization_quality.observe(source.snapshot(10.),time.monotonic())
        MapRecorder.process_pending(state)
        self.assertIsNotNone(state.pending)  # Existing IMU wait, not a quality rejection.
        state.status.assert_not_called()

    def test_pre_recovery_depth_and_future_depth_are_dropped(self):
        for stamp in (9.8,10.001):
            state=self.state(stamp);source=source_ready()
            state.localization_quality.observe(source.snapshot(10.),time.monotonic())
            MapRecorder.process_pending(state)
            self.assertIsNone(state.pending)
            state.terrain.add_frame.assert_not_called();state.uplink.enqueue.assert_not_called()

    def test_invalid_status_discards_pending_capture_immediately(self):
        state=self.state();MapRecorder.on_localization_quality(state,String(data='not-json'))
        self.assertIsNone(state.pending)
        state.terrain.add_frame.assert_not_called();state.uplink.enqueue.assert_not_called()

    def test_disabled_baseline_keeps_existing_sensor_wait(self):
        state=self.state(required=False);MapRecorder.process_pending(state)
        self.assertIsNotNone(state.pending);state.status.assert_not_called()


if __name__=='__main__':unittest.main()
