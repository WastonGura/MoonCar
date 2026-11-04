"""Independent joint capture keeps original measurement time and settle gates."""
from dataclasses import FrozenInstanceError
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data

sys.path.insert(0,str(Path(__file__).resolve().parent))
import test_gimbal_command_slew as fixtures
from experiment_motion import ExperimentMotion


class GimbalFeedbackCaptureTests(unittest.TestCase):
    setUp=fixtures.GimbalSlewTests.setUp
    tearDown=fixtures.GimbalSlewTests.tearDown
    motion=fixtures.GimbalSlewTests.motion
    clock=fixtures.GimbalSlewTests.clock
    commands=fixtures.GimbalSlewTests.commands

    def message(self,at,position=(0.,0.)):
        message=JointState();message.name=['camera_pan_joint','camera_tilt_joint']
        message.position=list(position);message.velocity=[0.,0.]
        value=round(at*1e9)
        message.header.stamp.sec,message.header.stamp.nanosec=divmod(value,1_000_000_000)
        return message

    def capture(self,at,position=(0.,0.)):
        message=self.message(at,position)
        self.node.capture_gimbal_joints(message)
        return message

    def test_capture_while_main_lock_held_is_nonblocking_and_copies_feedback(self):
        completed=threading.Event()
        message=self.message(10.,(.2,-.1))
        def capture():
            self.node.capture_gimbal_joints(message);completed.set()
        with self.node.lock:
            thread=threading.Thread(target=capture,daemon=True);thread.start()
            self.assertTrue(completed.wait(.5),'Independent capture waited for main motion lock')
            thread.join()
            self.assertIsNone(self.node.gimbal_position,'Capture itself must not mutate control feedback')
            immutable=self.node.gimbal_captures[-1]
            with self.assertRaises(FrozenInstanceError):immutable.stamp=99.
            message.position=[9.,9.];message.name=['wrong','wrong']
            self.node.pan_tilt(.8,-.3)
        self.assertEqual(self.node.gimbal_command,[.2,-.1])
        self.assertEqual(self.commands(),[])

    def test_actual_ros_subscription_captures_on_clock_executor_during_main_lock(self):
        self.assertIn(self.node.gimbal_joint_subscription,self.node.clock_node.subscriptions)
        publisher=self.node.create_publisher(JointState,'/joint_states',qos_profile_sensor_data)
        deadline=time.monotonic()+2.
        while publisher.get_subscription_count()==0 and time.monotonic()<deadline:
            threading.Event().wait(.01)
        self.assertGreater(publisher.get_subscription_count(),0)
        with self.node.lock:
            publisher.publish(self.message(10.,(.2,-.1)))
            deadline=time.monotonic()+1.
            while self.node.gimbal_capture_count==0 and time.monotonic()<deadline:
                threading.Event().wait(.005)
            self.assertEqual(self.node.gimbal_capture_count,1)
            self.assertIsNone(self.node.gimbal_position)
        self.node.tick()
        self.assertEqual(self.node.gimbal_position,[.2,-.1])

    def test_bounded_capture_queue_retains_only_32_latest_measurements(self):
        for number in range(40):self.capture(9.96+number*.001)
        self.assertEqual(len(self.node.gimbal_captures),32)
        self.assertEqual(self.node.gimbal_capture_count,40)
        self.assertEqual(self.node.gimbal_capture_omitted,8)
        self.assertAlmostEqual(self.node.gimbal_captures[0].stamp,9.968)
        self.assertIsNone(self.node.gimbal_position)

    def test_drain_preserves_captured_wall_receipt_and_rejects_old_receipt(self):
        wall=SimpleNamespace(now=time.monotonic())
        timer=SimpleNamespace(monotonic=lambda:wall.now)
        with patch('experiment_motion.time',timer):
            self.capture(10.)
            received=wall.now
            wall.now+=2.001
            with self.assertRaisesRegex(ValueError,'关节反馈'):
                self.node.pan_tilt(.3,-.2)
            self.assertEqual(self.node.gimbal_joint_received,received)
        self.assertFalse(self.node.active)
        self.assertEqual(self.commands(),[])

    def test_future_phase_at_50ms_is_accepted_beyond_it_is_discarded(self):
        self.capture(10.05001)
        self.node.tick()
        self.assertEqual(self.node.gimbal_joint_stamp,-1.)
        self.assertEqual(self.node.gimbal_capture_dropped_future,1)
        self.capture(10.05)
        self.node.pan_tilt(.3,-.2)
        self.assertTrue(self.node.active)
        self.assertAlmostEqual(self.node.gimbal_joint_stamp,10.05)
        self.assertEqual(self.commands(),[])

    def test_tick_accepts_small_future_phase_without_a_catchup_jump(self):
        self.capture(10.);self.node.pan_tilt(1.,0.)
        self.clock(10.1);self.capture(10.15)
        self.node.tick()
        self.assertAlmostEqual(self.commands()[-1][0],.035)
        self.assertFalse(self.node.permit.publish.call_args.args[0].data)

    def test_stale_queued_samples_are_discarded_without_polluting_joint_stamp(self):
        self.capture(9.89999)
        self.node.tick()
        self.assertEqual(self.node.gimbal_joint_stamp,-1.)
        self.assertIsNone(self.node.gimbal_position)
        self.assertEqual(self.node.gimbal_capture_dropped_old,1)
        self.capture(9.9)
        self.node.pan_tilt(.3,-.2)
        self.assertTrue(self.node.active)

    def test_reset_clears_queue_and_old_epoch_cannot_poison_new_low_stamp(self):
        self.capture(100.)
        self.node.reset()
        self.assertFalse(self.node.gimbal_captures)
        self.assertIsNone(self.node.gimbal_capture_last_stamp)
        self.assertEqual(self.node.gimbal_capture_count,0)
        self.assertIsNone(self.node.gimbal_final_command_wall)
        self.clock(.02)
        self.capture(100.)
        self.capture(.01,(.1,0.))
        self.node.pan_tilt(.3,-.2)
        self.assertEqual(self.node.gimbal_capture_dropped_future,1)
        self.assertAlmostEqual(self.node.gimbal_joint_stamp,.01)
        self.assertEqual(self.node.gimbal_command,[.1,0.])

    def test_final_command_still_requires_two_new_captured_real_feedback_samples(self):
        self.capture(10.);self.node.pan_tilt(.03,0.)
        self.clock(10.1);self.capture(10.1,(.03,0.));self.node.tick()
        self.assertEqual(self.commands()[-1],(.03,0.))
        self.assertEqual(self.node.gimbal_settled_samples,0)
        self.clock(10.11);self.capture(9.,(.03,0.));self.capture(10.11,(.03,0.));self.node.tick()
        self.assertTrue(self.node.active)
        self.assertEqual(self.node.gimbal_settled_samples,1)
        self.clock(10.12);self.capture(10.12,(.03,0.));self.node.tick()
        self.assertFalse(self.node.active)
        self.assertEqual(self.node.phase,'succeeded')

    def test_capture_before_final_publish_with_future_header_cannot_count_as_post_command_settle(self):
        self.capture(10.);self.node.pan_tilt(.03,0.)
        self.clock(10.1);self.capture(10.1,(.03,0.))
        # Reproduce the narrow causal window: independent capture runs after the
        # tick drained its queue, during the final tilt publisher's call.
        self.node.pub_camera_tilt.publish.side_effect=lambda msg:self.capture(10.13,(.03,0.))
        self.node.tick()
        self.node.pub_camera_tilt.publish.side_effect=None
        self.assertEqual(self.commands()[-1],(.03,0.))
        self.assertLess(self.node.gimbal_captures[-1].received_wall,self.node.gimbal_final_command_wall)
        self.clock(10.11);self.node.tick()
        self.assertAlmostEqual(self.node.gimbal_joint_stamp,10.13)
        self.assertEqual(self.node.gimbal_position,[.03,0.], 'Real position remains usable')
        self.assertEqual(self.node.gimbal_settled_samples,0, 'Future header cannot bypass receipt causality')
        self.assertTrue(self.node.active)
        self.clock(10.12);self.capture(10.14,(.03,0.));self.node.tick()
        self.assertEqual(self.node.gimbal_settled_samples,1)
        self.assertTrue(self.node.active)
        self.clock(10.13);self.capture(10.15,(.03,0.));self.node.tick()
        self.assertEqual(self.node.gimbal_settled_samples,2)
        self.assertFalse(self.node.active)
        self.assertEqual(self.node.phase,'succeeded')

    def test_real_state_drains_queue_and_reports_both_time_domains(self):
        self.capture(10.02,(.1,-.1))
        state=ExperimentMotion.state(self.node)
        diagnostics=state['diagnostics']
        self.assertEqual(diagnostics['gimbal_joint_stamp_sim'],10.02)
        self.assertAlmostEqual(diagnostics['gimbal_feedback_age_sim_seconds'],-.02)
        self.assertEqual(diagnostics['gimbal_queued_capture_count'],0)
        self.assertEqual(diagnostics['gimbal_capture_count'],1)
        self.assertEqual(diagnostics['gimbal_capture_last_stamp_sim'],10.02)
        self.assertEqual(diagnostics['gimbal_feedback_receipt_source'],'clock_executor_capture')

    def test_none_mode_keeps_direct_subscription_and_original_receipt_behavior(self):
        self.node.destroy_node();self.node=self.motion(None)
        self.assertIn(self.node.gimbal_joint_subscription,self.node.subscriptions)
        self.assertNotIn(self.node.gimbal_joint_subscription,self.node.clock_node.subscriptions)
        message=self.message(10.,(.1,-.1))
        self.node.on_gimbal_joints(message)
        self.assertEqual(self.node.gimbal_position,[.1,-.1])
        self.assertFalse(self.node.gimbal_captures)
        self.assertEqual(self.node.gimbal_capture_count,0)
        self.node.pan_tilt(.3,-.2)
        self.assertEqual(self.commands(),[(.3,-.2)])


if __name__=='__main__':unittest.main(verbosity=2)
