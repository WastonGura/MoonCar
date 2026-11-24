"""Isolated real ROS motion node: command limits never replace joint feedback."""
import math
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import Mock

import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'scripts'))
from experiment_motion import ExperimentMotion


class GimbalSlewTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=['--ros-args','-p','use_sim_time:=true'],domain_id=96)
        self.node=self.motion(.35)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def motion(self,rate):
        node=ExperimentMotion(gimbal_command_rate_rad_s=rate)
        node.state=Mock(return_value=dict(ready=True))
        node.pose=Mock(return_value=dict(x=0.,y=0.,z=0.,yaw=0.))
        node.record_stop=Mock()
        for name in ('pub_camera_pan','pub_camera_tilt','permit'):
            setattr(node,name,Mock())
        node.get_clock().set_ros_time_override(rclpy.time.Time(seconds=10.,clock_type=rclpy.clock.ClockType.ROS_TIME))
        return node

    def feedback(self,at,position=(0.,0.),velocity=(0.,0.)):
        self.clock(at)
        message=JointState();message.name=['camera_pan_joint','camera_tilt_joint']
        message.position=list(position);message.velocity=list(velocity)
        stamp=round(at*1e9)
        message.header.stamp.sec,message.header.stamp.nanosec=divmod(stamp,1_000_000_000)
        self.node.on_gimbal_joints(message)

    def clock(self,at):
        self.node.get_clock().set_ros_time_override(rclpy.time.Time(
            seconds=at,clock_type=rclpy.clock.ClockType.ROS_TIME))

    def commands(self):
        return list(zip([call.args[0].data for call in self.node.pub_camera_pan.publish.call_args_list],
                        [call.args[0].data for call in self.node.pub_camera_tilt.publish.call_args_list]))

    def test_rate_requires_a_positive_finite_number_or_none(self):
        for invalid in (0.,-1.,True,float('nan'),float('inf'),'0.35'):
            with self.subTest(rate=invalid),self.assertRaises(ValueError):
                ExperimentMotion(gimbal_command_rate_rad_s=invalid)

    def test_start_rejects_absent_stale_or_future_measured_joints(self):
        for invalid in ('missing','old_sim','old_wall','future'):
            with self.subTest(feedback=invalid):
                self.node.gimbal_position=[0.,0.]
                self.node.gimbal_joint_stamp=10.
                self.node.gimbal_joint_received=time.monotonic()
                if invalid=='missing':self.node.gimbal_position=None
                if invalid=='old_sim':self.node.gimbal_joint_stamp=9.8
                if invalid=='old_wall':self.node.gimbal_joint_received-=2.01
                if invalid=='future':self.node.gimbal_joint_stamp=10.051
                self.node.gimbal_target=[.3,-.2]
                with self.assertRaisesRegex(ValueError,'关节反馈'):
                    self.node.pan_tilt(.3,-.2)
                self.assertFalse(self.node.active)
        self.assertEqual(self.commands(),[])

    def test_start_uses_actual_joint_position_without_publishing_a_final_step(self):
        self.feedback(10.,(.2,-.1))
        self.node.gimbal_target=[-2.,-.5]  # An old desired target is not feedback.
        self.node.pan_tilt(.8,-.3)
        self.assertEqual(self.node.gimbal_command,[.2,-.1])
        self.assertEqual(self.node.gimbal_target,[.8,-.3])
        self.assertEqual(self.commands(),[])
        expected=math.hypot(.6,-.2)/.35+1.5+5.
        self.assertAlmostEqual(self.node.sim_budget,expected)

    def test_each_command_has_bounded_vector_rate_and_never_overshoots(self):
        self.feedback(10.,(.2,-.1));self.node.pan_tilt(.8,-.3)
        previous=(.2,-.1)
        previous_distance=math.hypot(.8-previous[0],-.3-previous[1])
        for number in range(1,16):
            self.feedback(10.+number*.1,previous)
            self.node.tick()
            current=self.commands()[-1]
            self.assertLessEqual(math.hypot(current[0]-previous[0],current[1]-previous[1]),.035+1e-12)
            remaining=math.hypot(.8-current[0],-.3-current[1])
            self.assertLessEqual(remaining,previous_distance+1e-12)
            self.assertTrue(.2<=current[0]<=.8 and -.3<=current[1]<=-.1)
            previous,previous_distance=current,remaining
            self.assertFalse(self.node.permit.publish.call_args.args[0].data)

    def test_paused_time_and_stale_feedback_do_not_accumulate_a_catchup_step(self):
        self.feedback(10.);self.node.pan_tilt(1.,0.)
        self.feedback(10.1);self.node.tick()
        before=self.commands()[-1]
        self.node.tick()
        self.assertEqual(len(self.commands()),1)
        self.clock(10.3);self.node.tick()
        self.assertEqual(len(self.commands()),1,'Stale real feedback cannot drive a command step')
        self.feedback(10.4);self.node.tick()
        after=self.commands()[-1]
        self.assertLessEqual(math.hypot(after[0]-before[0],after[1]-before[1]),.035+1e-12)

    def test_delayed_tick_with_fresh_feedback_caps_one_camera_period_without_debt(self):
        self.feedback(10.);self.node.pan_tilt(1.,0.)
        self.feedback(10.3);self.node.tick()
        self.assertAlmostEqual(self.commands()[-1][0],.035)
        self.feedback(10.4);self.node.tick()
        self.assertAlmostEqual(self.commands()[-1][0],.070)

    def test_cancel_stops_all_future_command_steps(self):
        self.feedback(10.);self.node.pan_tilt(1.,0.)
        self.feedback(10.1);self.node.tick()
        self.node.cancel('operator stop')
        count=len(self.commands())
        self.feedback(10.2);self.node.tick()
        self.assertFalse(self.node.active)
        self.assertEqual(len(self.commands()),count)
        self.assertFalse(self.node.permit.publish.call_args.args[0].data)

    def test_quality_fault_precedes_any_new_step_and_never_auto_resumes(self):
        self.feedback(10.);self.node.pan_tilt(1.,0.)
        self.node.localization_state=Mock(return_value=dict(healthy=False,reason='observation_stale'))
        self.feedback(10.1);self.node.tick()
        self.assertFalse(self.node.active)
        self.assertEqual(self.commands(),[])
        self.node.localization_state.return_value=dict(healthy=True,reason='healthy')
        self.feedback(10.2);self.node.tick()
        self.assertEqual(self.commands(),[])

    def test_lease_expiry_precedes_any_new_step(self):
        self.feedback(10.);self.node.pan_tilt(1.,0.)
        self.node.last_heartbeat=time.monotonic()-1.
        self.feedback(10.1);self.node.tick()
        self.assertFalse(self.node.active)
        self.assertEqual(self.commands(),[])

    def test_clock_backwards_cancels_and_latches_until_explicit_reset(self):
        self.feedback(10.);self.node.pan_tilt(1.,0.)
        self.feedback(10.1);self.node.tick()
        count=len(self.commands())
        self.clock(10.05);self.node.tick()
        self.assertFalse(self.node.active)
        self.assertEqual(self.node.gimbal_command_fault,'clock_moved_backwards')
        self.assertEqual(len(self.commands()),count)
        self.feedback(10.2)
        with self.assertRaisesRegex(ValueError,'clock_moved_backwards'):
            self.node.pan_tilt(1.,0.)
        self.node.reset()
        self.assertIsNone(self.node.gimbal_command_fault)

    def test_final_command_requires_two_new_measured_feedback_samples(self):
        self.feedback(10.);self.node.pan_tilt(.03,0.)
        self.feedback(10.02,(.03,0.))
        self.feedback(10.04,(.03,0.));self.node.tick()
        self.assertTrue(self.node.active,'Existing target-tolerance feedback cannot end an unfinished command ramp')
        self.assertLess(self.commands()[-1][0],.03)
        self.feedback(10.1,(.03,0.));self.node.tick()
        self.assertEqual(self.commands()[-1],(.03,0.))
        self.assertEqual(self.node.gimbal_settled_samples,0)
        self.assertTrue(self.node.active)
        self.feedback(10.11,(.03,0.));self.node.tick();self.assertTrue(self.node.active)
        self.feedback(10.12,(.03,0.));self.node.tick()
        self.assertFalse(self.node.active);self.assertEqual(self.node.phase,'succeeded')

    def test_command_arrival_cannot_replace_actual_target_settlement(self):
        self.node.set_parameters([Parameter('gimbal_position_tolerance_rad',value=math.radians(1)),
            Parameter('gimbal_velocity_tolerance_rad_s',value=.05),Parameter('gimbal_settle_duration_s',value=.2)])
        self.feedback(10.);self.node.pan_tilt(.03,0.)
        self.feedback(10.1);self.node.tick()
        self.assertEqual(self.commands()[-1],(.03,0.))
        self.assertEqual(self.node.gimbal_position,[0.,0.])
        self.assertTrue(self.node.active)
        for at in (10.15,10.2,10.25,10.3):
            self.feedback(at,(.03,0.));self.node.tick()
            self.assertTrue(self.node.active)
        self.feedback(10.35,(.03,0.));self.node.tick()
        self.feedback(10.4,(.03,0.));self.node.tick()
        self.assertFalse(self.node.active);self.assertEqual(self.node.phase,'succeeded')

    def test_explicit_timeout_is_honored_before_another_command_step(self):
        self.node.set_parameters([Parameter('gimbal_action_timeout_s',value=.2)])
        self.feedback(10.);self.node.pan_tilt(1.,0.)
        self.assertEqual(self.node.sim_budget,.2)
        self.feedback(10.1);self.node.tick()
        count=len(self.commands())
        self.feedback(10.201);self.node.tick()
        self.assertEqual(self.node.phase,'failed')
        self.assertFalse(self.node.active)
        self.assertEqual(len(self.commands()),count)

    def test_none_rate_retains_legacy_immediate_target_without_joint_feedback(self):
        self.node.destroy_node();self.node=self.motion(None)
        self.assertIsNone(self.node.gimbal_position)
        self.node.pan_tilt(1.,-.2)
        self.assertEqual(self.commands(),[(1.,-.2)])
        self.assertAlmostEqual(self.node.sim_budget,1./1.5+1.5+5.)


if __name__=='__main__':unittest.main(verbosity=2)
