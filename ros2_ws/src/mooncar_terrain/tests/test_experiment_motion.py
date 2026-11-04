"""Run in the existing ROS environment; verifies sensor health and lease/boundary stops."""
from pathlib import Path
import sys
import math
from types import SimpleNamespace
import threading
import time
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
try:
    from experiment_motion import ExperimentMotion, within_region, NAVIGATION_STALL_REASON
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
except ImportError:
    ExperimentMotion=None


@unittest.skipIf(ExperimentMotion is None,'Requires the existing sourced ROS environment')
class MotionTests(unittest.TestCase):
    def test_blocked_data_processing_does_not_block_simulation_clock(self):
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from rosgraph_msgs.msg import Clock
        rclpy.init(args=['--ros-args','-p','use_sim_time:=true'],domain_id=98)
        node=ExperimentMotion();publisher=Node('clock_isolation_test')
        entered=threading.Event();release=threading.Event()
        subscription=node.create_subscription(String,'/blocked_diagnostic',lambda m:(entered.set(),release.wait(3)),1,callback_group=node.data_group)
        data=publisher.create_publisher(String,'/blocked_diagnostic',1)
        clock=publisher.create_publisher(Clock,'/clock',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node);executor.add_node(publisher)
        thread=threading.Thread(target=executor.spin);thread.start()
        try:
            deadline=time.monotonic()+3
            while data.get_subscription_count()<1 and time.monotonic()<deadline:time.sleep(.01)
            data.publish(String(data='block'))
            self.assertTrue(entered.wait(2))
            msg=Clock();msg.clock.sec=11
            deadline=time.monotonic()+1
            while node.get_clock().now().nanoseconds!=11_000_000_000 and time.monotonic()<deadline:
                clock.publish(msg);time.sleep(.01)
            self.assertEqual(node.get_clock().now().nanoseconds,11_000_000_000)
            self.assertFalse(release.is_set())
        finally:
            release.set();executor.shutdown();thread.join(3)
            node.destroy_subscription(subscription);node.destroy_node();publisher.destroy_node();rclpy.shutdown()

    def test_clock_advances_while_all_motion_executor_threads_are_blocked(self):
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String
        rclpy.init(args=['--ros-args','-p','use_sim_time:=true'],domain_id=98)
        node=ExperimentMotion();publisher=Node('motion_executor_clock_test')
        # Remove the ordinary control timer so both workers deterministically
        # block in callbacks, reproducing local scheduling delay without Gazebo.
        for timer in list(node.timers):node.destroy_timer(timer)
        release=threading.Event();entered=[threading.Event(),threading.Event()]
        group=ReentrantCallbackGroup()
        def block(index):
            def callback(msg):
                entered[index].set();release.wait(5)
            return callback
        subscriptions=[node.create_subscription(String,f'/block_motion_{i}',block(i),1,callback_group=group) for i in range(2)]
        data=[publisher.create_publisher(String,f'/block_motion_{i}',1) for i in range(2)]
        clock=publisher.create_publisher(Clock,'/clock',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        executor=MultiThreadedExecutor(num_threads=2);executor.add_node(node)
        thread=threading.Thread(target=executor.spin);thread.start()
        helper=node.clock_node;clock_thread=node.clock_thread
        try:
            deadline=time.monotonic()+3
            while any(p.get_subscription_count()<1 for p in data) and time.monotonic()<deadline:time.sleep(.01)
            for p in data:p.publish(String(data='block'))
            self.assertTrue(all(event.wait(2) for event in entered))
            self.assertFalse(release.is_set())
            for stamp in (11,12):
                message=Clock();message.clock.sec=stamp
                deadline=time.monotonic()+1
                while node.get_clock().now().nanoseconds!=stamp*1_000_000_000 and time.monotonic()<deadline:
                    clock.publish(message);time.sleep(.01)
                self.assertEqual(node.get_clock().now().nanoseconds,stamp*1_000_000_000)
            self.assertEqual(sum(s.topic_name=='/clock' for s in node.subscriptions),0)
            self.assertEqual(sum(s.topic_name=='/clock' for s in helper.subscriptions),1)
            self.assertFalse(release.is_set())
        finally:
            release.set();executor.shutdown();thread.join(3)
            node.destroy_node();publisher.destroy_node();rclpy.shutdown()
        self.assertFalse(clock_thread.is_alive())

    def strict_gimbal_state(self):
        state=Mock();state.localization_state.return_value=dict(healthy=True,reason='disabled');state.lock=threading.RLock();state.active=True;state.goal_request_kind='gimbal'
        state.gimbal_target=[1.,-.2];state.gimbal_joint_stamp=-1.;state.gimbal_settled_samples=0
        state.gimbal_config=(math.radians(1),.05,.3);state.gimbal_settled_since=None
        state.gimbal_previous_feedback_stamp=10.;state.sim_started=10.;state.sim_budget=6.
        state.gimbal_deadline=time.monotonic()+30.;state.last_heartbeat=time.monotonic()
        return state

    def send_strict_feedback(self,state,t,velocity):
        from sensor_msgs.msg import JointState
        msg=JointState();msg.name=['camera_pan_joint','camera_tilt_joint'];msg.position=[1.,-.2]
        msg.velocity=list(velocity);msg.header.stamp.sec=int(t)
        msg.header.stamp.nanosec=round((t-int(t))*1e9)
        state.get_clock.return_value.now.return_value.nanoseconds=round(t*1e9)
        ExperimentMotion.on_gimbal_joints(state,msg)
        ExperimentMotion.tick(state)

    def test_v2_requires_continuous_low_velocity_feedback(self):
        state=self.strict_gimbal_state()
        for t in (10.,10.05,10.1,10.15,10.2,10.25):self.send_strict_feedback(state,t,[0.,0.])
        self.assertTrue(state.active)
        self.send_strict_feedback(state,10.3,[0.,0.])
        self.assertFalse(state.active);self.assertEqual(state.phase,'succeeded')

    def test_v2_rejects_missing_fast_or_nonfinite_velocity(self):
        for velocity in ([],[.1,0.],[float('nan'),0.]):
            state=self.strict_gimbal_state()
            for t in (10.,10.05,10.1,10.15,10.2,10.25,10.3):self.send_strict_feedback(state,t,velocity)
            self.assertTrue(state.active)
            self.assertEqual(state.gimbal_settled_samples,0)

    def test_v2_feedback_gap_resets_settling_window(self):
        state=self.strict_gimbal_state()
        for t in (10.,10.05,10.1,10.3,10.35):self.send_strict_feedback(state,t,[0.,0.])
        self.assertTrue(state.active)
        self.assertAlmostEqual(state.gimbal_settled_since,10.3)

    def test_v2_invalid_position_breaks_pending_settle(self):
        from sensor_msgs.msg import JointState
        for positions in ([],[float('nan'),-.2]):
            state=self.strict_gimbal_state();state.gimbal_settled_since=10.;state.gimbal_settled_samples=4
            msg=JointState();msg.name=['camera_pan_joint','camera_tilt_joint'];msg.position=positions
            msg.header.stamp.sec=10;msg.header.stamp.nanosec=200_000_000
            ExperimentMotion.on_gimbal_joints(state,msg)
            self.assertIsNone(state.gimbal_settled_since);self.assertEqual(state.gimbal_settled_samples,0)

    def test_v2_stale_feedback_cannot_complete_and_times_out(self):
        state=self.strict_gimbal_state();state.gimbal_settled_since=10.;state.gimbal_settled_samples=7
        state.gimbal_joint_stamp=10.3;state.gimbal_joint_received=time.monotonic()
        state.get_clock.return_value.now.return_value.nanoseconds=10_410_000_000
        ExperimentMotion.tick(state);self.assertTrue(state.active)
        state.get_clock.return_value.now.return_value.nanoseconds=17_000_000_000
        ExperimentMotion.tick(state);self.assertEqual(state.phase,'failed')

    def test_turn_rejects_invalid_yaw_before_action(self):
        for yaw in (True,float('nan'),float('inf'),4.,'0'):
            state=Mock();state.localization_state.return_value=dict(healthy=True,reason='disabled')
            with self.assertRaises(ValueError):ExperimentMotion.turn(state,yaw,dict(region=[-5,-5,5,5]))
            state.spin_client.send_goal_async.assert_not_called()

    def test_diagnostics_retain_first_cancel_and_do_not_weaken_stop(self):
        state=Mock();state.localization_state.return_value=dict(healthy=True,reason='disabled');state.lock=threading.RLock();state.active=True;state.phase='executing'
        state.stop_diagnostics=None;state.state.return_value=dict(diagnostics=dict(depth_age_wall_seconds=4.,pose_error='TF future'))
        state.record_stop=lambda reason:ExperimentMotion.record_stop(state,reason)
        ExperimentMotion.cancel(state,'传感器未更新')
        saved=state.stop_diagnostics.copy()
        state.state.return_value=dict(diagnostics=dict(depth_age_wall_seconds=.1,pose_error=''))
        ExperimentMotion.cancel(state,'用户停止')
        self.assertEqual(state.stop_diagnostics,saved)
        self.assertEqual(saved['health']['pose_error'],'TF future')
        self.assertFalse(state.permit.publish.call_args.args[0].data)
        state.phase='executing';state.state.side_effect=RuntimeError('diagnostic unavailable')
        ExperimentMotion.cancel(state,'停止仍需成功')
        self.assertEqual(state.phase,'cancelling')
        self.assertEqual(state.stop_diagnostics['diagnostic_error'],'diagnostic unavailable')

    def test_simulation_clock_reset_clears_previous_experiment_transforms(self):
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from rclpy.clock import ClockType
        rclpy.init(args=['--ros-args','-p','use_sim_time:=true'],domain_id=90)
        node=ExperimentMotion()
        try:
            clock=node.get_clock();self.assertIs(node.buffer.clock,clock)
            clock.set_ros_time_override(rclpy.time.Time(seconds=100,clock_type=ClockType.ROS_TIME))
            transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='test_base'
            transform.header.stamp.sec=100;transform.transform.rotation.w=1.
            node.buffer.set_transform(transform,'test')
            self.assertTrue(node.buffer.can_transform('map','test_base',rclpy.time.Time()))
            clock.set_ros_time_override(rclpy.time.Time(seconds=1,clock_type=ClockType.ROS_TIME))
            self.assertFalse(node.buffer.can_transform('map','test_base',rclpy.time.Time()))
        finally:node.destroy_node();rclpy.shutdown()

    def test_refresh_tf_listener_replays_latched_static_transform(self):
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from rclpy.node import Node
        from tf2_ros import StaticTransformBroadcaster
        rclpy.init(domain_id=90)
        node=ExperimentMotion();publisher=Node('mooncar_static_tf_test')
        try:
            node.listener.unregister()
            transform=TransformStamped();transform.header.frame_id='survey_odom';transform.child_frame_id='odom'
            transform.transform.rotation.w=1.
            broadcaster=StaticTransformBroadcaster(publisher)
            broadcaster.sendTransform(transform)
            self.assertFalse(node.buffer.can_transform('survey_odom','odom',rclpy.time.Time()))
            node.refresh_tf_listener()
            deadline=time.monotonic()+3
            while time.monotonic()<deadline and not node.buffer.can_transform('survey_odom','odom',rclpy.time.Time()):
                rclpy.spin_once(node,timeout_sec=.1)
            self.assertTrue(node.buffer.can_transform('survey_odom','odom',rclpy.time.Time()))
        finally:node.destroy_node();publisher.destroy_node();rclpy.shutdown()

    def test_reset_replaces_static_tf_cache_and_action_clients(self):
        import rclpy
        from geometry_msgs.msg import TransformStamped
        rclpy.init(args=['--ros-args','-p','use_sim_time:=true'],domain_id=90)
        node=ExperimentMotion()
        try:
            transform=TransformStamped();transform.header.frame_id='survey_odom';transform.child_frame_id='odom'
            transform.transform.translation.x=-1.4;transform.transform.rotation.w=1.
            node.buffer.set_transform_static(transform,'previous trial')
            self.assertTrue(node.buffer.can_transform('survey_odom','odom',rclpy.time.Time()))
            old_buffer,old_client=node.buffer,node.client
            node.reset()
            self.assertIsNot(node.buffer,old_buffer)
            self.assertIsNot(node.client,old_client)
            self.assertFalse(node.buffer.can_transform('survey_odom','odom',rclpy.time.Time()))
            self.assertFalse(node.start_checked)
        finally:node.destroy_node();rclpy.shutdown()

    def test_first_action_requires_planned_start(self):
        state=Mock();state.localization_state.return_value=dict(healthy=True,reason='disabled');state.start_checked=False
        task=dict(start=[6.5,-2.,2.4])
        with self.assertRaisesRegex(ValueError,'定位起点与本轮计划起点不一致'):
            ExperimentMotion.check_start(state,task,dict(x=-1.4,y=1.55))
        self.assertFalse(state.start_checked)
        ExperimentMotion.check_start(state,task,dict(x=6.55,y=-2.05))
        self.assertTrue(state.start_checked)

    def test_costmap_freshness_uses_simulation_time_with_wall_disconnect_guard(self):
        from nav_msgs.msg import OccupancyGrid
        state=Mock();state.localization_state.return_value=dict(healthy=True,reason='disabled');state.grid=OccupancyGrid();state.grid.header.stamp.sec=10
        state.grid_received=time.monotonic()-8
        state.get_clock.return_value.now.return_value.nanoseconds=11_000_000_000
        self.assertTrue(ExperimentMotion.grid_fresh(state))
        state.get_clock.return_value.now.return_value.nanoseconds=14_000_000_000
        self.assertFalse(ExperimentMotion.grid_fresh(state))
        state.get_clock.return_value.now.return_value.nanoseconds=9_000_000_000
        self.assertFalse(ExperimentMotion.grid_fresh(state))
        state.get_clock.return_value.now.return_value.nanoseconds=11_000_000_000
        state.grid_received=time.monotonic()-31
        self.assertFalse(ExperimentMotion.grid_fresh(state))
        state.grid=None;self.assertFalse(ExperimentMotion.grid_fresh(state))

    def test_pose_accepts_configured_tf_future_tolerance_but_rejects_larger_skew(self):
        from geometry_msgs.msg import TransformStamped
        transform=TransformStamped();transform.header.stamp.sec=10
        transform.header.stamp.nanosec=178_000_000
        transform.transform.rotation.w=1.
        state=Mock();state.localization_state.return_value=dict(healthy=True,reason='disabled');state.buffer.lookup_transform.return_value=transform
        state.get_clock.return_value.now.return_value.nanoseconds=10_000_000_000
        self.assertEqual(ExperimentMotion.pose(state)['x'],0.)
        transform.header.stamp.nanosec=301_000_000
        with self.assertRaisesRegex(ValueError,'定位时间失效'):
            ExperimentMotion.pose(state)
        transform.header.stamp.sec=7;transform.header.stamp.nanosec=999_000_000
        with self.assertRaisesRegex(ValueError,'定位时间失效'):
            ExperimentMotion.pose(state)

    def test_health_and_motion_stops(self):
        state=Mock();state.localization_state.return_value=dict(healthy=True,reason='disabled');state.last_depth=0
        ExperimentMotion.depth(state,DiagnosticArray(status=[DiagnosticStatus(name='map_recorder',level=DiagnosticStatus.OK,message='Archive active')]))
        self.assertEqual(state.last_depth,0)
        ExperimentMotion.depth(state,DiagnosticArray(status=[DiagnosticStatus(name='terrain_geometry',level=DiagnosticStatus.OK,message='Depth active')]))
        self.assertGreater(state.last_depth,0)
        self.assertTrue(within_region([-1,-2,8,6],3.5,3,0))
        self.assertFalse(within_region([-1,-2,8,6],7.8,3,0))
        for stale,map_fresh,pose,reason in [(True,True,dict(x=3.5,y=3,yaw=0),'页面失联'),(False,True,dict(x=7.8,y=3,yaw=0),'接近探测区域边界'),(False,False,dict(x=3.5,y=3,yaw=0),'导航地图失效')]:
            state=Mock();state.localization_state.return_value=dict(healthy=True,reason='disabled');state.lock=threading.RLock();state.active=True;state.phase='executing'
            state.started=state.last_depth=time.monotonic();state.last_heartbeat=0 if stale else time.monotonic()
            state.mission_limits=None
            state.grid_fresh.return_value=map_fresh
            state.sim_started=0.;state.sim_budget=180.;state.get_clock.return_value.now.return_value.nanoseconds=10_000_000_000
            state.track=[[3.5,3,0]]
            state.region=[-1,-2,8,6];state.pose.return_value=pose
            pose['z']=0.
            state.cancel.side_effect=lambda _:setattr(state,'phase','cancelling')
            ExperimentMotion.tick(state)
            self.assertIn(reason,state.cancel.call_args.args[0])
            self.assertFalse(state.permit.publish.call_args.args[0].data)

    def test_goal_limits_feedback_and_simulation_timeout(self):
        from nav_msgs.msg import OccupancyGrid
        from nav2_msgs.action import NavigateToPose
        from types import SimpleNamespace
        grid=OccupancyGrid();grid.info.width=grid.info.height=3;grid.info.resolution=1.
        grid.data=[0,0,0,0,-1,0,0,0,100]
        state=Mock();state.localization_state.return_value=dict(healthy=True,reason='disabled');state.grid=grid;state.grid_received=time.monotonic()
        ExperimentMotion.check_goal(state,[.2,.2])
        for target in ([4,0],[1.2,1.2],[2.2,2.2]):
            with self.assertRaises(ValueError):ExperimentMotion.check_goal(state,target)
        state.lock=threading.RLock();state.active=True;state.token='current';state.feedback={}
        state.best_distance=float('inf');state.last_progress_sim=0.
        state.get_clock.return_value.now.return_value.nanoseconds=0
        feedback=NavigateToPose.Feedback();feedback.distance_remaining=3.5
        ExperimentMotion.on_feedback(state,SimpleNamespace(feedback=feedback),'old')
        self.assertEqual(state.feedback,{})
        ExperimentMotion.on_feedback(state,SimpleNamespace(feedback=feedback),'current')
        self.assertEqual(state.feedback['distance_remaining'],3.5)
        state.phase='executing';state.started=time.monotonic()-200
        state.mission_limits=None
        state.last_depth=state.last_heartbeat=time.monotonic();state.sim_started=0.;state.sim_budget=180.
        state.get_clock.return_value.now.return_value.nanoseconds=30_000_000_000
        state.pose.return_value=dict(x=3.5,y=3,z=0,yaw=0);state.region=[-1,-2,8,6];state.track=[[3.5,3,0]]
        ExperimentMotion.tick(state);state.cancel.assert_not_called()
        state.get_clock.return_value.now.return_value.nanoseconds=181_000_000_000
        ExperimentMotion.tick(state);self.assertIn('仿真时间预算',state.cancel.call_args.args[0])
        state.get_clock.return_value.now.return_value.nanoseconds=30_000_000_000
        state.mission_limits=dict(origin=[3.5,3],radius=10,deadline=20)
        ExperimentMotion.tick(state);self.assertIn('自主探测时间预算',state.cancel.call_args.args[0])
        state.mission_limits=dict(origin=[0,0],radius=2,deadline=100)
        ExperimentMotion.tick(state);self.assertIn('活动距离限制',state.cancel.call_args.args[0])

    def test_unanswered_goal_fails_before_exhausting_mission_budget(self):
        state=Mock();state.localization_state.return_value=dict(healthy=True,reason='disabled');state.lock=threading.RLock();state.active=True;state.phase='planning'
        state.goal_request_future.done.return_value=False
        state.goal_sent_wall=time.monotonic()-6
        state.goal_request_id='diagnostic-id';state.goal_request_kind='spin'
        ExperimentMotion.tick(state)
        self.assertFalse(state.active)
        self.assertEqual(state.phase,'failed')
        self.assertIn('目标接收超过 5 秒',state.error)
        self.assertFalse(state.permit.publish.call_args.args[0].data)
        self.assertEqual(state.record_stop.call_args.kwargs['goal_request_id'],'diagnostic-id')
        self.assertFalse(state.record_stop.call_args.kwargs['goal_response_future_done'])

    def test_no_progress_cancels_navigation_without_waiting_for_recoveries(self):
        from nav2_msgs.action import NavigateToPose
        from types import SimpleNamespace
        state=Mock();state.localization_state.return_value=dict(healthy=True,reason='disabled');state.lock=threading.RLock();state.active=True;state.phase='executing'
        state.token='goal';state.goal_request_kind='navigate_to_pose'
        state.goal=[2.,0.];state.best_goal_distance=2.
        state.best_distance=float('inf');state.last_progress_sim=0.;state.feedback={}
        state.get_clock.return_value.now.return_value.nanoseconds=10_000_000_000
        feedback=NavigateToPose.Feedback();feedback.distance_remaining=2.;feedback.number_of_recoveries=0
        feedback.distance_remaining=0.
        ExperimentMotion.on_feedback(state,SimpleNamespace(feedback=feedback),'goal')
        self.assertEqual(state.best_distance,float('inf'))
        feedback.distance_remaining=2.
        ExperimentMotion.on_feedback(state,SimpleNamespace(feedback=feedback),'goal')
        self.assertEqual(state.last_progress_sim,10.)
        feedback.distance_remaining=1.8;feedback.number_of_recoveries=0
        state.get_clock.return_value.now.return_value.nanoseconds=20_000_000_000
        ExperimentMotion.on_feedback(state,SimpleNamespace(feedback=feedback),'goal')
        self.assertEqual(state.last_progress_sim,20.)
        state.last_depth=state.last_heartbeat=time.monotonic();state.started=time.monotonic()
        state.sim_started=0.;state.sim_budget=120.;state.mission_limits=None
        state.grid_fresh.return_value=True;state.region=[-5,-5,5,5]
        state.pose.return_value=dict(x=0.,y=0.,z=0.,yaw=0.);state.track=[[0.,0.,0.]]
        state.get_clock.return_value.now.return_value.nanoseconds=44_000_000_000
        ExperimentMotion.tick(state)
        state.cancel.assert_not_called()
        state.get_clock.return_value.now.return_value.nanoseconds=46_000_000_000
        ExperimentMotion.tick(state)
        state.cancel.assert_called_once_with(NAVIGATION_STALL_REASON)
        state.cancel.reset_mock();state.pose.return_value=dict(x=1.,y=0.,z=0.,yaw=0.)
        ExperimentMotion.tick(state)
        state.cancel.assert_not_called()
        self.assertEqual(state.best_goal_distance,1.)
        self.assertEqual(state.last_progress_sim,46.)
        state.get_clock.return_value.now.return_value.nanoseconds=72_000_000_000
        ExperimentMotion.tick(state)
        state.cancel.assert_called_once_with(NAVIGATION_STALL_REASON)
        state.cancel.reset_mock();state.feedback['distance_remaining']=.1
        ExperimentMotion.tick(state)
        state.cancel.assert_not_called()

    def test_goal_id_is_recorded_and_late_failure_does_not_stop_new_goal(self):
        from nav2_msgs.action import Spin
        from unique_identifier_msgs.msg import UUID
        state=Mock();state.localization_state.return_value=dict(healthy=True,reason='disabled');client=Mock();future=client.send_goal_async.return_value
        ExperimentMotion.send_goal(state,client,Spin.Goal(),'old','spin')
        goal_uuid=client.send_goal_async.call_args.kwargs['goal_uuid']
        self.assertIsInstance(goal_uuid,UUID)
        self.assertEqual(bytes(goal_uuid.uuid).hex(),state.goal_request_id)
        future.add_done_callback.call_args.args[0](future)
        state.accepted.assert_called_once_with(future,'old',state.goal_request_id)
        state.lock=threading.RLock();state.token='new';state.active=True;state.phase='executing'
        future.result.side_effect=RuntimeError('old response failed')
        ExperimentMotion.accepted(state,future,'old',state.goal_request_id)
        self.assertTrue(state.active)
        self.assertEqual(state.phase,'executing')
        state.token='old';state.active=False;state.phase='failed';state.error='response timeout'
        future.result.side_effect=None;future.result.return_value.accepted=False
        ExperimentMotion.accepted(state,future,'old',state.goal_request_id)
        self.assertEqual(state.error,'response timeout')

    def test_goal_callback_registration_does_not_call_slow_diagnostic_logger(self):
        from nav2_msgs.action import Spin
        state=Mock();state.localization_state.return_value=dict(healthy=True,reason='disabled');client=Mock()
        state.get_logger.return_value.info.side_effect=AssertionError('Diagnostic I/O must not enter the locked send path')
        ExperimentMotion.send_goal(state,client,Spin.Goal(),'lease','spin')
        client.send_goal_async.return_value.add_done_callback.assert_called_once()
        state.get_logger.assert_not_called()
        self.assertLessEqual(state.goal_dispatch['sent_wall'],state.goal_dispatch['send_return_wall'])
        self.assertLessEqual(state.goal_dispatch['send_return_wall'],state.goal_dispatch['callback_attach_started_wall'])

    def test_reset_keeps_action_clients_in_control_callback_group(self):
        import rclpy
        from rclpy.action import ActionClient
        rclpy.init(args=['--ros-args','-p','use_sim_time:=true'],domain_id=98)
        node=ExperimentMotion()
        try:
            with patch('experiment_motion.ActionClient',wraps=ActionClient) as created:
                node.reset()
            self.assertEqual(created.call_count,2)
            self.assertTrue(all(c.kwargs['callback_group'] is node.control_group for c in created.call_args_list))
            self.assertIsNone(node.goal_dispatch)
        finally:
            node.destroy_node();rclpy.shutdown()

    def test_completed_response_still_obeys_original_five_second_deadline(self):
        from experiment_motion import GOAL_RESPONSE_TIMEOUT_SECONDS
        self.assertEqual(GOAL_RESPONSE_TIMEOUT_SECONDS,5.)
        for elapsed,phase in ((4.999,'executing'),(5.,'executing'),(5.001,'failed')):
            with self.subTest(elapsed=elapsed):
                state=Mock();state.localization_state.return_value=dict(healthy=True,reason='disabled');state.lock=threading.RLock();state.token='lease';state.active=True;state.phase='planning'
                state.goal_request_id='request';state.goal_request_kind='navigate_to_pose';state.goal_sent_wall=100.
                state.goal_dispatch=dict(request_id='request')
                future=Mock();handle=future.result.return_value;handle.accepted=True
                with patch('experiment_motion.time.monotonic',return_value=100.+elapsed):
                    ExperimentMotion.accepted(state,future,'lease','request')
                self.assertEqual(state.phase,phase)
                self.assertAlmostEqual(state.goal_dispatch['response_latency_wall_seconds'],elapsed)
                state.get_logger.assert_not_called()
                if phase=='failed':
                    self.assertFalse(state.active);handle.cancel_goal_async.assert_called_once()
                    self.assertFalse(state.permit.publish.call_args.args[0].data)
                    handle.get_result_async.assert_not_called()
                else:
                    self.assertTrue(state.active);handle.cancel_goal_async.assert_not_called()
                    handle.get_result_async.assert_called_once()

    def test_path_updates_ignore_previous_goal_and_stopped_task(self):
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped
        state=Mock();state.localization_state.return_value=dict(healthy=True,reason='disabled');state.lock=threading.RLock();state.active=True;state.phase='executing';state.goal=[9,3]
        state.path=[]
        message=Path();message.header.frame_id='map';message.poses=[PoseStamped(),PoseStamped()]
        message.poses[-1].pose.position.x=4.
        ExperimentMotion.on_path(state,message);self.assertEqual(state.path,[])
        message.poses[-1].pose.position.x=9.;message.poses[-1].pose.position.y=3.
        ExperimentMotion.on_path(state,message);self.assertEqual(state.path[-1],[9,3,0])
        state.active=False;message.poses[0].pose.position.x=2.
        ExperimentMotion.on_path(state,message);self.assertEqual(state.path[0],[0,0,0])

    def test_pan_tilt_validation_and_lifecycle(self):
        state=Mock();state.localization_state.return_value=dict(healthy=True,reason='disabled');state.lock=threading.RLock()
        for bad_pan in (3.0, -3.0, float('nan'), '0'):
            with self.assertRaises(ValueError):
                ExperimentMotion.pan_tilt(state, bad_pan, 0.0)
        for bad_tilt in (0.5, -1.0, float('nan')):
            with self.assertRaises(ValueError):
                ExperimentMotion.pan_tilt(state, 0.0, bad_tilt)
        state.active=False;state.state.return_value=dict(ready=True)
        state.activate_localization_action.side_effect=lambda:setattr(state,'active',True)
        defaults={'gimbal_position_tolerance_rad':math.radians(5),'gimbal_velocity_tolerance_rad_s':0.,
                  'gimbal_settle_duration_s':0.,'gimbal_action_timeout_s':0.}
        state.get_parameter.side_effect=lambda name:SimpleNamespace(value=defaults[name])
        state.pose.return_value=dict(x=0.,y=0.,z=0.,yaw=0.)
        state.get_clock.return_value.now.return_value.nanoseconds=10_000_000_000
        res=ExperimentMotion.pan_tilt(state, 1.0, -0.2)
        self.assertIn('token', res)
        self.assertTrue(state.active)
        self.assertEqual(state.phase, 'executing')
        self.assertEqual(state.goal_request_kind, 'gimbal')
        self.assertEqual(state.pub_camera_pan.publish.call_args.args[0].data, 1.0)
        self.assertEqual(state.pub_camera_tilt.publish.call_args.args[0].data, -0.2)
        # Time alone cannot confirm a measured joint target.
        state.gimbal_joint_received=0.;state.gimbal_joint_stamp=-1.;state.gimbal_settled_samples=0
        state.last_heartbeat=time.monotonic()
        ExperimentMotion.tick(state)
        self.assertTrue(state.active)
        # Two current feedback samples at target confirm completion.
        state.gimbal_joint_received=time.monotonic();state.gimbal_joint_stamp=10.
        state.gimbal_settled_samples=2
        ExperimentMotion.tick(state)
        self.assertFalse(state.active)
        self.assertEqual(state.phase, 'succeeded')
        self.assertFalse(state.permit.publish.call_args.args[0].data)

    def test_gimbal_feedback_and_timeout(self):
        from sensor_msgs.msg import JointState
        state=Mock();state.localization_state.return_value=dict(healthy=True,reason='disabled');state.lock=threading.RLock();state.active=True;state.goal_request_kind='gimbal'
        state.gimbal_target=[1.,-.2];state.gimbal_joint_stamp=-1.;state.gimbal_settled_samples=0
        state.gimbal_config=(math.radians(5),0.,0.);state.gimbal_settled_since=None
        state.gimbal_previous_feedback_stamp=10.
        state.sim_started=10.;state.sim_budget=7.;state.gimbal_deadline=time.monotonic()+30.
        state.last_heartbeat=time.monotonic();state.get_clock.return_value.now.return_value.nanoseconds=10_000_000_000
        message=JointState();message.name=['camera_pan_joint','camera_tilt_joint']
        message.position=[0.,-.2];message.header.stamp.sec=10
        ExperimentMotion.on_gimbal_joints(state,message)
        self.assertEqual(state.gimbal_settled_samples,0)
        message.position=[1.,-.2];message.header.stamp.nanosec=100_000_000
        ExperimentMotion.on_gimbal_joints(state,message)
        self.assertEqual(state.gimbal_settled_samples,1)
        ExperimentMotion.on_gimbal_joints(state,message)
        self.assertEqual(state.gimbal_settled_samples,1,'duplicate feedback cannot complete scan')
        message.header.stamp.nanosec=200_000_000
        ExperimentMotion.on_gimbal_joints(state,message)
        self.assertEqual(state.gimbal_settled_samples,2)
        state.gimbal_joint_received=time.monotonic()-3.
        ExperimentMotion.tick(state)
        self.assertTrue(state.active,'stale samples cannot confirm completion')
        state.get_clock.return_value.now.return_value.nanoseconds=18_000_000_000
        ExperimentMotion.tick(state)
        self.assertFalse(state.active);self.assertEqual(state.phase,'failed')
        self.assertFalse(state.permit.publish.call_args.args[0].data)
        state.turn.assert_not_called()


if __name__=='__main__':unittest.main()
