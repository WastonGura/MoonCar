"""Run in the existing ROS environment; verifies sensor health and lease/boundary stops."""
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import Mock

sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
try:
    from experiment_motion import ExperimentMotion, within_region, NAVIGATION_STALL_REASON
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
except ImportError:
    ExperimentMotion=None


@unittest.skipIf(ExperimentMotion is None,'Requires the existing sourced ROS environment')
class MotionTests(unittest.TestCase):
    def test_turn_rejects_invalid_yaw_before_action(self):
        for yaw in (True,float('nan'),float('inf'),4.,'0'):
            state=Mock()
            with self.assertRaises(ValueError):ExperimentMotion.turn(state,yaw,dict(region=[-5,-5,5,5]))
            state.spin_client.send_goal_async.assert_not_called()

    def test_diagnostics_retain_first_cancel_and_do_not_weaken_stop(self):
        state=Mock();state.lock=threading.RLock();state.active=True;state.phase='executing'
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
        state=Mock();state.start_checked=False
        task=dict(start=[6.5,-2.,2.4])
        with self.assertRaisesRegex(ValueError,'定位起点与本轮计划起点不一致'):
            ExperimentMotion.check_start(state,task,dict(x=-1.4,y=1.55))
        self.assertFalse(state.start_checked)
        ExperimentMotion.check_start(state,task,dict(x=6.55,y=-2.05))
        self.assertTrue(state.start_checked)

    def test_costmap_freshness_uses_simulation_time_with_wall_disconnect_guard(self):
        from nav_msgs.msg import OccupancyGrid
        state=Mock();state.grid=OccupancyGrid();state.grid.header.stamp.sec=10
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
        state=Mock();state.buffer.lookup_transform.return_value=transform
        state.get_clock.return_value.now.return_value.nanoseconds=10_000_000_000
        self.assertEqual(ExperimentMotion.pose(state)['x'],0.)
        transform.header.stamp.nanosec=301_000_000
        with self.assertRaisesRegex(ValueError,'定位时间失效'):
            ExperimentMotion.pose(state)

    def test_health_and_motion_stops(self):
        state=Mock();state.last_depth=0
        ExperimentMotion.depth(state,DiagnosticArray(status=[DiagnosticStatus(name='map_recorder',level=DiagnosticStatus.OK,message='Archive active')]))
        self.assertEqual(state.last_depth,0)
        ExperimentMotion.depth(state,DiagnosticArray(status=[DiagnosticStatus(name='terrain_geometry',level=DiagnosticStatus.OK,message='Depth active')]))
        self.assertGreater(state.last_depth,0)
        self.assertTrue(within_region([-1,-2,8,6],3.5,3,0))
        self.assertFalse(within_region([-1,-2,8,6],7.8,3,0))
        for stale,map_fresh,pose,reason in [(True,True,dict(x=3.5,y=3,yaw=0),'页面失联'),(False,True,dict(x=7.8,y=3,yaw=0),'接近探测区域边界'),(False,False,dict(x=3.5,y=3,yaw=0),'导航地图失效')]:
            state=Mock();state.lock=threading.RLock();state.active=True;state.phase='executing'
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
        state=Mock();state.grid=grid;state.grid_received=time.monotonic()
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
        state=Mock();state.lock=threading.RLock();state.active=True;state.phase='planning'
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
        state=Mock();state.lock=threading.RLock();state.active=True;state.phase='executing'
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
        state.get_clock.return_value.now.return_value.nanoseconds=36_000_000_000
        ExperimentMotion.tick(state)
        state.cancel.assert_called_once_with(NAVIGATION_STALL_REASON)
        state.cancel.reset_mock();state.pose.return_value=dict(x=1.,y=0.,z=0.,yaw=0.)
        ExperimentMotion.tick(state)
        state.cancel.assert_not_called()
        self.assertEqual(state.best_goal_distance,1.)
        self.assertEqual(state.last_progress_sim,36.)
        state.get_clock.return_value.now.return_value.nanoseconds=52_000_000_000
        ExperimentMotion.tick(state)
        state.cancel.assert_called_once_with(NAVIGATION_STALL_REASON)
        state.cancel.reset_mock();state.feedback['distance_remaining']=.1
        ExperimentMotion.tick(state)
        state.cancel.assert_not_called()

    def test_goal_id_is_recorded_and_late_failure_does_not_stop_new_goal(self):
        from nav2_msgs.action import Spin
        from unique_identifier_msgs.msg import UUID
        state=Mock();client=Mock();future=client.send_goal_async.return_value
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

    def test_path_updates_ignore_previous_goal_and_stopped_task(self):
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped
        state=Mock();state.lock=threading.RLock();state.active=True;state.phase='executing';state.goal=[9,3]
        state.path=[]
        message=Path();message.header.frame_id='map';message.poses=[PoseStamped(),PoseStamped()]
        message.poses[-1].pose.position.x=4.
        ExperimentMotion.on_path(state,message);self.assertEqual(state.path,[])
        message.poses[-1].pose.position.x=9.;message.poses[-1].pose.position.y=3.
        ExperimentMotion.on_path(state,message);self.assertEqual(state.path[-1],[9,3,0])
        state.active=False;message.poses[0].pose.position.x=2.
        ExperimentMotion.on_path(state,message);self.assertEqual(state.path[0],[0,0,0])

    def test_pan_tilt_validation_and_lifecycle(self):
        state=Mock();state.lock=threading.RLock()
        for bad_pan in (3.0, -3.0, float('nan'), '0'):
            with self.assertRaises(ValueError):
                ExperimentMotion.pan_tilt(state, bad_pan, 0.0)
        for bad_tilt in (0.5, -1.0, float('nan')):
            with self.assertRaises(ValueError):
                ExperimentMotion.pan_tilt(state, 0.0, bad_tilt)
        state.active=False;state.state.return_value=dict(ready=True)
        state.pose.return_value=dict(x=0.,y=0.,z=0.,yaw=0.)
        state.get_clock.return_value.now.return_value.nanoseconds=10_000_000_000
        res=ExperimentMotion.pan_tilt(state, 1.0, -0.2)
        self.assertIn('token', res)
        self.assertTrue(state.active)
        self.assertEqual(state.phase, 'executing')
        self.assertEqual(state.goal_request_kind, 'gimbal')
        self.assertEqual(state.pub_camera_pan.publish.call_args.args[0].data, 1.0)
        self.assertEqual(state.pub_camera_tilt.publish.call_args.args[0].data, -0.2)
        # Test tick completing when deadline passed
        state.gimbal_deadline=time.monotonic()-1.0
        state.last_heartbeat=time.monotonic()
        ExperimentMotion.tick(state)
        self.assertFalse(state.active)
        self.assertEqual(state.phase, 'succeeded')
        self.assertFalse(state.permit.publish.call_args.args[0].data)


if __name__=='__main__':unittest.main()
