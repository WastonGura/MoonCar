import pathlib
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'scripts'))
import navigation_smoke_test as smoke  # noqa: E402


class NavigationTimeoutTest(unittest.TestCase):
    def test_lifecycle_readiness_without_cli_polling(self):
        node = Mock()
        node.get_parameter.return_value.value = 90.0
        node.create_client.return_value.call_async.return_value.result.return_value.current_state.id = 3
        with patch.object(smoke.rclpy, 'ok', return_value=True), \
                patch.object(smoke.rclpy, 'spin_once'):
            smoke.NavigationSmokeTest.wait_for_navigation(node)
        self.assertEqual(node.create_client.call_count, 6)

    def test_lifecycle_readiness_has_wall_deadline(self):
        node = Mock()
        node.get_parameter.return_value.value = 90.0
        with patch.object(smoke.rclpy, 'ok', return_value=True), \
                patch.object(smoke.time, 'monotonic', side_effect=[0.0, 91.0]):
            with self.assertRaisesRegex(RuntimeError, 'Nav2 not ready'):
                smoke.NavigationSmokeTest.wait_for_navigation(node)

    def run_goal(self, speed, finish_at, **overrides):
        params = {'goal_x': 3.5, 'goal_y': 3.0, 'goal_yaw': 0.0,
                  'server_timeout_s': 30.0, 'goal_timeout_s': 180.0,
                  'wall_timeout_s': 600.0, 'clock_stall_timeout_s': 30.0,
                  'min_displacement_m': 0.0}
        params.update(overrides)
        clock = {'wall': 0.0, 'ros': 0.0}
        result = Mock()
        result.done.side_effect = lambda: clock['ros'] >= finish_at
        result.result.return_value.status = smoke.GoalStatus.STATUS_SUCCEEDED
        handle = Mock(accepted=True)
        handle.get_result_async.return_value = result
        sent = Mock()
        sent.result.return_value = handle
        node = Mock(position=(0.0, 0.0))
        node.get_parameter.side_effect = lambda key: SimpleNamespace(value=params[key])
        node.client.send_goal_async.return_value = sent
        node.get_clock.return_value.now.side_effect = lambda: SimpleNamespace(
            nanoseconds=int(clock['ros'] * 1e9), to_msg=lambda: smoke.rclpy.time.Time().to_msg())

        def step(*args, **kwargs):
            clock['wall'] += 1.0
            clock['ros'] += speed

        with patch.object(smoke.time, 'monotonic', side_effect=lambda: clock['wall']), \
                patch.object(smoke.rclpy, 'spin_once', side_effect=step), \
                patch.object(smoke.rclpy, 'spin_until_future_complete'), \
                patch.object(smoke.rclpy, 'ok', return_value=True):
            try:
                smoke.NavigationSmokeTest.run(node)
            except RuntimeError:
                handle.cancel_goal_async.assert_called_once()
                raise
        handle.cancel_goal_async.assert_not_called()
        return clock

    def test_slow_simulation_can_finish_after_180_wall_seconds(self):
        clock = self.run_goal(0.2, 45.0)
        self.assertGreater(clock['wall'], 180.0)
        self.assertLess(clock['ros'], 180.0)

    def test_ros_timeout_cancels(self):
        with self.assertRaisesRegex(RuntimeError, 'ROS seconds'):
            self.run_goal(1.0, 200.0)

    def test_stopped_clock_cancels(self):
        with self.assertRaisesRegex(RuntimeError, 'clock stopped'):
            self.run_goal(0.0, 1.0)

    def test_wall_limit_cancels_slow_clock(self):
        with self.assertRaisesRegex(RuntimeError, 'wall seconds'):
            self.run_goal(0.01, 1.0, wall_timeout_s=10.0)


if __name__ == '__main__':
    unittest.main()
