"""Independent ROS output and Gazebo truth check after a gateway mission."""
import json
import math
from pathlib import Path
import sys
import time
from urllib.request import urlopen
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

sys.path.insert(0, '/work/ros2_ws/src/mooncar_navigation/scripts')
from mission_smoke_test import MissionTest

goal = tuple(map(float, sys.argv[1:3]))
expected, output = sys.argv[3], Path(sys.argv[4])
with urlopen('http://127.0.0.1:8766/api/navigation', timeout=5) as response:
    state = json.load(response)
rclpy.init()
node = Node('workbench_independent_witness')
commands = []
node.create_subscription(Twist, '/mooncar/cmd_vel_safe',
                         lambda msg: commands.append((msg.linear.x, msg.angular.z)), 10)
end = time.monotonic() + 2.
try:
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
finally:
    node.destroy_node()
    rclpy.shutdown()
truth = MissionTest.ground_truth()
error = math.hypot(truth[0] - goal[0], truth[1] - goal[1])
report = dict(state=state, goal=goal, ground_truth=truth, ground_truth_error_m=error,
              safe_command_samples=len(commands),
              safe_commands_zero=bool(len(commands) >= 5 and all(abs(x)+abs(z) < 1e-9 for x,z in commands)))
report['passed'] = (state['phase'] == expected and not state['active'] and not state['fault']
                    and report['safe_commands_zero'] and (expected != 'succeeded' or error <= .4))
output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
print(json.dumps({k:v for k,v in report.items() if k != 'state'}, indent=2))
if not report['passed']:
    raise SystemExit('Independent navigation check failed')
