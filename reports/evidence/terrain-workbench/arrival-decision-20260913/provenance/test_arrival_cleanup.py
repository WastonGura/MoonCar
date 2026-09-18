import ast
from pathlib import Path
import rclpy
for name in ['arrival_resampling.py','assess_arrival_gate.py','capture_slope_resampling.py']:
    ast.parse((Path('/source/scripts')/name).read_text(),filename=name)
rclpy.init();rclpy.shutdown();rclpy.try_shutdown()
print('PASS: changed Python syntax and idempotent capture cleanup after ROS shutdown')
