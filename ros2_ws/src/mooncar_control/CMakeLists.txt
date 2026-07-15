cmake_minimum_required(VERSION 3.8)
project(mooncar_control)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/cmd_vel_safety_relay.py
    scripts/drive_smoke_test.py
    scripts/keyboard_teleop.py
    scripts/wheel_odometry.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
