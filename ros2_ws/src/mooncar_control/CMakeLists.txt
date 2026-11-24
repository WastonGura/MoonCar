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
    scripts/serial_bridge.py
    scripts/pan_tilt_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES scripts/serial_protocol.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS ../../../scripts/r5_adaptive_turn_node.py DESTINATION lib/${PROJECT_NAME})
install(FILES ../../../scripts/r5_adaptive_turn.py ../../../scripts/r5_capture_io.py DESTINATION lib/${PROJECT_NAME})
install(FILES ../../../scripts/localization_quality.py DESTINATION lib/${PROJECT_NAME})

ament_package()
