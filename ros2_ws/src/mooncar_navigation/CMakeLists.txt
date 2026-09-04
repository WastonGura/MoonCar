cmake_minimum_required(VERSION 3.8)
project(mooncar_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/navigation_smoke_test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
