cmake_minimum_required(VERSION 3.8)
project(mooncar_slam)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/mapping_smoke_test.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
