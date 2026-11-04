cmake_minimum_required(VERSION 3.8)
project(mooncar_slam)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/mapping_smoke_test.sh
           scripts/laser_odometry_node.py
           scripts/terrain_odometry_node.py
           scripts/rgbd_odometry_node.py
           scripts/depth_localization_map.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES scripts/laser_odometry_core.py scripts/metric_laser_odometry.py scripts/metric_depth_odometry.py scripts/metric_rgbd_odometry.py DESTINATION lib/${PROJECT_NAME})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../../scripts/localization_quality.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/../../../scripts/rgbd_window_odometry.py
        DESTINATION lib/${PROJECT_NAME})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../../scripts/rgbd_window_shadow.py
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../scripts/offline_rgbd_window
        DESTINATION lib/${PROJECT_NAME} FILES_MATCHING PATTERN "*.py")

ament_package()
