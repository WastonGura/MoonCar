cmake_minimum_required(VERSION 3.8)
project(mooncar_terrain)

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
add_library(mooncar_risk_overlay SHARED src/grid_overlay.cpp)
target_include_directories(mooncar_risk_overlay PUBLIC include)
target_compile_features(mooncar_risk_overlay PUBLIC cxx_std_17)
target_compile_options(mooncar_risk_overlay PRIVATE -Wall -Wextra -Werror)
ament_target_dependencies(mooncar_risk_overlay nav2_costmap_2d pluginlib tf2_geometry_msgs)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)
install(TARGETS mooncar_risk_overlay LIBRARY DESTINATION lib)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_grid_overlay tests/test_grid_overlay.cpp)
  target_include_directories(test_grid_overlay PRIVATE include)
  ament_target_dependencies(test_grid_overlay nav2_costmap_2d pluginlib tf2_geometry_msgs)
endif()

install(PROGRAMS scripts/terrain_node.py scripts/keepout_node.py scripts/map_recorder.py scripts/terrain_planner_node.py scripts/level_scan_filter.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/terrain_geometry.py scripts/terrain_map.py scripts/terrain_planner.py scripts/navigation_gateway.py scripts/survey_uplink.py DESTINATION lib/${PROJECT_NAME})
install(FILES ../../../scripts/localization_quality.py DESTINATION lib/${PROJECT_NAME})
install(FILES ../../../scripts/literature_terrain.py DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
