cmake_minimum_required(VERSION 3.8)
project(mooncar_terrain)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/terrain_node.py scripts/keepout_node.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/terrain_geometry.py DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
