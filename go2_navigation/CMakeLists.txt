cmake_minimum_required(VERSION 3.8)
project(go2_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

ament_package()
