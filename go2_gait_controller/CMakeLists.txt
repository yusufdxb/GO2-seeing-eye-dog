cmake_minimum_required(VERSION 3.8)
project(go2_gait_controller)

# ── Compiler flags ────────────────────────────────────────────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ──────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  builtin_interfaces
)

# ── Library ───────────────────────────────────────────────────
add_library(gait_controller_lib
  src/gait_controller_node.cpp
)

target_include_directories(gait_controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gait_controller_lib ${DEPENDENCIES})

# ── Executable ────────────────────────────────────────────────
add_executable(go2_gait_controller_node
  src/main.cpp
)

target_link_libraries(go2_gait_controller_node
  gait_controller_lib
)

ament_target_dependencies(go2_gait_controller_node ${DEPENDENCIES})

# ── Install ───────────────────────────────────────────────────
install(TARGETS
  gait_controller_lib
  go2_gait_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config worlds scripts
  DESTINATION share/${PROJECT_NAME}
)

# ── Tests ─────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
