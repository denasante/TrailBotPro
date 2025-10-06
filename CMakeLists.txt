cmake_minimum_required(VERSION 3.8)
project(41068_ignition_bringup)

# Tooling flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# ---- Dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ros_ign_interfaces REQUIRED)  # keep since package uses ign elsewhere

# ---- Executables ----
# Waypoint-following mission (rover)
add_executable(trail_mission
  src/mission_interface.cpp
  src/rover_controller.cpp
)
ament_target_dependencies(trail_mission
  rclcpp rclcpp_action geometry_msgs nav2_msgs
)

# ---- Install ----
install(
  TARGETS trail_mission
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch models urdf worlds urdf_drone
  DESTINATION share/${PROJECT_NAME}
)

# ---- Tests/Lint ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
