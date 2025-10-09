# cmake_minimum_required(VERSION 3.8)
# project(41068_ignition_bringup)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)

# find_package(ros_ign_interfaces REQUIRED)

# install(
#   DIRECTORY config launch models urdf worlds urdf_drone
#   DESTINATION share/${PROJECT_NAME}
# )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()

#------------------#
# Rewritten


cmake_minimum_required(VERSION 3.8)
project(41068_ignition_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)            # <-- added
find_package(cv_bridge REQUIRED)
find_package(ros_ign_interfaces REQUIRED)

# --- Include directory for headers ---
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# --- Drone controller executable ---
add_executable(dronecontroller ${CMAKE_CURRENT_SOURCE_DIR}/dronecontroller.cpp)
ament_target_dependencies(dronecontroller
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs         # <-- added
  cv_bridge
  ros_ign_interfaces
)


# --- Mission Interface executable ---
add_executable(mission_interface
  ${CMAKE_CURRENT_SOURCE_DIR}/mission_interface.cpp
)

ament_target_dependencies(mission_interface
  rclcpp
  geometry_msgs
  nav_msgs
)

# Install both executables
install(TARGETS
  dronecontroller
  mission_interface
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install resources (optional) ---
install(DIRECTORY
  config launch models urdf urdf_drone worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()