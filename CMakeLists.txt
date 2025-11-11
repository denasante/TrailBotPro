cmake_minimum_required(VERSION 3.8)
project(41068_ignition_bringup LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------- Dependencies -------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ros_ign_interfaces REQUIRED)

# If your headers live in the package root, keep this; otherwise point to "include"
set(PKG_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# ------------------- Executables --------------------
# NOTE: If your sources are actually under "src/", change the paths accordingly.
add_executable(dronecontroller
  ${CMAKE_CURRENT_SOURCE_DIR}/dronecontroller.cpp
)
target_include_directories(dronecontroller PUBLIC ${PKG_INCLUDE_DIR})
ament_target_dependencies(dronecontroller
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs
  cv_bridge ros_ign_interfaces
)

add_executable(rover_controller
  ${CMAKE_CURRENT_SOURCE_DIR}/rover_controller.cpp
)
target_include_directories(rover_controller PUBLIC ${PKG_INCLUDE_DIR})
ament_target_dependencies(rover_controller
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs
  cv_bridge ros_ign_interfaces
)

add_executable(mission_interface
  ${CMAKE_CURRENT_SOURCE_DIR}/mission_interface.cpp
)
target_include_directories(mission_interface PUBLIC ${PKG_INCLUDE_DIR})
ament_target_dependencies(mission_interface
  rclcpp std_msgs geometry_msgs nav_msgs
)

# ------------------- Install binaries ----------------
install(TARGETS
  dronecontroller
  rover_controller
  mission_interface
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------- Install resources ----------------
# This ensures ros2 launch + ament can find your launch/xacro/worlds at runtime.
install(
  DIRECTORY config launch models urdf worlds urdf_drone
  DESTINATION share/${PROJECT_NAME}
)

# ------------------- Install Python nodes ----------------
install(PROGRAMS
  drone_object_detector_bamboo.py
  tree_detector.py
  drone_keyboard_teleop.py
  square_pattern_flight.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/object_landmark_mapper.py
  scripts/frontier_explorer.py
  scripts/drone_goal_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)


ament_package()
