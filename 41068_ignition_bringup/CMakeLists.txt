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

# ------------------ Dependencies ------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ros_ign_interfaces REQUIRED)

# Tell the compiler to search your src/ for headers
include_directories(src)

# ------------------ Executables -------------------
add_executable(drone_controller src/drone_controller.cpp)
ament_target_dependencies(drone_controller
  rclcpp sensor_msgs geometry_msgs visualization_msgs cv_bridge)
target_link_libraries(drone_controller OpenCV::opencv)

add_executable(rover_controller src/rover_controller.cpp)
ament_target_dependencies(rover_controller
  rclcpp geometry_msgs nav2_msgs)

add_executable(mission_interface src/mission_interface.cpp)
ament_target_dependencies(mission_interface
  rclcpp geometry_msgs visualization_msgs)

# ------------------ Install -----------------------
install(TARGETS
  drone_controller
  rover_controller
  mission_interface
  DESTINATION lib/${PROJECT_NAME})

# (Resources for bringup)
install(DIRECTORY
  config launch models urdf worlds urdf_drone
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()