
#include "drone_controller.h"

DroneController::DroneController() : Node("drone_controller") {
  waypoint_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/drone/waypoint", 10,
      std::bind(&DroneController::waypoint_callback, this, std::placeholders::_1));

  camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image", 10,
      std::bind(&DroneController::camera_callback, this, std::placeholders::_1));

  vel_pub_    = create_publisher<geometry_msgs::msg::Twist>("/drone/cmd_vel", 10);
  hazard_pub_ = create_publisher<visualization_msgs::msg::Marker>("/hazards", 10);

  RCLCPP_INFO(get_logger(), "DroneController started");
}

void DroneController::waypoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  RCLCPP_INFO(get_logger(), "Drone waypoint: (%.2f, %.2f, %.2f)",
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  // TODO: simple position controller -> publish Twist on /drone/cmd_vel
}

void DroneController::camera_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                       "Drone camera image: %ux%u", msg->width, msg->height);
  // TODO: quick hazard heuristic -> publish visualization_msgs/Marker on /hazards
}

// ---- main ----
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneController>());
  rclcpp::shutdown();
  return 0;
}