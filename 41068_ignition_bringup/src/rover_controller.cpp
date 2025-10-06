#include "mission_interface.h"

MissionInterface::MissionInterface() : Node("mission_interface") {
  cand_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/candidate_waypoints", 10,
      std::bind(&MissionInterface::candidate_callback, this, std::placeholders::_1));

  hazard_sub_ = create_subscription<visualization_msgs::msg::Marker>(
      "/hazards", 10,
      std::bind(&MissionInterface::hazard_callback, this, std::placeholders::_1));

  rover_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/waypoint", 10);

  RCLCPP_INFO(get_logger(), "MissionInterface started");
}

void MissionInterface::candidate_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
  if (msg->poses.empty()) {
    RCLCPP_WARN(get_logger(), "No candidate waypoints received");
    return;
  }
  geometry_msgs::msg::PoseStamped target;
  target.header = msg->header;
  target.pose   = msg->poses.front();

  RCLCPP_INFO(get_logger(), "Forwarding waypoint: (%.2f, %.2f)",
              target.pose.position.x, target.pose.position.y);

  rover_pub_->publish(target);
}

void MissionInterface::hazard_callback(const visualization_msgs::msg::Marker::SharedPtr msg) {
  RCLCPP_INFO(get_logger(), "Hazard marker received (id=%d)", msg->id);
}

// ---- main ----
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionInterface>());
  rclcpp::shutdown();
  return 0;
}