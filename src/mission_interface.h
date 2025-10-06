#pragma once
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class MissionInterface {
public:
  explicit MissionInterface(const rclcpp::Node::SharedPtr& node) : node_(node) {}
  std::vector<geometry_msgs::msg::PoseStamped> load_or_generate();

  static geometry_msgs::msg::PoseStamped make_pose(double x, double y, double yaw_rad,
                                                   const std::string& frame_id);
private:
  rclcpp::Node::SharedPtr node_;
};
