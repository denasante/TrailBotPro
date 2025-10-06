#pragma once
#include <vector>
#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

class RoverController {
public:
  enum class Status { IDLE, RUNNING, SUCCEEDED, FAILED };

  explicit RoverController(const rclcpp::Node::SharedPtr& node,
                           const std::string& action_name = "navigate_to_pose")
  : node_(node),
    action_client_(rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, action_name)),
    status_(Status::IDLE),
    current_idx_(0) {}

  bool setGoals(const std::vector<geometry_msgs::msg::PoseStamped>& goals) {
    if (goals.empty()) return false;
    goals_ = goals;
    current_idx_ = 0;
    status_ = Status::IDLE;
    return true;
  }

  Status status() const { return status_; }
  void run();  // non-blocking

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
  Status status_;
  size_t current_idx_;

  void send_current_goal_();
};
