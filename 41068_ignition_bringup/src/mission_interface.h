#pragma once
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class RoverController : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  RoverController();
private:
  void waypoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
};