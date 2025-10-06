#pragma once
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

class MissionInterface : public rclcpp::Node {
public:
  MissionInterface();
private:
  void candidate_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void hazard_callback(const visualization_msgs::msg::Marker::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cand_sub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr hazard_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr rover_pub_;
};