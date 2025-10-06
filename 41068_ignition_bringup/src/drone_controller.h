#pragma once
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/twist.hpp"

class DroneController : public rclcpp::Node {
public:
  DroneController();
private:
  void waypoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr hazard_pub_;
};