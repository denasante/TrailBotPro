#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

class DroneController : public rclcpp::Node
{
public:
  DroneController();

private:
  // --- Helpers ---
  double angle_wrap(double a);
  double yaw_from_quat(double x, double y, double z, double w);

  // --- ROS interfaces ---
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_target_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- Parameters ---
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string mission_target_topic_;
  std::string status_topic_;

  double kp_lin_, kp_yaw_;
  double max_speed_, max_yaw_rate_;
  double pos_tol_, yaw_tol_;
  double hold_time_s_;

  // --- Drone state ---
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  bool have_odom_{false};

  // --- Target state ---
  double target_x_{0.0}, target_y_{0.0}, target_yaw_{0.0};
  bool have_target_{false};
  rclcpp::Time within_tol_since_;

  // --- Callbacks ---
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void target_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // --- Control loop ---
  void control_tick();
};