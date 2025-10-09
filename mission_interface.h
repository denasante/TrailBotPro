// #ifndef MISSION_INTERFACE_H
// #define MISSION_INTERFACE_H

// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <nav_msgs/msg/odometry.hpp>

// // Interface mission: go forward until x >= 2, pause, reverse until x <= 0.
// class MissionInterface : public rclcpp::Node {
// public:
//   MissionInterface();

// private:
//   // Parameters
//   std::string odom_topic_;
//   std::string cmd_vel_topic_;
//   double fwd_speed_;
//   double rev_speed_;
//   double target_forward_x_;
//   double target_back_x_;
//   double pause_secs_;

//   // State variables
//   bool have_odom_;
//   double x_;
//   rclcpp::Time pause_start_;

//   // ROS interfaces
//   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
//   rclcpp::TimerBase::SharedPtr timer_;

//   // Enum for mission phases
//   enum class Phase { WAIT_ODOM, FORWARD, PAUSE, REVERSE, DONE };
//   Phase phase_;

//   // Functions
//   void tick();
//   void publish_stop();
// };

// #endif // MISSION_INTERFACE_H


// ----------------
// ----------------


#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

class MissionInterface : public rclcpp::Node
{
public:
  MissionInterface();

private:
  struct Waypoint {
    double x, y, yaw;
  };

  enum class Phase { WAIT_ODOM, WAIT_ARRIVAL, PAUSE, DONE };
  Phase phase_;

  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_target_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  std::string odom_topic_, target_topic_, status_topic_;
  double pause_secs_;

  // Mission state
  std::vector<Waypoint> waypoints_;
  size_t current_idx_;
  bool have_odom_;
  double x_;
  rclcpp::Time pause_start_;

  // Helpers
  void status_cb(const std_msgs::msg::String::SharedPtr msg);
  void send_target(double x, double y, double yaw);
  void tick();
};