#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>  // Pose2D
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <vector>
#include <cstdint>  // <-- ADD THIS

#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <limits>

class DroneController : public rclcpp::Node {
public:
  DroneController();

private:
  // --- ROS ---
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Odometry::SharedPtr odom_;

  // --- Params ---
  double min_x_, min_y_, max_x_, max_y_;
  double lane_spacing_;
  double v_, k_yaw_, waypoint_tol_;
  double timer_hz_;

  // NEW: safety-stop params
  bool use_obstacle_flag_{true};
  int  clear_required_{10};
<<<<<<< HEAD
  std::vector<int64_t> stop_tag_ids_{};   // <-- CHANGE TO int64_t
=======
>>>>>>> a0f2b1af31f16b5703c05b5c0130d6a351ebba3d

  // --- Plan / State ---
  std::vector<geometry_msgs::msg::Pose2D> waypoints_;
  std::size_t wpt_idx_{0};

  // NEW: safety-stop state
  bool blocked_{false};
  int  clear_count_{0};

  // --- Callbacks ---
  void odomCb(const nav_msgs::msg::Odometry & msg);
  void step();

  // --- Helpers ---
  void buildLawnmowerPlan();
  void driveToWaypoint();
  static double wrapToPi(double a);

  // --- Pause/Stop Control ---
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
  bool paused_{false};
  bool stopped_{false};

  // control callbacks
  void onPause(const std_msgs::msg::Bool &msg);
  void onStop(const std_msgs::msg::Empty &);
  void publishStop();

  // NEW: safety-stop inputs
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_flag_sub_;  // /detections/obstacle

  // NEW: safety-stop handlers & logger
  void onObstacle(const std_msgs::msg::Bool &msg);
  void logObstacle(const char* source);

  // --- Logging (you already had these; kept intact) ---
  nav_msgs::msg::Odometry current_odom_;
  std::vector<nav_msgs::msg::Odometry> obstacle_log_;
};
