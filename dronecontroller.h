#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <vector>
#include <string>
#include <cmath>
#include <limits>

class DroneController : public rclcpp::Node {
public:
  DroneController();

private:
  // --- Publishers / Subscribers / Timer ---
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   pause_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr  stop_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- State ---
  nav_msgs::msg::Odometry::SharedPtr odom_;
  std::vector<geometry_msgs::msg::Pose2D> waypoints_;
  std::size_t wpt_idx_{0};
  bool paused_{false};
  bool stopped_{false};

  // Avoidance state
  bool  blocked_{false};
  int   clear_count_{0};
  int   detour_dir_{+1};  // +1 = left, -1 = right

  // --- Parameters ---
  // sweep / control
  double min_x_, min_y_, max_x_, max_y_;
  double lane_spacing_;
  double v_, k_yaw_, waypoint_tol_;
  double timer_hz_;

  // lidar / avoidance
  std::string scan_topic_;
  double scan_stop_dist_;
  double scan_clear_dist_;
  double scan_front_half_angle_deg_;
  int    scan_required_hits_;

  // --- Callbacks ---
  void odomCb(const nav_msgs::msg::Odometry & msg);
  void onScan(const sensor_msgs::msg::LaserScan &msg);
  void onPause(const std_msgs::msg::Bool &msg);
  void onStop(const std_msgs::msg::Empty &);

  // --- Control loop & helpers ---
  void step();
  void buildLawnmowerPlan();
  void driveToWaypoint();
  static double wrapToPi(double a);
  void publishStop();
  void logObstacle(const char* source);
};