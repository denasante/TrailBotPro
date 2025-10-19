#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

#include <vector>
#include <string>

class RoverController : public rclcpp::Node {
public:
  RoverController();

  // canonical main helper (optional)
  friend int rover_controller_main(int argc, char **argv);

private:
  // ===== callbacks =====
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void target_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void tick();

  // ===== FTG pipeline bits =====
  void preprocess_scan(const sensor_msgs::msg::LaserScan &msg);
  bool select_gap_target(double goal_bearing, double &steer_dir, double &clearance);

  // ===== helpers =====
  static double clamp(double v, double lo, double hi);
  static double ang_wrap(double a);
  static double yaw_from_quat(const geometry_msgs::msg::Quaternion &q);

  // ===== pubs / subs / timer =====
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr   pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr        pub_status_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr   sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_target_;
  rclcpp::TimerBase::SharedPtr                               timer_;

  // ===== params / topics =====
  std::string odom_topic_;
  std::string scan_topic_;
  std::string cmd_topic_;
  std::string target_topic_;
  std::string status_topic_;

  double kp_lin_;
  double kp_yaw_;
  double max_speed_;
  double max_yaw_rate_;
  double pos_tol_;
  double hold_time_s_;

  // Obstacle gating (basic)
  double obs_stop_range_;
  double obs_slow_range_;
  double turn_rate_block_;

  // FTG params
  double ftg_front_fov_rad_;
  double ftg_min_range_;
  double ftg_max_range_;
  double ftg_safety_bubble_;
  int    ftg_smooth_kernel_;
  int    ftg_downsample_;

  // State flags
  bool have_odom_{false};
  bool have_scan_{false};
  bool have_target_{false};
  bool ftg_has_front_{false};

  // Pose / target
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  double tx_{0.0}, ty_{0.0};
  rclcpp::Time reached_since_{};

  // Scan cache
  std::vector<float> scan_preproc_;
  std::vector<double> scan_angles_;
  double angle_min_{0.0};
  double angle_increment_{0.0};
};