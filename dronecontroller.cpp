#include "dronecontroller.h"
#include <cmath>
#include <algorithm>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

// --- helpers ---
static inline double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

double DroneController::angle_wrap(double a) {
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double DroneController::yaw_from_quat(double x, double y, double z, double w) {
  return std::atan2(2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z));
}

// --- constructor ---
DroneController::DroneController() : rclcpp::Node("dronecontroller") {
  // Parameters
  odom_topic_        = declare_parameter<std::string>("odom_topic", "/parrot/odometry");
  cmd_vel_topic_     = declare_parameter<std::string>("cmd_vel_topic", "/parrot/cmd_vel");
  mission_target_topic_ = declare_parameter<std::string>("mission_target_topic", "/mission/target");
  status_topic_      = declare_parameter<std::string>("status_topic", "/mission/status");

  kp_lin_            = declare_parameter<double>("kp_lin", 0.8);
  kp_yaw_            = declare_parameter<double>("kp_yaw", 1.0);
  max_speed_         = declare_parameter<double>("max_speed", 0.8);
  max_yaw_rate_      = declare_parameter<double>("max_yaw_rate", 0.8);
  pos_tol_           = declare_parameter<double>("pos_tol", 0.25);
  yaw_tol_           = declare_parameter<double>("yaw_tol", 0.25);
  hold_time_s_       = declare_parameter<double>("hold_time_s", 0.5);

  auto qos = rclcpp::QoS(10).reliable();

  // Publishers
  pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, qos);
  pub_status_ = create_publisher<std_msgs::msg::String>(status_topic_, qos);

  // Subscribers
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, qos, std::bind(&DroneController::odom_cb, this, _1));

  sub_target_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      mission_target_topic_, qos, std::bind(&DroneController::target_cb, this, _1));

  timer_ = create_wall_timer(50ms, std::bind(&DroneController::control_tick, this));

  have_odom_ = false;
  have_target_ = false;
  within_tol_since_ = rclcpp::Time{};

  RCLCPP_INFO(get_logger(),
              "DroneController up. Tracking mission targets on '%s', publishing status on '%s'.",
              mission_target_topic_.c_str(), status_topic_.c_str());
}

// --- Callbacks ---
void DroneController::odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;
  yaw_ = yaw_from_quat(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
  have_odom_ = true;
}

void DroneController::target_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  target_x_ = msg->pose.position.x;
  target_y_ = msg->pose.position.y;
  target_yaw_ = yaw_from_quat(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w);
  have_target_ = true;
  within_tol_since_ = rclcpp::Time{};
  RCLCPP_INFO(get_logger(), "Received new mission target (x=%.2f, y=%.2f, yaw=%.2f)",
              target_x_, target_y_, target_yaw_);
}

// --- Control loop ---
void DroneController::control_tick() {
  geometry_msgs::msg::Twist cmd;

  if (!have_odom_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for odometry...");
    pub_cmd_->publish(cmd);
    return;
  }

  if (!have_target_) {
    pub_cmd_->publish(cmd); // stop
    return;
  }

  // Compute errors
  double ex = target_x_ - x_;
  double ey = target_y_ - y_;
  double dist = std::hypot(ex, ey);
  double heading_to_wp = std::atan2(ey, ex);
  double yaw_err = angle_wrap(heading_to_wp - yaw_);

  // Check if within tolerance
  if (dist < pos_tol_) {
    if (within_tol_since_.nanoseconds() == 0)
      within_tol_since_ = now();

    double held = (now() - within_tol_since_).seconds();
    if (held >= hold_time_s_) {
      std_msgs::msg::String done;
      done.data = "ARRIVED";
      pub_status_->publish(done);
      RCLCPP_INFO(get_logger(), "✅ Arrived at target (x=%.2f, y=%.2f). Holding.", target_x_, target_y_);
      have_target_ = false;
      pub_cmd_->publish(cmd);
      return;
    }
  } else {
    within_tol_since_ = rclcpp::Time{};
  }

  // Compute control
  double vx_cmd = kp_lin_ * dist;
  double wz_cmd = kp_yaw_ * yaw_err;
  vx_cmd = clamp(vx_cmd, -max_speed_, max_speed_);
  wz_cmd = clamp(wz_cmd, -max_yaw_rate_, max_yaw_rate_);

  cmd.linear.x = vx_cmd;
  cmd.angular.z = wz_cmd;
  pub_cmd_->publish(cmd);
}

// --- main ---
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneController>());
  rclcpp::shutdown();
  return 0;
}

