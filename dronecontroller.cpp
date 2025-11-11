#include "dronecontroller.h"
#include <algorithm>
#include <cmath>

using std::placeholders::_1;

DroneController::DroneController()
: rclcpp::Node("dronecontroller")
{
  // ---- Sweep boundaries ----
  min_x_        = this->declare_parameter("min_x", -10.0);
  min_y_        = this->declare_parameter("min_y", -10.0);
  max_x_        = this->declare_parameter("max_x",  10.0);
  max_y_        = this->declare_parameter("max_y",  10.0);
  lane_spacing_ = this->declare_parameter("lane_spacing", 2.0);
  v_            = this->declare_parameter("v", 0.8);
  k_yaw_        = this->declare_parameter("k_yaw", 1.0);
  waypoint_tol_ = this->declare_parameter("waypoint_tol", 0.25);
  timer_hz_     = this->declare_parameter("timer_hz", 20.0);

  // ---- LiDAR parameters ----
  scan_topic_                = this->declare_parameter<std::string>("scan_topic", "/scan");
  scan_stop_dist_            = this->declare_parameter("scan_stop_dist", 1.2);
  scan_clear_dist_           = this->declare_parameter("scan_clear_dist", 1.8);
  scan_front_half_angle_deg_ = this->declare_parameter("scan_front_half_angle_deg", 40.0);
  scan_required_hits_        = this->declare_parameter("scan_required_hits", 2);

  // ---- Pubs/Subs ----
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", 10, std::bind(&DroneController::odomCb, this, _1));

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DroneController::onScan, this, _1));

  RCLCPP_INFO(get_logger(),
    "DroneController active with avoidance on '%s' (stop<%.2fm, clear>%.2fm, ±%.1f°)",
    scan_topic_.c_str(), scan_stop_dist_, scan_clear_dist_, scan_front_half_angle_deg_);

  // Build sweep plan
  buildLawnmowerPlan();

  using namespace std::chrono_literals;
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / timer_hz_)),
      std::bind(&DroneController::step, this));
}

void DroneController::odomCb(const nav_msgs::msg::Odometry & msg) {
  odom_ = std::make_shared<nav_msgs::msg::Odometry>(msg);
}

// --- Scan callback: detect obstacles + pick avoidance direction ---
void DroneController::onScan(const sensor_msgs::msg::LaserScan &msg)
{
  const double half = scan_front_half_angle_deg_ * M_PI / 180.0;
  int hits = 0;
  double min_front = std::numeric_limits<double>::infinity();
  double left_avg = 0, right_avg = 0;
  int left_count = 0, right_count = 0;

  for (size_t i = 0; i < msg.ranges.size(); ++i) {
    double ang = msg.angle_min + i * msg.angle_increment;
    double r = msg.ranges[i];
    if (!std::isfinite(r)) continue;

    if (std::abs(ang) <= half) {
      if (r < scan_stop_dist_) hits++;
      min_front = std::min(min_front, r);
    }

    // Collect side averages for steering decisions
    if (ang > 0 && ang < 1.0) { left_avg += r; left_count++; }
    if (ang < 0 && ang > -1.0){ right_avg += r; right_count++; }
  }

  double left_mean = (left_count > 0) ? left_avg / left_count : msg.range_max;
  double right_mean = (right_count > 0) ? right_avg / right_count : msg.range_max;

  if (hits >= scan_required_hits_) {
    // Obstacle detected → choose side with more open space
    blocked_ = true;
    clear_count_ = 0;
    logObstacle("LiDAR");

    if (left_mean > right_mean) {
      RCLCPP_WARN(get_logger(), "🟠 Obstacle %.2fm ahead → detouring LEFT (L=%.2f, R=%.2f)",
                  min_front, left_mean, right_mean);
      detour_dir_ = +1; // left
    } else {
      RCLCPP_WARN(get_logger(), "🟠 Obstacle %.2fm ahead → detouring RIGHT (L=%.2f, R=%.2f)",
                  min_front, left_mean, right_mean);
      detour_dir_ = -1; // right
    }
  } else if (blocked_) {
    // Check if clear
    clear_count_++;
    if (min_front > scan_clear_dist_ && clear_count_ > 5) {
      blocked_ = false;
      RCLCPP_INFO(get_logger(), "🟢 Path clear → resuming toward waypoint.");
    }
  }
}

// --- Main control loop ---
void DroneController::step() {
  if (!odom_) return;

  geometry_msgs::msg::Twist cmd;

  if (blocked_) {
    // Simple avoidance: yaw in chosen direction while creeping forward
    cmd.linear.x = 0.2 * v_;
    cmd.angular.z = detour_dir_ * 0.8;
    cmd_pub_->publish(cmd);
    return;
  }

  if (wpt_idx_ >= waypoints_.size()) {
    publishStop();
    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
                         "Mission complete. Hovering.");
    return;
  }

  driveToWaypoint();
}

void DroneController::buildLawnmowerPlan() {
  waypoints_.clear();
  const int cols = std::max(1, static_cast<int>(
      std::floor((max_x_ - min_x_) / std::max(0.01, lane_spacing_)) + 1));

  for (int i = 0; i < cols; ++i) {
    const double x = min_x_ + i * lane_spacing_;
    geometry_msgs::msg::Pose2D p1, p2;
    if ((i % 2) == 0) {
      p1.x = x; p1.y = min_y_;
      p2.x = x; p2.y = max_y_;
    } else {
      p1.x = x; p1.y = max_y_;
      p2.x = x; p2.y = min_y_;
    }
    waypoints_.push_back(p1);
    waypoints_.push_back(p2);
  }
  wpt_idx_ = 0;
}

double DroneController::wrapToPi(double a) {
  while (a >  M_PI) a -= 2*M_PI;
  while (a < -M_PI) a += 2*M_PI;
  return a;
}

void DroneController::driveToWaypoint() {
  const auto &o = *odom_;
  const double x = o.pose.pose.position.x;
  const double y = o.pose.pose.position.y;

  const auto &q = o.pose.pose.orientation;
  const double yaw = std::atan2(2.0*(q.w*q.z + q.x*q.y),
                                1.0 - 2.0*(q.y*q.y + q.z*q.z));

  const auto &wpt = waypoints_[wpt_idx_];
  const double dx = wpt.x - x;
  const double dy = wpt.y - y;
  const double dist = std::hypot(dx, dy);

  const double target_yaw = std::atan2(dy, dx);
  const double err_yaw = wrapToPi(target_yaw - yaw);

  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = std::clamp(k_yaw_ * err_yaw, -1.2, 1.2);
  cmd.linear.x  = std::clamp(v_ * std::cos(err_yaw), 0.0, v_);

  cmd_pub_->publish(cmd);

  if (dist < waypoint_tol_) {
    ++wpt_idx_;
    RCLCPP_INFO(get_logger(), "✅ Reached waypoint %zu/%zu (%.2f, %.2f)",
                wpt_idx_, waypoints_.size(), wpt.x, wpt.y);
  }
}

void DroneController::publishStop() {
  geometry_msgs::msg::Twist zero{};
  cmd_pub_->publish(zero);
}

void DroneController::logObstacle(const char* source) {
  if (odom_) {
    const auto &p = odom_->pose.pose.position;
    RCLCPP_WARN(get_logger(), "🚧 Logged obstacle (%s) at (x=%.2f, y=%.2f, z=%.2f)",
                source, p.x, p.y, p.z);
  }
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DroneController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
