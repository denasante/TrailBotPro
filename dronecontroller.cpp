#include "dronecontroller.h"

#include <algorithm>

// NEW: for tag/flag handling & logging
<<<<<<< HEAD


=======
>>>>>>> a0f2b1af31f16b5703c05b5c0130d6a351ebba3d
using std::placeholders::_1;

DroneController::DroneController()
: rclcpp::Node("dronecontroller")
{
  // ---- Parameters (with sensible defaults) ----
  min_x_        = this->declare_parameter("min_x", -7.0);
  min_y_        = this->declare_parameter("min_y", -7.0);
  max_x_        = this->declare_parameter("max_x",  7.0);
  max_y_        = this->declare_parameter("max_y",  7.0);
  lane_spacing_ = this->declare_parameter("lane_spacing", 1.0);
  v_            = this->declare_parameter("v", 0.8);       // m/s
  k_yaw_        = this->declare_parameter("k_yaw", 1.0);   // rad/s per rad
  waypoint_tol_ = this->declare_parameter("waypoint_tol", 0.25);
  timer_hz_     = this->declare_parameter("timer_hz", 20.0);

  // NEW: safety-stop parameters
  use_obstacle_flag_ = this->declare_parameter("use_obstacle_flag", true);
<<<<<<< HEAD
  stop_on_tags_      = this->declare_parameter("stop_on_tags", true);
  stop_tag_ids_ = this->declare_parameter<std::vector<int64_t>>("stop_tag_ids", std::vector<int64_t>{7, 42});
=======
>>>>>>> a0f2b1af31f16b5703c05b5c0130d6a351ebba3d
  clear_required_    = this->declare_parameter("clear_required", 10);  // cycles to auto-resume

  // ---- Pubs / Subs ----
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Subscribe to raw odometry (bridged from /model/parrot/odometry -> /odometry)
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", 10, std::bind(&DroneController::odomCb, this, _1));

  pause_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/mission/pause", 10, std::bind(&DroneController::onPause, this, std::placeholders::_1));

  stop_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/mission/stop", 10, std::bind(&DroneController::onStop, this, std::placeholders::_1));

  // NEW: optional obstacle flag (std_msgs/Bool) that latches a stop
  if (use_obstacle_flag_) {
    obstacle_flag_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/detections/obstacle", 10, std::bind(&DroneController::onObstacle, this, std::placeholders::_1));
  }

  RCLCPP_INFO(get_logger(),
    "DroneController ready. Topics: /cmd_vel, /odometry, /mission/pause, /mission/stop%s",
    use_obstacle_flag_ ? ", /detections/obstacle" : "");

  // Build rectangular boustrophedon plan
  buildLawnmowerPlan();

  RCLCPP_INFO(get_logger(),
      "DroneController: rectangular sweep; rect=[%.2f,%.2f]→[%.2f,%.2f], "
      "spacing=%.2f m, lanes=%zu, v=%.2f m/s, k_yaw=%.2f, tol=%.2f m",
      min_x_, min_y_, max_x_, max_y_, lane_spacing_,
      waypoints_.size(), v_, k_yaw_, waypoint_tol_);

  // Timer (control loop)
  using namespace std::chrono_literals;
  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, timer_hz_));
  timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&DroneController::step, this));
}

void DroneController::odomCb(const nav_msgs::msg::Odometry & msg) {
  odom_ = std::make_shared<nav_msgs::msg::Odometry>(msg);
}

void DroneController::step() {
  // Auto-clear (resume) after stable all-clear
  if (blocked_ && clear_count_ >= clear_required_) {
    blocked_ = false;
    clear_count_ = 0;
    RCLCPP_INFO(get_logger(), "RESUME: Clear condition met");
  }

  if (stopped_ || paused_ || blocked_) {
    publishStop();
    if (stopped_) return;

    // Friendly status throttles
    if (paused_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000, "Paused → holding.");
    } else if (blocked_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "Blocked → safety stop.");
    }
    return;
  }

  if (!odom_) {
    // no odometry yet → hold still
    geometry_msgs::msg::Twist zero;
    cmd_pub_->publish(zero);
    return;
  }

  if (wpt_idx_ >= waypoints_.size()) {
    geometry_msgs::msg::Twist zero;
    cmd_pub_->publish(zero);
    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
                         "Survey complete. Holding position.");
    return;
  }

  driveToWaypoint();
}

void DroneController::buildLawnmowerPlan() {
  waypoints_.clear();
  if (max_x_ < min_x_) std::swap(max_x_, min_x_);
  if (max_y_ < min_y_) std::swap(max_y_, min_y_);

  // Number of columns (ensure inclusive of max_x)
  const int cols = std::max(1, static_cast<int>(
      std::floor((max_x_ - min_x_) / std::max(0.01, lane_spacing_)) + 1));

  for (int i = 0; i < cols; ++i) {
    const double x = min_x_ + i * lane_spacing_;
    geometry_msgs::msg::Pose2D p1, p2;

    if (i % 2 == 0) {
      // Up column
      p1.x = x; p1.y = min_y_;
      p2.x = x; p2.y = max_y_;
    } else {
      // Down column
      p1.x = x; p1.y = max_y_;
      p2.x = x; p2.y = min_y_;
    }
    waypoints_.push_back(p1);
    waypoints_.push_back(p2);
  }

  // Keep index at first waypoint
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

  // yaw from quaternion (robust, no tf dependency)
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
  // Simple heading P: turn toward target
  cmd.angular.z = k_yaw_ * err_yaw;
  // Forward speed modulated by alignment (keeps it from arcing too wide)
  cmd.linear.x  = v_ * std::cos(err_yaw);

  // Clamp for safety
  cmd.angular.z = std::clamp(cmd.angular.z, -1.5, 1.5);
  cmd.linear.x  = std::clamp(cmd.linear.x,   0.0,  v_);

  cmd_pub_->publish(cmd);

  if (dist < waypoint_tol_) {
    ++wpt_idx_;
    RCLCPP_INFO(get_logger(), "Reached waypoint %zu/%zu (%.2f, %.2f)",
                wpt_idx_, waypoints_.size(), wpt.x, wpt.y);
  }
}

// ----------------- Safety Stop Callbacks -----------------

void DroneController::onPause(const std_msgs::msg::Bool &msg) {
  paused_ = msg.data;
  if (paused_) {
    RCLCPP_WARN(get_logger(), "PAUSE received → holding position (zero velocity).");
    publishStop();
  } else {
    RCLCPP_INFO(get_logger(), "PAUSE=false → resuming motion.");
  }
}

void DroneController::onStop(const std_msgs::msg::Empty &) {
  if (!stopped_) {
    RCLCPP_ERROR(get_logger(), "STOP received → permanent stop for this run.");
  }
  stopped_ = true;
  publishStop();
}

void DroneController::onObstacle(const std_msgs::msg::Bool &msg) {
  if (msg.data) {
    if (!blocked_) {
      blocked_ = true;
      clear_count_ = 0;
      logObstacle("ObstacleFlag");
    }
  } else {
    // contribute to clear window
    if (blocked_) clear_count_++;
  }
}

<<<<<<< HEAD
void DroneController::onTags(const apriltag_msgs::msg::AprilTagDetectionArray &msg) {
  bool hit = false;
  for (const auto & det : msg.detections) {
    // In your version, 'det.id' is a single int (not a vector)
    const int64_t tag_id = static_cast<int64_t>(det.id);
    if (std::find(stop_tag_ids_.begin(), stop_tag_ids_.end(), tag_id) != stop_tag_ids_.end()) {
      hit = true;
      break;
    }
  }

  if (hit) {
    if (!blocked_) {
      blocked_ = true;
      clear_count_ = 0;
      logObstacle("AprilTag");
    }
  } else {
    if (blocked_) clear_count_++;
  }
}

=======
>>>>>>> a0f2b1af31f16b5703c05b5c0130d6a351ebba3d
void DroneController::logObstacle(const char* source) {
  // record current odom snapshot (if available)
  if (odom_) {
    obstacle_log_.push_back(*odom_);
    const auto &p = odom_->pose.pose.position;
    RCLCPP_WARN(get_logger(), "📍 Logged obstacle (%s) at (x=%.2f, y=%.2f, z=%.2f)",
                source, p.x, p.y, p.z);
  } else {
    RCLCPP_WARN(get_logger(), "📍 Logged obstacle (%s) but odometry not yet available.", source);
  }
}

// ----------------- Utilities -----------------

void DroneController::publishStop() {
  geometry_msgs::msg::Twist zero{};
  cmd_pub_->publish(zero);
}

// ----------------- main -----------------
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DroneController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
