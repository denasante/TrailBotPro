#include "rover_controller.h"
#include <algorithm>
#include <cmath>
#include <limits>

using std::placeholders::_1;

// ===== Helpers =====
double RoverController::clamp(double v, double lo, double hi){
  return std::max(lo, std::min(hi, v));
}
double RoverController::ang_wrap(double a){
  while(a >  M_PI) a -= 2.0*M_PI;
  while(a < -M_PI) a += 2.0*M_PI;
  return a;
}
double RoverController::yaw_from_quat(const geometry_msgs::msg::Quaternion &q){
  return std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
}

// ===== Node =====
RoverController::RoverController() : rclcpp::Node("rover_controller") {
  // topics (default odom is EKF output; change if you want raw)
  odom_topic_    = declare_parameter<std::string>("odom_topic", "/husky/odometry/filtered");
  scan_topic_    = declare_parameter<std::string>("scan_topic", "/scan");
  cmd_topic_     = declare_parameter<std::string>("cmd_vel_topic", "/husky/cmd_vel");
  target_topic_  = declare_parameter<std::string>("mission_target_topic", "/mission/target");
  status_topic_  = declare_parameter<std::string>("status_topic", "/mission/status");

  // control gains
  kp_lin_        = declare_parameter<double>("kp_lin", 0.8);
  kp_yaw_        = declare_parameter<double>("kp_yaw", 1.2);
  max_speed_     = declare_parameter<double>("max_speed", 0.8);
  max_yaw_rate_  = declare_parameter<double>("max_yaw_rate", 1.0);
  pos_tol_       = declare_parameter<double>("pos_tol", 0.25);
  hold_time_s_   = declare_parameter<double>("hold_time_s", 0.5);

  // simple obstacle gating (fallback if FTG finds no gap)
  obs_stop_range_  = declare_parameter<double>("obs_stop_range", 0.6);
  obs_slow_range_  = declare_parameter<double>("obs_slow_range", 1.0);
  turn_rate_block_ = declare_parameter<double>("turn_rate_on_block", 0.6);

  // FTG parameters
  ftg_front_fov_rad_ = declare_parameter<double>("ftg_front_fov_deg", 120.0) * M_PI/180.0;
  ftg_min_range_     = declare_parameter<double>("ftg_min_range", 0.20);
  ftg_max_range_     = declare_parameter<double>("ftg_max_range", 8.0);
  ftg_safety_bubble_ = declare_parameter<double>("ftg_safety_bubble", 0.35);
  ftg_smooth_kernel_ = declare_parameter<int>("ftg_smooth_kernel", 3);
  ftg_downsample_    = declare_parameter<int>("ftg_downsample", 2);

  auto qos = rclcpp::QoS(50).reliable();

  pub_cmd_    = create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, qos);
  pub_status_ = create_publisher<std_msgs::msg::String>(status_topic_, qos);

  sub_odom_   = create_subscription<nav_msgs::msg::Odometry>(odom_topic_, qos,
                  std::bind(&RoverController::odom_cb, this, _1));
  sub_scan_   = create_subscription<sensor_msgs::msg::LaserScan>(scan_topic_, qos,
                  std::bind(&RoverController::scan_cb, this, _1));
  sub_target_ = create_subscription<geometry_msgs::msg::PoseStamped>(target_topic_, qos,
                  std::bind(&RoverController::target_cb, this, _1));

  using namespace std::chrono_literals;
  timer_ = create_wall_timer(50ms, std::bind(&RoverController::tick, this));

  have_odom_ = have_scan_ = have_target_ = false;
  ftg_has_front_ = false;

  RCLCPP_INFO(get_logger(), "RoverController listening: odom=%s scan=%s, cmd=%s",
              odom_topic_.c_str(), scan_topic_.c_str(), cmd_topic_.c_str());
}

// ===== callbacks =====
void RoverController::odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg){
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;
  yaw_ = yaw_from_quat(msg->pose.pose.orientation);
  have_odom_ = true;
}

void RoverController::scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg){
  preprocess_scan(*msg);
  have_scan_ = true;
}

void RoverController::target_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
  tx_ = msg->pose.position.x;
  ty_ = msg->pose.position.y;
  have_target_ = true;
  reached_since_ = rclcpp::Time{};
  RCLCPP_INFO(get_logger(), "New target: (%.2f, %.2f)", tx_, ty_);
}

// ===== FTG: preprocess scan =====
void RoverController::preprocess_scan(const sensor_msgs::msg::LaserScan &msg){
  angle_min_ = msg.angle_min;
  angle_increment_ = msg.angle_increment;

  // Only keep a front arc (±ftg_front_fov_rad_/2)
  int i_min = std::max(0, (int)std::floor((-ftg_front_fov_rad_/2 - msg.angle_min)/msg.angle_increment));
  int i_max = std::min((int)msg.ranges.size()-1,
                       (int)std::ceil(( ftg_front_fov_rad_/2 - msg.angle_min)/msg.angle_increment));

  // Downsample + clamp to [min,max] + simple moving-average smoothing
  std::vector<float> tmp;
  std::vector<double> angs;
  tmp.reserve((i_max - i_min + 1) / std::max(1, ftg_downsample_));
  angs.reserve(tmp.capacity());

  for(int i = i_min; i <= i_max; i += std::max(1, ftg_downsample_)){
    float r = msg.ranges[i];
    if(!std::isfinite(r)) r = ftg_max_range_;
    r = (float)clamp(r, ftg_min_range_, ftg_max_range_);
    tmp.push_back(r);
    angs.push_back(msg.angle_min + i * msg.angle_increment);
  }

  // smooth
  scan_preproc_.resize(tmp.size());
  int k = std::max(1, ftg_smooth_kernel_);
  for(size_t i=0; i<tmp.size(); ++i){
    int a = std::max<int>(0, (int)i - k);
    int b = std::min<int>((int)tmp.size()-1, (int)i + k);
    double acc = 0.0;
    int cnt = 0;
    for(int j=a; j<=b; ++j){ acc += tmp[j]; ++cnt; }
    scan_preproc_[i] = (float)(acc / std::max(1, cnt));
  }

  scan_angles_ = std::move(angs);
  ftg_has_front_ = !scan_preproc_.empty();
}

// ===== FTG: choose gap direction =====
bool RoverController::select_gap_target(double goal_bearing, double &steer_dir, double &clearance){
  if(!ftg_has_front_ || scan_preproc_.empty()) return false;

  // Build "bubble" mask around close obstacles
  std::vector<float> bubble = scan_preproc_;
  for(size_t i=0; i<bubble.size(); ++i){
    if(bubble[i] < ftg_safety_bubble_){
      // inflate neighbors by reducing their range
      int spread = 2; // cells on either side
      for(int d=-spread; d<=spread; ++d){
        int j = (int)i + d;
        if(j>=0 && j<(int)bubble.size()){
          bubble[j] = std::min(bubble[j], (float)ftg_safety_bubble_);
        }
      }
    }
  }

  // Find the longest contiguous segment with ranges > safety_bubble
  int best_a=-1, best_b=-1;
  int cur_a=-1;
  for(int i=0; i<(int)bubble.size(); ++i){
    if(bubble[i] > ftg_safety_bubble_){
      if(cur_a < 0) cur_a = i;
    } else {
      if(cur_a >= 0){
        if(best_a < 0 || (i-1 - cur_a) > (best_b - best_a)){
          best_a = cur_a; best_b = i-1;
        }
        cur_a = -1;
      }
    }
  }
  if(cur_a >= 0){
    if(best_a < 0 || ((int)bubble.size()-1 - cur_a) > (best_b - best_a)){
      best_a = cur_a; best_b = (int)bubble.size()-1;
    }
  }

  if(best_a < 0) return false; // no gap

  // Center of the best gap
  int mid = (best_a + best_b)/2;
  double gap_angle = scan_angles_[mid];
  steer_dir = gap_angle;               // where to steer
  clearance = bubble[mid];             // how open it is
  return true;
}

// ===== control loop =====
void RoverController::tick(){
  geometry_msgs::msg::Twist cmd;

  if(!have_odom_ || !have_scan_){ pub_cmd_->publish(cmd); return; }
  if(!have_target_){ pub_cmd_->publish(cmd); return; }

  // go-to-goal
  double ex = tx_ - x_, ey = ty_ - y_;
  double dist = std::hypot(ex, ey);
  double heading_to_wp = std::atan2(ey, ex);
  double rel_goal = ang_wrap(heading_to_wp - yaw_);

  // arrival check
  if(dist < pos_tol_){
    if(reached_since_.nanoseconds() == 0) reached_since_ = now();
    if((now() - reached_since_).seconds() >= hold_time_s_){
      std_msgs::msg::String s; s.data = "ARRIVED";
      pub_status_->publish(s);
      have_target_ = false;
      pub_cmd_->publish(cmd);
      RCLCPP_INFO(get_logger(), "✅ Arrived at target.");
      return;
    }
  } else {
    reached_since_ = rclcpp::Time{};
  }

  // FTG steering
  double steer_dir = 0.0, clearance = 0.0;
  bool have_gap = select_gap_target(rel_goal, steer_dir, clearance);

  // Fallback simple gating if no gap found
  if(!have_gap){
    // approximate “front min range” from preprocessed arc around 0 rad
    float front_min = std::numeric_limits<float>::infinity();
    if(!scan_preproc_.empty()){
      int mid = (int)scan_preproc_.size()/2;
      for(int i=std::max(0, mid-3); i<=std::min(mid+3, (int)scan_preproc_.size()-1); ++i){
        front_min = std::min(front_min, scan_preproc_[i]);
      }
    }

    double scale = 1.0;
    if(std::isfinite(front_min)){
      if(front_min < obs_stop_range_){
        // blocked — rotate toward goal
        cmd.angular.z = clamp(kp_yaw_ * rel_goal, -turn_rate_block_, turn_rate_block_);
        pub_cmd_->publish(cmd);
        return;
      } else if(front_min < obs_slow_range_){
        scale = 0.3;
      }
    }

    cmd.linear.x  = clamp(kp_lin_ * dist, -max_speed_, max_speed_) * scale;
    cmd.angular.z = clamp(kp_yaw_ * rel_goal, -max_yaw_rate_, max_yaw_rate_);
    pub_cmd_->publish(cmd);
    return;
  }

  // Blend FTG steer with goal direction: here we just steer to gap center
  double yaw_err = steer_dir; // already relative in rover frame
  double speed_scale = std::clamp((clearance - ftg_safety_bubble_) / (ftg_max_range_ - ftg_safety_bubble_), 0.2, 1.0);

  cmd.linear.x  = clamp(kp_lin_ * dist, -max_speed_, max_speed_) * speed_scale;
  cmd.angular.z = clamp(kp_yaw_ * yaw_err, -max_yaw_rate_, max_yaw_rate_);
  pub_cmd_->publish(cmd);
}

// simple C-style main wrapper (optional)
int rover_controller_main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoverController>());
  rclcpp::shutdown();
  return 0;
}

// canonical main
int main(int argc, char **argv){ return rover_controller_main(argc, argv); }