#include "mission_interface.h"

#include <chrono>
#include <fstream>
#include <sstream>
#include <cmath>

using namespace std::chrono_literals;

// ---------- helpers ----------
bool MissionInterface::isZeroQuat(const geometry_msgs::msg::Quaternion &q) {
  return q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0;
}

geometry_msgs::msg::Quaternion MissionInterface::yawToQuat(double yaw_rad) {
  geometry_msgs::msg::Quaternion q;
  const double half = yaw_rad * 0.5;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
}

// ---------- ctor ----------
MissionInterface::MissionInterface() : Node("mission_interface")
{
  // Declare + read params
  this->declare_parameter<std::string>("frame_id",   frame_id_);
  this->declare_parameter<double>("timeout_sec",     timeout_sec_);
  this->declare_parameter<std::string>("path_csv",   path_csv_);
  this->declare_parameter<bool>("loop",              loop_path_);

  this->get_parameter("frame_id",   frame_id_);
  this->get_parameter("timeout_sec",timeout_sec_);
  this->get_parameter("path_csv",   path_csv_);
  this->get_parameter("loop",       loop_path_);

  // Action client
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  // Wait for server (prints once) then start CSV mission if provided
  connect_timer_ = this->create_wall_timer(500ms, [this]{
    if (!nav_client_->wait_for_action_server(0s)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Waiting for action server 'navigate_to_pose'…");
      return;
    }
    RCLCPP_INFO(get_logger(), "MissionInterface ready. Publish PoseStamped to /waypoint, "
                              "or supply -p path_csv:=<file>.");
    connect_timer_->cancel();
    startCsvMissionIfAny();
  });

  // Optional manual single-goal input
  wp_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/waypoint", rclcpp::QoS(10),
    std::bind(&MissionInterface::waypointCb, this, std::placeholders::_1));
}

// ---------- CSV loading ----------
bool MissionInterface::loadCsv(const std::string &path,
                               std::vector<geometry_msgs::msg::PoseStamped> &out)
{
  out.clear();
  std::ifstream f(path);
  if (!f.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open CSV: %s", path.c_str());
    return false;
  }

  std::string line;
  int line_no = 0;
  while (std::getline(f, line)) {
    ++line_no;
    if (line.empty()) continue;

    std::stringstream ss(line);
    std::string tok;
    std::vector<double> vals;
    while (std::getline(ss, tok, ',')) {
      try {
        vals.push_back(std::stod(tok));
      } catch (...) {
        RCLCPP_WARN(get_logger(), "CSV parse warning at line %d: '%s'", line_no, tok.c_str());
      }
    }
    if (vals.size() < 2) {
      RCLCPP_WARN(get_logger(), "CSV line %d has <2 values; skipping", line_no);
      continue;
    }

    const double x = vals[0];
    const double y = vals[1];
    const double yaw_deg = (vals.size() >= 3) ? vals[2] : 0.0;

    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = frame_id_;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.position.z = 0.0;
    p.pose.orientation = yawToQuat(yaw_deg * M_PI / 180.0);
    out.push_back(p);
  }

  RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from '%s'", out.size(), path.c_str());
  return !out.empty();
}

void MissionInterface::startCsvMissionIfAny()
{
  if (path_csv_.empty()) return;

  if (!loadCsv(path_csv_, waypoints_)) {
    RCLCPP_ERROR(get_logger(), "CSV mission not started (load failed).");
    return;
  }
  next_idx_ = 0;
  sendNextFromList();
}

void MissionInterface::sendNextFromList()
{
  if (waypoints_.empty()) return;

  if (next_idx_ >= waypoints_.size()) {
    if (loop_path_) {
      next_idx_ = 0;
      RCLCPP_INFO(get_logger(), "Looping mission back to start.");
    } else {
      RCLCPP_INFO(get_logger(), "🎉 Mission complete.");
      return;
    }
  }

  auto pose = waypoints_[next_idx_];
  pose.header.stamp = now();
  RCLCPP_INFO(get_logger(), "[%zu/%zu] Sending waypoint (%.2f, %.2f)",
              next_idx_ + 1, waypoints_.size(),
              pose.pose.position.x, pose.pose.position.y);
  sendGoal(pose);
}

// ---------- manual /waypoint path (single goal) ----------
void MissionInterface::waypointCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!nav_client_->wait_for_action_server(0s)) {
    RCLCPP_WARN(get_logger(), "Nav2 action server not available yet.");
    return;
  }
  if (goal_active_) {
    RCLCPP_WARN(get_logger(), "A goal is already active; ignoring /waypoint.");
    return;
  }

  auto pose = *msg;
  if (pose.header.frame_id.empty())
    pose.header.frame_id = frame_id_;
  if (isZeroQuat(pose.pose.orientation))
    pose.pose.orientation.w = 1.0;

  sendGoal(pose);
}

// ---------- action send ----------
void MissionInterface::sendGoal(const geometry_msgs::msg::PoseStamped &pose)
{
  goal_active_ = true;

  NavigateToPose::Goal goal;
  goal.pose = pose;

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;

  // Feedback (throttled)
  opts.feedback_callback =
    [this](GoalHandleNav::SharedPtr,
           const std::shared_ptr<const NavigateToPose::Feedback> fb)
    {
      if (!fb) return;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Distance remaining: %.2f m", fb->distance_remaining);
    };

  // Result
  opts.result_callback =
    [this](const GoalHandleNav::WrappedResult &result)
    {
      goal_active_ = false;

      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "✅ Goal reached.");
        if (!waypoints_.empty()) {
          ++next_idx_;
          // Send next after a short breather so TF/Costmaps settle
          this->create_wall_timer(750ms, [this]{ sendNextFromList(); });
        }
        return;
      }

      if (result.code == rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_WARN(get_logger(), "⚠️ Goal canceled (timeout/user).");
      } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
        RCLCPP_ERROR(get_logger(), "❌ Goal aborted by Nav2.");
      } else {
        RCLCPP_ERROR(get_logger(), "❌ Unknown result code.");
      }

      // On failure in a CSV mission, try next waypoint anyway
      if (!waypoints_.empty()) {
        ++next_idx_;
        this->create_wall_timer(750ms, [this]{ sendNextFromList(); });
      }
    };

  auto gh_future = nav_client_->async_send_goal(goal, opts);

  // Timeout watchdog
  auto timeout_ms = std::chrono::milliseconds(
      static_cast<int>(timeout_sec_ * 1000.0));

  auto weak_client = nav_client_;
  this->create_wall_timer(timeout_ms,
    [this, weak_client, gh_future]()
    {
      if (!goal_active_) return;
      auto gh = gh_future.get();
      if (gh) {
        RCLCPP_WARN(get_logger(), "⏱️ Nav2 goal timeout — canceling goal.");
        weak_client->async_cancel_goal(gh);
      }
    });
}

// ---------- main ----------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionInterface>());
  rclcpp::shutdown();
  return 0;
}
