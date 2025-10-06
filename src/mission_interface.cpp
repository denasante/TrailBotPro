#include <cmath>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "mission_interface.h"
#include "rover_controller.h"

// ---------- helpers ----------
static void yaw_to_quat(double yaw, double& qx, double& qy, double& qz, double& qw) {
  const double cz = std::cos(yaw * 0.5);
  const double sz = std::sin(yaw * 0.5);
  qx = 0.0; qy = 0.0; qz = sz; qw = cz;
}

geometry_msgs::msg::PoseStamped
MissionInterface::make_pose(double x, double y, double yaw, const std::string& frame_id) {
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame_id;
  p.header.stamp = rclcpp::Clock().now();
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.position.z = 0.0;
  double qx, qy, qz, qw;
  yaw_to_quat(yaw, qx, qy, qz, qw);
  p.pose.orientation.x = qx;
  p.pose.orientation.y = qy;
  p.pose.orientation.z = qz;
  p.pose.orientation.w = qw;
  return p;
}

std::vector<geometry_msgs::msg::PoseStamped> MissionInterface::load_or_generate() {
  std::vector<double> flat;
  std::string frame = "map";

  // Be explicit with defaults (avoid ParameterDescriptor ctor warnings)
  if (!node_->has_parameter("waypoints"))
    node_->declare_parameter<std::vector<double>>("waypoints", std::vector<double>{});
  if (!node_->has_parameter("frame_id"))
    node_->declare_parameter<std::string>("frame_id", frame);

  node_->get_parameter("waypoints", flat);
  node_->get_parameter("frame_id", frame);

  std::vector<geometry_msgs::msg::PoseStamped> out;

  if (flat.size() >= 3 && (flat.size() % 3 == 0)) {
    for (size_t i = 0; i < flat.size(); i += 3)
      out.emplace_back(make_pose(flat[i], flat[i+1], flat[i+2], frame));
    RCLCPP_INFO(node_->get_logger(), "Loaded %zu waypoints from param.", out.size());
    return out;
  }

  // Fallback demo trail
  const double dx = 2.0;
  const int n = 8;
  double x = 0.0, y = 0.0;
  out.emplace_back(make_pose(x, y, 0.0, frame));
  for (int i = 1; i < n; ++i) {
    x += dx;
    out.emplace_back(make_pose(x, y, 0.0, frame));
  }
  out.emplace_back(make_pose(x, y + 2.0, M_PI_2, frame));
  out.emplace_back(make_pose(x, y + 4.0, M_PI_2, frame));
  RCLCPP_WARN(node_->get_logger(), "Param 'waypoints' missing/malformed. Generated %zu demo waypoints.", out.size());
  return out;
}

// -------- runner (main) --------
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("trail_mission");

  MissionInterface mission(node);
  auto goals = mission.load_or_generate();

  RoverController ctrl(node);
  if (!ctrl.setGoals(goals)) {
    RCLCPP_FATAL(node->get_logger(), "No goals provided — abort.");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  rclcpp::Rate rate(10.0); // 10 Hz control loop
  while (rclcpp::ok()) {
    ctrl.run();
    exec.spin_some();

    const auto st = ctrl.status();
    if (st == RoverController::Status::SUCCEEDED) {
      RCLCPP_INFO(node->get_logger(), "Mission complete.");
      break;
    }
    if (st == RoverController::Status::FAILED) {
      RCLCPP_ERROR(node->get_logger(), "Mission failed.");
      break;
    }
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
