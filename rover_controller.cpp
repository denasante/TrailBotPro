#include "rover_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::OccupancyGrid;

RoverController::RoverController() : Node("rover_controller")
{
  // Declare + read parameters
  this->declare_parameter<std::string>("map_frame", map_frame_);
  this->declare_parameter<int>("min_cluster_cells", min_cluster_cells_);
  this->declare_parameter<double>("goal_offset_m", goal_offset_m_);
  this->declare_parameter<double>("replan_period_sec", replan_period_sec_);
  this->declare_parameter<double>("min_goal_sep_m", min_goal_sep_m_);

  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("min_cluster_cells", min_cluster_cells_);
  this->get_parameter("goal_offset_m", goal_offset_m_);
  this->get_parameter("replan_period_sec", replan_period_sec_);
  this->get_parameter("min_goal_sep_m", min_goal_sep_m_);

  // Publisher: waypoints to MissionInterface
  wp_pub_ = this->create_publisher<PoseStamped>("/waypoint", 10);

  // Subscriber: occupancy grid from Nav2/map_server or SLAM
  map_sub_ = this->create_subscription<OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&RoverController::mapCb, this, std::placeholders::_1));

  // Periodic replan timer
  replan_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(replan_period_sec_),
      std::bind(&RoverController::timerTick, this));

  RCLCPP_INFO(get_logger(),
              "RoverController up. frame=%s, min_cluster=%d, offset=%.2f m, "
              "replan=%.1f s, min_goal_sep=%.2f m",
              map_frame_.c_str(), min_cluster_cells_, goal_offset_m_,
              replan_period_sec_, min_goal_sep_m_);
}

void RoverController::mapCb(const OccupancyGrid::SharedPtr msg)
{
  latest_map_ = msg;
}

void RoverController::timerTick()
{
  if (!latest_map_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                    "Waiting for /map before searching frontiers…");
    return;
  }

  double gx = 0.0, gy = 0.0;
  if (!findBestFrontier(gx, gy)) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                    "No suitable frontier found (maybe fully explored?)");
    return;
  }

  // Enforce minimum separation from last goal to avoid spam
  if (last_goal_) {
    const auto &lg = last_goal_.value();
    double dx = gx - lg.pose.position.x;
    double dy = gy - lg.pose.position.y;
    if ((dx * dx + dy * dy) < sqr(min_goal_sep_m_)) {
      // too close to previous goal; skip this tick
      return;
    }
  }

  PoseStamped out;
  out.header.stamp = now();
  out.header.frame_id = map_frame_;
  out.pose.position.x = gx;
  out.pose.position.y = gy;
  out.pose.position.z = 0.0;
  out.pose.orientation.w = 1.0; // yaw=0, Nav2 can refine orientation

  wp_pub_->publish(out);
  last_goal_ = out;

  RCLCPP_INFO(get_logger(),
              "Published frontier goal: (%.2f, %.2f) in %s",
              gx, gy, map_frame_.c_str());
}

bool RoverController::inBounds(int ix, int iy) const
{
  if (!latest_map_) return false;
  const auto &info = latest_map_->info;
  return (ix >= 0 && iy >= 0 &&
          ix < static_cast<int>(info.width) &&
          iy < static_cast<int>(info.height));
}

int RoverController::idx(int ix, int iy) const
{
  const auto &info = latest_map_->info;
  return iy * static_cast<int>(info.width) + ix;
}

bool RoverController::isFrontierCell(int ix, int iy) const
{
  // Frontier = free cell adjacent to at least one unknown
  if (!inBounds(ix, iy)) return false;
  int i = idx(ix, iy);
  const auto &data = latest_map_->data;

  // Free?
  if (data[i] != 0) return false; // 0=free, 100=occ, -1=unknown

  // 8-neighborhood: look for unknown
  static const int nb[8][2] = {
      {-1, -1}, {0, -1}, {1, -1},
      {-1,  0},          {1,  0},
      {-1,  1}, {0,  1}, {1,  1}
  };

  for (auto &d : nb) {
    int nx = ix + d[0];
    int ny = iy + d[1];
    if (!inBounds(nx, ny)) continue;
    int ni = idx(nx, ny);
    if (latest_map_->data[ni] == -1) {
      return true; // adjacent unknown
    }
  }
  return false;
}

bool RoverController::findBestFrontier(double &gx, double &gy)
{
  const auto &info = latest_map_->info;
  //const auto &data = latest_map_->data;

  const double res = info.resolution;
  const double ox  = info.origin.position.x;
  const double oy  = info.origin.position.y;

  // Collect frontier cells
  std::vector<std::pair<int,int>> frontiers;
  frontiers.reserve(4096);

  for (int iy = 1; iy < static_cast<int>(info.height) - 1; ++iy) {
    for (int ix = 1; ix < static_cast<int>(info.width) - 1; ++ix) {
      if (isFrontierCell(ix, iy)) {
        frontiers.emplace_back(ix, iy);
      }
    }
  }
  if (frontiers.empty())
    return false;

  // Very simple clustering: scan buckets by coarse tiles to count density
  // and pick the densest cell as goal seed.
  const int tile = 5; // cells
  int best_count = 0;
  int best_ix = frontiers.front().first;
  int best_iy = frontiers.front().second;

  for (auto [ix, iy] : frontiers) {
    int count = 0;
    for (auto [jx, jy] : frontiers) {
      if (std::abs(ix - jx) <= tile && std::abs(iy - jy) <= tile)
        ++count;
    }
    if (count > best_count) {
      best_count = count;
      best_ix = ix;
      best_iy = iy;
    }
  }

  if (best_count < min_cluster_cells_) {
    RCLCPP_DEBUG(get_logger(),
                 "Frontier density too low (cluster=%d < min=%d)",
                 best_count, min_cluster_cells_);
  }

  // Convert to world; nudge a bit into free space along +x
  const double wx = ox + (best_ix + 0.5) * res + goal_offset_m_;
  const double wy = oy + (best_iy + 0.5) * res;

  gx = wx;
  gy = wy;
  return true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoverController>());
  rclcpp::shutdown();
  return 0;
}