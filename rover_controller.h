#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class RoverController : public rclcpp::Node {
public:
  RoverController();

private:
  // --- Parameters ---
  std::string map_frame_{"map"};
  int         min_cluster_cells_{40};
  double      goal_offset_m_{0.6};
  double      replan_period_sec_{3.0};
  double      min_goal_sep_m_{0.8};

  // --- ROS I/O ---
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr wp_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr replan_timer_;

  // --- State ---
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  std::optional<geometry_msgs::msg::PoseStamped> last_goal_;

  // --- Callbacks ---
  void mapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void timerTick();

  // --- Helpers (grid / frontier) ---
  bool inBounds(int ix, int iy) const;
  int  idx(int ix, int iy) const;
  bool isFrontierCell(int ix, int iy) const;
  bool findBestFrontier(double &gx, double &gy);

  // --- Utility ---
  static double sqr(double v) { return v * v; }
};