#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <string>
#include <vector>

class MissionInterface : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  MissionInterface();

private:
  // --- callbacks / flow ---
  void waypointCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void sendGoal(const geometry_msgs::msg::PoseStamped &pose);

  // --- CSV mission helpers ---
  bool loadCsv(const std::string &path,
               std::vector<geometry_msgs::msg::PoseStamped> &out);
  void startCsvMissionIfAny();
  void sendNextFromList();

  // --- small helpers (declared only; defined in .cpp) ---
  static bool isZeroQuat(const geometry_msgs::msg::Quaternion &q);
  static geometry_msgs::msg::Quaternion yawToQuat(double yaw_rad);

  // --- params ---
  std::string frame_id_   = "map";
  double      timeout_sec_ = 120.0;
  std::string path_csv_    = "";     // empty = no CSV mission
  bool        loop_path_   = false;  // loop waypoints when true

  // --- ROS entities ---
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::TimerBase::SharedPtr connect_timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr wp_sub_;

  // --- state ---
  bool goal_active_ = false;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  std::size_t next_idx_ = 0;
};