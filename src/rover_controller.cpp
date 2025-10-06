#include <functional>
#include "rclcpp/rclcpp.hpp"    // <- add (for logging types)
#include "rover_controller.h"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

void RoverController::run() {
  using namespace std::chrono_literals;
  if (goals_.empty()) { status_ = Status::FAILED; return; }

  if (!action_client_->wait_for_action_server(50ms)) {
    // Nav2 not up yet
    return;
  }

  if (status_ == Status::IDLE) {
    send_current_goal_();
    status_ = Status::RUNNING;
  }
}

void RoverController::send_current_goal_() {
  if (current_idx_ >= goals_.size()) {
    status_ = Status::SUCCEEDED;
    RCLCPP_INFO(node_->get_logger(), "All waypoints complete.");
    return;
  }

  NavigateToPose::Goal goal_msg;
  goal_msg.pose = goals_[current_idx_];

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;

  // Humble expects a *GoalHandle* (not a future) here
  options.goal_response_callback =
    [this](GoalHandleNav::SharedPtr goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "Goal rejected by server.");
        status_ = Status::FAILED;
      } else {
        RCLCPP_INFO(node_->get_logger(), "Goal accepted.");
      }
    };

  options.feedback_callback =
    [this](GoalHandleNav::SharedPtr,
           const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
      if (feedback) {
        RCLCPP_DEBUG(node_->get_logger(), "Distance remaining: %.2f", feedback->distance_remaining);
      }
    };

  options.result_callback =
    [this](const GoalHandleNav::WrappedResult & result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(node_->get_logger(), "Reached waypoint %zu", current_idx_);
          ++current_idx_;
          send_current_goal_(); // chain next goal
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(node_->get_logger(), "Goal aborted at idx %zu", current_idx_);
          status_ = Status::FAILED;
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(node_->get_logger(), "Goal canceled at idx %zu", current_idx_);
          status_ = Status::FAILED;
          break;
        default:
          RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
          status_ = Status::FAILED;
          break;
      }
    };

  RCLCPP_INFO(node_->get_logger(), "Sending waypoint %zu: (%.2f, %.2f)",
              current_idx_,
              goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);

  action_client_->async_send_goal(goal_msg, options);
}
