#include "rover_controller.h"

RoverController::RoverController() : Node("rover_controller") {
  waypoint_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/waypoint", 10,
      std::bind(&RoverController::waypoint_callback, this, std::placeholders::_1));

  nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  RCLCPP_INFO(get_logger(), "RoverController started");
}

void RoverController::waypoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(get_logger(), "Nav2 action server not available");
    return;
  }

  NavigateToPose::Goal goal; 
  goal.pose = *msg;

  RCLCPP_INFO(get_logger(), "Sending goal: (%.2f, %.2f)",
              goal.pose.pose.position.x, goal.pose.pose.position.y);

  auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  opts.result_callback = [this](const GoalHandleNav::WrappedResult& r){
    if (r.code == rclcpp_action::ResultCode::SUCCEEDED)
      RCLCPP_INFO(get_logger(), "Rover reached goal");
    else
      RCLCPP_WARN(get_logger(), "Rover failed to reach goal (code %d)", static_cast<int>(r.code));
  };

  nav_client_->async_send_goal(goal, opts);
}

// ---- main ----
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoverController>());
  rclcpp::shutdown();
  return 0;
}