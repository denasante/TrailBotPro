// #include "mission_interface.h"
// #include <chrono>

// using namespace std::chrono_literals;

// MissionInterface::MissionInterface()
// : rclcpp::Node("mission_interface"),
//   have_odom_(false),
//   x_(0.0),
//   phase_(Phase::WAIT_ODOM)
// {
//   // --- Parameters ---
//   odom_topic_        = declare_parameter<std::string>("odom_topic", "/parrot/odometry");
//   cmd_vel_topic_     = declare_parameter<std::string>("cmd_vel_topic", "/parrot/cmd_vel");
//   fwd_speed_         = declare_parameter<double>("fwd_speed", 0.5);
//   rev_speed_         = declare_parameter<double>("rev_speed", -0.5);
//   target_forward_x_  = declare_parameter<double>("target_forward_x", 2.0);
//   target_back_x_     = declare_parameter<double>("target_back_x", 0.0);
//   pause_secs_        = declare_parameter<double>("pause_secs", 2.0);

//   // --- Interfaces ---
//   auto qos = rclcpp::QoS(10).reliable();

//   pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, qos);

//   sub_ = create_subscription<nav_msgs::msg::Odometry>(
//     odom_topic_, qos,
//     [this](nav_msgs::msg::Odometry::SharedPtr msg)
//     {
//       x_ = msg->pose.pose.position.x;
//       have_odom_ = true;
//     });

//   timer_ = create_wall_timer(50ms, std::bind(&MissionInterface::tick, this));

//   RCLCPP_INFO(get_logger(),
//               "MissionInterface up. odom='%s' cmd_vel='%s' → forward to x>=%.2f, pause %.1fs, back to x<=%.2f",
//               odom_topic_.c_str(), cmd_vel_topic_.c_str(),
//               target_forward_x_, pause_secs_, target_back_x_);
// }

// void MissionInterface::publish_stop() {
//   geometry_msgs::msg::Twist t;
//   pub_->publish(t);
// }

// void MissionInterface::tick() {
//   geometry_msgs::msg::Twist cmd;

//   switch (phase_) {
//     case Phase::WAIT_ODOM:
//       if (!have_odom_) {
//         RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
//                              "Waiting for odometry on %s ...", odom_topic_.c_str());
//         publish_stop();
//         return;
//       }
//       RCLCPP_INFO(get_logger(), "Odom ready (x=%.3f). Starting FORWARD.", x_);
//       phase_ = Phase::FORWARD;
//       return;

//     case Phase::FORWARD:
//       if (x_ >= target_forward_x_) {
//         publish_stop();
//         pause_start_ = now();
//         phase_ = Phase::PAUSE;
//         RCLCPP_INFO(get_logger(),
//                     "Reached x >= %.2f (x=%.3f). Pausing %.1fs.",
//                     target_forward_x_, x_, pause_secs_);
//         return;
//       }
//       cmd.linear.x = fwd_speed_;
//       pub_->publish(cmd);
//       return;

//     case Phase::PAUSE:
//       publish_stop();
//       if ((now() - pause_start_).seconds() >= pause_secs_) {
//         RCLCPP_INFO(get_logger(), "Pause done. Starting REVERSE.");
//         phase_ = Phase::REVERSE;
//       }
//       return;

//     case Phase::REVERSE:
//       if (x_ <= target_back_x_) {
//         publish_stop();
//         RCLCPP_INFO(get_logger(), "Reached x <= %.2f (x=%.3f). Mission DONE.", target_back_x_, x_);
//         phase_ = Phase::DONE;
//         return;
//       }
//       cmd.linear.x = rev_speed_;
//       pub_->publish(cmd);
//       return;

//     case Phase::DONE:
//       publish_stop();
//       return;
//   }
// }

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MissionInterface>());
//   rclcpp::shutdown();
//   return 0;
// }



// ---------------------------------------------------
// ---------------------------------------------------

#include "mission_interface.h"
#include <chrono>

using namespace std::chrono_literals;

MissionInterface::MissionInterface()
: rclcpp::Node("mission_interface"),
  phase_(Phase::WAIT_ODOM),
  have_odom_(false)
{
  // Parameters
  odom_topic_   = declare_parameter<std::string>("odom_topic", "/parrot/odometry");
  target_topic_ = declare_parameter<std::string>("target_topic", "/mission/target");
  status_topic_ = declare_parameter<std::string>("status_topic", "/mission/status");

  pause_secs_   = declare_parameter<double>("pause_secs", 2.0);

  // Mission waypoints
  waypoints_ = {
    {2.0, 0.0, 0.0},  // forward
    {0.0, -3.0, 0.0},
    {-2.0, 0.0, 0.0},
    {0.0, 0.0, 0.0}   // back to start
  };
  current_idx_ = 0;

  auto qos = rclcpp::QoS(10).reliable();

  // Publishers/subscribers
  pub_target_ = create_publisher<geometry_msgs::msg::PoseStamped>(target_topic_, qos);
  sub_status_ = create_subscription<std_msgs::msg::String>(
      status_topic_, qos,
      std::bind(&MissionInterface::status_cb, this, std::placeholders::_1));
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, qos,
      [this](nav_msgs::msg::Odometry::SharedPtr msg)
      {
        x_ = msg->pose.pose.position.x;
        have_odom_ = true;
      });

  timer_ = create_wall_timer(200ms, std::bind(&MissionInterface::tick, this));

  RCLCPP_INFO(get_logger(), "MissionInterface up. Waiting for odom...");
}

// --- Status callback ---
void MissionInterface::status_cb(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "ARRIVED")
  {
    RCLCPP_INFO(get_logger(), "✅ Drone reported ARRIVED");
    phase_ = Phase::PAUSE;
    pause_start_ = now();
  }
}

// --- Publish next waypoint ---
void MissionInterface::send_target(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped t;
  t.header.frame_id = "map";
  t.pose.position.x = x;
  t.pose.position.y = y;
  t.pose.orientation.z = std::sin(yaw/2);
  t.pose.orientation.w = std::cos(yaw/2);
  pub_target_->publish(t);
  RCLCPP_INFO(get_logger(), "🚀 Sent new target (x=%.2f, y=%.2f, yaw=%.2f)", x, y, yaw);
}

// --- Main tick ---
void MissionInterface::tick()
{
  switch (phase_)
  {
    case Phase::WAIT_ODOM:
      if (!have_odom_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for odometry...");
        return;
      }
      RCLCPP_INFO(get_logger(), "Odom ready. Starting mission.");
      send_target(waypoints_[current_idx_].x, waypoints_[current_idx_].y, waypoints_[current_idx_].yaw);
      phase_ = Phase::WAIT_ARRIVAL;
      break;

    case Phase::WAIT_ARRIVAL:
      // just wait for "ARRIVED" callback
      break;

    case Phase::PAUSE:
      if ((now() - pause_start_).seconds() >= pause_secs_) {
        current_idx_++;
        if (current_idx_ >= waypoints_.size()) {
          RCLCPP_INFO(get_logger(), "🛑 Mission complete!");
          phase_ = Phase::DONE;
          return;
        }
        send_target(waypoints_[current_idx_].x, waypoints_[current_idx_].y, waypoints_[current_idx_].yaw);
        phase_ = Phase::WAIT_ARRIVAL;
      }
      break;

    case Phase::DONE:
      // nothing left
      break;
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionInterface>());
  rclcpp::shutdown();
  return 0;
}


