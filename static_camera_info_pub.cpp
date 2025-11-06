#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <yaml-cpp/yaml.h>

class StaticCameraInfoPub : public rclcpp::Node {
public:
  StaticCameraInfoPub() : Node("static_camera_info_pub") {
    declare_parameter<std::string>("camera_info_yaml", "");
    auto path = get_parameter("camera_info_yaml").as_string();
    YAML::Node y = YAML::LoadFile(path);

    info_.width  = y["image_width"].as<int>();
    info_.height = y["image_height"].as<int>();
    info_.k = { y["camera_matrix"]["data"][0].as<double>(),
                y["camera_matrix"]["data"][1].as<double>(),
                y["camera_matrix"]["data"][2].as<double>(),
                y["camera_matrix"]["data"][3].as<double>(),
                y["camera_matrix"]["data"][4].as<double>(),
                y["camera_matrix"]["data"][5].as<double>(),
                y["camera_matrix"]["data"][6].as<double>(),
                y["camera_matrix"]["data"][7].as<double>(),
                y["camera_matrix"]["data"][8].as<double>() };
    info_.p = { y["projection_matrix"]["data"][0].as<double>(),
                y["projection_matrix"]["data"][1].as<double>(),
                y["projection_matrix"]["data"][2].as<double>(),
                y["projection_matrix"]["data"][3].as<double>(),
                y["projection_matrix"]["data"][4].as<double>(),
                y["projection_matrix"]["data"][5].as<double>(),
                y["projection_matrix"]["data"][6].as<double>(),
                y["projection_matrix"]["data"][7].as<double>(),
                y["projection_matrix"]["data"][8].as<double>(),
                y["projection_matrix"]["data"][9].as<double>(),
                y["projection_matrix"]["data"][10].as<double>(),
                y["projection_matrix"]["data"][11].as<double>() };
    info_.d.assign(5, 0.0);
    info_.distortion_model = "plumb_bob";

    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr img){
        info_.header = img->header;  // keep frame + stamp in sync
        pub_->publish(info_);
      });
    pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10);
  }
private:
  sensor_msgs::msg::CameraInfo info_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticCameraInfoPub>());
  rclcpp::shutdown();
  return 0;
}