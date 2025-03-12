#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "common_messages/msg/camera_control_custom.hpp"
#include "common_messages/msg/lamp_control_custom.hpp"
#include "common_messages/msg/pan_and_tilt_control_custom.hpp"

using namespace std::chrono_literals;

class ControlPublisher : public rclcpp::Node
{
public:
  ControlPublisher()
  : Node("control_publisher")
  {
    // Declare parameters for Camera Control
    this->declare_parameter("camera_id", 9);
    this->declare_parameter("camera_power", true);
    this->declare_parameter("camera_light", true);
    this->declare_parameter("camera_focus", 22.0);
    this->declare_parameter("camera_zoom", 25.0);

    // Declare parameters for Lamp Control
    this->declare_parameter("lamp_id", 9);
    this->declare_parameter("lamp_intensity", 80.0);
    this->declare_parameter("lamp_power", true);

    // Declare parameters for Pan and Tilt Control
    this->declare_parameter("panandtilt_id", 9);
    this->declare_parameter("panandtilt_x", 80.0);
    this->declare_parameter("panandtilt_y", 20.0);
    this->declare_parameter("panandtilt_z", 23.0);

    // Publishers for each topic
    camera_publisher_ =
      this->create_publisher<common_messages::msg::CameraControlCustom>("rt/CameraSetterTopic", 10);
    lamp_publisher_ =
      this->create_publisher<common_messages::msg::LampControlCustom>("rt/LampSetterTopic",
      10);
    panandtilt_publisher_ =
      this->create_publisher<common_messages::msg::PanAndTiltControlCustom>(
      "rt/PanAndTiltSetterTopic",
      10);

    // Timers for periodic publishing (1 Hz frequency)
    timer_camera_ = this->create_wall_timer(
      1s, std::bind(&ControlPublisher::publish_camera_data, this));
    timer_lamp_ = this->create_wall_timer(
      1s, std::bind(&ControlPublisher::publish_lamp_data, this));
    timer_panandtilt_ = this->create_wall_timer(
      1s, std::bind(&ControlPublisher::publish_panandtilt_data, this));
  }

private:
  void publish_camera_data()
  {
    auto message = common_messages::msg::CameraControlCustom();
    message.cameraid = this->get_parameter("camera_id").as_int();
    message.power = this->get_parameter("camera_power").as_bool();
    message.light = this->get_parameter("camera_light").as_bool();
    message.focus = this->get_parameter("camera_focus").as_double();
    message.zoom = this->get_parameter("camera_zoom").as_double();

    camera_publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(),
      "Published Camera Data: [ID: %d, Power: %s, Light: %s, Focus: %.2f, Zoom: %.2f]",
                message.cameraid, message.power ? "true" : "false",
      message.light ? "true" : "false",
                message.focus, message.zoom);
  }

  void publish_lamp_data()
  {
    auto message = common_messages::msg::LampControlCustom();
    message.lampid = this->get_parameter("lamp_id").as_int();
    message.intensity = this->get_parameter("lamp_intensity").as_double();
    message.power = this->get_parameter("lamp_power").as_bool();

    lamp_publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published Lamp Data: [ID: %d, Intensity: %.2f, Power: %s]",
                message.lampid, message.intensity, message.power ? "true" : "false");
  }

  void publish_panandtilt_data()
  {
    auto message = common_messages::msg::PanAndTiltControlCustom();
    message.panandtiltid = this->get_parameter("panandtilt_id").as_int();
    message.x = this->get_parameter("panandtilt_x").as_double();
    message.y = this->get_parameter("panandtilt_y").as_double();
    message.z = this->get_parameter("panandtilt_z").as_double();

    panandtilt_publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(),
      "Published Pan and Tilt Data: [ID: %d, X: %.2f, Y: %.2f, Z: %.2f]",
                message.panandtiltid, message.x, message.y, message.z);
  }

  rclcpp::TimerBase::SharedPtr timer_camera_;
  rclcpp::TimerBase::SharedPtr timer_lamp_;
  rclcpp::TimerBase::SharedPtr timer_panandtilt_;
  rclcpp::Publisher<common_messages::msg::CameraControlCustom>::SharedPtr camera_publisher_;
  rclcpp::Publisher<common_messages::msg::LampControlCustom>::SharedPtr lamp_publisher_;
  rclcpp::Publisher<common_messages::msg::PanAndTiltControlCustom>::SharedPtr panandtilt_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlPublisher>());
  rclcpp::shutdown();
  return 0;
}
