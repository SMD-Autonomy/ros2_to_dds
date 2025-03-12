#ifndef CONTROL_PUBLISHER_HPP
#define CONTROL_PUBLISHER_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "common_messages/msg/camera_control_custom.hpp"
#include "common_messages/msg/lamp_control_custom.hpp"
#include "common_messages/msg/pan_and_tilt_control_custom.hpp"

class ControlPublisher : public rclcpp::Node
{
public:
  ControlPublisher();

private:
  void publish_camera_data();
  void publish_lamp_data();
  void publish_panandtilt_data();

  rclcpp::TimerBase::SharedPtr timer_camera_;
  rclcpp::TimerBase::SharedPtr timer_lamp_;
  rclcpp::TimerBase::SharedPtr timer_panandtilt_;
  rclcpp::Publisher<common_messages::msg::CameraControlCustom>::SharedPtr camera_publisher_;
  rclcpp::Publisher<common_messages::msg::LampControlCustom>::SharedPtr lamp_publisher_;
  rclcpp::Publisher<common_messages::msg::PanAndTiltControlCustom>::SharedPtr panandtilt_publisher_;
};

#endif // CONTROL_PUBLISHER_HPP
