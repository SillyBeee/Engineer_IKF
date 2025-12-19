#include "driver_gamepad.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <atomic>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("joy_node");
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub =
      node->create_publisher<std_msgs::msg::Float64MultiArray>("/joy_data", 1);

  drivers::GamePad gamepad;
  if (!gamepad.Init()) {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize SDL GamePad.");
    rclcpp::shutdown();
    return 1;
  }

  std::atomic_bool running{true};
  std::thread reader([&]() {
    rclcpp::WallRate rate(100.0);
    const auto lx =
        static_cast<int>(drivers::GamePad::GamePadInput::LEFT_STICK_X);
    const auto ly =
        static_cast<int>(drivers::GamePad::GamePadInput::LEFT_STICK_Y);
    const auto rx =
        static_cast<int>(drivers::GamePad::GamePadInput::RIGHT_STICK_X);
    const auto ry =
        static_cast<int>(drivers::GamePad::GamePadInput::RIGHT_STICK_Y);

    while (rclcpp::ok() && running.load()) {
      const auto left_x_raw = gamepad.GetInputState(lx);
      const auto left_y_raw = gamepad.GetInputState(ly);
      const auto right_x_raw = gamepad.GetInputState(rx);
      const auto right_y_raw = gamepad.GetInputState(ry);

      auto left_x = static_cast<double>(left_x_raw) / 32768.0;
      auto left_y = static_cast<double>(left_y_raw) / 32768.0;
      auto right_x = static_cast<double>(right_x_raw) / 32768.0;
      auto right_y = static_cast<double>(right_y_raw) / 32768.0;
      left_x = left_x < 0.5 ? 0: left_x;
      left_y = left_y < 0.5 ? 0: left_y;
      right_x = right_x < 0.5 ? 0: right_x;
      right_y = right_y < 0.5 ? 0: right_y;

      RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 100,
                           "Left stick: x=%f y=%f , right: x=%f y=%f", left_x, left_y, right_x, right_y);
      std_msgs::msg::Float64MultiArray msg;
      msg.data.push_back(left_x);
      msg.data.push_back(left_y);
      msg.data.push_back(right_x);
      msg.data.push_back(right_y);
      pub->publish(msg);
      rate.sleep();
    }
  });

  rclcpp::spin(node);

  running.store(false);
  if (reader.joinable()) {
    reader.join();
  }
  gamepad.Shutdown();
  rclcpp::shutdown();
  return 0;
}