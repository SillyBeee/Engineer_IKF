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
    rclcpp::WallRate rate(50.0);
    const auto lx =
        static_cast<int>(drivers::GamePad::GamePadInput::LEFT_STICK_X);
    const auto ly =
        static_cast<int>(drivers::GamePad::GamePadInput::LEFT_STICK_Y);

    while (rclcpp::ok() && running.load()) {
      const auto left_x_raw = gamepad.GetInputState(lx);
      const auto left_y_raw = gamepad.GetInputState(ly);
      auto left_x = static_cast<double>(left_x_raw) / 32768.0;
      auto left_y = static_cast<double>(left_y_raw) / 32768.0;
      RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 100,
                           "Left stick: x=%f y=%f", left_x, left_y);
      std_msgs::msg::Float64MultiArray msg;
      msg.data.push_back(left_x);
      msg.data.push_back(left_y);
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