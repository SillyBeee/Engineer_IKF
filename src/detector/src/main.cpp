#include "detector/detector_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions();
    auto node = std::make_shared<DetectorNode>(options);
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions{}, 4);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}