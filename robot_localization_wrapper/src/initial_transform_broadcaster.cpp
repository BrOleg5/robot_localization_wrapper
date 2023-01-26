#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "robot_localization_wrapper/InitialTransformBroadcaster.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitialTransformBroadcasterNode>());
    rclcpp::shutdown();

    return 0;
}