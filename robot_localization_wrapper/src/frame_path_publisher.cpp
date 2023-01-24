#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_localization_wrapper/FramePathPublisher.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramePathPublisher>());
    rclcpp::shutdown();

    return 0;
}