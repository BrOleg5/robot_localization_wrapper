#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_localization/CameraLocalizationNode.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraLocalizationNode>());
    rclcpp::shutdown();

    return 0;
}