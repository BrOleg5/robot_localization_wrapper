#ifndef CAMERALOCALIZATIONNODE_HPP
#   define CAMERALOCALIZATIONNODE_HPP

#include <string>
#include <memory>
#include <chrono>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "robot_localization/ArucoLocalization.hpp"
#include "robot_localization/ReadSaveCameraParameters.hpp"

#include "opencv2/videoio.hpp"

using namespace std::chrono;

class CameraLocalizationNode : public rclcpp::Node {
    public:
        CameraLocalizationNode();
        ~CameraLocalizationNode();

    private:
        void declare_node_parameters();
        void timer_callback();
        void video_capture_init();

        cv::VideoCapture video_capture;
        int marker_id;
        ArucoLocalization cv_system;
        td::TransferData transfer;
        cv::Mat frame;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr position_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr change_position_publisher_;
};

#endif