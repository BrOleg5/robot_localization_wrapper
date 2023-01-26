#ifndef CAMERALOCALIZATIONNODE_HPP
#   define CAMERALOCALIZATIONNODE_HPP

#include <string>
#include <memory>
#include <chrono>
#include <cstdlib>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "opencv2/videoio.hpp"

#include "aruco_robot_localization/arucolocalization.hpp"

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

        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

        geometry_msgs::msg::TransformStamped transform;
        tf2::Quaternion q;

        nav_msgs::msg::Path path;
};

#endif