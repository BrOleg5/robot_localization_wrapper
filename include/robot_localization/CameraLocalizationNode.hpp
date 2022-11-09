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
        CameraLocalizationNode(): Node("camera_loc") {
            declare_node_parameters();

            std::string node_name = this->get_name();
            position_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(node_name + "/pos", 10);
            change_position_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(node_name + "/change_pos", 10);

            int dictionary_id = static_cast<int>(this->get_parameter("dict_id").get_parameter_value().get<long long>());

            video_capture_init();

            marker_id = static_cast<int>(this->get_parameter("marker_id").get_parameter_value().get<long long>());
            cv::Point2f pixelResolution;
            std::string cam_param_file = this->get_parameter("cam_param_file").get_parameter_value().get<std::string>();
            RCLCPP_INFO(this->get_logger(), "File camera parameters: %s", cam_param_file);
            if (!readCameraParameters(cam_param_file, pixelResolution)) {
                RCLCPP_ERROR(this->get_logger(), "Read camera parameters error.");
            }
            RCLCPP_INFO(this->get_logger(), "Pixel resolutions: x: %f, y = %f", pixelResolution.x, pixelResolution.y);
            // validate data
            if((pixelResolution.x <= 0) || (pixelResolution.y <= 0)) {
                RCLCPP_ERROR(this->get_logger(), "Get invalid camera parameters.");
            }

            transfer.pixelResolution = pixelResolution;
            cv_system.setMarkerDictionary(dictionary_id);

            milliseconds sample_period = milliseconds(this->get_parameter("sample_period").get_parameter_value().get<long long>());
            timer_ = this->create_wall_timer(sample_period, std::bind(&CameraLocalizationNode::timer_callback, this));

            RCLCPP_INFO(this->get_logger(), "Sample period: %d ms", sample_period.count());
            RCLCPP_INFO(this->get_logger(), "Dictionary: %s", getDictionaryName(dictionary_id));
            RCLCPP_INFO(this->get_logger(), "Marker id: %d", marker_id);
        }

        ~CameraLocalizationNode() {
            video_capture.release();
        }

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