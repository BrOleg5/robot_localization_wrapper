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

#include "opencv2/highgui.hpp"

using namespace std::chrono;

class CameraLocalizationNode : public rclcpp::Node {
    public:
        CameraLocalizationNode(): Node("camera_loc") {
            this->declare_parameter<long long>("sample_time", 40);
            this->declare_parameter<long long>("dict_num", 0);
            this->declare_parameter<long long>("cam_id", 1);
            this->declare_parameter<long long>("cam_width", 1920);
            this->declare_parameter<long long>("cam_height", 1080);
            this->declare_parameter<double>("cam_focus", 0);
            this->declare_parameter<double>("cam_exposure", -7.5);
            this->declare_parameter<long long>("marker_id", 0);
            this->declare_parameter<std::string>("cam_param", "camera_params.json");

            std::string node_name = this->get_name();
            position_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(node_name + "/pos", 10);
            change_position_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(node_name + "/change_pos", 10);

            int dictionary_id = static_cast<int>(this->get_parameter("dict_num").get_parameter_value().get<long long>());
            cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name = cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id);

            int cam_id = static_cast<int>(this->get_parameter("cam_id").get_parameter_value().get<long long>());
            #ifdef WIN32
                video_capture.open(cam_id, cv::CAP_DSHOW);
            #else
                video_capture.open(cam_id);
            #endif
            int frame_width = static_cast<int>(this->get_parameter("cam_width").get_parameter_value().get<long long>());
            int frame_height = static_cast<int>(this->get_parameter("cam_height").get_parameter_value().get<long long>());
            video_capture.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(frame_width));
            video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(frame_height));
            video_capture.set(cv::CAP_PROP_FOCUS, this->get_parameter("cam_focus").get_parameter_value().get<double>()); // min: 0, max: 255, increment:5
            video_capture.set(cv::CAP_PROP_BUFFERSIZE, 1);
            // link: https://stackoverflow.com/a/70074022
            #ifdef WIN32
                video_capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            #endif
            if(this->has_parameter("cam_exposure")) {
                video_capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);
                double cam_exposure = this->get_parameter("cam_exposure").get_parameter_value().get<double>();
                video_capture.set(cv::CAP_PROP_EXPOSURE, cam_exposure);
                RCLCPP_INFO(this->get_logger(), "Camera exposure: %f", cam_exposure);
            }
            else {
                video_capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
            }
            //Checking for the camera to be connected 
            if (video_capture.isOpened()) {
                RCLCPP_INFO(this->get_logger(), "Camera connected.");
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "Camera not connected.");
            }

            markerID = static_cast<int>(this->get_parameter("marker_id").get_parameter_value().get<long long>());
            RCLCPP_INFO(this->get_logger(), "Marker id: %d", markerID);
            cv::Point2f pixelResolution;
            std::string camParamFile = this->get_parameter("cam_param").get_parameter_value().get<std::string>();
            if (!readCameraParameters(camParamFile, pixelResolution)) {
                RCLCPP_ERROR(this->get_logger(), "Read camera parameters error.");
            }
            RCLCPP_INFO(this->get_logger(), "Camera parameters: pixel resolution x = %f, pixel resolution y = %f", 
                        pixelResolution.x, pixelResolution.y);
            // validate data
            if((pixelResolution.x <= 0) || (pixelResolution.y <= 0)) {
                RCLCPP_ERROR(this->get_logger(), "Get invalid camera parameters.");
            }

            transfer.pixelResolution = pixelResolution;
            cv_system.setMarkerDictionary(dictionary_name);
            cv_system.setFrameSize(frame_width, frame_height);

            milliseconds sample_time = milliseconds(this->get_parameter("sample_time").get_parameter_value().get<long long>());
            RCLCPP_INFO(this->get_logger(), "Sample_time: %d ms", sample_time.count());
            timer_ = this->create_wall_timer(sample_time, std::bind(&CameraLocalizationNode::timer_callback, this));
        }

        ~CameraLocalizationNode() {
            video_capture.release();
        }

    private:
        void timer_callback();

        cv::VideoCapture video_capture;
        int markerID;
        ArucoLocalization cv_system;
        td::TransferData transfer;
        cv::Mat frame;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr position_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr change_position_publisher_;
};

#endif