#include "robot_localization/CameraLocalizationNode.hpp"

void CameraLocalizationNode::video_capture_init() {
    int cam_id = static_cast<int>(this->get_parameter("cam_id").get_parameter_value().get<long long>());
    RCLCPP_INFO(this->get_logger(), "Camera id: %d", cam_id);
    #ifdef WIN32
        video_capture.open(cam_id, cv::CAP_DSHOW);
    #else
        video_capture.open(cam_id);
    #endif
    int frame_width = static_cast<int>(this->get_parameter("cam_width").get_parameter_value().get<long long>());
    int frame_height = static_cast<int>(this->get_parameter("cam_height").get_parameter_value().get<long long>());
    int cam_focus = static_cast<int>(this->get_parameter("cam_focus").get_parameter_value().get<long long>());
    cv_system.setFrameSize(frame_width, frame_height);
    video_capture.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(frame_width));
    video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(frame_height));
    video_capture.set(cv::CAP_PROP_FOCUS, static_cast<double>(cam_focus)); // min: 0, max: 255, increment:5
    video_capture.set(cv::CAP_PROP_BUFFERSIZE, 1);
    RCLCPP_INFO(this->get_logger(), "Frame size: %dx%d", frame_width, frame_height);
    RCLCPP_INFO(this->get_logger(), "Camera focus: %d", cam_focus);
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
        RCLCPP_INFO(this->get_logger(), "Camera auto-exposure");
    }
    //Checking for the camera to be connected 
    if (video_capture.isOpened()) {
        RCLCPP_INFO(this->get_logger(), "Camera connected.");
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Camera not connected.");
    }
}

void CameraLocalizationNode::timer_callback() {
    video_capture >> frame;
    int status = cv_system.detectMarkers(frame);
    switch (status) {
    case ArucoLocalization::Status::OK:
        if(cv_system.estimatePosition(&transfer, marker_id)) {
            geometry_msgs::msg::Twist msg;
            msg.linear.x = static_cast<double>(transfer.currGlobalCartesian.x);
            msg.linear.y = static_cast<double>(transfer.currGlobalCartesian.y);
            msg.linear.z = 0.0;
            msg.angular.x = 0.0;
            msg.angular.y = 0.0;
            msg.angular.z = static_cast<double>(transfer.currAngle);
            position_publisher_->publish(msg);
            msg.linear.x = static_cast<double>(transfer.deltaEigenCartesian.x);
            msg.linear.y = static_cast<double>(transfer.deltaEigenCartesian.y);
            msg.angular.z = static_cast<double>(transfer.deltaAngle);
            change_position_publisher_->publish(msg);
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Marker estimate position failed.");
        }
        break;
    case ArucoLocalization::Status::MARKER_NOT_DETECTED:
        RCLCPP_ERROR(this->get_logger(), "Marker detection failed.");

    default:
        break;
    }
}