#include "robot_localization/CameraLocalizationNode.hpp"

void CameraLocalizationNode::timer_callback() {
    video_capture >> frame;
    int status = cv_system.detectMarkers(frame);
    switch (status) {
    case ArucoLocalization::Status::OK:
        if(cv_system.estimatePosition(&transfer, markerID)) {
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