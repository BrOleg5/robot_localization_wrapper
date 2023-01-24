#ifndef FRAMEPATHPUBLISHER_HPP
#   define FRAMEPATHPUBLISHER_HPP

#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/path.hpp"

#include "aruco_robot_localization/arucolocalization.hpp"

class FramePathPublisher : public rclcpp::Node {
    public:
        FramePathPublisher();

    private:
        void handle_robot_pose(const geometry_msgs::msg::TwistStamped& msg);

        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr pose_subscriber;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

        geometry_msgs::msg::TransformStamped transform;
        tf2::Quaternion q;

        geometry_msgs::msg::PoseStamped robot_pose;
        nav_msgs::msg::Path path;
};

#endif