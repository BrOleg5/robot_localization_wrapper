#include "robot_localization/FramePathPublisher.hpp"

FramePathPublisher::FramePathPublisher(): Node("frame_path_broadcaster") {
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pose_subscriber = this->create_subscription<geometry_msgs::msg::TwistStamped>("/robot/pose", 10, 
                        std::bind(&FramePathPublisher::handle_robot_pose, this, std::placeholders::_1));

    std::string node_name = this->get_name();
    path_publisher = this->create_publisher<nav_msgs::msg::Path>("/" + node_name + "/path", 10);
}

void FramePathPublisher::handle_robot_pose(const geometry_msgs::msg::TwistStamped& msg) {
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "world";
    transform.child_frame_id = "robot";

    transform.transform.translation.x = msg.twist.linear.y;
    transform.transform.translation.y = msg.twist.linear.x;
    transform.transform.translation.z = msg.twist.linear.z;

    q.setRPY(0, 0, msg.twist.angular.z - 3.14159);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    tf_broadcaster->sendTransform(transform);

    robot_pose.header.stamp = msg.header.stamp;
    robot_pose.pose.position.x = transform.transform.translation.x;
    robot_pose.pose.position.y = transform.transform.translation.y;
    robot_pose.pose.position.z = transform.transform.translation.z;

    robot_pose.pose.orientation.x = transform.transform.rotation.x;
    robot_pose.pose.orientation.y = transform.transform.rotation.y;
    robot_pose.pose.orientation.z = transform.transform.rotation.z;
    robot_pose.pose.orientation.w = transform.transform.rotation.w;

    path.header.stamp = msg.header.stamp;
    path.header.frame_id = "world";
    path.poses.push_back(robot_pose);

    path_publisher->publish(path);
}