// Copyright 2022 BrOleg5

#include "robot_localization_wrapper/InitialTransformBroadcaster.hpp"

InitialTransformBroadcasterNode::InitialTransformBroadcasterNode():
    Node("initial_transform_broadcaster")
{
    this->declare_parameter<std::string>("parent_frame_id", "map");
    this->declare_parameter<std::string>("child_frame_id", "init_pose");

    using geometry_msgs::msg::PoseStamped;
    rclcpp::Subscription<PoseStamped>::SharedPtr pose_subscription =
        this->create_subscription<PoseStamped>(
            "/pose",
            10,
            [](const PoseStamped::ConstSharedPtr msg) {}
        );

    tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    PoseStamped pose_stamped;
    using namespace std::chrono_literals;
    // source: https://github.com/ros2/rclcpp/blob/8e6a6fb32d8d6a818b483660e326f2c5313b64ae/rclcpp/include/rclcpp/wait_for_message.hpp#L78-L94
    if(rclcpp::wait_for_message(
        pose_stamped,
        pose_subscription,
        this->get_node_options().context(),
        120s)
    ) {
        RCLCPP_INFO(this->get_logger(), "Get pose message.");
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = this->get_parameter("parent_frame_id")
                                         .get_parameter_value().get<std::string>();
        transform.child_frame_id = this->get_parameter("child_frame_id")
                                        .get_parameter_value().get<std::string>();

        transform.transform.translation.x = pose_stamped.pose.position.x;
        transform.transform.translation.y = pose_stamped.pose.position.y;
        transform.transform.translation.z = pose_stamped.pose.position.z;

        transform.transform.rotation.x = pose_stamped.pose.orientation.x;
        transform.transform.rotation.y = pose_stamped.pose.orientation.y;
        transform.transform.rotation.z = pose_stamped.pose.orientation.z;
        transform.transform.rotation.w = pose_stamped.pose.orientation.w;

        tf_static_broadcaster->sendTransform(transform);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Timeout waiting of pose message.");
    }
}
