// Copyright 2022 BrOleg5

#ifndef ROBOT_LOCALIZATION_WRAPPER__INITIALTRANSFORMBROADCASTER_HPP_
#define ROBOT_LOCALIZATION_WRAPPER__INITIALTRANSFORMBROADCASTER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/static_transform_broadcaster.h"

class InitialTransformBroadcasterNode : public rclcpp::Node {
    public:
        InitialTransformBroadcasterNode();

    private:
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster;
};

#endif  // ROBOT_LOCALIZATION_WRAPPER__INITIALTRANSFORMBROADCASTER_HPP_
