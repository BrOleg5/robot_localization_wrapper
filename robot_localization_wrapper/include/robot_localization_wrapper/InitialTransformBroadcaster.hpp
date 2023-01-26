#ifndef INITIALTRANSFORMBROADCASTER_HPP
#   define INITIALTRANSFORMBROADCASTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

class InitialTransformBroadcasterNode : public rclcpp::Node {
    public:
        InitialTransformBroadcasterNode();
};

#endif