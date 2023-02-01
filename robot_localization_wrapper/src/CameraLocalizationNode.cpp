// Copyright 2022 BrOleg5

#include "robot_localization_wrapper/CameraLocalizationNode.hpp"

CameraLocalizationNode::CameraLocalizationNode(): Node("camera_loc") {
    declare_node_parameters();

    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform.header.frame_id = this->get_parameter("parent_frame_id")
                                     .get_parameter_value().get<std::string>();
    transform.child_frame_id = this->get_parameter("child_frame_id")
                                    .get_parameter_value().get<std::string>();

    std::string node_name = this->get_name();
    pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/" + node_name + "/pose",
        10
    );
    path_publisher = this->create_publisher<nav_msgs::msg::Path>(
        "/" + node_name + "/path",
        10
    );

    path.header.frame_id = this->get_parameter("parent_frame_id")
                                .get_parameter_value().get<std::string>();

    int dictionary_id = static_cast<int>(
        this->get_parameter("dict_id").get_parameter_value().get<int64_t>()
    );
    cv_system.setMarkerDictionary(dictionary_id);
    marker_id = static_cast<int>(
        this->get_parameter("marker_id").get_parameter_value().get<int64_t>()
    );

    video_capture_init();
    cv::Point2f pixelResolution;
    std::map<std::string, double> pixel_resolution;
    this->get_parameters("pixel_resolution", pixel_resolution);
    pixelResolution.x = static_cast<float>(pixel_resolution["x"]);
    pixelResolution.y = static_cast<float>(pixel_resolution["y"]);
    RCLCPP_INFO(this->get_logger(), "Pixel resolutions: x: %f, y = %f",
                pixelResolution.x, pixelResolution.y);
    // validate data
    if((pixelResolution.x < 0) || (pixelResolution.y < 0)) {
        RCLCPP_ERROR(this->get_logger(), "Get invalid camera parameters.");
    }
    transfer.pixelResolution = pixelResolution;

    using std::chrono::milliseconds;
    milliseconds sample_period = milliseconds(
        this->get_parameter("sample_period").get_parameter_value().get<int64_t>()
    );
    timer = this->create_wall_timer(
        sample_period,
        std::bind(&CameraLocalizationNode::timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Sample period: %d ms", sample_period.count());
    RCLCPP_INFO(this->get_logger(), "Dictionary: %s", getDictionaryName(dictionary_id));
    RCLCPP_INFO(this->get_logger(), "Marker id: %d", marker_id);
}

CameraLocalizationNode::~CameraLocalizationNode() {
    video_capture.release();
}

void CameraLocalizationNode::declare_node_parameters() {
    rcl_interfaces::msg::ParameterDescriptor sample_period_desc{};
    sample_period_desc.description = "Sample period of localization. "
                                     "Sample period in ms.";
    this->declare_parameter<int64_t>("sample_period", 40, sample_period_desc);

    rcl_interfaces::msg::ParameterDescriptor dict_id_desc{};
    dict_id_desc.description = "Marker dictionary id: DICT_4X4_50=0, DICT_4X4_100=1, "
                               "DICT_4X4_250=2, DICT_4X4_1000=3, DICT_5X5_50=4, "
                               "DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
                               "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, "
                               "DICT_7X7_50=12, DICT_7X7_100=13, DICT_7X7_250=14, "
                               "DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16, "
                               "DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, "
                               "DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20";
    this->declare_parameter<int64_t>("dict_id", 0, dict_id_desc);

    rcl_interfaces::msg::ParameterDescriptor marker_id_desc{};
    marker_id_desc.description = "Marker id.";
    this->declare_parameter<int64_t>("marker_id", 0, marker_id_desc);

    rcl_interfaces::msg::ParameterDescriptor cam_id_desc{};
    cam_id_desc.description = "Camera id.";
    this->declare_parameter<int64_t>("cam_id", 0, cam_id_desc);

    this->declare_parameters<int64_t>(
        "camera_resolution",
        std::map<std::string, int64_t>({{"width", 1920}, {"height", 1080}})
    );

    rcl_interfaces::msg::ParameterDescriptor cam_focus_desc{};
    cam_focus_desc.description = "Camera focus.";
    cam_focus_desc.additional_constraints = "From 0 to 255 with step 5";
    this->declare_parameter<int64_t>("cam_focus", 0, cam_focus_desc);

    rcl_interfaces::msg::ParameterDescriptor cam_exposure_desc{};
    cam_exposure_desc.description = "Camera exposure.";
    this->declare_parameter<double>("cam_exposure", 0.0, cam_exposure_desc);

    this->declare_parameter<std::string>("parent_frame_id", "map");
    this->declare_parameter<std::string>("child_frame_id", "robot");

    this->declare_parameters<double>(
        "pixel_resolution",
        std::map<std::string, double>({{"x", 1.0}, {"y", 1.0}})
    );
}

void CameraLocalizationNode::video_capture_init() {
    int cam_id = static_cast<int>(
        this->get_parameter("cam_id").get_parameter_value().get<int64_t>()
    );
    RCLCPP_INFO(this->get_logger(), "Camera id: %d", cam_id);
    #ifdef WIN32
        video_capture.open(cam_id, cv::CAP_DSHOW);
    #else
        video_capture.open(cam_id);
    #endif
    std::map<std::string, int64_t> frame_resolution;
    this->get_parameters("camera_resolution", frame_resolution);
    int frame_width = static_cast<int>(frame_resolution["width"]);
    int frame_height = static_cast<int>(frame_resolution["height"]);
    double cam_focus = static_cast<double>(
        this->get_parameter("cam_focus").get_parameter_value().get<int64_t>()
    );
    cv_system.setFrameSize(frame_width, frame_height);
    video_capture.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(frame_width));
    video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(frame_height));
    // Focus: min = 0, max = 255, step = 5
    video_capture.set(cv::CAP_PROP_FOCUS, cam_focus);
    video_capture.set(cv::CAP_PROP_BUFFERSIZE, 1);
    RCLCPP_INFO(this->get_logger(), "Frame size: %dx%d", frame_width, frame_height);
    RCLCPP_INFO(this->get_logger(), "Camera focus: %f", cam_focus);
    // link: https://stackoverflow.com/a/70074022
    #ifdef WIN32
        video_capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    #endif
    if(this->has_parameter("cam_exposure")) {
        video_capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);
        double cam_exposure = this->get_parameter("cam_exposure")
                                   .get_parameter_value().get<double>();
        video_capture.set(cv::CAP_PROP_EXPOSURE, cam_exposure);
        RCLCPP_INFO(this->get_logger(), "Camera exposure: %f", cam_exposure);
    } else {
        video_capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
        RCLCPP_INFO(this->get_logger(), "Camera auto-exposure");
    }
    // Checking for the camera to be connected
    if (video_capture.isOpened()) {
        RCLCPP_INFO(this->get_logger(), "Camera connected.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Camera not connected.");
    }
}

void CameraLocalizationNode::timer_callback() {
    video_capture >> frame;
    if(frame.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Camera frame empty.");
        return;
    }
    if(cv_system.detectMarkers(frame)) {
        if(cv_system.estimatePosition(&transfer, marker_id)) {
            robot_pose.header.stamp = this->get_clock()->now();
            robot_pose.pose.position.x = static_cast<double>(
                transfer.currGlobalCartesian.y * transfer.pixelResolution.y) / 1000.0;
            robot_pose.pose.position.y = static_cast<double>(
                transfer.currGlobalCartesian.x * transfer.pixelResolution.x) / 1000.0;
            robot_pose.pose.position.z = 0.0;

            q.setRPY(0.0f, 0.0f, td::deg2rad(transfer.currAngle - 180.f));
            robot_pose.pose.orientation.x = q.x();
            robot_pose.pose.orientation.y = q.y();
            robot_pose.pose.orientation.z = q.z();
            robot_pose.pose.orientation.w = q.w();
            pose_publisher->publish(robot_pose);

            transform.header.stamp = robot_pose.header.stamp;

            transform.transform.translation.x = robot_pose.pose.position.x;
            transform.transform.translation.y = robot_pose.pose.position.y;
            transform.transform.translation.z = robot_pose.pose.position.z;

            transform.transform.rotation.x = q.x();
            transform.transform.rotation.y = q.y();
            transform.transform.rotation.z = q.z();
            transform.transform.rotation.w = q.w();

            tf_broadcaster->sendTransform(transform);

            path.header.stamp = robot_pose.header.stamp;
            path.poses.push_back(robot_pose);
            path_publisher->publish(path);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Marker with ID %d is not found.", marker_id);
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Marker detection failed.");
    }
}
