#include "robot_localization/CameraLocalizationNode.hpp"

CameraLocalizationNode::CameraLocalizationNode(): Node("camera_loc") {
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

CameraLocalizationNode::~CameraLocalizationNode() {
    video_capture.release();
}

void CameraLocalizationNode::declare_node_parameters() {
    rcl_interfaces::msg::ParameterDescriptor sample_period_desc{};
    sample_period_desc.description = "Sample period of localization. Sample period in ms.";
    this->declare_parameter<long long>("sample_period", 40, sample_period_desc);
    
    rcl_interfaces::msg::ParameterDescriptor dict_id_desc{};
    dict_id_desc.description = "Marker dictionary id: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
                                "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
                                "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
                                "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
                                "DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20";
    this->declare_parameter<long long>("dict_id", 0, dict_id_desc);
    rcl_interfaces::msg::ParameterDescriptor marker_id_desc{};
    marker_id_desc.description = "Marker id.";
    this->declare_parameter<long long>("marker_id", 0, marker_id_desc);

    rcl_interfaces::msg::ParameterDescriptor cam_id_desc{};
    cam_id_desc.description = "Camera id.";
    this->declare_parameter<long long>("cam_id", 0, cam_id_desc);
    rcl_interfaces::msg::ParameterDescriptor cam_width_desc{};
    cam_width_desc.description = "Width of camera frame in pixels.";
    this->declare_parameter<long long>("cam_width", 1920, cam_width_desc);
    rcl_interfaces::msg::ParameterDescriptor cam_height_desc{};
    cam_height_desc.description = "Height of camera frame in pixels.";
    this->declare_parameter<long long>("cam_height", 1080, cam_height_desc);
    rcl_interfaces::msg::ParameterDescriptor cam_focus_desc{};
    cam_focus_desc.description = "Camera focus.";
    cam_focus_desc.additional_constraints = "From 0 to 255 with step 5";
    this->declare_parameter<long long>("cam_focus", 0, cam_focus_desc);
    rcl_interfaces::msg::ParameterDescriptor cam_exposure_desc{};
    cam_exposure_desc.description = "Camera exposure.";
    this->declare_parameter<double>("cam_exposure", -7.5, cam_exposure_desc);
    rcl_interfaces::msg::ParameterDescriptor cam_param_file_desc{};
    cam_param_file_desc.description = "JSON file with camera parameters.";
    this->declare_parameter<std::string>("cam_param_file", "camera_params.json", cam_param_file_desc);
}

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