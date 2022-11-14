# Robot localization

ROS2 package to localize robot. This node based on [ArucoLocalization CMake package](https://github.com/BrOleg5/mobile-robot-localization).

## Topics and services of node

### camera_localization_node

Published topics:

- `/camera_loc/pos` &ndash; robot position in global coordinate frame
- `/camera_loc/change_pos` &ndash; robot speed in local coordinate frame (robot coordinate frame)

## Dependencies

- ROS2 Foxy or later
- OpenCV v4.2.0 or later with extra modules (aruco)
