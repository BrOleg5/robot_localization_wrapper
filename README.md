# Robot localization

ROS2 robot localization package. The Aruco marker, which is placed on the top side of the robot, is detected by the top-view camera.

This package based on [aruco_robot_localization](https://github.com/BrOleg5/aruco_robot_localization) CMake package.

## Nodes description

### CameraLocalizationNode

Published topics:

- `/camera_loc/pose` &ndash; robot position in global coordinate frame
- `/camera_loc/path` &ndash; robot path

Also, node broadcast transform from `map` to `robot` frame according to robot position.

Node parameters:

- `sample_period` &ndash; sample period of localization (ms). Default: 40.
- `dict_id` &ndash; marker dictionary ID (see table below). Default: 0.
- `marker_id` &ndash; marker ID. Default: 0.
- `cam_id` &ndash; camera ID. ID is assigned incrementally to each camera starting from 0. Default: 0.
- `camera_resolution.width` and `camera_resolution.heigh` &ndash; width and height of camera frame in pixels. Default: 1920x1080.
- `cam_focus` &ndash; camera focus from 0 to 255 with step 5. Zero is autofocus. Default: 0.
- `cam_exposure` &ndash; camera exposure. Default: 0.
- `parent_frame_id` &ndash; parent frame ID. Default: "map".
- `child_frame_id` &ndash; robot (or Aruco marker) frame ID. Default: "robot".
- `pixel_resolution.x` and `pixel_resolution.y` &ndash; pixel resolution along X (columns) and Y (rows) axes. Default: 1.0 and 1.0.

| `dict_id` | marker dictionary   |
|:---------:|:--------------------|
| 0         | DICT_4X4_50         |
| 1         | DICT_4X4_100        |
| 2         | DICT_4X4_250        |
| 3         | DICT_4X4_1000       |
| 4         | DICT_5X5_50         |
| 5         | DICT_5X5_100        |
| 6         | DICT_5X5_250        |
| 7         | DICT_5X5_1000       |
| 8         | DICT_6X6_50         |
| 9         | DICT_6X6_100        |
| 10        | DICT_6X6_250        |
| 11        | DICT_6X6_1000       |
| 12        | DICT_7X7_50         |
| 13        | DICT_7X7_100        |
| 14        | DICT_7X7_250        |
| 15        | DICT_7X7_1000       |
| 16        | DICT_ARUCO_ORIGINAL |
| 17        | DICT_APRILTAG_16h5  |
| 18        | DICT_APRILTAG_25h9  |
| 19        | DICT_APRILTAG_36h10 |
| 20        | DICT_APRILTAG_36h11 |

### InitialTransformBroadcasterNode

Node gets first message from `/pose` topic and broadcast static transform that shows a start robot position in global coordinate frame.
This transform is used for show local navigation (e.g. wheel odometry, IMU).

Subscribed topics:

- `/pose` &ndash; robot position in global coordinate frame

Node parameters:

- `parent_frame_id` &ndash; parent frame ID. Default: "map".
- `child_frame_id` &ndash; initial position frame ID. Default: "init_pose".

## Requirements

- ROS2 Foxy or later
- OpenCV v4.2.0 or later
