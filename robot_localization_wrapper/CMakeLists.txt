cmake_minimum_required(VERSION 3.8)

include("cmake/read_package_version.cmake")
read_package_version_from_xml()

project(robot_localization_wrapper VERSION ${PACKAGE_VERSION} DESCRIPTION "Robot localization package")

unset(PACKAGE_VERSION)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  set(CMAKE_CXX_FLAGS_RELEASE "/O2")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(OpenCV 4.2.0 REQUIRED core highgui imgproc videoio)

find_package(aruco_robot_localization REQUIRED)

add_library(camera_localization_node src/CameraLocalizationNode.cpp)

set_target_properties(camera_localization_node PROPERTIES
  VERSION ${PROJECT_VERSION}
  PUBLIC_HEADER "include/${PROJECT_NAME}/CameraLocalizationNode.hpp"
  DEBUG_POSTFIX d
)

target_link_libraries(camera_localization_node arucolocalization ${OpenCV_LIBS})

target_include_directories(camera_localization_node PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(camera_localization_node rclcpp geometry_msgs tf2 tf2_ros nav_msgs)



add_executable(camera_localization_node_bin src/camera_localization_node.cpp)

# source: https://forum.kde.org/viewtopic.php?f=74&t=153982
if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
  # You must acknowledge that you understand MSVC resolved a byte alignment issue in this compiler
  # We get this due to using Eigen objects and allocating those objects with make_shared
  target_compile_definitions(camera_localization_node_bin PRIVATE _ENABLE_EXTENDED_ALIGNED_STORAGE)
endif()

set_target_properties(camera_localization_node_bin PROPERTIES
  OUTPUT_NAME camera_localization_node
)

target_link_libraries(camera_localization_node_bin camera_localization_node)



add_executable(initial_transform_broadcaster
  src/initial_transform_broadcaster.cpp
  src/InitialTransformBroadcaster.cpp
  include/${PROJECT_NAME}/InitialTransformBroadcaster.hpp
)

target_include_directories(initial_transform_broadcaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}
)

ament_target_dependencies(initial_transform_broadcaster rclcpp geometry_msgs tf2 tf2_ros)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  camera_localization_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  PUBLIC_HEADER DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  camera_localization_node_bin
  initial_transform_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
