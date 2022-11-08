cmake_minimum_required(VERSION 3.8)
project(robot_localization VERSION 0.0.1 DESCRIPTION "Robot localization package")

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

find_package(OpenCV 4.2.0 REQUIRED core aruco highgui imgproc videoio)

add_library(aruco_localization src/ArucoLocalization.cpp)

set_target_properties(aruco_localization PROPERTIES
                      VERSION 1.4.4
                      PUBLIC_HEADER "include/${PROJECT_NAME}/ArucoLocalization.hpp;include/${PROJECT_NAME}/ReadSaveCameraParameters.hpp"
                      DEBUG_POSTFIX d)

target_link_libraries(aruco_localization ${OpenCV_LIBS})

target_include_directories(aruco_localization PUBLIC
                           $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}
                           ${OpenCV_INCLUDE_DIRS})


add_library(camera_localization_node src/CameraLocalizationNode.cpp)

set_target_properties(camera_localization_node PROPERTIES
                      VERSION ${PROJECT_VERSION}
                      PUBLIC_HEADER "include/${PROJECT_NAME}/CameraLocalizationNode.hpp"
                      DEBUG_POSTFIX d)
              
target_link_libraries(camera_localization_node aruco_localization ${OpenCV_LIBS})

target_include_directories(camera_localization_node PUBLIC
                           $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}
                           ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(camera_localization_node rclcpp geometry_msgs)



add_executable(camera_localization_node_bin src/camera_localization_node.cpp)

set_target_properties(camera_localization_node_bin PROPERTIES
                      OUTPUT_NAME camera_localization_node)

target_link_libraries(camera_localization_node_bin camera_localization_node)

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
        PUBLIC_HEADER DESTINATION include/${PROJECT_NAME})

install(TARGETS
        camera_localization_node_bin
        DESTINATION lib/${PROJECT_NAME})

ament_package()
