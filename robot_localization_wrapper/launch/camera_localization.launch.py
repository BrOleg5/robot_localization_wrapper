from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'sample_period', default_value='40'
        ),
        DeclareLaunchArgument(
            'dict_id', default_value='0'
        ),
        DeclareLaunchArgument(
            'marker_id', default_value='0'
        ),
        DeclareLaunchArgument(
            'cam_id', default_value='0'
        ),
        DeclareLaunchArgument(
            'cam_width', default_value='1920'
        ),
        DeclareLaunchArgument(
            'cam_height', default_value='1080'
        ),
        DeclareLaunchArgument(
            'cam_focus', default_value='0'
        ),
        DeclareLaunchArgument(
            'cam_exposure', default_value='0.0'
        ),
        DeclareLaunchArgument(
            'cam_param_file', default_value='camera_params.json'
        ),
        Node(
            package='robot_localization_wrapper',
            executable='camera_localization_node',
            parameters=[{
                'sample_period': LaunchConfiguration('sample_period'),
                'dict_id': LaunchConfiguration('dict_id'),
                'marker_id': LaunchConfiguration('marker_id'),
                'cam_id': LaunchConfiguration('cam_id'),
                'cam_width': LaunchConfiguration('cam_width'),
                'cam_height': LaunchConfiguration('cam_height'),
                'cam_focus': LaunchConfiguration('cam_focus'),
                'cam_exposure': LaunchConfiguration('cam_exposure'),
                'cam_param_file': LaunchConfiguration('cam_param_file')
            }]
        )
    ])