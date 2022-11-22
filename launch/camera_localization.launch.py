from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    sample_period_launch_arg = DeclareLaunchArgument(
        'sample_period', default_value='40'
    )
    dict_id_launch_arg = DeclareLaunchArgument(
        'dict_id', default_value='0'
    )
    marker_id_launch_arg = DeclareLaunchArgument(
        'marker_id', default_value='0'
    )
    cam_id_launch_arg = DeclareLaunchArgument(
        'cam_id', default_value='0'
    )
    cam_width_launch_arg = DeclareLaunchArgument(
        'cam_width', default_value='1920'
    )
    cam_height_launch_arg = DeclareLaunchArgument(
        'cam_height', default_value='1080'
    )
    cam_focus_launch_arg = DeclareLaunchArgument(
        'cam_focus', default_value='0'
    )
    cam_exposure_launch_arg = DeclareLaunchArgument(
        'cam_exposure', default_value='0'
    )
    cam_param_file_launch_arg = DeclareLaunchArgument(
        'cam_param_file', default_value='camera_params.json'
    )
    
    return LaunchDescription([
        sample_period_launch_arg,
        dict_id_launch_arg,
        marker_id_launch_arg,
        cam_id_launch_arg,
        cam_width_launch_arg,
        cam_height_launch_arg,
        cam_focus_launch_arg,
        cam_exposure_launch_arg,
        cam_param_file_launch_arg,
        Node(
            package='robot_localization',
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