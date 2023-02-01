from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'parent_frame_id', default_value='map'
        ),
        DeclareLaunchArgument(
            'child_frame_id', default_value='init_pose'
        ),
        DeclareLaunchArgument(
            'topic_name', default_value='/pose'
        ),
        Node(
            package='robot_localization_wrapper',
            executable='initial_transform_broadcaster',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'parent_frame_id': LaunchConfiguration('parent_frame_id'),
                'child_frame_id': LaunchConfiguration('child_frame_id')
            }],
            remappings=[
                ('/pose', LaunchConfiguration('topic_name'))
            ]
        )
    ])
