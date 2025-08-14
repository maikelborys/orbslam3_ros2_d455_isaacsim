from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='/home/robot/ros2_test/src/ros2_orb_slam3/rviz/orbslam3.rviz',
        description='Path to RViz config file'
    )

    return LaunchDescription([
        rviz_config_arg,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')]
        )
    ])


