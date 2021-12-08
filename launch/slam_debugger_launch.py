import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    voc_file = LaunchConfiguration('voc_file')

    params_file_for_pub = LaunchConfiguration('params_file_for_pub')

    params_file_for_debug = LaunchConfiguration('params_file_for_debug')

    print(os.path.join(
                get_package_share_directory("orb_slam2davinci"),
                'config', 'orb_slam_debugger.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file_for_debug',
            default_value=os.path.join(
                get_package_share_directory("orb_slam2davinci"),
                'config', 'orb_slam_debugger.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        Node(
            parameters=[
                params_file_for_debug,
            ],
            package='orb_slam_debugger',
            executable='slam_debugger',
            name='slam_debugger',
            output='screen',
        )
    ])
