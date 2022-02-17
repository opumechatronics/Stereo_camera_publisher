import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    params_file = LaunchConfiguration('params_file')
    
    return LaunchDescription([

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory("orb_slam2davinci"),
                'config', 'depthai_publisher.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),


        Node(
            parameters=[
                params_file,
            ],
            package='orb_slam2davinci',
            node_executable='stereo_camera_pub',
            node_name='stereo_camera_pub',
            output='screen',
        )
    ])