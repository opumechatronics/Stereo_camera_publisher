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

    remappings = [
        ('/image_left/image_color_rect', '/mask_generator/davinci/left'),
        ('/image_right/image_color_rect', '/mask_generator/davinci/right'),
        ('/image_left/image_mask_rect', '/mask_generator/davinci/left_mask'),
        ('/image_right/image_mask_rect', '/mask_generator/davinci/right_mask'),
        ('/camera/camera_info', '/stereo_camera_pub/camera_info'),
    ]
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory("orb_slam2davinci"),
                'config', 'orb_slam_davincixi_stereo.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        DeclareLaunchArgument(
            'voc_file',
            default_value=os.path.join(
                get_package_share_directory("orb_slam2_ros"),
                'orb_slam2', 'Vocabulary', 'ORBvoc.txt'),
            description='Full path to vocabulary file to use'),

        Node(
            parameters=[
                params_file,
                {"voc_file": voc_file,
                 "use_sim_time": use_sim_time},
            ],
            package='orb_slam2_ros',
            executable='orb_slam2_ros_stereo',
            name='orb_slam2_stereo',
            output='screen',
            remappings=remappings
        ),

        DeclareLaunchArgument(
            'params_file_for_pub',
            default_value=os.path.join(
                get_package_share_directory("orb_slam2davinci"),
                'config', 'stereo_publisher.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),


        Node(
            parameters=[
                params_file_for_pub,
            ],
            package='orb_slam2davinci',
            executable='stereo_camera_pub',
            name='stereo_camera_pub',
            output='screen',
        ),

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
