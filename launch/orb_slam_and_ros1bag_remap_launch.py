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

    print(os.path.join(
                get_package_share_directory("orb_slam2davinci"),
                'config', 'davincixi_stereo.yaml'))

    remappings = [
        ('/image_left/image_color_rect', '/camera/left/image_rect'),
        ('/image_right/image_color_rect', '/camera/right/image_rect'),
        ('/camera/camera_info', '/camera/left/camera_info'),
    ]

    image_proc_left_remap = [
        ('/image_raw', '/camera/left/image_raw'),
        ('/camera_info', '/camera/left/camera_info'),
        ('/image', '/camera/left/image_raw'),
        ('/image_rect', '/camera/left/image_rect'),
    ]
    image_proc_right_remap = [
        ('/image_raw', '/camera/right/image_raw'),
        ('/camera_info', '/camera/right/camera_info'),
        ('/image', '/camera/right/image_raw'),
        ('/image_rect', '/camera/right/image_rect'),
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
                'config', 'orb_slam_elp_stereo.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        DeclareLaunchArgument(
            'voc_file',
            default_value=os.path.join(
                get_package_share_directory("orb_slam2_ros"),
                'orb_slam2', 'Vocabulary', 'ORBvoc.txt'),
            description='Full path to vocabulary file to use'),

        Node(
            package='image_proc',
            node_executable='image_proc',
            node_name='image_proc_left',
            output='screen',
            remappings=image_proc_left_remap
        ),

        Node(
            package='image_proc',
            node_executable='image_proc',
            node_name='image_proc_right',
            output='screen',
            remappings=image_proc_right_remap
        ),

        Node(
            parameters=[
                params_file,
                {"voc_file": voc_file,
                 "use_sim_time": use_sim_time},
            ],
            package='orb_slam2_ros',
            node_executable='orb_slam2_ros_stereo',
            node_name='orb_slam2_stereo',
            output='screen',
            remappings=remappings
        )

    ])