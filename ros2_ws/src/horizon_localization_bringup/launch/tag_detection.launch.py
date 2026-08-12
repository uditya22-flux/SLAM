import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('horizon_localization_bringup')
    default_config_file = os.path.join(bringup_dir, 'config', 'aruco_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_config_file,
        description='Full path to the ROS2 parameters file for ArUco detector'
    )

    use_mock_arg = DeclareLaunchArgument(
        'use_mock',
        default_value='false',
        description='Whether to launch synthetic Mock ZED publisher for testing'
    )

    aruco_detector_node = Node(
        package='horizon_localization_core',
        executable='aruco_tag_detector.py',
        name='aruco_tag_detector',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    mock_zed_node = Node(
        package='horizon_localization_core',
        executable='mock_zed_publisher.py',
        name='mock_zed_publisher',
        output='screen',
        condition=None  # Can be dynamically enabled via launch arguments
    )

    return LaunchDescription([
        params_file_arg,
        use_mock_arg,
        aruco_detector_node,
    ])
