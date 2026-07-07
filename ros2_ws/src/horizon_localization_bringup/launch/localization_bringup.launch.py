import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory('horizon_localization_bringup')
    
    # Path to parameter file
    default_params_file = os.path.join(bringup_dir, 'config', 'tf_params.yaml')
    
    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameter file to use'
    )
    
    # Read the YAML file to extract the static transform values
    try:
        with open(default_params_file, 'r') as f:
            params = yaml.safe_load(f)
    except Exception as e:
        print(f"Error reading parameters from {default_params_file}: {e}")
        params = {}

    # Extract ZED camera transform parameters
    zed_config = params.get('zed_camera_transform', {})
    zed_x = str(zed_config.get('x', 0.25))
    zed_y = str(zed_config.get('y', 0.0))
    zed_z = str(zed_config.get('z', 0.35))
    zed_roll = str(zed_config.get('roll', 0.0))
    zed_pitch = str(zed_config.get('pitch', 0.0))
    zed_yaw = str(zed_config.get('yaw', 0.0))
    zed_frame = zed_config.get('frame_id', 'base_link')
    zed_child_frame = zed_config.get('child_frame_id', 'zed_camera_link')

    nodes = []

    # 1. ZED static transform publisher node (base_link -> zed_camera_link)
    zed_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_zed_camera',
        arguments=[
            '--x', zed_x, '--y', zed_y, '--z', zed_z,
            '--yaw', zed_yaw, '--pitch', zed_pitch, '--roll', zed_roll,
            '--frame-id', zed_frame, '--child-frame-id', zed_child_frame
        ]
    )
    nodes.append(zed_tf_publisher)

    # 2. Dummy static transform publishers (for earth -> map and map -> odom)
    # Note: These are helper transforms for visualization and baseline setups.
    # In live fusion, map -> odom will be dynamically published by EKF/Fusion nodes.
    dummy_config = params.get('dummy_transforms', {})
    if dummy_config.get('publish_dummy_world', True):
        # Earth to map
        e2m = dummy_config.get('earth_to_map', {})
        earth_tf_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='earth_to_map_publisher',
            arguments=[
                '--x', str(e2m.get('x', 0.0)), '--y', str(e2m.get('y', 0.0)), '--z', str(e2m.get('z', 0.0)),
                '--yaw', str(e2m.get('yaw', 0.0)), '--pitch', str(e2m.get('pitch', 0.0)), '--roll', str(e2m.get('roll', 0.0)),
                '--frame-id', e2m.get('frame_id', 'earth'), '--child-frame-id', e2m.get('child_frame_id', 'map')
            ]
        )
        nodes.append(earth_tf_publisher)

        # Map to odom
        m2o = dummy_config.get('map_to_odom', {})
        map_tf_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_publisher',
            arguments=[
                '--x', str(m2o.get('x', 0.0)), '--y', str(m2o.get('y', 0.0)), '--z', str(m2o.get('z', 0.0)),
                '--yaw', str(m2o.get('yaw', 0.0)), '--pitch', str(m2o.get('pitch', 0.0)), '--roll', str(m2o.get('roll', 0.0)),
                '--frame-id', m2o.get('frame_id', 'map'), '--child-frame-id', m2o.get('child_frame_id', 'odom')
            ]
        )
        nodes.append(map_tf_publisher)

    # 3. RViz2 node
    rviz_config_path = os.path.join(bringup_dir, 'rviz', 'localization.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    nodes.append(rviz_node)

    return LaunchDescription([
        params_file_arg,
        *nodes
    ])
