import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    SLAM-only (No RViz, No LiDAR)
    - Assumes LiDAR is already running
    - No RViz (saves resources on RPi)
    """
    cwd = os.path.dirname(os.path.abspath(__file__))
    slam_params_file = os.path.join(cwd, 'mapper.yaml')

    ld = LaunchDescription()

    # TF: odom -> base_link
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', 
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'odom', '--child-frame-id', 'base_link'],
        output='log'
    ))

    # TF: base_link -> laser
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        output='log'
    ))

    # SLAM Toolbox (no RViz!)
    ld.add_action(Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file]
    ))

    return ld
