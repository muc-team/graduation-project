import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    SLAM-only launch file
    Assumes LiDAR is ALREADY running separately
    """
    cwd = os.path.dirname(os.path.abspath(__file__))
    slam_params_file = os.path.join(cwd, 'mapper.yaml')
    rviz_config_file = os.path.join(cwd, 'config.rviz')

    ld = LaunchDescription()

    # TF: odom -> base_link
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    ))

    # TF: base_link -> laser
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    ))

    # SLAM Toolbox
    ld.add_action(Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file]
    ))

    # RViz2 (optional)
    if os.path.exists(rviz_config_file):
        ld.add_action(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config_file], output='screen'
        ))

    return ld
