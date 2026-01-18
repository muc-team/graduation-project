#!/usr/bin/env python3
"""
Autonomous Navigation Launch File
Starts: SLAM + Motor Controller + Fake Odometry + Nav2
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    cwd = os.path.dirname(os.path.abspath(__file__))
    nav2_params_file = os.path.join(cwd, 'nav2_params.yaml')
    
    # Get paths
    mapping_dir = os.path.join(cwd, '..', 'mapping')
    slam_params_file = os.path.join(mapping_dir, 'mapper.yaml')
    rviz_config_file = os.path.join(mapping_dir, 'config.rviz')
    
    # Serial port parameter
    serial_port = LaunchConfiguration('serial_port')
    
    ld = LaunchDescription()
    
    # Declare arguments
    ld.add_action(DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Arduino'
    ))
    
    # --- RPLiDAR ---
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')
        ])
    ))
    
    # --- Static TF: odom -> base_link (will be overwritten by fake_odom) ---
    # We still need base_link -> laser
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    ))
    
    # --- SLAM Toolbox ---
    ld.add_action(Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file]
    ))
    
    # --- Fake Odometry (publishes odom -> base_link TF) ---
    ld.add_action(Node(
        package='navigation',
        executable='fake_odom',
        name='fake_odom',
        output='screen',
        parameters=[{
            'linear_scale': 0.2,
            'angular_scale': 0.5,
            'publish_rate': 30.0
        }]
    ))
    
    # --- Motor Controller ---
    ld.add_action(Node(
        package='navigation',
        executable='motor_controller',
        name='motor_controller',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': 9600,
            'linear_threshold': 0.1,
            'angular_threshold': 0.3
        }]
    ))
    
    # --- Nav2 BT Navigator ---
    ld.add_action(Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file]
    ))
    
    # --- Nav2 Controller Server ---
    ld.add_action(Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file]
    ))
    
    # --- Nav2 Planner Server ---
    ld.add_action(Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file]
    ))
    
    # --- Nav2 Behavior Server ---
    ld.add_action(Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file]
    ))
    
    # --- Nav2 Lifecycle Manager ---
    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator'
            ]
        }]
    ))
    
    # --- Local Costmap ---
    ld.add_action(Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[('costmap', 'local_costmap')]
    ))
    
    # --- Global Costmap ---
    ld.add_action(Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[('costmap', 'global_costmap')]
    ))
    
    # --- RViz2 ---
    if os.path.exists(rviz_config_file):
        ld.add_action(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config_file], output='screen'
        ))
    else:
        ld.add_action(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen'
        ))
    
    return ld
