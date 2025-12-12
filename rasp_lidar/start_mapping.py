import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    cwd = os.getcwd()
    slam_params_file = os.path.join(cwd, 'mapper.yaml')
    rviz_config_file = os.path.join(cwd, 'config.rviz')

    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')
        ])
    ))

    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    ))

    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    ))

    ld.add_action(Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file]
    ))

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
