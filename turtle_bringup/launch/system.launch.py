import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('turtle_bringup'),
                                  'config', 'speed_limits.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='YAML with max_linear_speed / max_angular_speed'),
        Node(package='turtlesim', executable='turtlesim_node', name='turtlesim', output='screen'),
        Node(package='turtle_control', executable='goto_pose_server', name='goto_pose_server',
             output='screen', parameters=[LaunchConfiguration('params_file')]),
        Node(package='turtle_control', executable='reset_service', name='reset_service',
             output='screen'),
        Node(package='turtle_control', executable='square_server', name='square_server',
             output='screen'),
    ])
