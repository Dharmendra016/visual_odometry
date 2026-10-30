"""Depth Anything V2 depth + point clouds, and (optionally) visual odometry."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('depth_source', default_value='model',
                              description="'model' = Depth Anything V2, 'gt' = sim depth camera"),
        DeclareLaunchArgument('use_vo', default_value='true'),
        Node(package='rover4ws_perception', executable='depth_node.py', output='screen',
             parameters=[{'use_sim_time': True,
                          'source': LaunchConfiguration('depth_source')}]),
        Node(package='rover4ws_perception', executable='visual_odometry.py', output='screen',
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('use_vo'))),
    ])
