"""Wheel odometry + EKF (optionally fusing visual odometry). Publishes odom -> base_footprint."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('rover4ws_localization')
    use_vo = LaunchConfiguration('use_vo')

    def ekf(config, condition):
        return Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            parameters=[PathJoinSubstitution([pkg, 'config', config])],
            condition=condition,
        )

    return LaunchDescription([
        DeclareLaunchArgument('use_vo', default_value='true',
                              description='Fuse /vo/odom into the EKF'),
        Node(package='rover4ws_localization', executable='wheel_odometry.py',
             parameters=[{'use_sim_time': True}]),
        ekf('ekf_vo.yaml', IfCondition(use_vo)),
        ekf('ekf.yaml', UnlessCondition(use_vo)),
    ])
