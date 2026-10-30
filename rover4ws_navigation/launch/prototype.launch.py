"""
Full prototype: Gazebo + 4WS control + localization + camera perception + Nav2 + RViz.

  ros2 launch rover4ws_navigation prototype.launch.py
  -> in RViz click "2D Goal Pose" and click/drag a target. The rover plans and drives there.

Arguments:
  world:=test_yard|stone_yard  stone_yard = gravel, large rocks, walking person, moving cart
  depth_source:=model|gt  Depth Anything V2 (default) or the sim-only perfect depth camera
  use_vo:=true|false       fuse visual odometry into the EKF
  gui:=true|false          Gazebo window
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include(pkg, name, args):
    # Scoped group: without it, arguments given to an included launch file (e.g. rviz:=false
    # for sim.launch.py) leak back and override this file's own arguments.
    return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(pkg), 'launch', name])),
        launch_arguments=args.items())])


def generate_launch_description():
    cfg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='test_yard',
                              description='test_yard | stone_yard (rover4ws_gazebo/worlds)'),
        DeclareLaunchArgument('depth_source', default_value='model'),
        # sim-only perfect depth camera: on for depth_source:=gt, off otherwise (it costs
        # camera frame rate). Set gt_depth:=true with depth_source:=model to evaluate depth.
        DeclareLaunchArgument('gt_depth', default_value=PythonExpression(
            ["'true' if '", cfg('depth_source'), "' == 'gt' else 'false'"])),
        DeclareLaunchArgument('use_vo', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        include('rover4ws_gazebo', 'sim.launch.py',
                {'gui': cfg('gui'), 'rviz': 'false', 'gt_depth': cfg('gt_depth'),
                 'world': PathJoinSubstitution([FindPackageShare('rover4ws_gazebo'), 'worlds',
                                                [cfg('world'), '.world']]),
                 'x': cfg('x'), 'y': cfg('y'), 'yaw': cfg('yaw')}),
        include('rover4ws_localization', 'localization.launch.py', {'use_vo': cfg('use_vo')}),
        include('rover4ws_perception', 'perception.launch.py',
                {'depth_source': cfg('depth_source'), 'use_vo': cfg('use_vo')}),
        # give Gazebo, controllers and the depth model time to come up
        TimerAction(period=8.0, actions=[include('rover4ws_navigation', 'nav.launch.py', {})]),

        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', PathJoinSubstitution(
                 [FindPackageShare('rover4ws_navigation'), 'rviz', 'prototype.rviz'])],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(cfg('rviz'))),
    ])
