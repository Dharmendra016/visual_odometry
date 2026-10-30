"""
RGB-only prototype: Gazebo + 4WS control + localization (wheels + IMU + ground-flow VO)
+ RGB perception (SegFormer, YOLO, corridor, optical flow) + steering-aware planner + RViz.

  ros2 launch rover4ws_rgb_navigation rgb_prototype.launch.py [world:=stone_yard]
  -> RViz "2D Goal Pose", drag the arrow in the direction of travel.

The previous Depth Anything V2 prototype is unchanged:
  ros2 launch rover4ws_navigation prototype.launch.py   (git tag v0.1-depth-anything)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include(pkg, name, args):
    # scoped: arguments given to an include must not leak back into this file
    return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(pkg), 'launch', name])),
        launch_arguments=args.items())])


def generate_launch_description():
    cfg = LaunchConfiguration
    pkg = FindPackageShare('rover4ws_rgb_navigation')
    params = PathJoinSubstitution([pkg, 'config', 'rgb_nav2.yaml'])
    nav = [
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[params]),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', output='screen', parameters=[params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_rgb', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['planner_server', 'collision_monitor']}]),
        Node(package='rover4ws_navigation', executable='safety_gate.py', output='screen',
             parameters=[{'use_sim_time': True, 'perception_topic': '/rgb/obstacles'}]),
        Node(package='rover4ws_rgb_navigation', executable='steering_planner.py', output='screen',
             parameters=[{'use_sim_time': True}]),
    ]
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='stone_yard',
                              description='test_yard | stone_yard'),
        DeclareLaunchArgument('use_vo', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        include('rover4ws_gazebo', 'sim.launch.py', {
            'gui': cfg('gui'), 'rviz': 'false', 'gt_depth': 'false',
            'world': PathJoinSubstitution([FindPackageShare('rover4ws_gazebo'), 'worlds',
                                           [cfg('world'), '.world']]),
            'x': cfg('x'), 'y': cfg('y'), 'yaw': cfg('yaw')}),
        include('rover4ws_localization', 'localization.launch.py', {'use_vo': cfg('use_vo')}),
        Node(package='rover4ws_rgb_perception', executable='rgb_perception_node.py',
             output='screen', parameters=[{'use_sim_time': True}]),
        TimerAction(period=8.0, actions=nav),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'rgb_prototype.rviz'])],
             parameters=[{'use_sim_time': True}], condition=IfCondition(cfg('rviz'))),
    ])
