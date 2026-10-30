"""Nav2 servers + collision monitor + safety gate for the 4WS rover (odom frame, no map)."""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('rover4ws_navigation')
    params = PathJoinSubstitution([pkg, 'config', 'nav2_params.yaml'])
    bt_xml = PathJoinSubstitution([pkg, 'behavior_trees', 'navigate_4ws.xml'])
    bt_poses_xml = PathJoinSubstitution([pkg, 'behavior_trees', 'navigate_through_poses_4ws.xml'])
    to_nav = [('cmd_vel', 'cmd_vel_nav')]
    lifecycle_nodes = ['controller_server', 'planner_server', 'behavior_server',
                       'bt_navigator', 'collision_monitor']

    return LaunchDescription([
        Node(package='nav2_controller', executable='controller_server', output='screen',
             parameters=[params], remappings=to_nav),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[params]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=[params], remappings=to_nav),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=[params, {'default_nav_to_pose_bt_xml': bt_xml,
                                                   'default_nav_through_poses_bt_xml': bt_poses_xml}]),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', output='screen', parameters=[params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': lifecycle_nodes}]),
        Node(package='rover4ws_navigation', executable='safety_gate.py', output='screen',
             parameters=[{'use_sim_time': True}]),
    ])
