"""
Spawn the 4WS rover in Gazebo Classic with ros2_control and the cmd_vel -> 4WS node.

  ros2 launch rover4ws_gazebo sim.launch.py
  ros2 run teleop_twist_keyboard teleop_twist_keyboard   # drive it
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg = FindPackageShare('rover4ws_description')
    sim_pkg = FindPackageShare('rover4ws_gazebo')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    controllers_file = PathJoinSubstitution([sim_pkg, 'config', 'controllers.yaml'])
    robot_description = ParameterValue(Command([
        'xacro ', PathJoinSubstitution([desc_pkg, 'urdf', 'rover4ws.urdf.xacro']),
        ' use_sim:=true',
        ' controllers_file:=', controllers_file,
        ' gt_depth:=', LaunchConfiguration('gt_depth'),
    ]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])),
        launch_arguments={'world': world, 'gui': gui}.items(),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'rover4ws',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.02', '-Y', LaunchConfiguration('yaw')],
        output='screen',
    )

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager', '/controller_manager'])

    controllers_after_spawn = RegisterEventHandler(OnProcessExit(
        target_action=spawn,
        on_exit=[spawner('joint_state_broadcaster'),
                 spawner('steering_controller'),
                 spawner('wheel_controller')],
    ))

    kinematics = Node(
        package='rover4ws_gazebo',
        executable='four_ws_kinematics.py',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([desc_pkg, 'rviz', 'rover4ws.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution(
            [sim_pkg, 'worlds', 'test_yard.world'])),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('gt_depth', default_value='false',
                              description='Add the simulation-only ground-truth depth camera'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        gazebo,
        rsp,
        spawn,
        controllers_after_spawn,
        kinematics,
        rviz_node,
    ])
