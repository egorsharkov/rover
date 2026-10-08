import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def static_tf(x, y, z, parent, child):
    return Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=[str(x), str(y), str(z), '0', '0', '0', parent, child],
        parameters=[{'use_sim_time': True}])

def generate_launch_description():
    pkg = get_package_share_directory('rover')
    gz_ros = get_package_share_directory('gazebo_ros')

    world = os.path.join(pkg, 'world', 'wall.world')
    robot_sdf = os.path.join(pkg, 'models', 'rover', 'model.sdf')

    # Запуск Gazebo с явным подключением ROS Factory плагина
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world,
            'gui': LaunchConfiguration('gui'),
            'extra_gazebo_args': '-s libgazebo_ros_factory.so'  # <-- КЛЮЧЕВОЙ ФИКС
        }.items())

    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-entity', 'rover', '-file', robot_sdf,
                   '-x', '-6.0', '-y', '0.0', '-z', '0.01'])

    navigator = TimerAction(
        period=10.0,
        actions=[Node(package='rover_nav', executable='navigator',
                      output='screen',
                      parameters=[{'use_sim_time': True}])])
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg, 'rviz', 'rover.rviz')],
        parameters=[{'use_sim_time': True}])

    camera_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0.05', '0', '0.12', '0', '0', '0',
                   'base_link', 'camera_link'])
    optical_tf = Node(
    package='tf2_ros', executable='static_transform_publisher',
    arguments=['0', '0', '0', '-1.5708', '0', '-1.5708',
               'camera_link', 'camera_depth_optical_frame'],
    parameters=[{'use_sim_time': True}])

    base_tf   = static_tf(0.0,    0.0, 0.010, 'base_footprint', 'base_link')
    camera_tf = static_tf(0.05,   0.0, 0.12,  'base_link',      'camera_link')
    scan_tf   = static_tf(-0.032, 0.0, 0.171, 'base_link',      'base_scan')

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH',
            os.path.join(pkg, 'models') + ':/opt/ros/humble/share/turtlebot3_gazebo/models:'
            + os.environ.get('GAZEBO_MODEL_PATH', '')),
        DeclareLaunchArgument('gui', default_value='true'),
        gazebo, spawn, navigator, rviz, base_tf, camera_tf, scan_tf, optical_tf
    ])