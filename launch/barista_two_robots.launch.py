#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import random
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import xacro

def generate_launch_description():
    #files
    description_package_name = "barista_robot_description"
    xacro_file='barista_robot_model.urdf.xacro'
    rviz_file = 'vis4.rviz'
    world_selected = 'obstacles.world'
    # Position and orientation
    robot_base_name = "barista"
    position1 = [-3.0, 0.0, 0.2]
    orientation1 = [0.0, 0.0, 0.0]
    position2 = [4.0, 3.0, 0.2]
    orientation2 = [0.0, 0.0, 0.0]
    #fetching
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_gazebo = get_package_share_directory(description_package_name)
    install_dir = get_package_prefix(description_package_name)

    #instalation
    gazebo_models_path = os.path.join(pkg_robot_gazebo, 'meshes')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    print("\nGAZEBO MODELS PATH==" + str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH==" + str(os.environ["GAZEBO_PLUGIN_PATH"]) + '\n')

    # launch argument for the world file
    world_file_arg=DeclareLaunchArgument('world', default_value=[os.path.join(pkg_robot_gazebo, 'worlds', world_selected), ''], description='Path to the Gazebo world file')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    # set WORLD tf
    static_tf_publisher_node1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_turtle_odom',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'rick/odom']
    )
    static_tf_publisher_node2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_turtle_odom',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'morty/odom']
    )

    # Robot State Publishers
    robot_desc_path = os.path.join(get_package_share_directory(description_package_name))
    xacro_path = os.path.join(robot_desc_path, 'xacro', xacro_file)
    # set arguments for xacro 1: Rick
    robot_name1='rick'
    include_laser1='true'
    robot_color1='Gazebo/Blue'
    robot_state_publisher_node1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node1',
        namespace=robot_name1,
        emulate_tty=True,
        parameters=[{ 'use_sim_time': True,
                     'robot_description': Command(['xacro ', xacro_path, ' robot_name:=', robot_name1, ' include_laser:=', include_laser1, ' robot_color:=', robot_color1])}],
        output='screen'
    )
    #set arguments for xacro 2: Morty
    robot_name2='morty'
    include_laser2='true'
    robot_color2='Gazebo/Red'
    robot_state_publisher_node2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node2',
        namespace=robot_name2,
        emulate_tty=True,
        parameters=[{ 'use_sim_time': True,
                     'robot_description': Command(['xacro ', xacro_path, ' robot_name:=', robot_name2, ' include_laser:=', include_laser2, ' robot_color:=', robot_color2])}],
        output='screen'
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(description_package_name), 'rviz', rviz_file)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

    # Spawn ROBOT Set Gazebo: Rick
    entity_name1 = robot_base_name + "-" + str(int(random.random() * 100000))
    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', entity_name1,
                   '-x', str(position1[0]), '-y', str(position1[1]), '-z', str(position1[2]),
                   '-R', str(orientation1[0]), '-P', str(orientation1[1]), '-Y', str(orientation1[2]),
                   '-topic', robot_name1+'/robot_description']
    )
    # Spawn ROBOT Set Gazebo: Morty
    entity_name2 = robot_base_name + "-" + str(int(random.random() * 100000))
    spawn_robot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', entity_name2,
                   '-x', str(position2[0]), '-y', str(position2[1]), '-z', str(position2[2]),
                   '-R', str(orientation2[0]), '-P', str(orientation2[1]), '-Y', str(orientation2[2]),
                   '-topic', robot_name2+'/robot_description']
    )

    return LaunchDescription([
        world_file_arg,
        gazebo,
        static_tf_publisher_node1,
        static_tf_publisher_node2,
        robot_state_publisher_node1,
        robot_state_publisher_node2,
        rviz_node,
        spawn_robot1,
        spawn_robot2,
    ])

#ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/rick/cmd_vel
#ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/morty/cmd_vel