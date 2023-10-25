import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix
from launch.substitutions import Command
import xacro


def generate_launch_description():
    ####### DATA INPUT ##########
    description_package_name = "barista_robot_description"
    urdf_file = 'barista_robot_model.urdf'
    rviz_file='vis1.rviz'
    install_dir=get_package_share_directory(description_package_name)
    world_selected='obstacles.world'
    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(install_dir, "urdf", urdf_file)

    # This is to find the models inside the models folder in my_robot_gazebo package
    gazebo_models_path = os.path.join(description_package_name, 'meshes')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    print("\nGAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"])+'\n')


    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py', name='spawn_entity_node',
                        arguments=['-entity', 'barista', '-x', '1.0', '-y', '1.0', '-z', '0.2',
                                   '-topic', '/robot_description'],
                        output='screen')

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(description_package_name), 'rviz', rviz_file)

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])    


    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        DeclareLaunchArgument(
                'world',
                default_value=[os.path.join(install_dir, 'worlds', world_selected), ''],
                description='SDF world file'),
        spawn_entity
    ])
