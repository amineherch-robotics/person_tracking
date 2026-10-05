"""Simulation Gazebo : monde avec personne + TurtleBot3 Burger equipe d'une webcam."""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tb_bringup')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    # Meshes du Burger (model://turtlebot3_common/...) fournis par turtlebot3_gazebo
    tb3_models = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')

    model_sdf = os.path.join(pkg_share, 'models', 'tb3_burger_cam', 'model.sdf')
    urdf_path = os.path.join(pkg_share, 'urdf', 'tb3_burger_cam.urdf')
    bridge_config = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    gui_config = os.path.join(pkg_share, 'config', 'gz_gui.config')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')

    declare_args = [
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([pkg_share, 'worlds', 'person_world.sdf']),
            description='Fichier monde Gazebo'),
        DeclareLaunchArgument('gui', default_value='true', description='Ouvrir la fenetre Gazebo'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    resource_paths = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', tb3_models),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg_share, 'models')),
    ]

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items(),
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-g -v2 --gui-config {gui_config}', 'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(gui),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'tb3_burger_cam',
            '-file', model_sdf,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '-Y', yaw,
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen',
    )

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Le serveur `gz sim` est lance via un wrapper ruby : a l'arret, le wrapper meurt mais le serveur
    # peut survivre et entrer en conflit avec la simulation suivante (/clock publie deux fois).
    # Idem pour la fenetre (client -g), reconnaissable grace a --gui-config pointant dans ce package.
    # On tue donc le serveur et la fenetre de ce package avant le demarrage et a l'arret.
    def kill_stale_gazebo(*_):
        subprocess.run(['pkill', '-9', '-f', 'gz sim -(r -s|g) .*tb_bringup'], check=False)

    cleanup_before_start = OpaqueFunction(function=kill_stale_gazebo)
    # OnShutdown peut etre emis plusieurs fois : un simple callable Python supporte les appels repetes
    cleanup_on_shutdown = RegisterEventHandler(OnShutdown(on_shutdown=kill_stale_gazebo))

    return LaunchDescription(
        declare_args + resource_paths + [
            cleanup_before_start,
            cleanup_on_shutdown,
            gz_server,
            gz_client,
            spawn_robot,
            robot_state_publisher,
            bridge,
            image_bridge,
        ]
    )
