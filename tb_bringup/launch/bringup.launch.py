"""Point d'entree unique du projet (EF-11) : ros2 launch tb_bringup bringup.launch.py mode:=sim|real"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tb_bringup')

    mode = LaunchConfiguration('mode')
    gui = LaunchConfiguration('gui')

    declare_mode = DeclareLaunchArgument(
        'mode', default_value='sim', choices=['sim', 'real'],
        description='sim : Gazebo ; real : TurtleBot3 reel en WiFi')
    declare_gui = DeclareLaunchArgument('gui', default_value='true')

    pipeline = LaunchConfiguration('pipeline')
    declare_pipeline = DeclareLaunchArgument(
        'pipeline', default_value='true',
        description='Lancer la chaine detection -> cible -> commande')

    # Parametres de tous les nodes de la chaine : config/sim.yaml ou config/real.yaml
    params_file = PathJoinSubstitution([pkg_share, 'config', [mode, '.yaml']])

    joystick = LaunchConfiguration('joystick')
    declare_joystick = DeclareLaunchArgument(
        'joystick', default_value='false',
        description='Simulation : ouvrir le joystick a l\'ecran qui deplace la balle de tennis')
    ball_joystick = Node(
        package='tb_bringup', executable='ball_joystick', output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim' and '", joystick, "' == 'true'"])))

    pipeline_nodes = [
        Node(package='tb_perception', executable='detector_node',
             parameters=[params_file], output='screen', condition=IfCondition(pipeline)),
        Node(package='tb_tracking', executable='target_selector_node',
             parameters=[params_file], output='screen', condition=IfCondition(pipeline)),
        Node(package='tb_control', executable='follower_controller',
             parameters=[params_file], output='screen', condition=IfCondition(pipeline)),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'sim.launch.py')),
        launch_arguments={'gui': gui}.items(),
        condition=IfCondition(EqualsSubstitution(mode, 'sim')),
    )

    real_info = LogInfo(
        msg='mode:=real : lancer sur le robot `ros2 launch turtlebot3_bringup robot.launch.py` '
            'et le driver de la webcam (usb_cam). Meme ROS_DOMAIN_ID sur le robot et le laptop.',
        condition=IfCondition(EqualsSubstitution(mode, 'real')),
    )

    return LaunchDescription([
        declare_mode,
        declare_gui,
        declare_pipeline,
        declare_joystick,
        LogInfo(msg=['Parametres : ', params_file]),
        sim,
        real_info,
        ball_joystick,
    ] + pipeline_nodes)
