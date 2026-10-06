"""Point d'entree unique du projet (EF-11) : ros2 launch tb_bringup bringup.launch.py mode:=sim|real"""

from datetime import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
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

    target = LaunchConfiguration('target')
    declare_target = DeclareLaunchArgument(
        'target', default_value='person', choices=['person', 'ball'],
        description='person : YOLO + distance LiDAR ; ball : couleur HSV + distance par la bbox')

    # Parametres de la chaine : config/<mode>.yaml (commun), puis config/target_<cible>.yaml (detection),
    # le second completant le premier.
    params_file = PathJoinSubstitution([pkg_share, 'config', [mode, '.yaml']])
    target_file = PathJoinSubstitution([pkg_share, 'config', ['target_', target, '.yaml']])

    joystick = LaunchConfiguration('joystick')
    declare_joystick = DeclareLaunchArgument(
        'joystick', default_value='false',
        description='Simulation : ouvrir le joystick a l\'ecran qui deplace la balle de tennis')
    ball_joystick = Node(
        package='tb_bringup', executable='ball_joystick', output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim' and '", joystick, "' == 'true'"])))

    chooser = LaunchConfiguration('chooser')
    declare_chooser = DeclareLaunchArgument(
        'chooser', default_value='true',
        description='target:=person : ouvrir la fenetre de choix de la personne a suivre')
    # En reel, la fenetre lit l'image compressee (WiFi) ; les parametres image_* viennent de <mode>.yaml
    target_chooser = Node(
        package='tb_bringup', executable='target_chooser', output='screen', parameters=[params_file],
        condition=IfCondition(PythonExpression(["'", target, "' == 'person' and '", chooser, "' == 'true'"])))

    pipeline_nodes = [
        Node(package='tb_perception', executable='detector_node',
             parameters=[params_file, target_file, {'tracker_dir': os.path.join(pkg_share, 'config')}],
             output='screen', condition=IfCondition(pipeline),
             # Bibliotheques de calcul limitees a 1 thread chacune (sinon une par coeur : 490 % CPU mesures,
             # Gazebo ralenti a 0,24x le temps reel) ; PyTorch est regle par le parametre num_threads.
             additional_env={'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1'}),
        Node(package='tb_tracking', executable='target_selector_node',
             parameters=[params_file, target_file], output='screen', condition=IfCondition(pipeline)),
        Node(package='tb_control', executable='follower_controller',
             parameters=[params_file], output='screen', condition=IfCondition(pipeline)),
    ]

    # Enregistrement rosbag (ENF-05) : image compressee + toute la chaine, dans ~/rosbags/<mode>_<date>
    declare_record = DeclareLaunchArgument(
        'record', default_value='false', description='Enregistrer un rosbag dans ~/rosbags')
    declare_bag_dir = DeclareLaunchArgument(
        'bag_dir', default_value=os.path.expanduser('~/rosbags'), description='Dossier des rosbags')

    def start_recording(context):
        if LaunchConfiguration('record').perform(context) != 'true':
            return []
        sim_mode = mode.perform(context) == 'sim'
        image = '/camera/image_raw' if sim_mode else '/image_raw'
        camera_info = '/camera/camera_info' if sim_mode else '/camera_info'
        topics = [image + '/compressed', camera_info, '/scan', '/odom', '/tf', '/tf_static',
                  '/detections', '/tracks', '/target', '/target/select', '/cmd_vel', '/follower/enable']
        if sim_mode:
            topics += ['/ball/cmd_vel']  # pas /clock : 'ros2 bag play --clock' la regenere
        name = f"{mode.perform(context)}_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
        output = os.path.join(LaunchConfiguration('bag_dir').perform(context), name)
        os.makedirs(os.path.dirname(output), exist_ok=True)
        return [
            LogInfo(msg=f'Enregistrement rosbag : {output}'),
            ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', output] + topics, output='screen'),
        ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'sim.launch.py')),
        launch_arguments={'gui': gui}.items(),
        condition=IfCondition(EqualsSubstitution(mode, 'sim')),
    )

    real_info = LogInfo(
        msg='mode:=real : sur le robot, lancer `ros2 launch turtlebot3_bringup robot.launch.py` et '
            '`ros2 launch tb_bringup robot_camera.launch.py` (webcam). Meme ROS_DOMAIN_ID partout. '
            'Le suivi demarre desactive : `ros2 run tb_control estop_keyboard`, touche g.',
        condition=IfCondition(EqualsSubstitution(mode, 'real')),
    )

    return LaunchDescription([
        declare_mode,
        declare_gui,
        declare_pipeline,
        declare_target,
        declare_chooser,
        declare_joystick,
        declare_record,
        declare_bag_dir,
        LogInfo(msg=['Parametres : ', params_file, ' + ', target_file]),
        sim,
        real_info,
        ball_joystick,
        target_chooser,
        OpaqueFunction(function=start_recording),
    ] + pipeline_nodes)
