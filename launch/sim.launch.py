"""Gazebo Classic + UR3/UR3e + ros2_control + MoveIt 2 + RViz.

    ros2 launch ur3_llm_control sim.launch.py                 # ur_type from scene.yaml (ur3e)
    ros2 launch ur3_llm_control sim.launch.py ur_type:=ur3
    ros2 launch ur3_llm_control sim.launch.py gazebo_gui:=false   # lighter (WSL2)

Start-up order (each step waits for the previous one, so slow machines/WSL2
do not hit spawner time-outs):
    gzserver(+world from scene.yaml) + robot_state_publisher
      -> spawn_entity (robot "ur")
      -> joint_state_broadcaster -> joint_trajectory_controller
      -> MoveIt 2 move_group + RViz  (ur_moveit_config/ur_moveit.launch.py)

The world (table, 3 cubes, 3 zones, 2 buffer pads) is generated from
config/scene.yaml, the same file the planning scene and the skills use.
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PKG = 'ur3_llm_control'
SUPPORTED = ('ur3', 'ur3e')


def _default_scene():
    return os.path.join(get_package_share_directory(PKG), 'config', 'scene.yaml')


def _scene_ur_type(path):
    try:
        with open(path, encoding='utf-8') as f:
            return str(yaml.safe_load(f).get('ur_type', 'ur3e'))
    except OSError:
        return 'ur3e'


def launch_setup(context, *args, **kwargs):
    from ur3_llm_control.scene_model import SceneConfig
    from ur3_llm_control.world_gen import generate

    ur_type = LaunchConfiguration('ur_type').perform(context)
    scene_file = LaunchConfiguration('scene_file').perform(context)
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    launch_rviz = LaunchConfiguration('launch_rviz')
    safety_limits = LaunchConfiguration('safety_limits')

    if ur_type not in SUPPORTED:
        raise RuntimeError(f'ur_type:={ur_type} is not supported (use one of {SUPPORTED})')

    scene = SceneConfig.from_file(scene_file)
    actions = []
    if scene.ur_type != ur_type:
        actions.append(LogInfo(msg=f'[sim.launch] WARNING: ur_type:={ur_type} but {scene_file} '
                                   f'says ur_type: {scene.ur_type}. Reachability was checked for '
                                   f'{scene.ur_type}; run `ros2 run {PKG} check_scene`.'))

    # ------------------------------------------------------------ world file
    world_path = os.path.join(tempfile.gettempdir(), 'ur3_llm_world.world')
    with open(world_path, 'w', encoding='utf-8') as f:
        f.write(generate(scene))
    actions.append(LogInfo(msg=f'[sim.launch] Gazebo world generated: {world_path}'))

    # ----------------------------------------------------- robot description
    controllers_file = PathJoinSubstitution(
        [FindPackageShare('ur_simulation_gazebo'), 'config', 'ur_controllers.yaml'])
    initial_positions = PathJoinSubstitution(
        [FindPackageShare('ur_description'), 'config', 'initial_positions.yaml'])
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('ur_description'), 'urdf', 'ur.urdf.xacro']), ' ',
        'name:=ur', ' ',
        'ur_type:=', ur_type, ' ',
        'safety_limits:=', safety_limits, ' ',
        'sim_gazebo:=true', ' ',
        'simulation_controllers:=', controllers_file, ' ',
        'initial_positions_file:=', initial_positions,
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)}

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='both',
        parameters=[{'use_sim_time': True}, robot_description])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('gazebo_ros'), '/launch', '/gazebo.launch.py']),
        launch_arguments={'world': world_path, 'gui': gazebo_gui, 'verbose': 'false'}.items())

    spawn_robot = Node(
        package='gazebo_ros', executable='spawn_entity.py', name='spawn_ur', output='screen',
        arguments=['-entity', 'ur', '-topic', 'robot_description', '-timeout', '120'])

    jsb_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '120'])
    jtc_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '120'])

    # use_sim_time:=true makes ur_moveit.launch.py use joint_trajectory_controller
    # (the scaled controller of the real robot does not exist in Gazebo).
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('ur_moveit_config'), '/launch', '/ur_moveit.launch.py']),
        launch_arguments={
            'ur_type': ur_type,
            'safety_limits': safety_limits,
            'use_sim_time': 'true',
            'launch_rviz': launch_rviz,
            'launch_servo': 'false',
        }.items())

    actions += [
        robot_state_publisher,
        gazebo,
        spawn_robot,
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[jsb_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=[jtc_spawner])),
        RegisterEventHandler(OnProcessExit(
            target_action=jtc_spawner,
            on_exit=[LogInfo(msg='[sim.launch] controllers up -> starting MoveIt 2'), moveit])),
    ]
    return actions


def generate_launch_description():
    scene = _default_scene()
    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value=_scene_ur_type(scene),
                              choices=list(SUPPORTED),
                              description='UR3 or UR3e (default: ur_type in scene.yaml)'),
        DeclareLaunchArgument('scene_file', default_value=scene,
                              description='scene description (table, cubes, zones)'),
        DeclareLaunchArgument('gazebo_gui', default_value='true',
                              description='start the Gazebo client window'),
        DeclareLaunchArgument('launch_rviz', default_value='true',
                              description='start RViz with the MoveIt plugin'),
        DeclareLaunchArgument('safety_limits', default_value='true',
                              description='UR safety limits in the URDF'),
        OpaqueFunction(function=launch_setup),
    ])
