"""Full system in ONE terminal: simulation (sim.launch.py) + llm_robot_node.

    ros2 launch ur3_llm_control llm_robot.launch.py
    # then, in another terminal:
    ros2 run ur3_llm_control send_command "Put the red cube in zone B."

The node has no keyboard here (launch does not forward stdin), so it takes
commands from the /llm_robot/command topic. For the interactive "Command>"
prompt, use two terminals instead (see HUONG_DAN_CHAY.md):
    ros2 launch ur3_llm_control sim.launch.py
    ros2 run ur3_llm_control llm_robot_node --ros-args -p use_sim_time:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ur3_llm_control'), 'launch', 'sim.launch.py'])),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'gazebo_gui': LaunchConfiguration('gazebo_gui'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
        }.items())

    llm_node = Node(
        package='ur3_llm_control', executable='llm_robot_node', name='llm_robot_node',
        output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': True,
            'interactive': False,
            'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
            'llm_model': ParameterValue(LaunchConfiguration('llm_model'), value_type=str),
        }])

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur3e', choices=['ur3', 'ur3e']),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='validate plans only, do not move the robot'),
        DeclareLaunchArgument('llm_model', default_value='',
                              description='override llm.model of student_config.yaml'),
        sim,
        # the node itself waits (up to 180 s) for move_group; the delay only keeps
        # its banner from being buried under the Gazebo/MoveIt start-up log
        TimerAction(period=8.0, actions=[llm_node]),
    ])
