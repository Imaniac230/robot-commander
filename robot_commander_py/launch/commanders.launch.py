# Launch file to start the commanders for chat and ros publisher agents for a robot.
#
# Created by Tomas on 3.7.2024
#

from launch import LaunchDescription, LaunchContext, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml

from ament_index_python.packages import get_package_share_directory

import os
from typing import Optional, List


def launch_setup(context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:
    # Get the necessary directories

    # Create the launch configuration variables
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')
    launch_with_gamepad = LaunchConfiguration('launch_with_gamepad')

    # Process launch variables

    # Create our own temporary YAML files that include substitutions
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,  # must be specified if using namespaces in node launches
        param_rewrites={},
        convert_types=True)

    # Actions - nodes
    start_chat_commander_action_server_node_cmd = Node(
        package='robot_commander_py',
        executable='chat_commander_action_server',
        name='chat_agent_commander',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[configured_params],
    )

    start_goal_commander_action_server_node_cmd = Node(
        package='robot_commander_py',
        executable='goal_commander_action_server',
        name='ros_agent_commander',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[configured_params],
        remappings=[
            ("odometry", "localization/odometry_filtered"),
        ],
    )

    start_commander_action_client_node_cmd = Node(
        package='robot_commander_py',
        executable='commander_action_client',
        name='agent_commander_client',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[],
    )

    start_gamepad_node_cmd = Node(
        condition=IfCondition(launch_with_gamepad),
        package='input',
        namespace=namespace,
        executable='gamepad_node',
        name='dualsense_gamepad',
        parameters=[
            {"default_state": "Disabled"},
        ],
        remappings=[
            ("ros_agent_state", "goal_agent_state"),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Actions and Handlers
    ld.add_action(start_chat_commander_action_server_node_cmd)
    ld.add_action(start_goal_commander_action_server_node_cmd)
    ld.add_action(start_commander_action_client_node_cmd)
    ld.add_action(start_gamepad_node_cmd)

    return [ld]


def generate_launch_description():
    # Get the necessary directories
    commander_dir = get_package_share_directory('robot_commander_py')

    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument(
            name='params_file',
            default_value=os.path.join(commander_dir, 'params', 'commander_params.yaml'),
            description='Path to the commander action server nodes parameters file.'
        ),
        DeclareLaunchArgument(
            name='namespace',
            default_value='',
            description='Namespace for the commander action server nodes launch.'
        ),
        DeclareLaunchArgument(
            name='launch_with_gamepad',
            default_value='False',
            description='Run the customized dualsense gamepad interface.'
        ),
        # Perform the launch setup
        OpaqueFunction(function=launch_setup)
    ])
