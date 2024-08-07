# Launch file to start the chat and ros publisher agents for a robot.
#
# Created by Tomas on 27.6.2024
#

from launch import LaunchDescription, LaunchContext, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

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

    # Process launch variables

    # Create our own temporary YAML files that include substitutions
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,  # must be specified if using namespaces in node launches
        param_rewrites={},
        convert_types=True)

    # Actions - nodes
    start_chat_agent_server_node_cmd = Node(
        package='robot_commander_py',
        executable='agent_node',
        name='chat_agent',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[configured_params],
    )

    start_ros_agent_server_node_cmd = Node(
        package='robot_commander_py',
        executable='agent_node',
        name='ros_agent',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[configured_params],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Actions and Handlers
    ld.add_action(start_chat_agent_server_node_cmd)
    ld.add_action(start_ros_agent_server_node_cmd)

    return [ld]


def generate_launch_description():
    # Get the necessary directories
    commander_dir = get_package_share_directory('robot_commander_py')

    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument(
            name='params_file',
            default_value=os.path.join(commander_dir, 'params', 'agent_params.yaml'),
            description='Path to the agent server nodes parameters file.'
        ),
        DeclareLaunchArgument(
            name='namespace',
            default_value='',
            description='Namespace for the agent server nodes launch.'
        ),
        # Perform the launch setup
        OpaqueFunction(function=launch_setup)
    ])
