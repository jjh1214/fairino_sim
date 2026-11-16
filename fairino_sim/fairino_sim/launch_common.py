"""Launch arguments and helpers shared by the fairino_sim launch files."""

import os

from ament_index_python.packages import get_package_share_directory
from fairino_sim import description
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TRUE_VALUES = ('true', '1', 'yes', 'on')


def declare_robot_arguments():
    """Return the launch arguments that describe the simulated robot."""
    return [
        DeclareLaunchArgument(
            'robot_name', default_value='fairino5_v6',
            description='Robot model: ' + ', '.join(description.SUPPORTED_MODELS)),
        DeclareLaunchArgument(
            'tool', default_value='none',
            description='Tool on the flange: none, a tool name under fairino_sim/tools, '
                        'or the path of a tool YAML file'),
        DeclareLaunchArgument(
            'base_xyz', default_value='0 0 0',
            description='Position of base_link in the world frame, in metres'),
        DeclareLaunchArgument(
            'base_rpy', default_value='0 0 0',
            description='Roll, pitch and yaw of base_link in the world frame, in radians'),
        DeclareLaunchArgument(
            'initial_positions', default_value='0 -1.571 0 0 0 0',
            description='Initial angles of j1 to j6, in radians'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo /clock'),
    ]


def robot_settings(context):
    """Read the robot launch arguments and load the tool file, if one is given."""
    def value(name):
        return LaunchConfiguration(name).perform(context)

    tool_file = description.resolve_tool_file(value('tool'))
    return {
        'robot_name': value('robot_name'),
        'tool': description.load_tool(tool_file) if tool_file else None,
        'base_xyz': value('base_xyz'),
        'base_rpy': value('base_rpy'),
        'initial_positions': value('initial_positions'),
        'use_sim_time': value('use_sim_time').lower() in TRUE_VALUES,
    }


def gazebo_actions(gz_args=' -r -v 1 empty.sdf'):
    """Return the actions that start Gazebo Sim and bridge its /clock to ROS."""
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items())
    bridge_config = os.path.join(
        get_package_share_directory('fairino_sim'), 'config', 'gz_bridge.yaml')
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'])
    return [gz_sim, clock_bridge]
