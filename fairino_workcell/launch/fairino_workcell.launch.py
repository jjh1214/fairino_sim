"""
Simulate a FAIRINO work cell: the robot from fairino_sim plus the equipment of a cell file.

  ros2 launch fairino_workcell fairino_workcell.launch.py cell:=/path/to/cell.yaml

The cell file sets the robot model, base pose, initial joint angles and tool, and lists the
equipment. Edit the file and call /workcell/reload to update the scene without restarting.
With MoveIt (the default), workcell_executor runs recipe steps: see fairino_workcell.api.
"""

import os

from ament_index_python.packages import get_package_share_directory
from fairino_workcell.cell import load_cell
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TRUE_VALUES = ('true', '1', 'yes', 'on')


def _vector(values):
    return ' '.join(repr(float(v)) for v in values)


def launch_setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    cell_file = os.path.abspath(os.path.expanduser(value('cell')))
    cell = load_cell(cell_file)  # report problems in the cell file before anything starts
    robot = cell.robot
    use_moveit = value('use_moveit').lower() in TRUE_VALUES
    use_sim_time = value('use_sim_time').lower() in TRUE_VALUES

    sim_share = get_package_share_directory('fairino_sim')
    launch_file = 'fairino_gz_moveit2.launch.py' if use_moveit else 'fairino_gz.launch.py'
    arguments = {
        'robot_name': robot.model,
        'tool': robot.tool_file or 'none',
        'base_xyz': _vector(robot.base.xyz),
        'base_rpy': _vector(robot.base.rpy),
        'initial_positions': _vector(robot.initial_positions),
        'use_sim_time': value('use_sim_time'),
    }
    if use_moveit:
        arguments['use_rviz'] = value('use_rviz')
        arguments['rviz_config'] = os.path.join(
            get_package_share_directory('fairino_workcell'), 'config', 'workcell.rviz')

    robot_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', launch_file)),
        launch_arguments=arguments.items())
    manager = Node(
        package='fairino_workcell',
        executable='cell_manager',
        output='screen',
        parameters=[{'cell_file': cell_file, 'use_moveit': use_moveit,
                     'use_sim_time': use_sim_time,
                     # with MoveIt, workcell_executor publishes the TF of movable objects
                     'publish_movable_tf': not use_moveit}])
    actions = [robot_simulation, manager]
    if use_moveit:
        actions.append(Node(
            package='fairino_workcell',
            executable='workcell_executor',
            output='screen',
            parameters=[{'cell_file': cell_file, 'use_sim_time': use_sim_time}]))
    return actions


def generate_launch_description():
    example = os.path.join(get_package_share_directory('fairino_workcell'), 'examples',
                           'conveyor_cell', 'cell.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('cell', default_value=example, description='Cell file (YAML)'),
        DeclareLaunchArgument('use_moveit', default_value='true',
                              description='Plan with MoveIt 2 (false: joint position control)'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Start RViz (with use_moveit only)'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the Gazebo /clock'),
        OpaqueFunction(function=launch_setup),
    ])
