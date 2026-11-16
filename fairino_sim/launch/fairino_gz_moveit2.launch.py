"""
Gazebo simulation of a FAIRINO cobot planned and executed through MoveIt 2.

Gazebo, robot_state_publisher, move_group and RViz all use the same URDF and SRDF,
built by fairino_sim.description (world frame, flange, optional tool).
"""

import os

from ament_index_python.packages import get_package_share_directory
from fairino_sim import description, launch_common
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

# Joint damping used with the MoveIt joint trajectory controller in Gazebo.
JOINT_DAMPING = 0.1
PLANNING_PIPELINES = ['ompl', 'chomp', 'pilz_industrial_motion_planner', 'stomp']


def launch_setup(context):
    robot = launch_common.robot_settings(context)
    robot_name = robot['robot_name']
    description.check_model(robot_name, moveit=True)
    use_sim_time = {'use_sim_time': robot['use_sim_time']}
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() in (
        launch_common.TRUE_VALUES)

    moveit_package = description.moveit_config_package(robot_name)
    moveit_share = get_package_share_directory(moveit_package)
    controllers_file = os.path.join(moveit_share, 'config', 'ros2_controllers.yaml')
    joint_limits_file = os.path.join(
        get_package_share_directory('fairino_sim'), 'config', 'custom_joint_limits.yaml')

    urdf = description.build_urdf(
        robot_name, controllers_file,
        base_xyz=robot['base_xyz'], base_rpy=robot['base_rpy'],
        initial_positions=robot['initial_positions'], tool=robot['tool'],
        damping=JOINT_DAMPING)
    srdf = description.build_srdf(robot_name, robot['tool'])

    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name=moveit_package)
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=PLANNING_PIPELINES)
        .joint_limits(file_path=joint_limits_file)
        .to_moveit_configs()
    )
    # The MoveIt config package points at the plain fairino_description URDF. Replace it
    # with the simulated robot so that MoveIt plans with the same model Gazebo runs.
    moveit_config.robot_description = {'robot_description': urdf}
    moveit_config.robot_description_semantic = {'robot_description_semantic': srdf}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description, use_sim_time])
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', robot_name,
                   '-allow_renaming', 'true'])
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file', controllers_file],
        parameters=[use_sim_time])
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[description.trajectory_controller_name(robot_name),
                   '--param-file', controllers_file],
        parameters=[use_sim_time])

    moveit_nodes = [Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(), use_sim_time])]
    rviz_config = LaunchConfiguration('rviz_config').perform(context) or os.path.join(
        moveit_share, 'config', 'moveit.rviz')
    if use_rviz:
        moveit_nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', rviz_config],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                use_sim_time,
            ]))

    return [
        robot_state_publisher,
        gz_spawn_entity,
        RegisterEventHandler(OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner, arm_controller_spawner])),
        RegisterEventHandler(OnProcessExit(
            target_action=arm_controller_spawner, on_exit=moveit_nodes)),
    ]


def generate_launch_description():
    return LaunchDescription(
        launch_common.declare_robot_arguments()
        + [DeclareLaunchArgument('use_rviz', default_value='true',
                                 description='Start RViz with the MoveIt plugin'),
           DeclareLaunchArgument('rviz_config', default_value='',
                                 description='RViz config file (default: moveit.rviz of the '
                                             'MoveIt config package)')]
        + launch_common.gazebo_actions()
        + [OpaqueFunction(function=launch_setup)])
