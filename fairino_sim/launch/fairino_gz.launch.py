"""
Gazebo simulation of a FAIRINO cobot driven by a joint position controller.

Move the robot with:
  ros2 topic pub -1 /frcobot_position_controller/commands std_msgs/msg/Float64MultiArray
  "{data: [0.0, -1.571, 0.0, 0.0, 0.0, 0.0]}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from fairino_sim import description, launch_common
from launch import LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

# Joint damping that keeps the position controller stable in Gazebo.
JOINT_DAMPING = 300


def launch_setup(context):
    robot = launch_common.robot_settings(context)
    robot_name = robot['robot_name']
    description.check_model(robot_name)
    use_sim_time = {'use_sim_time': robot['use_sim_time']}

    controllers_file = os.path.join(
        get_package_share_directory('fairino_sim'), 'config', 'frcobot_controller2.yaml')
    urdf = description.build_urdf(
        robot_name, controllers_file,
        base_xyz=robot['base_xyz'], base_rpy=robot['base_rpy'],
        initial_positions=robot['initial_positions'], tool=robot['tool'],
        damping=JOINT_DAMPING)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf}, use_sim_time])
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', robot_name,
                   '-allow_renaming', 'true'])
    spawners = [
        Node(package='controller_manager',
             executable='spawner',
             arguments=[controller, '--param-file', controllers_file],
             parameters=[use_sim_time])
        for controller in ('joint_state_broadcaster', 'frcobot_position_controller')
    ]
    if robot['tool'] is not None and robot['tool'].fingers is not None:
        # move the fingers with a trajectory on /gripper_controller/joint_trajectory
        spawners.append(Node(
            package='controller_manager',
            executable='spawner',
            arguments=[description.GRIPPER_CONTROLLER,
                       '--param-file', description.gripper_controllers_file()],
            parameters=[use_sim_time]))
    return [
        robot_state_publisher,
        gz_spawn_entity,
        RegisterEventHandler(OnProcessExit(target_action=gz_spawn_entity, on_exit=spawners)),
    ]


def generate_launch_description():
    return LaunchDescription(
        launch_common.declare_robot_arguments()
        + launch_common.gazebo_actions()
        + [OpaqueFunction(function=launch_setup)])
