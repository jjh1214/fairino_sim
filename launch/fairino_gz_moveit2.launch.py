import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

import shutil

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="fairino5_v6",
        description="Robot name for selecting the xacro model"
    )
    robot_name = LaunchConfiguration("robot_name")

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    use_sim_time_param = {"use_sim_time": use_sim_time}

    def robot_state_publisher(context):
        xacro_exe = shutil.which("xacro")
        if not xacro_exe:
            raise RuntimeError("❌ The xacro executable was not found. Please make sure it is installed by running: sudo apt install ros-jazzy-xacro")

        select_robot = robot_name.perform(context)
        xacro_file = os.path.join(
            get_package_share_directory('fairino_sim'),
            'config',
            'frcobot_gz_moveit2.urdf.xacro'
        )

        robot_description_raw = os.popen(f'"{xacro_exe}" "{xacro_file}" robot_name:={select_robot}').read()

        package_path = get_package_share_directory('fairino_description')
        robot_description_raw = robot_description_raw.replace(
            'package://fairino_description',
            f'file://{package_path}',
        )

        robot_description_raw = robot_description_raw.replace(
            'damping="0"',
            f'damping="0.1"'
        )

        robot_description = {'robot_description': robot_description_raw}

        return [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[robot_description]
            )
        ]

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('fairino_sim'),
            'config',
            'frcobot_gz_moveit_controller2.yaml',
        ],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   robot_name, '-allow_renaming', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--param-file',
                   robot_controllers,
                   ],
        parameters=[use_sim_time_param],
    )

    fairino5_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fairino5_controller',
                   '--param-file',
                   robot_controllers,
                   ],
        parameters=[use_sim_time_param],
    )

    bridge_params = os.path.join(get_package_share_directory('fairino_sim'),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
    )

    moveit_config = (
        MoveItConfigsBuilder("fairino_cobot", package_name="fairino5_v6_moveit2_config")
        .robot_description_semantic(file_path="config/fairino5_v6_robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"]
        )
        .to_moveit_configs()
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('fairino5_v6_moveit2_config'), 'config', 'moveit.rviz']
    )

    moveit_spawner = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), use_sim_time_param],
    )

    rviz_spawner = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            use_sim_time_param,
        ],
    )

    ld = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 1 empty.sdf'])]),
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[fairino5_controller_spawner],
            )
        ),        

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=fairino5_controller_spawner,
                on_exit=[moveit_spawner, rviz_spawner],
            )
        ),

        robot_name_arg,
        ros_gz_bridge,
        gz_spawn_entity,

    ])

    ld.add_action(OpaqueFunction(function=robot_state_publisher))
    return ld