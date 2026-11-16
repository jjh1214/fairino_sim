import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution

import shutil

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="fairino5_v6",
        description="Robot name for xacro, controllers, MoveIt config"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use /clock from Gazebo if true"
    )

    robot_name   = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_param = {"use_sim_time": use_sim_time}

    def robot_state_publisher(context, *args, **kwargs):
        xacro_exe = shutil.which("xacro")
        if not xacro_exe:
            raise RuntimeError(
                "❌ xacro not found. sudo apt install ros-jazzy-xacro"
            )

        select_robot = context.perform_substitution(robot_name)
        xacro_file = os.path.join(
            get_package_share_directory('fairino_sim'),
            'config',
            'frcobot_gz_moveit2.urdf.xacro'
        )
        raw = os.popen(
            f'"{xacro_exe}" "{xacro_file}" '
            f'robot_name:={select_robot} pkg_name:={select_robot}_moveit2_config'
        ).read()

        package_path = get_package_share_directory('fairino_description')
        raw = raw.replace('package://fairino_description', f'file://{package_path}')
        raw = raw.replace('damping="0"', 'damping="0.1"')
        return [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': raw}]
            )
        ]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': ' -r -v 1 empty.sdf'}.items()
    )

    bridge_params = os.path.join(
        get_package_share_directory('fairino_sim'),
        'config',
        'gz_bridge.yaml'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   robot_name, '-allow_renaming', 'true'],
    )

    find_pkg_name = PythonExpression([
        TextSubstitution(text="'"),
        robot_name,
        TextSubstitution(text="_moveit2_config'")
    ])

    robot_controllers = PathJoinSubstitution([
        FindPackageShare(find_pkg_name),
        'config',
        'moveit_controllers.yaml',
    ])

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--param-file',
                   robot_controllers,
                   ],
        parameters=[use_sim_time_param],
    )

    fairino_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            PythonExpression([
                TextSubstitution(text="'"),
                robot_name,
                TextSubstitution(text="'"),
                TextSubstitution(text=".rsplit('_',1)[0] + '_controller'"),
            ]),
            '--param-file',
            robot_controllers,
        ],
        parameters=[use_sim_time_param],
    )

    def launch_moveit_and_rviz(context, *args, **kwargs):

        robot = context.perform_substitution(robot_name)
        pkg_name = context.perform_substitution(find_pkg_name)

        custom_joint_limits_path = os.path.join(
            get_package_share_directory("fairino_sim"),
            "config",
            "custom_joint_limits.yaml",
        )

        moveit_config = (
            MoveItConfigsBuilder(robot, package_name=pkg_name)
            .robot_description_semantic(file_path=f"config/{robot}_robot.srdf")
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .planning_pipelines(
                pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"]
            )
            .joint_limits(file_path=custom_joint_limits_path)
            .to_moveit_configs()
        )

        rviz_cfg = os.path.join(
            get_package_share_directory(pkg_name),
            'config',
            'moveit.rviz'
        )

        return [

            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[moveit_config.to_dict(), use_sim_time_param],
            ),

            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", rviz_cfg],
                parameters=[
                    moveit_config.robot_description_semantic,
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    use_sim_time_param,
                ],
            ),
        ]

    ld = LaunchDescription([
        robot_name_arg,
        use_sim_time_arg,

        gz_sim,
        ros_gz_bridge,
        gz_spawn_entity,

        RegisterEventHandler(
            OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[fairino_controller_spawner]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=fairino_controller_spawner,
                on_exit=[OpaqueFunction(function=launch_moveit_and_rviz)]
            )
        ),

        OpaqueFunction(function=robot_state_publisher),
    ])

    return ld