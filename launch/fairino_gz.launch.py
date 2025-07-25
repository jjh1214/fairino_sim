import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

import shutil

def generate_launch_description():

    def robot_state_publisher(context):
        xacro_exe = shutil.which("xacro")
        if not xacro_exe:
            raise RuntimeError("❌ The xacro executable was not found. Please make sure it is installed by running: sudo apt install ros-jazzy-xacro")

        xacro_file = os.path.join(
            get_package_share_directory('fairino_sim'),
            'config',
            'frcobot_gz.urdf.xacro'
        )

        robot_description_raw = os.popen(f'"{xacro_exe}" "{xacro_file}"').read()

        package_path = get_package_share_directory('fairino_description')
        robot_description_raw = robot_description_raw.replace(
            'package://fairino_description',
            f'file://{package_path}'
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
            'frcobot_controller2.yaml',
        ],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'fairino5_v6', '-allow_renaming', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--param-file',
                   robot_controllers,
                   ],
    )

    frcobot_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['frcobot_position_controller',
                   '--param-file',
                   robot_controllers,
                   ],
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
                on_exit=[frcobot_position_controller_spawner],
            )
        ),        

        ros_gz_bridge,
        gz_spawn_entity,

    ])

    ld.add_action(OpaqueFunction(function=robot_state_publisher))
    return ld