import os
from typing import List

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _append_env_path(current: str, new_path: str) -> str:
    paths: List[str] = [p for p in [new_path, current] if p]
    return os.pathsep.join(paths)


def generate_launch_description():
    package_share = get_package_share_directory('stewart_platform')
    package_prefix = get_package_prefix('stewart_platform')

    # Use your ArduPilot world here (WITHOUT the stewart model included inside)
    # Either the original runway world, or the edited one with the stewart block removed.
    world_file = os.path.join(
        os.environ.get('HOME', ''),
        'projects',
        'ardupilot_gazebo',
        'worlds',
        'iris_arducopter_runway.world'
    )

    # Stewart SDF and plugin path as before
    sdf_file = os.path.join(package_share, 'stewart_sdf_model', 'stewart_fractal.sdf')
    plugin_dir = os.path.join(package_prefix, 'lib')
    model_dir = os.path.join(package_share, 'stewart')

    gui = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Environment setup for Gazebo
    set_plugin_path = SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        _append_env_path(os.environ.get('GAZEBO_PLUGIN_PATH', ''), plugin_dir))

    set_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        _append_env_path(os.environ.get('GAZEBO_MODEL_PATH', ''), model_dir))

    disable_model_db = SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', '')

    # Launch Gazebo (classic) via gazebo_ros
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )),
        launch_arguments={
            'world': world_file,
            'verbose': verbose,
            'gui': gui
        }.items(),
    )

    # Explicitly spawn the Stewart platform model from SDF
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'stewart',
            '-file', sdf_file,
            '-x', '3', '-y', '0', '-z', '0'
        ],
        output='screen',
    )

    # Inverse Kinematics Node
    ik_node = Node(
        package='stewart_platform',
        executable='ik',
        name='inverse_kinematic',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Platfor pose controller Node
    platform_pose_pub_node = Node(
        package='stewart_platform',
        executable='controller_platform_ros2.py',
        name='platform_pose_controller',
        output='screen',
        parameters=[
            {'use_csv': False},
            {'csv_path': '/home/rycker/projects/ros2_ws/src/stewart_platform_learning/src/stewart_platform/D1H3_heave.csv'},
            {'x_amplitude': 0.0},
            {'y_amplitude': 0.0},
            {'command_rate': 20.0},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        set_plugin_path,
        set_model_path,
        disable_model_db,

        gazebo,
        spawn_entity,
        ik_node,
        platform_pose_pub_node,
    ])
