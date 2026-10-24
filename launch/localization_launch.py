"""Localize in a saved SLAM Toolbox pose graph, with optional simulation and RViz."""

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def _cleanup_parameters(context, parameters):
    parameters.cleanup()


def _launch_setup(context):
    project_dir = Path(__file__).resolve().parent.parent
    map_file = Path(LaunchConfiguration('map_file').perform(context)).expanduser().resolve()
    if map_file.suffix in ('.posegraph', '.data', '.pgm', '.yaml'):
        raise ValueError('map_file must be a filename prefix without an extension, e.g. maps/my_map')
    for suffix in ('.posegraph', '.data'):
        path = Path(str(map_file) + suffix)
        if not path.is_file() or path.stat().st_size == 0:
            raise FileNotFoundError(f'Required SLAM Toolbox map file is missing or empty: {path}')

    # Quote the path as a YAML string, including paths containing spaces or punctuation.
    context.launch_configurations['localization_map_file'] = json.dumps(str(map_file))
    parameters = ParameterFile(
        LaunchConfiguration('localization_params_file'), allow_substs=True,
    )
    expanded_parameters = parameters.evaluate(context)
    slam_share = Path(get_package_share_directory('slam_toolbox'))

    return [
        # Keep ParameterFile alive until shutdown, then remove its expanded temporary file.
        RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=_cleanup_parameters, kwargs={'parameters': parameters}),
        ])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(project_dir / 'launch' / 'sim_launch.py')),
            condition=IfCondition(LaunchConfiguration('start_simulation')),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'verbosity': LaunchConfiguration('verbosity'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(slam_share / 'launch' / 'localization_launch.py')),
            launch_arguments={
                'slam_params_file': str(expanded_parameters),
                'use_sim_time': 'true',
                'autostart': 'true',
                'use_lifecycle_manager': 'false',
            }.items(),
        ),
        Node(
            package='rviz2', executable='rviz2', name='localization_rviz',
            arguments=['-d', str(project_dir / 'rviz' / 'localization.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ]


def generate_launch_description():
    project_dir = Path(__file__).resolve().parent.parent
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_simulation', default_value='true', choices=['true', 'false'],
            description='Start Gazebo and bridges; false reuses an existing simulation',
        ),
        DeclareLaunchArgument(
            'gui', default_value='true', choices=['true', 'false'],
            description='Show the Gazebo GUI when starting the simulation',
        ),
        DeclareLaunchArgument(
            'verbosity', default_value='3', choices=['0', '1', '2', '3', '4'],
            description='Gazebo console verbosity',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true', choices=['true', 'false'],
            description='Start RViz with the known map and 2D Pose Estimate tool',
        ),
        DeclareLaunchArgument(
            'map_file', default_value=str(project_dir / 'maps' / 'my_map'),
            description='Saved map filename prefix without .posegraph or .data',
        ),
        DeclareLaunchArgument(
            'localization_params_file',
            default_value=str(project_dir / 'config' / 'slam_toolbox_localization.yaml'),
            description='Localization parameter template',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
