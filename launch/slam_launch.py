"""Run online SLAM, optionally together with the simulation and RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project_dir = Path(__file__).resolve().parent.parent
    slam_share = Path(get_package_share_directory('slam_toolbox'))

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
            description='Start RViz with the online SLAM displays',
        ),
        DeclareLaunchArgument(
            'slam_params_file', default_value=str(project_dir / 'config' / 'slam_toolbox.yaml'),
            description='SLAM Toolbox mapping parameters',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(project_dir / 'launch' / 'sim_launch.py')),
            condition=IfCondition(LaunchConfiguration('start_simulation')),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'verbosity': LaunchConfiguration('verbosity'),
            }.items(),
        ),
        # The official launch configures and activates the lifecycle node.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(slam_share / 'launch' / 'online_async_launch.py')),
            launch_arguments={
                'slam_params_file': LaunchConfiguration('slam_params_file'),
                'use_sim_time': 'true',
                'autostart': 'true',
                'use_lifecycle_manager': 'false',
            }.items(),
        ),
        Node(
            package='rviz2', executable='rviz2', name='slam_rviz',
            arguments=['-d', str(project_dir / 'rviz' / 'slam.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
