"""Start the field simulation, topic bridges, and the LiDAR static TF."""

import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def _lidar_tf_arguments(robot_path):
    """Use the sensor pose in the SDF instead of maintaining a second copy."""
    model = ET.parse(robot_path).getroot().find('model')
    link = model.find("link[@name='base_link']")
    sensor = link.find("sensor[@name='rplidar_a2m8']")
    if sensor is None:
        raise ValueError(f'LiDAR sensor rplidar_a2m8 is missing from {robot_path}')

    pose = sensor.find('pose')
    if pose is not None and pose.get('relative_to', '') not in ('', link.get('name')):
        raise ValueError('The LiDAR pose must be expressed relative to base_link')
    values = pose.text.split() if pose is not None else ['0'] * 6
    if len(values) != 6 or not all(math.isfinite(float(value)) for value in values):
        raise ValueError('The LiDAR pose must contain six finite values: x y z roll pitch yaw')
    if pose is not None and (
        pose.get('degrees', 'false').lower() not in ('false', '0')
        or pose.get('rotation_format', 'euler_rpy') != 'euler_rpy'
    ):
        raise ValueError('The LiDAR pose must use Euler angles in radians')

    frame = sensor.findtext('gz_frame_id', '').strip()
    if not frame:
        raise ValueError('The LiDAR sensor must define gz_frame_id')

    arguments = []
    for name, value in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), values):
        arguments.extend([f'--{name}', value])
    return arguments + ['--frame-id', link.get('name'), '--child-frame-id', frame]


def _launch_setup(context):
    project_dir = Path(__file__).resolve().parent.parent
    resource_dir = project_dir / 'src'
    world_path = resource_dir / 'worlds' / '2026_nhk_field.sdf'
    robot_path = resource_dir / 'model' / 'robot' / '2026a_robot.sdf'
    bridge_path = project_dir / 'config' / 'ros_gz_bridge.yaml'
    for path in (world_path, robot_path, bridge_path):
        if not path.is_file():
            raise FileNotFoundError(f'Required simulation file is missing: {path}')

    lidar_tf_arguments = _lidar_tf_arguments(robot_path)
    resource_paths = [str(resource_dir)]
    existing_paths = context.environment.get('GZ_SIM_RESOURCE_PATH', '')
    if existing_paths:
        resource_paths.append(existing_paths)

    gazebo_command = [
        FindExecutable(name='gz'), 'sim', '-r', '-v',
        LaunchConfiguration('verbosity').perform(context),
    ]
    if LaunchConfiguration('gui').perform(context) == 'false':
        gazebo_command.extend(['-s', '--headless-rendering'])
    gazebo_command.append(str(world_path))

    return [
        ExecuteProcess(
            cmd=gazebo_command,
            name='gazebo',
            cwd=str(project_dir),
            additional_env={'GZ_SIM_RESOURCE_PATH': os.pathsep.join(resource_paths)},
            output='screen',
            on_exit=Shutdown(reason='Gazebo exited'),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='sim_bridge',
            parameters=[{'config_file': str(bridge_path), 'use_sim_time': True}],
            output='screen',
            on_exit=Shutdown(reason='Simulation bridge exited'),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=lidar_tf_arguments,
            parameters=[{'use_sim_time': True}],
            output='screen',
            on_exit=Shutdown(reason='LiDAR static TF publisher exited'),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true', choices=['true', 'false'],
            description='Start the Gazebo GUI; false uses headless rendering',
        ),
        DeclareLaunchArgument(
            'verbosity', default_value='3', choices=['0', '1', '2', '3', '4'],
            description='Gazebo console verbosity',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
