"""Applied robot camera configuration plus platform mock feedback; no hardware execution."""
import hashlib
import json
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from humanoid_manager.deployment import DEFAULT_PLUGIN_ROOT, resolve_robot_deployment
from humanoid_manager.runtime_state import acquire_deployment_lock, acquire_robot_run_lock, configuration_identity

_LEASES = []


def _launch(context):
    root = Path(LaunchConfiguration('plugin_root').perform(context)).expanduser().resolve()
    robot_id = LaunchConfiguration('robot_id').perform(context)
    # Same leases and applied composition as the real managed launch.
    _LEASES.append(acquire_robot_run_lock(root))
    _LEASES.append(acquire_deployment_lock(root, shared=True))
    deployment = resolve_robot_deployment(root, robot_id)
    camera_config = deployment.manifest_path.parent / 'cameras.yaml'
    from humanoid_camera.configuration import validate_cameras
    document = yaml.safe_load(camera_config.read_text())
    if not isinstance(document, dict) or document.get('schema_version') != 1:
        raise RuntimeError('Applied camera configuration must have schema_version: 1')
    cameras = validate_cameras(document.get('cameras'))
    if not any(camera['enabled'] for camera in cameras):
        raise RuntimeError('No enabled cameras in the applied robot configuration; configure and apply cameras first')
    identity = {**configuration_identity(root, robot_id), 'execution_mode': 'mock_feedback'}
    params = deployment.driver_parameters
    arm_names = params['joint_names']
    gripper_names = [name for instance in deployment.gripper_instances for name in instance.parameters['gripper_names']]
    arm_state = params.get('platform_joint_state_topic', '/hc_teleop/joint_states')
    arm_command = params.get('platform_joint_command_topic', '/hc_teleop/joint_cmd')
    grip_states = {i.parameters.get('platform_gripper_state_topic', '/hc_teleop/gripper_states') for i in deployment.gripper_instances}
    grip_commands = {i.parameters.get('platform_gripper_command_topic', '/hc_teleop/gripper_commands') for i in deployment.gripper_instances}
    if len(grip_states) > 1 or len(grip_commands) > 1:
        raise RuntimeError('Mock feedback currently requires gripper instances to share platform topics')
    return [
        LogInfo(msg=f'Mock execution; applied robot={robot_id}; revision={identity["revision"]}'),
        LogInfo(msg=f'Camera config={camera_config}; sha256={hashlib.sha256(camera_config.read_bytes()).hexdigest()}'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('humanoid_camera')) / 'launch/multi_camera.launch.py')),
            launch_arguments={'camera_config': str(camera_config)}.items()),
        Node(package='humanoid_manager', executable='mock_robot_feedback.py', output='screen',
             arguments=['--arm-joints', ','.join(arm_names), '--gripper-joints', ','.join(gripper_names),
                        '--domain-id', LaunchConfiguration('domain_id'),
                        '--arm-hz', str(params.get('control_frequency_hz', 100.)),
                        '--gripper-hz', str(deployment.gripper_instances[0].parameters.get('control_frequency_hz', 50.)) if deployment.gripper_instances else '50'],
             remappings=[('/hc_teleop/joint_states', arm_state), ('/hc_teleop/joint_cmd', arm_command),
                         ('/hc_teleop/gripper_states', next(iter(grip_states), '/hc_teleop/gripper_states')),
                         ('/hc_teleop/gripper_commands', next(iter(grip_commands), '/hc_teleop/gripper_commands'))],
             on_exit=Shutdown(reason='mock feedback stopped')),
        Node(package='humanoid_manager', executable='configuration_status.py', output='screen',
             parameters=[{'identity': json.dumps(identity), 'expected_nodes': ['mock_robot_feedback']}]),
    ]


def generate_launch_description():
    import os
    from launch.actions import SetEnvironmentVariable
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', description='Applied robot composition ID from the manager'),
        DeclareLaunchArgument('plugin_root', default_value=str(DEFAULT_PLUGIN_ROOT)),
        DeclareLaunchArgument('domain_id', default_value=os.environ.get('ROS_DOMAIN_ID', '199')),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        OpaqueFunction(function=_launch),
    ])
