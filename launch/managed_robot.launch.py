#!/usr/bin/env python3
"""Launch the core stack from a robot composition managed by the adapter manager."""

from pathlib import Path
import json
import math
from typing import List
import yaml
from ament_index_python.packages import get_package_share_directory
from humanoid_manager.runtime_state import acquire_deployment_lock, acquire_robot_run_lock, configuration_identity
from humanoid_manager.startup import bringup_command, default_plan
from humanoid_manager.plugin_startup import startup_actions

_LEASES = []

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, Shutdown, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from humanoid_manager.deployment import (
    DEFAULT_PLUGIN_ROOT,
    DeploymentError,
    resolve_robot_deployment,
)


# These are the double declarations of the managed driver, gripper and motion
# runtimes. JSON numbers from the editor do not preserve 1.0 versus 1.
_DOUBLE_PARAMETERS = {
    'control_frequency_hz', 'diagnostic_frequency_hz', 'command_watchdog_ms',
    'input_stamp_max_age_s', 'input_stamp_future_tolerance_s', 'default_move_timeout_s',
    'move_j_position_tolerance_rad', 'stopped_velocity_tolerance_rad_s',
    'cartesian_position_tolerance_m', 'cartesian_orientation_tolerance_rad', 'stable_duration_s',
    'joint_max_velocity_rad_s', 'joint_max_acceleration_rad_s2', 'joint_max_jerk_rad_s3',
    'cartesian_max_linear_velocity_m_s', 'cartesian_max_linear_acceleration_m_s2',
    'cartesian_max_linear_jerk_m_s3', 'cartesian_max_angular_velocity_rad_s',
    'cartesian_max_angular_acceleration_rad_s2', 'cartesian_max_angular_jerk_rad_s3',
}
_DOUBLE_ARRAY_PARAMETERS = {
    'vendor_to_logical_scales', 'vendor_to_logical_offsets_rad', 'vendor_to_logical_offsets',
}


def _double(value, key):
    if type(value) not in (int, float):
        raise DeploymentError(f'{key} 必须是有限数值')
    try:
        number = float(value)
    except OverflowError as error:
        raise DeploymentError(f'{key} 必须是有限数值') from error
    if not math.isfinite(number):
        raise DeploymentError(f'{key} 必须是有限数值')
    return number


def _typed_parameters(parameters):
    """Restore declared runtime types without changing saved, checksummed files."""
    result = {}
    for key, value in parameters.items():
        if key in _DOUBLE_PARAMETERS:
            result[key] = ParameterValue(_double(value, key), value_type=float)
        elif key in _DOUBLE_ARRAY_PARAMETERS or key.startswith(('group_lower_limits.', 'group_upper_limits.')):
            if not isinstance(value, list):
                raise DeploymentError(f'{key} 必须是有限数值数组')
            result[key] = ParameterValue([_double(item, key) for item in value], value_type=List[float])
        elif isinstance(value, str):
            result[key] = ParameterValue(value, value_type=str)
        elif isinstance(value, list) and all(isinstance(item, str) for item in value):
            result[key] = ParameterValue(value, value_type=List[str])
        else:
            result[key] = value
    return result


def _motion_parameters(path):
    # The ROS YAML parser rejects mixed integer/double arrays before overrides
    # can apply. Read with Python and normalize before generating launch YAML.
    document = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    return _typed_parameters(document['humanoid_motion_control']['ros__parameters'])


def _launch_registered_robot(context):
    robot_id = LaunchConfiguration("robot_id").perform(context)
    if not robot_id:
        raise RuntimeError("robot_id is required")

    plugin_root = Path(LaunchConfiguration("plugin_root").perform(context)).resolve()
    _LEASES.append(acquire_robot_run_lock(plugin_root))
    lease = acquire_deployment_lock(plugin_root, shared=True)
    _LEASES.append(lease)
    deployment = resolve_robot_deployment(plugin_root, robot_id)
    identity = configuration_identity(plugin_root, robot_id)
    resources = deployment.resources
    driver_environment = deployment.environment()
    resource_environment = deployment.resource_environment()
    start_driver = LaunchConfiguration("start_driver")
    start_gripper = LaunchConfiguration("start_gripper")
    start_motion = LaunchConfiguration("start_motion")
    start_teleop = LaunchConfiguration("start_teleop")
    start_cameras = LaunchConfiguration("start_cameras")
    plan = default_plan(robot_id)
    plan['bringup'] = json.loads(LaunchConfiguration('bringup_json').perform(context))
    vendor = bringup_command(plan)

    actions = [
        Node(
            package="humanoid_driver_runtime",
            executable="humanoid_driver_runtime_node",
            name="humanoid_driver_runtime",
            output="screen",
            parameters=[
                _typed_parameters(deployment.driver_parameters),
                {
                    "plugin_class": deployment.driver_class,
                    "plugin_xml_paths": [
                        str(path) for path in deployment.driver_plugin_xml_paths
                    ],
                },
            ],
            additional_env=driver_environment,
            condition=IfCondition(start_driver),
            on_exit=Shutdown(reason="humanoid driver runtime exited"),
        ),
        Node(
            package="humanoid_motion_server",
            executable="humanoid_motion_control_node",
            name="humanoid_motion_control",
            output="screen",
            parameters=[
                _motion_parameters(resources["motion_params"]),
                {
                    "channel_config_file": str(resources["channel_config"]),
                    "sdk_config_file": str(resources["sdk_config"]),
                    "tool_config_file": str(resources["tool_config"]),
                    "urdf_file": str(resources["urdf"]),
                },
            ],
            additional_env=resource_environment,
            condition=IfCondition(start_motion),
            on_exit=Shutdown(reason="humanoid motion server exited"),
        ),
    ]
    vendor_actions = []
    if vendor:
        # The run lease is acquired before any vendor hardware can start.
        # Normal successful controller-spawner exits are not failures.
        vendor_actions = [
            RegisterEventHandler(OnProcessExit(on_exit=lambda event, _context: (
                [Shutdown(reason=f'底层启动进程异常退出: {event.returncode}')]
                if event.returncode else []))),
            GroupAction(scoped=True, actions=[
                IncludeLaunchDescription(AnyLaunchDescriptionSource(vendor[2]),
                                         launch_arguments=plan['bringup']['arguments'].items()),
            ]),
        ]

    for instance in deployment.gripper_instances:
        actions.insert(1, Node(
            package="humanoid_driver_runtime", executable="humanoid_gripper_runtime_node",
            name=instance.node_name, output="screen",
            parameters=[_typed_parameters(instance.parameters), {
                "plugin_class": instance.plugin_class,
                "plugin_xml_paths": [str(instance.plugin_xml)],
                "filter_unowned_commands": len(deployment.gripper_instances) > 1,
            }], additional_env=instance.environment(), condition=IfCondition(start_gripper),
            on_exit=Shutdown(reason=f"gripper instance {instance.instance_id} exited"),
        ))

    if "hc_teleop_config" in resources:
        actions.append(
            Node(
                package="hc_teleop_recv",
                executable="hc_teleop_recv_node",
                name="hc_teleop_recv",
                output="screen",
                parameters=[{"config_file": str(resources["hc_teleop_config"])}],
                additional_env=resource_environment,
                condition=IfCondition(start_teleop),
                on_exit=Shutdown(reason="teleoperation frontend exited"),
            )
        )
    elif LaunchConfiguration("start_teleop").perform(context).lower() in {
        "1",
        "true",
        "yes",
        "on",
    }:
        raise RuntimeError(
            "start_teleop is true but the deployed profile has no hc_teleop_config; select a model with hc_teleop_recv configuration"
        )

    camera_config = deployment.manifest_path.parent / "cameras.yaml"
    if camera_config.is_file():
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(Path(get_package_share_directory("humanoid_camera")) / "launch" / "multi_camera.launch.py")
                ),
                launch_arguments={"camera_config": str(camera_config)}.items(),
                condition=IfCondition(start_cameras),
            )
        )

    expected_pairs = [
        ("start_driver", "humanoid_driver_runtime"),
        ("start_motion", "humanoid_motion_control"),
        ("start_teleop", "hc_teleop_recv"),
    ]
    expected_pairs.extend(("start_gripper", instance.node_name)
                          for instance in deployment.gripper_instances)
    expected = [name for argument, name in expected_pairs
        if LaunchConfiguration(argument).perform(context).lower() in {"1", "true", "yes", "on"}]
    # An empty list needs an explicit ROS string-array type.  Without it ROS 2
    # launch normalizes [] to an untyped tuple before configuration_status.py
    # can declare the parameter.  Camera-only/no-hardware launches therefore
    # failed before any camera process could start.
    status_parameters = {"identity": json.dumps(identity)}
    if expected:
        status_parameters["expected_nodes"] = expected
    else:
        status_parameters["expected_nodes"] = ParameterValue([], value_type=List[str])
    actions.append(Node(package="humanoid_manager", executable="configuration_status.py",
        name="humanoid_configuration_status", output="screen",
        parameters=[status_parameters]))
    steps = []
    for enabled, entries, environment in (
        (start_driver, deployment.driver_startup, driver_environment),
        *((start_gripper, instance.startup, instance.environment())
          for instance in deployment.gripper_instances),
    ):
        if enabled.perform(context).lower() in {'1', 'true', 'yes', 'on'}:
            steps.extend((entry, environment) for entry in entries)
    return vendor_actions + startup_actions(steps, actions)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('bringup_json', default_value='{"package":"","launch_file":"","arguments":{}}',
                                  description='Optional vendor ROS launch supplied by the public bringup entry.'),
            DeclareLaunchArgument(
                "robot_id",
                description="Deployed robot-composition ID below plugin_root.",
            ),
            DeclareLaunchArgument(
                "plugin_root",
                default_value=str(DEFAULT_PLUGIN_ROOT),
                description="Adapter-manager deployment root.",
            ),
            DeclareLaunchArgument("start_driver", default_value="true"),
            DeclareLaunchArgument("start_gripper", default_value="true"),
            DeclareLaunchArgument("start_motion", default_value="true"),
            DeclareLaunchArgument("start_teleop", default_value="true"),
            DeclareLaunchArgument("start_cameras", default_value="true"),
            OpaqueFunction(function=_launch_registered_robot),
        ]
    )
