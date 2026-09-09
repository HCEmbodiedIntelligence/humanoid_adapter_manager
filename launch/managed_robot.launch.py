#!/usr/bin/env python3
"""Launch the core stack from a robot composition managed by the adapter manager."""

from pathlib import Path
import json
from ament_index_python.packages import get_package_share_directory
from humanoid_manager.runtime_state import acquire_deployment_lock, acquire_robot_run_lock, configuration_identity
from humanoid_manager.startup import bringup_command, default_plan

_LEASES = []

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, Shutdown, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from humanoid_manager.deployment import (
    DEFAULT_PLUGIN_ROOT,
    resolve_robot_deployment,
)


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
    gripper_environment = deployment.gripper_environment()
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
                str(resources["driver_params"]),
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
                str(resources["motion_params"]),
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
    if vendor:
        # The run lease is acquired before any vendor hardware can start.
        # Normal successful controller-spawner exits are not failures.
        actions[0:0] = [
            RegisterEventHandler(OnProcessExit(on_exit=lambda event, _context: (
                [Shutdown(reason=f'底层启动进程异常退出: {event.returncode}')]
                if event.returncode else []))),
            GroupAction(scoped=True, actions=[
                IncludeLaunchDescription(AnyLaunchDescriptionSource(vendor[2]),
                                         launch_arguments=plan['bringup']['arguments'].items()),
            ]),
        ]

    if deployment.gripper_class:
        actions.insert(
            1,
            Node(
                package="humanoid_driver_runtime",
                executable="humanoid_gripper_runtime_node",
                name="humanoid_gripper_runtime",
                output="screen",
                parameters=[
                    str(resources["gripper_params"]),
                    {
                        "plugin_class": deployment.gripper_class,
                        "plugin_xml_paths": [
                            str(path) for path in deployment.gripper_plugin_xml_paths
                        ],
                    },
                ],
                additional_env=gripper_environment,
                condition=IfCondition(start_gripper),
                on_exit=Shutdown(reason="humanoid gripper runtime exited"),
            ),
        )

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
    if deployment.gripper_class:
        expected_pairs.append(("start_gripper", "humanoid_gripper_runtime"))
    expected = [name for argument, name in expected_pairs
        if LaunchConfiguration(argument).perform(context).lower() in {"1", "true", "yes", "on"}]
    actions.append(Node(package="humanoid_manager", executable="configuration_status.py",
        name="humanoid_configuration_status", output="screen",
        parameters=[{"identity": json.dumps(identity), "expected_nodes": expected}]))
    return actions


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
