"""Exercise launch output with the real ROS parser, without hardware drivers."""
import copy
import importlib.util
from pathlib import Path

import pytest
import yaml

from humanoid_manager.deployment import DeploymentError


@pytest.fixture
def managed_launch():
    path = Path(__file__).resolve().parents[1] / 'launch/managed_robot.launch.py'
    spec = importlib.util.spec_from_file_location('runtime_parameter_launch_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('values,types', [
    ({'vendor_to_logical_scales': [1, -1], 'vendor_to_logical_offsets_rad': [0, 0],
      'control_frequency_hz': 100, 'command_watchdog_ms': 250},
     {'vendor_to_logical_scales': 'DOUBLE_ARRAY', 'vendor_to_logical_offsets_rad': 'DOUBLE_ARRAY',
      'control_frequency_hz': 'DOUBLE', 'command_watchdog_ms': 'DOUBLE'}),
    ({'vendor_to_logical_scales': [1, 1], 'vendor_to_logical_offsets': [0, 0.04],
      'diagnostic_frequency_hz': 10},
     {'vendor_to_logical_scales': 'DOUBLE_ARRAY', 'vendor_to_logical_offsets': 'DOUBLE_ARRAY',
      'diagnostic_frequency_hz': 'DOUBLE'}),
    ({'group_lower_limits.arm': [0, -2.5], 'group_upper_limits.arm': [2.4, 0],
      'default_move_timeout_s': 60, 'joint_max_velocity_rad_s': 1,
      'feedback_max_age_ms': 100, 'servo_lease_ms': 100, 'test_pause_driver_feedback': False},
     {'group_lower_limits.arm': 'DOUBLE_ARRAY', 'group_upper_limits.arm': 'DOUBLE_ARRAY',
      'default_move_timeout_s': 'DOUBLE', 'joint_max_velocity_rad_s': 'DOUBLE',
      'feedback_max_age_ms': 'INTEGER', 'servo_lease_ms': 'INTEGER', 'test_pause_driver_feedback': 'BOOL'}),
])
def test_integer_json_values_reach_ros_with_declared_runtime_types(
        managed_launch, tmp_path, monkeypatch, values, types):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from launch import LaunchContext
    from launch_ros.utilities import evaluate_parameters, normalize_parameters

    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    original = copy.deepcopy(values)
    evaluated = evaluate_parameters(LaunchContext(), normalize_parameters([
        managed_launch._typed_parameters(values)]))[0]
    path = tmp_path / 'launch_parameters.yaml'
    path.write_text(yaml.safe_dump({'parameter_type_probe': {'ros__parameters': evaluated}}))
    context, node = Context(), None
    try:
        rclpy.init(args=[], context=context, domain_id=228)
        node = Node('parameter_type_probe', context=context, enable_rosout=False,
                    start_parameter_services=False, use_global_arguments=False,
                    cli_args=['--ros-args', '--params-file', str(path)])
        for key, expected in types.items():
            parameter = node.declare_parameter(key, getattr(Parameter.Type, expected))
            assert parameter.type_.name == expected
            assert parameter.value == values[key]
    finally:
        if node is not None:
            node.destroy_node()
        context.try_shutdown()
    assert values == original


@pytest.mark.parametrize('key,value', [
    ('control_frequency_hz', True), ('control_frequency_hz', '100'),
    ('control_frequency_hz', float('inf')),
    ('vendor_to_logical_scales', [True, 1]),
    ('vendor_to_logical_offsets_rad', ['0', 0]),
    ('group_lower_limits.arm', [0, float('nan')]),
])
def test_invalid_numeric_types_are_not_silently_coerced(managed_launch, key, value):
    with pytest.raises(DeploymentError, match=key):
        managed_launch._typed_parameters({key: value})
