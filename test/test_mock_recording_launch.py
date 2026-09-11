"""The mock launch must reuse the applied camera file without camera parameter overrides."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from launch import LaunchContext


@pytest.mark.parametrize('enabled', [True, False])
def test_applied_cameras_pass_through_and_no_hardware_execution(tmp_path, monkeypatch, enabled):
    path = Path(__file__).resolve().parents[1] / 'launch/mock_robot_recording.launch.py'
    spec = importlib.util.spec_from_file_location('mock_recording_launch_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    robot_dir = tmp_path / 'robots/robot_test'
    robot_dir.mkdir(parents=True)
    camera_path = robot_dir / 'cameras.yaml'
    camera_path.write_text(yaml.safe_dump({'schema_version': 1, 'cameras': [{
        'id': 'head', 'enabled': enabled, 'device_type': 'd435', 'serial_no': '0012345',
        'width': 640, 'height': 480, 'fps': 30, 'color_exposure_us': 3900,
        'color_gain': 72, 'rgb_topic': '/existing_camera/rgb',
    }]}))
    original = camera_path.read_bytes()
    deployment = SimpleNamespace(manifest_path=robot_dir / 'manifest.yaml',
        driver_parameters={'joint_names': ['actual_left', 'actual_right'],
                           'platform_joint_state_topic': '/actual/state',
                           'platform_joint_command_topic': '/actual/command', 'control_frequency_hz': 80.},
        gripper_instances=[SimpleNamespace(parameters={'gripper_names': ['actual_gripper']})])
    monkeypatch.setattr(module, 'resolve_robot_deployment', lambda root, robot_id: deployment)
    monkeypatch.setattr(module, 'configuration_identity', lambda root, robot_id: {'revision': 'applied-revision'})
    monkeypatch.setattr(module, 'acquire_robot_run_lock', lambda root: None)
    monkeypatch.setattr(module, 'acquire_deployment_lock', lambda root, **kw: None)
    monkeypatch.setattr(module, 'Node', lambda **kw: {'node': kw})
    monkeypatch.setattr(module, 'IncludeLaunchDescription', lambda source, **kw: {'include': source, **kw})
    context = LaunchContext()
    context.launch_configurations.update(plugin_root=str(tmp_path), robot_id='robot_test', domain_id='199')
    if not enabled:
        with pytest.raises(RuntimeError, match='No enabled cameras'):
            module._launch(context)
        return
    actions = module._launch(context)
    includes = [x for x in actions if isinstance(x, dict) and 'include' in x]
    assert len(includes) == 1
    assert dict(includes[0]['launch_arguments']) == {'camera_config': str(camera_path)}
    assert camera_path.read_bytes() == original
    nodes = [x['node'] for x in actions if isinstance(x, dict) and 'node' in x]
    assert {x['executable'] for x in nodes} == {'mock_robot_feedback.py', 'configuration_status.py'}
    feedback = nodes[0]
    assert feedback['arguments'][:4] == ['--arm-joints', 'actual_left,actual_right', '--gripper-joints', 'actual_gripper']
    assert ('/hc_teleop/joint_cmd', '/actual/command') in feedback['remappings']
    assert '80.0' in feedback['arguments']
