"""Plugin selection, initialization ordering, and controller restart behavior."""
import asyncio
import copy
import json
import os
from pathlib import Path
import signal
import sys
from types import SimpleNamespace

import pytest
import yaml

from humanoid_manager.configuration import ConfigurationManager
from humanoid_manager.deployment import DeploymentError, deploy_archive, pack_directory, resolve_robot_deployment
from humanoid_manager.plugin_startup import startup_actions, validate_startup
from humanoid_manager.ros2_controllers import ensure_controllers
from test_deployment_plugins import _hardware_tree, _gripper_tree, _model_tree, _write_yaml


def node_step(package='fake_gripper', executable='initialize', wait=True):
    return dict(kind='node', package=package, executable=executable,
                arguments=[], wait_for_exit=wait)


@pytest.mark.parametrize('invalid', [
    {}, [None], [{'kind': []}], [dict(node_step(), executable='../script')],
    [dict(node_step(), arguments=['$(exec something)'])],
    [dict(node_step(), arguments=[True])], [dict(node_step(), wait_for_exit='true')],
    [dict(node_step(), shell=True)],
    [dict(kind='launch', package='vendor', launch_file='robot.launch.py', arguments={'fake': True})],
    [dict(kind='launch', package='vendor', launch_file='robot.launch.py', arguments={'namespace': ''})],
])
def test_invalid_startup_rejected_during_bundle_validation(tmp_path, invalid):
    tree = _gripper_tree(tmp_path / 'gripper')
    manifest = yaml.safe_load((tree / 'manifest.yaml').read_text())
    manifest['startup'] = invalid
    _write_yaml(tree / 'manifest.yaml', manifest)
    with pytest.raises(DeploymentError, match='startup'):
        pack_directory(tree, tmp_path / 'invalid.zip')


def test_switching_and_removing_gripper_preserves_only_selected_startup(tmp_path):
    root = tmp_path / 'plugins'
    driver = _hardware_tree(tmp_path / 'driver')
    manifest = yaml.safe_load((driver / 'manifest.yaml').read_text())
    hardware_step = dict(kind='launch', package='some_vendor',
                         launch_file='device.launch.py', arguments={'port': '/dev/ttyUSB1'})
    manifest['startup'] = [hardware_step]
    _write_yaml(driver / 'manifest.yaml', manifest)
    for name, tree in [('driver', driver), ('model', _model_tree(tmp_path / 'model'))]:
        deploy_archive(pack_directory(tree, tmp_path / f'{name}.zip'), root)
    for plugin_id in ('tool_a', 'tool_b'):
        tree = _gripper_tree(tmp_path / plugin_id)
        manifest = yaml.safe_load((tree / 'manifest.yaml').read_text())
        manifest.update(plugin_id=plugin_id, startup=[node_step(executable=plugin_id)])
        _write_yaml(tree / 'manifest.yaml', manifest)
        deploy_archive(pack_directory(tree, tmp_path / f'{plugin_id}.zip'), root)
    manager = ConfigurationManager(root, tmp_path / 'state')
    robot = manager.create('lab', 'Any robot', driver_id='fake_driver',
                           model_id='test_model', gripper_id='tool_a')
    manager.apply('lab', robot['latest'], robot['etag'])
    resolved = resolve_robot_deployment(root, 'lab')
    assert resolved.driver_startup == (hardware_step,)
    assert resolved.gripper_startup == (node_step(executable='tool_a'),)

    document = copy.deepcopy(robot['draft'])
    document['gripper_driver'] = {'plugin_id': 'tool_b', 'name': 'Another gripper'}
    document['resources']['gripper_params'] = manager.catalog()['gripper_drivers']['tool_b']['template']
    robot = manager.save('lab', document, robot['etag'])
    manager.apply('lab', robot['latest'], robot['etag'])
    resolved = resolve_robot_deployment(root, 'lab')
    assert resolved.driver_startup == (hardware_step,)
    assert resolved.gripper_startup == (node_step(executable='tool_b'),)

    document = copy.deepcopy(robot['draft'])
    document['gripper_driver'] = None
    del document['resources']['gripper_params']
    robot = manager.save('lab', document, robot['etag'])
    manager.apply('lab', robot['latest'], robot['etag'])
    assert resolve_robot_deployment(root, 'lab').gripper_startup == ()


class Controllers:
    def __init__(self, states, fail=''):
        self.states = dict(states)
        self.calls = []
        self.fail = fail

    def list_controllers(self, node, manager, timeout):
        assert manager == '/some_robot/controller_manager'
        assert timeout == 12
        return SimpleNamespace(controller=[SimpleNamespace(name=k, state=v) for k, v in self.states.items()])

    def load_controller(self, node, manager, name, timeout):
        self.calls.append(('load', name))
        self.states[name] = 'unconfigured'
        return SimpleNamespace(ok=self.fail != 'load')

    def configure_controller(self, node, manager, name, timeout):
        self.calls.append(('configure', name))
        self.states[name] = 'inactive'
        return SimpleNamespace(ok=self.fail != 'configure')

    def switch_controllers(self, node, manager, deactivate, activate, strict, asap, timeout):
        assert deactivate == [] and strict
        self.calls.append(('activate', tuple(activate)))
        if self.fail != 'switch':
            self.states.update({name: 'active' for name in activate})
        return SimpleNamespace(ok=self.fail != 'switch')


@pytest.mark.parametrize('state', [None, 'unconfigured', 'inactive', 'active'])
def test_controller_startup_is_idempotent_and_preserves_other_controllers(state):
    initial = {'tool_b': 'active', 'arm': 'active'}
    if state is not None:
        initial['tool_a'] = state
    api = Controllers(initial)
    ensure_controllers(api, None, '/some_robot/controller_manager', ['tool_a', 'tool_b'], 12)
    assert api.states == {'tool_a': 'active', 'tool_b': 'active', 'arm': 'active'}
    assert all('tool_b' not in str(call) and 'arm' not in str(call) for call in api.calls)
    before = list(api.calls)
    ensure_controllers(api, None, '/some_robot/controller_manager', ['tool_a', 'tool_b'], 12)
    assert api.calls == before


@pytest.mark.parametrize('failure', ['load', 'configure', 'switch'])
def test_controller_failures_abort_initialization(failure):
    api = Controllers({}, fail=failure)
    with pytest.raises(RuntimeError, match='Failed'):
        ensure_controllers(api, None, '/some_robot/controller_manager', ['tool_a'], 12)


@pytest.mark.parametrize('failure', [False, True])
def test_real_launch_waits_for_initialization_and_owns_service_processes(tmp_path, failure):
    prefix = tmp_path / 'prefix'
    marker = prefix / 'share/ament_index/resource_index/packages/test_device'
    marker.parent.mkdir(parents=True)
    marker.touch()
    binaries = prefix / 'lib/test_device'
    binaries.mkdir(parents=True)
    service = binaries / 'service'
    service.write_text(f'#!{sys.executable}\n' + '''
import os, signal, sys
from pathlib import Path
root = Path(os.environ['TEST_STARTUP_ROOT'])
def stop(*args):
    (root / 'stopped').touch()
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
(root / 'pid').write_text(str(os.getpid()))
while True: signal.pause()
''')
    service.chmod(0o755)
    initializer = binaries / 'initialize'
    initializer.write_text(f'#!{sys.executable}\n' + '''
import os, sys, time
from pathlib import Path
root = Path(os.environ['TEST_STARTUP_ROOT'])
for _ in range(500):
    if (root / 'pid').exists(): break
    time.sleep(.01)
else: sys.exit(9)
(root / 'initialized').touch()
sys.exit(int(os.environ['TEST_STARTUP_FAILURE']))
''')
    initializer.chmod(0o755)
    vendor = prefix / 'share/test_device/launch/device.launch.py'
    vendor.parent.mkdir(parents=True)
    vendor.write_text('''
from launch import LaunchDescription
from launch.actions import ExecuteProcess
def generate_launch_description():
    return LaunchDescription([ExecuteProcess(cmd=[%r])])
''' % str(service))
    steps = [dict(kind='launch', package='test_device', launch_file='device.launch.py', arguments={}),
             node_step('test_device')]
    entry = tmp_path / 'test.launch.py'
    entry.write_text('''
import json, os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from humanoid_manager.plugin_startup import startup_actions
def runtime(context):
    root = Path(os.environ['TEST_STARTUP_ROOT'])
    assert (root / 'initialized').exists()
    (root / 'runtime').touch()
    return []
def generate_launch_description():
    steps = json.loads(%r)
    return LaunchDescription(startup_actions([(step, {}) for step in steps], [OpaqueFunction(function=runtime)]))
''' % json.dumps(steps))

    async def check():
        log_path = tmp_path / 'launch.log'
        with log_path.open('w') as log:
            process = await asyncio.create_subprocess_exec('ros2', 'launch', str(entry),
                env={**os.environ, 'AMENT_PREFIX_PATH': str(prefix) + ':' + os.environ.get('AMENT_PREFIX_PATH', ''),
                     'TEST_STARTUP_ROOT': str(tmp_path), 'TEST_STARTUP_FAILURE': str(int(failure)),
                     'ROS_LOG_DIR': str(tmp_path / 'logs')},
                stdout=log, stderr=log, start_new_session=True)
            try:
                if failure:
                    await asyncio.wait_for(process.wait(), 15)
                    assert not (tmp_path / 'runtime').exists(), log_path.read_text()
                    assert '插件启动进程退出' in log_path.read_text()
                else:
                    for _ in range(150):
                        if (tmp_path / 'runtime').exists(): break
                        if process.returncode is not None: pytest.fail(log_path.read_text())
                        await asyncio.sleep(.1)
                    else: pytest.fail(log_path.read_text())
                    assert process.returncode is None
                    process.send_signal(signal.SIGINT)
                    await asyncio.wait_for(process.wait(), 15)
                assert (tmp_path / 'stopped').exists(), log_path.read_text()
                with pytest.raises(ProcessLookupError):
                    os.kill(int((tmp_path / 'pid').read_text()), 0)
            finally:
                if process.returncode is None:
                    os.killpg(process.pid, signal.SIGKILL)
                    await process.wait()
    asyncio.run(check())


def test_missing_dependency_preflights_even_after_waiting_step(tmp_path):
    marker = tmp_path / 'share/ament_index/resource_index/packages/test'
    marker.parent.mkdir(parents=True)
    marker.touch()
    executable = tmp_path / 'lib/test/initialize'
    executable.parent.mkdir(parents=True)
    executable.write_text('#!/bin/sh\nexit 0\n')
    executable.chmod(0o755)
    environment = {'AMENT_PREFIX_PATH': str(tmp_path)}
    with pytest.raises(DeploymentError, match='missing'):
        startup_actions([(node_step('test'), environment), (node_step('missing'), environment)], [])


@pytest.mark.parametrize('driver_enabled,gripper_enabled', [('true', 'true'), ('false', 'true'), ('true', 'false'), ('false', 'false')])
def test_managed_launch_uses_only_enabled_plugin_steps(monkeypatch, tmp_path, driver_enabled, gripper_enabled):
    import importlib.util
    from launch import LaunchContext

    path = Path(__file__).resolve().parents[1] / 'launch/managed_robot.launch.py'
    spec = importlib.util.spec_from_file_location('managed_startup_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    deployment = SimpleNamespace(
        resources={name: tmp_path / (name + '.yaml') for name in (
            'driver_params', 'gripper_params', 'motion_params', 'channel_config', 'sdk_config', 'tool_config', 'urdf')},
        driver_class='test/Arm', gripper_class='test/Gripper', driver_parameters={},
        gripper_instances=(SimpleNamespace(node_name='humanoid_gripper_runtime', instance_id='default',
            plugin_class='test/Gripper', parameters={}, plugin_xml=tmp_path / 'gripper.xml',
            environment=lambda: {'DEVICE': 'tool'}, startup=(node_step('tool_vendor'),)),),
        driver_plugin_xml_paths=[], gripper_plugin_xml_paths=[],
        manifest_path=tmp_path / 'manifest.yaml',
        driver_startup=(node_step('arm_vendor'),), gripper_startup=(node_step('tool_vendor'),),
        environment=lambda: {'DEVICE': 'arm'}, gripper_environment=lambda: {'DEVICE': 'tool'},
        resource_environment=lambda: {})
    monkeypatch.setattr(module, 'resolve_robot_deployment', lambda *args: deployment)
    monkeypatch.setattr(module, 'acquire_robot_run_lock', lambda *args: None)
    monkeypatch.setattr(module, 'acquire_deployment_lock', lambda *args, **kwargs: None)
    monkeypatch.setattr(module, 'configuration_identity', lambda *args: {})
    selected = []
    def capture(steps, continuation):
        selected.extend(steps)
        return continuation
    monkeypatch.setattr(module, 'startup_actions', capture)
    context = LaunchContext()
    context.launch_configurations.update(
        robot_id='any_robot', plugin_root=str(tmp_path), start_driver=driver_enabled,
        start_gripper=gripper_enabled, start_motion='false', start_teleop='false',
        start_cameras='false', bringup_json='{"package":"","launch_file":"","arguments":{}}')
    module._launch_registered_robot(context)
    expected = []
    if driver_enabled == 'true': expected.append((node_step('arm_vendor'), {'DEVICE': 'arm'}))
    if gripper_enabled == 'true': expected.append((node_step('tool_vendor'), {'DEVICE': 'tool'}))
    assert selected == expected


def test_resolved_ros_parameters_preserve_empty_lists_and_literal_names():
    import importlib.util
    from launch import LaunchContext
    from launch_ros.utilities import evaluate_parameters, normalize_parameters
    path = Path(__file__).resolve().parents[1] / 'launch/managed_robot.launch.py'
    spec = importlib.util.spec_from_file_location('managed_parameter_types', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    parameters = {'plugin_parameters': [], 'vendor_gripper_names': ['on', '007'],
                  'namespace': 'false', 'control_frequency_hz': 50.0}
    evaluated = evaluate_parameters(LaunchContext(), normalize_parameters([module._typed_parameters(parameters)]))
    assert evaluated[0] == parameters
