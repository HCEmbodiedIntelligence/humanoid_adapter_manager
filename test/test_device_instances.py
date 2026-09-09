"""Regression coverage for device-independent deployment and editable instances."""
import copy
import importlib.util
import os
from pathlib import Path
import stat
import subprocess
import sys

import pytest
import yaml

from humanoid_manager.configuration import ConfigurationManager, resolve_gripper_test, validate_cameras
from humanoid_manager.deployment import (DeploymentError, _safe_extract, deploy_archive,
    pack_directory, resolve_robot_deployment, validate_tree, write_checksums)
from humanoid_manager.plugin_metadata import settings_from_manifest, validate_parameter_schema
from humanoid_manager.plugin_startup import startup_command
from humanoid_manager.runtime_state import configuration_identity
from test_deployment_plugins import _hardware_tree, _model_tree, _gripper_tree, _write_yaml


def gripper(root, plugin_id, joint):
    tree = _gripper_tree(root)
    manifest = yaml.safe_load((tree / 'manifest.yaml').read_text())
    manifest.update(plugin_id=plugin_id, instance_parameters={'namespace': '/' + plugin_id},
        startup=[{'kind': 'node', 'package': 'fake_gripper', 'executable': 'initialize',
                  'arguments': ['${namespace}'], 'wait_for_exit': True}],
        parameter_schema={'type': 'object', 'properties': {'port': {'type': 'string'}},
                          'required': ['port'], 'additionalProperties': False},
        capabilities={'grippers': {joint: {'open_position': .03, 'closed_position': .01}}})
    # A different exported class stands for an independent vendor implementation.
    original = manifest['plugin_class']
    manifest['plugin_class'] = plugin_id + '/Driver'
    xml = tree / manifest['plugin_xml']
    xml.write_text(xml.read_text().replace(original, manifest['plugin_class']))
    config_path = tree / manifest['resources']['gripper_params']
    config = yaml.safe_load(config_path.read_text())
    params = config['humanoid_gripper_runtime']['ros__parameters']
    params.update(plugin_class=manifest['plugin_class'], gripper_names=[joint], vendor_gripper_names=[joint],
                  position_units=['m'], vendor_to_logical_scales=[1.], vendor_to_logical_offsets=[0.],
                  platform_gripper_command_topic='/tools/commands', platform_gripper_state_topic='/tools/states',
                  plugin_parameters=['port=${namespace}/device'])
    executable = tree / 'prefix/lib/fake_gripper/initialize'
    executable.parent.mkdir(parents=True)
    executable.write_text('#!/bin/sh\nexit 0\n')
    executable.chmod(0o755)
    _write_yaml(config_path, config)
    _write_yaml(tree / 'manifest.yaml', manifest)
    return tree


@pytest.fixture
def devices(tmp_path):
    root = tmp_path / 'plugins'
    for name, tree in [('driver', _hardware_tree(tmp_path / 'driver')), ('model', _model_tree(tmp_path / 'model')),
                       ('one', gripper(tmp_path / 'one', 'vendor_a', 'tool_a')),
                       ('two', gripper(tmp_path / 'two', 'vendor_b', 'tool_b'))]:
        deploy_archive(pack_directory(tree, tmp_path / f'{name}.zip'), root)
    return ConfigurationManager(root, tmp_path / 'state')


def test_two_vendors_roundtrip_edit_remove_restore_and_copy(devices, tmp_path):
    manager = devices
    robot = manager.create('lab', 'Robot', driver_id='fake_driver', model_id='test_model',
                           gripper_ids={'clamp': 'vendor_a', 'hand': 'vendor_b'})
    document = copy.deepcopy(robot['draft'])
    assert document['gripper_driver'] is None
    assert len(document['gripper_instances']) == 2
    document['gripper_instances'][1]['settings']['instance_parameters']['namespace'] = '/changed'
    saved = manager.save('lab', document, robot['etag'])
    manager.apply('lab', saved['latest'], saved['etag'])
    resolved = resolve_robot_deployment(manager.plugin_root, 'lab')
    assert [item.plugin_class for item in resolved.gripper_instances] == ['vendor_a/Driver', 'vendor_b/Driver']
    assert resolved.gripper_instances[1].parameters['plugin_parameters'] == ['port=/changed/device']
    assert resolved.gripper_instances[1].startup[0]['arguments'] == ['/changed']
    command = resolve_gripper_test(saved['saved'], 'tool_b', 'open')
    assert command['position'] == .03
    assert command['runtime_node'] == 'humanoid_gripper_runtime_hand'
    before = configuration_identity(manager.plugin_root, 'lab')['fingerprint']
    complete = tmp_path / 'whole.zip'
    manager.export('lab', saved['latest'], complete)
    imported = manager.import_workspace(complete, 'copy', 'Copied')
    manager.apply('copy', imported['latest'], imported['etag'])
    assert resolve_robot_deployment(manager.plugin_root, 'copy').gripper_instances[1].startup[0]['arguments'] == ['/changed']
    cloned = manager.create('clone', 'Cloned', source_workspace='lab')
    assert len(cloned['draft']['gripper_instances']) == 2
    doc = copy.deepcopy(saved['draft'])
    doc['gripper_instances'].pop()
    removed = manager.save('lab', doc, saved['etag'])
    manager.apply('lab', removed['latest'], removed['etag'])
    assert not (manager.plugin_root / 'gripper_drivers/lab.gripper.hand').exists()
    assert len(resolve_robot_deployment(manager.plugin_root, 'lab').gripper_instances) == 1
    assert configuration_identity(manager.plugin_root, 'lab')['fingerprint'] != before
    restored = manager.restore('lab', saved['latest'], removed['etag'])
    restored = manager.save('lab', restored['draft'], restored['etag'])
    manager.apply('lab', restored['latest'], restored['etag'])
    assert len(resolve_robot_deployment(manager.plugin_root, 'lab').gripper_instances) == 2


def test_conversion_of_unapplied_legacy_snapshot(devices):
    robot = devices.create('legacy', 'Legacy', driver_id='fake_driver', model_id='test_model', gripper_id='vendor_a')
    doc = copy.deepcopy(robot['draft'])
    doc['gripper_instances'] = [{'instance_id': 'default', **doc['gripper_driver'],
        'parameters': doc['resources'].pop('gripper_params'), 'settings': doc['plugin_settings'].pop('gripper')}]
    doc['gripper_driver'] = None
    saved = devices.save('legacy', doc, robot['etag'])
    devices.apply('legacy', saved['latest'], saved['etag'])
    assert resolve_robot_deployment(devices.plugin_root, 'legacy').gripper_instances[0].node_name == 'humanoid_gripper_runtime'


def test_instances_reject_duplicate_logical_names_without_changing_revision(devices):
    robot = devices.create('lab', 'Robot', driver_id='fake_driver', model_id='test_model', gripper_ids={'a': 'vendor_a'})
    doc = copy.deepcopy(robot['draft'])
    second = copy.deepcopy(doc['gripper_instances'][0]); second['instance_id'] = 'b'
    doc['gripper_instances'].append(second)
    with pytest.raises(DeploymentError, match='unique across instances'):
        devices.save('lab', doc, robot['etag'])
    assert devices.get('lab')['latest'] == robot['latest']


def test_bundled_initializer_keeps_only_normal_executable_bits(tmp_path):
    tree = gripper(tmp_path / 'plugin', 'vendor', 'tool')
    script = tree / 'prefix/lib/fake_gripper/initialize'
    script.chmod(0o6755)
    packed = pack_directory(tree, tmp_path / 'device.zip')
    destination = deploy_archive(packed, tmp_path / 'installed')
    executable = destination / 'prefix/lib/fake_gripper/initialize'
    assert stat.S_IMODE(executable.stat().st_mode) == 0o755
    manifest = validate_tree(destination)
    from humanoid_manager.plugin_metadata import resolved_document
    command = startup_command(resolved_document(manifest['startup'][0], manifest),
                              {'AMENT_PREFIX_PATH': str(destination / 'prefix')})
    subprocess.run(command, check=True)
    assert stat.S_IMODE((destination / 'manifest.yaml').stat().st_mode) == 0o644


@pytest.mark.parametrize('kind', ['prismatic', 'fixed', 'floating', 'planar'])
def test_model_rejects_unsupported_controlled_joints(tmp_path, kind):
    tree = _model_tree(tmp_path / 'model')
    path = tree / 'resources/robot.urdf'
    path.write_text(path.read_text().replace('type="revolute"', f'type="{kind}"', 1))
    with pytest.raises(DeploymentError, match='unsupported controlled joint'):
        pack_directory(tree, tmp_path / 'model.zip')


def test_uncontrolled_prismatic_joint_is_allowed(tmp_path):
    tree = _model_tree(tmp_path / 'model')
    path = tree / 'resources/robot.urdf'
    path.write_text(path.read_text().replace('</robot>', '''<link name="passive"/>
        <joint name="passive_slide" type="prismatic"><parent link="base"/><child link="passive"/>
        <limit lower="0" upper="0.1" effort="1" velocity="1"/></joint></robot>'''))
    pack_directory(tree, tmp_path / 'model.zip')


def test_unknown_vendor_uses_own_schema_without_class_dispatch():
    manifest = {'plugin_class': 'new_vendor/Claw', 'parameter_schema': {
        'type': 'object', 'properties': {'baud': {'type': 'integer', 'minimum': 1}},
        'required': ['baud'], 'additionalProperties': False}}
    validate_parameter_schema(manifest, {'baud': '115200'})
    with pytest.raises(DeploymentError, match='plugin parameter schema'):
        validate_parameter_schema(manifest, {'baud': '-1'})
    with pytest.raises(DeploymentError, match='plugin parameter schema'):
        validate_parameter_schema(manifest, {'baud': '115200', 'old_vendor_field': 'x'})


def test_plugin_schema_supports_boolean_schemas():
    validate_parameter_schema({'parameter_schema': True}, {'port': '/dev/device'})
    validate_parameter_schema({'parameter_schema': {'properties': {'port': True}}},
                              {'port': '/dev/device'})
    for schema in (False, {'properties': {'port': False}}):
        with pytest.raises(DeploymentError, match='plugin parameter schema'):
            validate_parameter_schema({'parameter_schema': schema}, {'port': '/dev/device'})


def test_arbitrary_camera_driver_startup_and_output_contract(tmp_path, monkeypatch):
    camera = {'id': 'ceiling', 'backend': 'new_vendor', 'instance_parameters': {'port': '/dev/video8', 'ns': '/ceiling'},
              'startup': [{'kind': 'launch', 'package': 'new_camera_driver', 'launch_file': 'camera.launch.py',
                           'arguments': {'device': '${port}', 'namespace': '${ns}'}}],
              'rgbd_topic': '${ns}/rgbd', 'metadata_topic': '${ns}/metadata'}
    normalized = validate_cameras([camera])
    path = Path(__file__).resolve().parents[2] / 'humanoid_camera/launch/multi_camera.launch.py'
    spec = importlib.util.spec_from_file_location('camera_launch', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    selected = []
    monkeypatch.setattr(module, 'startup_actions', lambda steps, actions: selected.extend(steps) or actions)
    from launch import LaunchContext
    context = LaunchContext(); config = tmp_path / 'cameras.yaml'
    _write_yaml(config, {'schema_version': 1, 'cameras': normalized})
    context.launch_configurations['camera_config'] = str(config)
    module._launch(context)
    assert selected[0][0]['arguments'] == {'device': '/dev/video8', 'namespace': '/ceiling'}
    normalized[0]['enabled'] = False
    _write_yaml(config, {'schema_version': 1, 'cameras': normalized}); selected.clear()
    module._launch(context)
    assert selected == []


def test_shared_commands_reach_only_the_owning_runtime(tmp_path):
    # Isolate DDS initialization from other tests' already-loaded middleware contexts.
    transport = tmp_path / 'dds.xml'
    transport.write_text('''<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors><transport_descriptor><transport_id>loopback_udp</transport_id>
    <type>UDPv4</type><interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>
  </transport_descriptor></transport_descriptors>
  <participant profile_name="loopback" is_default_profile="true"><rtps>
    <defaultUnicastLocatorList><locator><udpv4><address>127.0.0.1</address></udpv4></locator></defaultUnicastLocatorList>
    <userTransports><transport_id>loopback_udp</transport_id></userTransports>
    <useBuiltinTransports>false</useBuiltinTransports>
  </rtps></participant>
</profiles>''')
    # Local traffic is enforced by the XML; the Humble localhost flag adds SHM.
    environment = dict(os.environ, ROS_DOMAIN_ID='227', ROS_LOCALHOST_ONLY='0',
                       FASTRTPS_DEFAULT_PROFILES_FILE=str(transport))
    process = subprocess.run([sys.executable, str(Path(__file__).resolve()), str(tmp_path)],
        env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=30)
    assert process.returncode == 0, process.stdout


def _shared_runtime_simulation(tmp_path):
    """Two real runtime processes with the topic adapter, synthetic feedback only."""
    import time
    import signal
    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64
    from rclpy.qos import qos_profile_sensor_data
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    binary = Path(get_package_prefix('humanoid_driver_runtime')) / 'lib/humanoid_driver_runtime/humanoid_gripper_runtime_node'
    plugin_xml = Path(get_package_share_directory('humanoid_gripper')) / 'plugins/gripper_plugins.xml'
    observed = {name: [] for name in ('tool_a', 'tool_b')}
    node = executor = context = None
    processes, logs = [], []
    try:
        for name in observed:
            params = {'plugin_class': 'humanoid_gripper/RosTopicGripperDriver',
                'plugin_xml_paths': [str(plugin_xml)], 'gripper_names': [name],
                'vendor_gripper_names': [name], 'position_units': ['m'], 'filter_unowned_commands': True,
                'platform_gripper_command_topic': '/test_tools/commands',
                'platform_gripper_state_topic': '/test_tools/states',
                'plugin_parameters': [f'{name}.command_topic=/test_tools/{name}',
                    f'{name}.feedback_topic=/test_tools/feedback', f'{name}.command_type=float64',
                    f'{name}.feedback_type=joint_state', f'{name}.min_position=0', f'{name}.max_position=0.1']}
            path = tmp_path / f'{name}.yaml'; _write_yaml(path, {'/**': {'ros__parameters': params}})
            log = (tmp_path / f'{name}.log').open('w'); logs.append(log)
            processes.append(subprocess.Popen([str(binary), '--ros-args', '-r', '__node:=' + name,
                '--params-file', str(path)], stdout=log, stderr=subprocess.STDOUT))
        ready = time.monotonic() + 4
        while time.monotonic() < ready and not all('awaiting feedback' in path.read_text() for path in tmp_path.glob('tool_*.log')):
            time.sleep(.02)
        rclpy.init()
        context = rclpy.get_default_context()
        node = rclpy.create_node('synthetic_gripper_vendor', context=context)
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        commands = node.create_publisher(JointState, '/test_tools/commands', 10)
        feedback = node.create_publisher(JointState, '/test_tools/feedback', 10)
        observed = {name: [] for name in ('tool_a', 'tool_b')}
        subscriptions = [node.create_subscription(Float64, '/test_tools/' + name,
            lambda msg, key=name: observed[key].append(msg.data), qos_profile_sensor_data) for name in observed]
        discovery = time.monotonic() + 8
        while time.monotonic() < discovery:
            executor.spin_once(timeout_sec=.05)
            if commands.get_subscription_count() >= 2 and feedback.get_subscription_count() >= 2:
                break
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            assert all(p.poll() is None for p in processes), '\n'.join(p.read_text() for p in tmp_path.glob('*.log'))
            feedback.publish(JointState(name=['tool_a', 'tool_b'], position=[.02, .02]))
            commands.publish(JointState(name=['tool_b', 'tool_a'], position=[.03, .01]))
            executor.spin_once(timeout_sec=.02)
            time.sleep(.02)
            if any(abs(v-.01) < 1e-6 for v in observed['tool_a']) and any(abs(v-.03) < 1e-6 for v in observed['tool_b']):
                break
        detail = repr(observed) + repr(node.get_node_names_and_namespaces()) + repr(node.get_topic_names_and_types()) + '\n' + '\n'.join(path.read_text() for path in tmp_path.glob('*.log'))
        assert any(abs(v-.01) < 1e-6 for v in observed['tool_a']), detail
        assert any(abs(v-.03) < 1e-6 for v in observed['tool_b']), detail
        assert not any(abs(v-.03) < 1e-6 for v in observed['tool_a']), observed
        assert not any(abs(v-.01) < 1e-6 for v in observed['tool_b']), observed
    finally:
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
        for log in logs: log.close()
        if executor: executor.shutdown()
        if node: node.destroy_node()
        if context: rclpy.shutdown(context=context)


@pytest.mark.parametrize('kind', ['prismatic', 'mimic'])
def test_manual_motion_launch_rejects_unsupported_joints_before_sdk_execution(tmp_path, kind):
    from ament_index_python.packages import get_package_prefix
    tree = _model_tree(tmp_path / 'model')
    urdf = tree / 'resources/robot.urdf'
    text = urdf.read_text()
    text = text.replace('type="revolute"', 'type="prismatic"', 1) if kind == 'prismatic' else text.replace('</joint>', '<mimic joint="joint2"/></joint>', 1)
    urdf.write_text(text)
    motion = yaml.safe_load((tree / 'resources/motion.yaml').read_text())
    motion['humanoid_motion_control']['ros__parameters'].update(
        urdf_file=str(urdf), channel_config_file=str(tree / 'resources/channels.yaml'),
        sdk_config_file=str(tree / 'resources/sdk.yaml'), tool_config_file=str(tree / 'resources/tools.yaml'))
    path = tmp_path / 'motion.yaml'; _write_yaml(path, motion)
    executable = Path(get_package_prefix('humanoid_motion_server')) / 'lib/humanoid_motion_server/humanoid_motion_control_node'
    env = dict(os.environ, ROS_DOMAIN_ID='226', ROS_LOCALHOST_ONLY='1')
    process = subprocess.run([str(executable), '--ros-args', '--params-file', str(path)],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=10)
    assert process.returncode != 0
    assert 'unsupported controlled joint' in process.stdout, process.stdout


def test_same_plugin_can_be_instantiated_with_different_names_and_ports(devices, tmp_path):
    tree = gripper(tmp_path / 'template', 'reusable', 'tool')
    manifest = yaml.safe_load((tree / 'manifest.yaml').read_text())
    manifest['instance_parameters']['logical_name'] = 'tool'
    manifest['capabilities']['grippers']['${logical_name}'] = manifest['capabilities']['grippers'].pop('tool')
    path = tree / manifest['resources']['gripper_params']
    params = yaml.safe_load(path.read_text()); params['humanoid_gripper_runtime']['ros__parameters']['gripper_names'] = ['${logical_name}']
    _write_yaml(path, params); _write_yaml(tree / 'manifest.yaml', manifest)
    deploy_archive(pack_directory(tree, tmp_path / 'reusable.zip'), devices.plugin_root)
    robot = devices.create('two_identical', 'Two identical grippers', driver_id='fake_driver', model_id='test_model')
    template = devices.catalog()['gripper_drivers']['reusable']
    for name in ('one', 'two'):
        settings = copy.deepcopy(template['settings'])
        settings['instance_parameters'].update(logical_name=name, namespace='/' + name)
        robot['draft']['gripper_instances'].append({'instance_id': name, 'plugin_id': 'reusable',
            'name': name, 'parameters': copy.deepcopy(template['template']), 'settings': settings})
    saved = devices.save('two_identical', robot['draft'], robot['etag'])
    devices.apply('two_identical', saved['latest'], saved['etag'])
    resolved = resolve_robot_deployment(devices.plugin_root, 'two_identical')
    assert [item.parameters['gripper_names'] for item in resolved.gripper_instances] == [['one'], ['two']]
    assert resolve_gripper_test(saved['saved'], 'two', 'close')['position'] == .01


if __name__ == '__main__':
    _shared_runtime_simulation(Path(sys.argv[1]))
