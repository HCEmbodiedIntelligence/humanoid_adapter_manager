from pathlib import Path
import copy
import json

import pytest

from humanoid_manager.configuration import ConfigurationManager, ConfigurationConflict, resolve_initial_pose
from humanoid_manager.deployment import DeploymentError, deploy_archive, pack_directory, resolve_robot_deployment
from humanoid_manager.runtime_state import configuration_identity, deployment_lock
from test_deployment_plugins import _hardware_tree, _gripper_tree, _model_tree, _composition_tree


@pytest.fixture
def manager(tmp_path):
    root = tmp_path / 'deployed'
    for name, factory in [('driver', _hardware_tree), ('model', _model_tree),
                          ('gripper', _gripper_tree), ('robot', _composition_tree)]:
        archive = pack_directory(factory(tmp_path / name), tmp_path / f'{name}.zip')
        deploy_archive(archive, root)
    return ConfigurationManager(root, tmp_path / 'state')


def create(manager):
    return manager.create('lab', '实验机器人', source_robot='test_robot')


def test_edit_validate_apply_restore_export_roundtrip(manager, tmp_path):
    robot = create(manager)
    original = robot['latest']
    before = configuration_identity(manager.plugin_root, 'test_robot')
    document = copy.deepcopy(robot['draft'])
    document['resources']['driver_params']['humanoid_driver_runtime']['ros__parameters']['command_watchdog_ms'] = 180.0
    document['recording']['directory'] = '/tmp/robot-recordings'
    robot = manager.draft('lab', document, robot['etag'])
    assert len(robot['diff']) == 2
    robot = manager.validate('lab', robot['etag'], save=True)
    assert robot['latest'] != original
    assert configuration_identity(manager.plugin_root, 'test_robot') == before
    manager.apply('lab', robot['latest'], robot['etag'])
    assert configuration_identity(manager.plugin_root, 'lab')['revision'] == robot['latest']
    assert configuration_identity(manager.plugin_root, 'test_robot') == before
    exported = tmp_path / 'export.zip'
    manager.export('lab', robot['latest'], exported)
    imported = manager.import_workspace(exported, 'second', '第二台')
    assert imported['draft']['recording']['directory'] == '/tmp/robot-recordings'
    assert imported['draft']['cameras'] == []
    assert imported['draft']['resources'] == robot['draft']['resources']
    restored = manager.restore('lab', original, robot['etag'])
    assert restored['diff']
    assert configuration_identity(manager.plugin_root, 'lab')['revision'] == robot['latest']


def test_conflict_invalid_model_and_live_lease_do_not_change_deployment(manager):
    robot = create(manager)
    etag = robot['etag']
    doc = copy.deepcopy(robot['draft'])
    doc['resources']['motion_params']['humanoid_motion_control']['ros__parameters']['groups.arm'] = ['absent', 'joint2']
    robot = manager.draft('lab', doc, etag)
    with pytest.raises(ConfigurationConflict):
        manager.draft('lab', doc, etag)
    with pytest.raises(DeploymentError):
        manager.validate('lab', robot['etag'], save=True)
    assert len(manager.get('lab')['history']) == 1
    with deployment_lock(manager.plugin_root, shared=True):
        with pytest.raises(DeploymentError, match='正在运行'):
            manager.apply('lab', robot['latest'], robot['etag'])
    assert not (manager.plugin_root / 'robots/lab').exists()


def test_apply_rolls_back_all_components_on_failure(manager, monkeypatch):
    import humanoid_manager.configuration as module
    robot = create(manager)
    manager.apply('lab', robot['latest'], robot['etag'])
    original = configuration_identity(manager.plugin_root, 'lab')
    doc = copy.deepcopy(robot['draft'])
    doc['name'] = '新版'
    robot = manager.draft('lab', doc, robot['etag'])
    robot = manager.validate('lab', robot['etag'], save=True)
    real_deploy = module.deploy_archive
    calls = []
    def fail_second(*args, **kwargs):
        calls.append(args)
        if len(calls) == 2:
            raise OSError('simulated disk failure')
        return real_deploy(*args, **kwargs)
    monkeypatch.setattr(module, 'deploy_archive', fail_second)
    with pytest.raises(OSError, match='disk failure'):
        manager.apply('lab', robot['latest'], robot['etag'])
    assert configuration_identity(manager.plugin_root, 'lab') == original
    assert resolve_robot_deployment(manager.plugin_root, 'lab').name == '实验机器人'


@pytest.mark.parametrize('robot_id', ['../escape','/tmp/escape','UPPER','', 'x' * 65])
def test_workspace_ids_are_bounded(manager, robot_id):
    with pytest.raises(DeploymentError):
        manager.create(robot_id, 'invalid', source_robot='test_robot')


def test_unknown_vendor_parameter_preserved_and_invalid_number_rejected(manager):
    robot = create(manager)
    doc = copy.deepcopy(robot['draft'])
    p = doc['resources']['driver_params']['humanoid_driver_runtime']['ros__parameters']
    p['plugin_parameters'] = ['vendor_extension=custom']
    p['control_frequency_hz'] = -5
    robot = manager.draft('lab', doc, robot['etag'])
    with pytest.raises(DeploymentError, match='非负数'):
        manager.validate('lab', robot['etag'], save=True)
    p['control_frequency_hz'] = 100
    robot = manager.draft('lab', doc, robot['etag'])
    robot = manager.validate('lab', robot['etag'], save=True)
    assert robot['draft']['resources']['driver_params']['humanoid_driver_runtime']['ros__parameters']['plugin_parameters'] == ['vendor_extension=custom']


def test_add_receiver_resource_and_restore_without_repacking_model(manager):
    robot=create(manager)
    original=robot['latest']
    doc=robot['draft']
    doc['resources']['channel_config']['channels']=[{'name':'arm_teleop','kind':'servo_p',
        'endpoint':'/teleop/arm/servo_p','fk_pose_topic':'/teleop/arm/fk_pose','priority':50,
        'group':'arm','base_frame':'base','tip_frame':'link2'}]
    doc['resources']['hc_teleop_config']={'schema_version':1,'adapter':{'robot_id':'lab'},
        'channels':[{'id':'arm','controller':'right','target_pose_topic':'/teleop/arm/servo_p',
            'fk_pose_topic':'/teleop/arm/fk_pose','base_frame':'base','tool_frame':'link2',
            'axis_mapping':[[1,0,0],[0,1,0],[0,0,1]]}]}
    robot=manager.draft('lab',doc,robot['etag'])
    robot=manager.validate('lab',robot['etag'],save=True)
    manager.apply('lab',robot['latest'],robot['etag'])
    assert 'hc_teleop_config' in resolve_robot_deployment(manager.plugin_root,'lab').resources
    restored=manager.restore('lab',original,robot['etag'])
    restored=manager.validate('lab',restored['etag'],save=True)
    manager.apply('lab',restored['latest'],restored['etag'])
    assert 'hc_teleop_config' not in resolve_robot_deployment(manager.plugin_root,'lab').resources


def test_attach_configure_export_and_remove_gripper_from_existing_robot(manager, tmp_path):
    robot = create(manager)
    assert robot['draft']['gripper_driver'] is None
    manager.apply('lab', robot['latest'], robot['etag'])
    catalog = manager.catalog()
    template = catalog['gripper_drivers']['fake_gripper']['template']
    document = copy.deepcopy(robot['draft'])
    document['gripper_driver'] = {
        'plugin_id': 'fake_gripper',
        'name': catalog['gripper_drivers']['fake_gripper']['name'],
    }
    document['resources']['gripper_params'] = copy.deepcopy(template)
    document['resources']['hc_teleop_config'] = {
        'schema_version': 1,
        'adapter': {'robot_id': 'lab'},
        'channels': [],
        'grippers': [{
            'id': 'left_gripper',
            'command_type': 'joint_state',
            'command_topic': '/hc_teleop/gripper_commands',
            'feedback_type': 'joint_state',
            'feedback_topic': '/hc_teleop/gripper_states',
            'joint_name': 'left_gripper',
            'position_unit': 'm',
        }],
    }
    robot = manager.draft('lab', document, robot['etag'])
    robot = manager.validate('lab', robot['etag'], save=True)
    manager.apply('lab', robot['latest'], robot['etag'])
    deployment = resolve_robot_deployment(manager.plugin_root, 'lab')
    assert deployment.gripper_class == 'fake_gripper/FakeGripperDriver'
    assert deployment.resources['gripper_params'].is_file()
    assert (manager.plugin_root / 'gripper_drivers/lab.gripper').is_dir()

    exported = tmp_path / 'with-gripper.zip'
    manager.export('lab', robot['latest'], exported)
    imported = manager.import_workspace(exported, 'gripper_copy', '夹爪副本')
    assert imported['draft']['resources']['gripper_params'] == robot['draft']['resources']['gripper_params']
    assert imported['draft']['gripper_driver'] is not None

    document = copy.deepcopy(robot['draft'])
    document['gripper_driver'] = None
    document['resources'].pop('gripper_params')
    document['resources'].pop('hc_teleop_config')
    robot = manager.draft('lab', document, robot['etag'])
    robot = manager.validate('lab', robot['etag'], save=True)
    manager.apply('lab', robot['latest'], robot['etag'])
    assert resolve_robot_deployment(manager.plugin_root, 'lab').gripper_class is None


def test_robot_camera_configuration_is_versioned_deployed_and_exported(manager, tmp_path):
    robot = create(manager)
    doc = copy.deepcopy(robot['draft'])
    doc['cameras'] = [
        {'id': 'front', 'device_type': 'd405', 'serial_no': '405001'},
        {'id': 'left', 'device_type': 'd405', 'serial_no': '405002'},
        {'id': 'right', 'device_type': 'd435', 'serial_no': '435001',
         'pointcloud': True, 'color_auto_exposure': False},
    ]
    robot = manager.draft('lab', doc, robot['etag'])
    robot = manager.validate('lab', robot['etag'], save=True)
    manager.apply('lab', robot['latest'], robot['etag'])
    deployed = manager.plugin_root / 'robots/lab/cameras.yaml'
    camera_config = __import__('yaml').safe_load(deployed.read_text())
    assert [camera['device_type'] for camera in camera_config['cameras']] == ['d405', 'd405', 'd435']
    assert camera_config['cameras'][2]['color_exposure_us'] == 4500
    assert all(camera['sync_rgb_depth'] for camera in camera_config['cameras'])
    assert all(camera['timestamp_alignment'] for camera in camera_config['cameras'])
    assert all(camera['max_actual_exposure_us'] == 5000 for camera in camera_config['cameras'])
    exported = tmp_path / 'cameras.zip'
    manager.export('lab', robot['latest'], exported)
    imported = manager.import_workspace(exported, 'camera_copy', '相机副本')
    assert imported['draft']['cameras'] == robot['draft']['cameras']


def test_multiple_realsense_require_unique_serial_numbers(manager):
    robot = create(manager)
    doc = copy.deepcopy(robot['draft'])
    doc['cameras'] = [
        {'id': 'front', 'device_type': 'd405', 'serial_no': ''},
        {'id': 'right', 'device_type': 'd435', 'serial_no': ''},
    ]
    robot = manager.draft('lab', doc, robot['etag'])
    with pytest.raises(DeploymentError, match='序列号'):
        manager.validate('lab', robot['etag'])


def test_initial_pose_is_versioned_resolved_and_limited(manager, tmp_path):
    robot = create(manager)
    document = copy.deepcopy(robot['draft'])
    document['initial_poses'] = [{
        'id': 'teleop_home',
        'name': '遥操作初始姿态',
        'velocity_scale': 0.2,
        'acceleration_scale': 0.15,
        'jerk_scale': 0.1,
        'timeout_sec': 45,
        'targets': [{'channel': 'arm_move_j', 'positions_rad': [0.25, -0.5]}],
    }]
    robot = manager.draft('lab', document, robot['etag'])
    robot = manager.validate('lab', robot['etag'], save=True)
    manager.apply('lab', robot['latest'], robot['etag'])
    deployed = __import__('yaml').safe_load((manager.plugin_root / 'robots/lab/initial_poses.yaml').read_text())
    assert deployed['initial_poses'][0]['targets'][0]['positions_rad'] == [0.25, -0.5]
    resolved = resolve_initial_pose(robot['saved'], 'teleop_home')
    assert resolved['goals'] == [{
        'channel': 'arm_move_j', 'endpoint': '/motion/arm/move_j', 'group': 'arm',
        'joint_names': ['joint1', 'joint2'], 'positions_rad': [0.25, -0.5],
        'velocity_scale': 0.2, 'acceleration_scale': 0.15, 'jerk_scale': 0.1,
        'timeout_sec': 45.0,
    }]
    exported = tmp_path / 'pose.zip'
    manager.export('lab', robot['latest'], exported)
    imported = manager.import_workspace(exported, 'pose_copy', '姿态副本')
    assert imported['draft']['initial_poses'] == robot['saved']['initial_poses']

    invalid = copy.deepcopy(imported['draft'])
    invalid['initial_poses'][0]['targets'][0]['positions_rad'][0] = 1.5
    imported = manager.draft('pose_copy', invalid, imported['etag'])
    with pytest.raises(DeploymentError, match='超出限位'):
        manager.validate('pose_copy', imported['etag'])
