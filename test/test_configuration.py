from pathlib import Path
import copy
import json

import pytest

from humanoid_manager.configuration import (
    ConfigurationManager,
    ConfigurationConflict,
    resolve_gripper_test,
    resolve_initial_pose,
    resolve_joint_jog,
)
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


@pytest.mark.parametrize('gripper_id', ['', 'fake_gripper'])
@pytest.mark.parametrize('robot_id', ['openarmx_01', ' \topenarmx_01\n', 'a' * 64])
def test_create_accepts_workspace_ids_and_trims_pasted_input(manager, robot_id, gripper_id):
    robot = manager.create(robot_id, '  实验机器人  ', driver_id=' fake_driver ',
                           model_id=' test_model ', gripper_id=gripper_id)
    expected = robot_id.strip()
    assert robot['robot_id'] == expected
    assert robot['name'] == '实验机器人'
    # A maximum-length workspace ID creates longer private plugin IDs. Saving
    # must still work, including validation of the private gripper selection.
    saved = manager.save(expected, robot['draft'], robot['etag'])
    assert saved['robot_id'] == expected
    assert ConfigurationManager(manager.plugin_root, manager.state_root).get(expected)['latest'] == saved['latest']


@pytest.mark.parametrize('field,value,label', [
    ('robot_id', '', '配置 ID（robot_id）'),
    ('robot_id', 'openarmx_01\u200b', '配置 ID（robot_id）'),
    ('robot_id', 'openarmx 01', '配置 ID（robot_id）'),
    ('driver_id', '', '机械臂驱动插件 ID（driver_id）'),
    ('driver_id', None, '机械臂驱动插件 ID（driver_id）'),
    ('driver_id', '../escape', '机械臂驱动插件 ID（driver_id）'),
    ('model_id', '', '模型插件 ID（model_id）'),
    ('model_id', 'TestModel', '模型插件 ID（model_id）'),
    ('gripper_id', [], '夹爪插件 ID（gripper_id）'),
    ('source_workspace', '../escape', '来源配置 ID（source_workspace）'),
    ('source_robot', '../escape', '来源机器人 ID（source_robot）'),
])
def test_create_reports_the_actual_invalid_field_without_persisting(manager, field, value, label):
    arguments = dict(robot_id='openarmx_01', name='实验机器人',
                     driver_id='fake_driver', model_id='test_model')
    arguments[field] = value
    with pytest.raises(DeploymentError) as caught:
        manager.create(**arguments)
    assert label in str(caught.value)
    if value == 'openarmx_01\u200b':
        assert '\\u200b' in str(caught.value)
    assert manager.catalog()['workspaces'] == []


@pytest.mark.parametrize('field,label', [('driver_id', '机械臂驱动插件'),
                                        ('model_id', '模型插件'), ('gripper_id', '夹爪插件')])
def test_create_reports_missing_catalog_plugin(manager, field, label):
    arguments = dict(robot_id='openarmx_01', name='实验机器人',
                     driver_id='fake_driver', model_id='test_model')
    arguments[field] = 'missing_plugin'
    with pytest.raises(DeploymentError, match=f'{label} missing_plugin 不存在'):
        manager.create(**arguments)
    assert manager.catalog()['workspaces'] == []


def test_create_can_copy_and_reject_duplicate_normalized_id(manager):
    original = create(manager)
    copied = manager.create(' openarmx_01 ', '复制的机器人', source_workspace=' lab ')
    assert copied['robot_id'] == 'openarmx_01'
    assert manager.get('lab')['latest'] == original['latest']
    with pytest.raises(ConfigurationConflict):
        manager.create(' openarmx_01 ', '不能覆盖', source_workspace='lab')
    assert manager.get('openarmx_01')['latest'] == copied['latest']


def test_creation_accepts_imported_plugin_ids_longer_than_workspace_ids(manager, tmp_path):
    from test_deployment_plugins import _write_yaml
    import yaml

    driver = _hardware_tree(tmp_path / 'long-driver')
    manifest = yaml.safe_load((driver / 'manifest.yaml').read_text())
    manifest['plugin_id'] = 'driver_' + 'a' * 64
    _write_yaml(driver / 'manifest.yaml', manifest)
    deploy_archive(pack_directory(driver, tmp_path / 'long-driver.zip'), manager.plugin_root)
    robot = manager.create('openarmx_01', '实验机器人', driver_id=manifest['plugin_id'], model_id='test_model')
    assert robot['robot_id'] == 'openarmx_01'


def test_workspace_import_uses_normalized_id_for_followup_save(manager, tmp_path):
    original = create(manager)
    archive = tmp_path / 'workspace.zip'
    manager.export('lab', original['latest'], archive)
    imported = manager.import_workspace(archive, ' openarmx_01 ', '导入配置')
    assert imported['robot_id'] == 'openarmx_01'
    assert manager.get('openarmx_01')['latest'] == imported['latest']


def test_single_save_validates_versions_and_normalizes_joint_space_channels(manager):
    robot = create(manager)
    previous_revision = robot['latest']
    document = copy.deepcopy(robot['draft'])
    document['name'] = '一次保存'
    channel = document['resources']['channel_config']['channels'][0]
    channel.update(base_frame='base', tip_frame='link2')
    saved = manager.save('lab', document, robot['etag'])
    assert saved['latest'] != previous_revision
    assert saved['draft']['name'] == '一次保存'
    assert 'base_frame' not in saved['draft']['resources']['channel_config']['channels'][0]
    assert 'tip_frame' not in saved['draft']['resources']['channel_config']['channels'][0]

    invalid = copy.deepcopy(saved['draft'])
    invalid['resources']['motion_params']['humanoid_motion_control']['ros__parameters'][
        'groups.arm'] = ['missing_joint', 'joint2']
    with pytest.raises(DeploymentError):
        manager.save('lab', invalid, saved['etag'])
    unchanged = manager.get('lab')
    assert unchanged['latest'] == saved['latest']
    assert unchanged['draft'] == saved['draft']


def test_direct_gripper_import_rejects_other_plugin_types_before_deploy(manager, tmp_path):
    archive = pack_directory(_hardware_tree(tmp_path / 'other-driver'), tmp_path / 'driver.zip')
    before = sorted(path.name for path in (manager.plugin_root / 'hardware_drivers').iterdir())
    with pytest.raises(DeploymentError, match='gripper_driver'):
        manager.import_bundle(archive, expected_plugin_type='gripper_driver')
    assert sorted(path.name for path in (manager.plugin_root / 'hardware_drivers').iterdir()) == before


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


def test_joint_jog_resolves_smallest_move_j_group_and_rejects_unsafe_step(manager):
    robot = create(manager)
    jog = resolve_joint_jog(robot['saved'], 'joint2', 0.01)
    assert jog['endpoint'] == '/motion/arm/move_j'
    assert jog['joint_names'] == ['joint1', 'joint2']
    assert jog['state_topic'] == '/hc_teleop/joint_states'
    assert jog['delta_rad'] == 0.01
    with pytest.raises(DeploymentError, match='步长'):
        resolve_joint_jog(robot['saved'], 'joint2', 0.3)


def test_gripper_test_uses_managed_topics_and_configured_open_close_positions(manager):
    robot = create(manager)
    document = copy.deepcopy(robot['saved'])
    document['resources']['gripper_params'] = copy.deepcopy(
        manager.catalog()['gripper_drivers']['fake_gripper']['template']
    )
    document['resources']['hc_teleop_config'] = {'grippers': [{
        'id': 'left', 'joint_name': 'left_gripper', 'open_position': 0.04,
        'closed_position': 0.002, 'max_effort': 12.0, 'max_speed': 0.02,
    }]}
    opened = resolve_gripper_test(document, 'left_gripper', 'open')
    closed = resolve_gripper_test(document, 'left_gripper', 'close')
    assert opened['command_topic'] == '/hc_teleop/gripper_commands'
    assert opened['state_topic'] == '/hc_teleop/gripper_states'
    assert opened['position'] == 0.04 and closed['position'] == 0.002
    assert opened['max_effort'] == 12.0


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
        {'id': 'rear', 'device_type': 'd455', 'serial_no': '00455001',
         'rgb_topic': '/rear/rgb', 'depth_topic': '/rear/depth',
         'rgbd_topic': '/rear/rgbd', 'metadata_topic': '/rear/metadata'},
    ]
    robot = manager.draft('lab', doc, robot['etag'])
    robot = manager.validate('lab', robot['etag'], save=True)
    manager.apply('lab', robot['latest'], robot['etag'])
    deployed = manager.plugin_root / 'robots/lab/cameras.yaml'
    camera_config = __import__('yaml').safe_load(deployed.read_text())
    assert [camera['device_type'] for camera in camera_config['cameras']] == ['d405', 'd405', 'd435', 'd455']
    assert camera_config['cameras'][3]['serial_no'] == '00455001'
    assert camera_config['cameras'][3]['rgb_topic'] == '/rear/rgb'
    assert camera_config['cameras'][3]['rgbd_topic'] == '/rear/rgbd'
    assert camera_config['cameras'][2]['color_exposure_us'] == 3900
    assert all(camera['sync_rgb_depth'] for camera in camera_config['cameras'])
    assert all(camera['timestamp_alignment'] for camera in camera_config['cameras'])
    assert all(camera['max_actual_exposure_us'] == 5000 for camera in camera_config['cameras'])
    exported = tmp_path / 'cameras.zip'
    manager.export('lab', robot['latest'], exported)
    imported = manager.import_workspace(exported, 'camera_copy', '相机副本')
    assert imported['draft']['cameras'] == robot['draft']['cameras']


@pytest.mark.parametrize('save_draft', [False, True])
def test_save_normalizes_pasted_camera_id_and_rejects_bad_entries_atomically(manager, save_draft):
    robot = create(manager)
    doc = copy.deepcopy(robot['draft'])
    doc['cameras'] = [{'id': ' \tcamera_left\n', 'namespace': ' \tcamera_left\n'}]
    if save_draft:
        draft = manager.draft('lab', doc, robot['etag'])
        saved = manager.validate('lab', draft['etag'], save=True)
    else:
        saved = manager.save('lab', doc, robot['etag'])
    camera = saved['saved']['cameras'][0]
    assert camera['id'] == camera['namespace'] == 'camera_left'
    assert camera['rgb_topic'] == '/camera_left/camera/color/image_raw'
    assert camera['rgbd_topic'] == '/camera_left/normalized/rgbd'
    assert manager.get('lab')['draft']['cameras'] == [camera]

    invalid = copy.deepcopy(saved['draft'])
    invalid['cameras'].append({'id': 'camera_right\u200b'})
    with pytest.raises(DeploymentError, match=r'cameras\[1\].id'):
        manager.save('lab', invalid, saved['etag'])
    unchanged = manager.get('lab')
    assert unchanged['latest'] == saved['latest']
    assert unchanged['draft'] == saved['draft']


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
