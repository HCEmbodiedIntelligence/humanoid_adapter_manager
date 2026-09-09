"""Configurable RealSense count/identity/topics and one-shot previews; no hardware."""
import base64
import copy
import importlib.util
import io
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from PIL import Image
from humanoid_camera.configuration import validate_cameras
from humanoid_manager.deployment import DeploymentError
from humanoid_manager.web.camera_snapshot import (
    CameraSnapshotError, encode_snapshot, resolve_camera_snapshot,
)


def cameras(count):
    return [{'id': f'camera{i}', 'device_type': 'd405' if i % 2 else 'd435',
             'serial_no': f'0000000000{i}', 'namespace': f'cam{i}',
             'rgb_topic': f'/camera{i}/rgb', 'depth_topic': f'/camera{i}/depth',
             'rgbd_topic': f'/camera{i}/rgbd', 'metadata_topic': f'/camera{i}/metadata',
             'pointcloud': True, 'pointcloud_topic': f'/camera{i}/cloud',
             'pointcloud_metadata_topic': f'/camera{i}/cloud_info'} for i in range(count)]


@pytest.mark.parametrize('count', [0, 1, 4, 6])
def test_camera_count_is_driven_by_config(count):
    result = validate_cameras(cameras(count))
    assert len(result) == count
    for camera in result:
        assert resolve_camera_snapshot({'cameras': result}, camera['id'])['topic'] == camera['rgb_topic']


@pytest.mark.parametrize('field', ['id', 'serial_no', 'rgb_topic', 'depth_topic', 'rgbd_topic', 'metadata_topic'])
def test_active_cameras_cannot_share_identity_or_outputs(field):
    values = cameras(4)
    values[3][field] = values[0][field]
    with pytest.raises(DeploymentError):
        validate_cameras(values)


def test_multiple_cameras_require_serials_and_snapshot_requires_enabled_camera():
    values = cameras(4)
    values[-1]['serial_no'] = ''
    with pytest.raises(DeploymentError, match='序列号'):
        validate_cameras(values)
    values[-1]['enabled'] = False
    with pytest.raises(DeploymentError, match='禁用'):
        resolve_camera_snapshot({'cameras': values}, values[-1]['id'])
    with pytest.raises(DeploymentError, match='没有这台相机'):
        resolve_camera_snapshot({'cameras': values}, 'absent')


def test_legacy_serial_string_prefix_cannot_bypass_duplicate_detection():
    values = cameras(2)
    values[1]['serial_no'] = '_' + values[0]['serial_no']
    with pytest.raises(DeploymentError, match='序列号重复'):
        validate_cameras(values)


def test_launch_binds_each_serial_and_remaps_driver_and_adapter_consistently(tmp_path, monkeypatch):
    from launch import LaunchContext
    from launch_ros.utilities import evaluate_parameters, normalize_parameters
    path = Path(__file__).resolve().parents[2] / 'humanoid_camera/launch/realsense_camera.launch.py'
    spec = importlib.util.spec_from_file_location('realsense_launch_test', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    values = cameras(4)
    values[-1]['parameters'] = {'serial_no': 'wrong', 'device_type': 'wrong'}
    # Distinct nodes can share a namespace when every output is explicit.
    values[1].update(namespace='cam0', camera_name='second_camera')
    config = tmp_path / 'cameras.yaml'
    config.write_text(yaml.safe_dump({'schema_version': 1, 'cameras': values}))
    context = LaunchContext(); context.launch_configurations['camera_config'] = str(config)
    actions = module._launch(context)
    assert len(actions) == 8
    node_names = set()
    for camera, driver, adapter in zip(values, actions[::2], actions[1::2]):
        params = evaluate_parameters(context, normalize_parameters(driver['parameters']))[0]
        assert params['serial_no'] == camera['serial_no']  # Includes leading zeroes; must stay a string.
        assert params['device_type'] == camera['device_type']
        prefix = '/' + driver['namespace'] + '/' + driver['name']
        assert dict(driver['remappings'])[prefix + '/color/image_raw'] == camera['rgb_topic']
        assert dict(adapter['remappings'])[prefix + '/color/image_raw'] == camera['rgb_topic']
        assert dict(adapter['remappings'])['normalized/rgbd'] == camera['rgbd_topic']
        assert dict(adapter['remappings'])['normalized/points'] == camera['pointcloud_topic']
        for node in (driver, adapter):
            full_name = node['namespace'] + '/' + node['name']
            assert full_name not in node_names
            node_names.add(full_name)


def frame(encoding='rgb8', data=None):
    return SimpleNamespace(width=2, height=2, encoding=encoding, step=8,
        data=data if data is not None else bytes([255, 0, 0, 255, 0, 0, 42, 42] * 2),
        header=SimpleNamespace(frame_id='optical', stamp=SimpleNamespace(sec=1, nanosec=2)))


@pytest.mark.parametrize('encoding,pixels', [
    ('rgb8', [255, 0, 0, 255, 0, 0, 42, 42] * 2),
    ('bgr8', [0, 0, 255, 0, 0, 255, 42, 42] * 2),
])
def test_photo_encoding_handles_row_padding_and_color_order(encoding, pixels):
    result = encode_snapshot(frame(encoding, bytes(pixels)))
    image = Image.open(io.BytesIO(base64.b64decode(result['image_data_url'].split(',')[1])))
    assert image.size == (2, 2)
    r, g, b = image.getpixel((0, 0))
    assert r > 240 and g < 10 and b < 10
    assert result['stamp_ns'] == 1_000_000_002


def test_photo_rejects_malformed_or_unsupported_frames():
    for bad in [frame(data=b''), frame(encoding='16UC1')]:
        with pytest.raises(CameraSnapshotError):
            encode_snapshot(bad)


def test_photo_has_bounded_preview_dimensions():
    message = frame()
    message.width, message.height, message.step = 1600, 1200, 4800
    message.data = bytes(message.step * message.height)
    result = encode_snapshot(message)
    assert (result['width'], result['height']) == (1600, 1200)
    assert (result['preview_width'], result['preview_height']) == (1280, 960)
