"""Manual acceptance checker: deterministic good/bad observations, no camera hardware."""
import copy
import json

import pytest
from humanoid_camera.configuration import validate_cameras
from humanoid_camera.health_check import CameraCheck, expected_parameters, mapped_midpoint_ns
from humanoid_camera.health_report import assemble, write_report


def camera():
    return validate_cameras([{'id': 'front', 'device_type': 'd405', 'serial_no': 'TEST_ONLY'}])[0]


def observed(check, *, count=60, exposure=4000, skew_us=100, mode=1, mapping_error=0,
             images=True, dynamic=False, domain='global_time', parameter_epoch=0):
    for index in range(count):
        header = 1_780_000_000_000_000_000 + index * 33_333_333
        anchor = f'{header // 1_000_000}.{header % 1_000_000:06d}'
        raw = {'clock_domain': domain, 'frame_timestamp': anchor,
               'hw_timestamp': 100000 + index * 33333,
               'sensor_timestamp': 98000 + index * 33333, 'frame_number': index + 1,
               'actual_exposure': exposure - (index % 5) * 100 if dynamic else exposure,
               'gain_level': 16 + index % 3 if dynamic else 16, 'auto_exposure': mode}
        depth = {**raw, 'sensor_timestamp': raw['sensor_timestamp'] + skew_us}
        capture = header - 2_000_000
        doc = {'source_id': 'front', 'clock_epoch': parameter_epoch, 'capture_time_ns': capture,
               'driver_header_timestamp_ns': header,
               'rgb': {'capture_time_ns': capture + mapping_error, 'source_seq': index + 1},
               'depth': {'capture_time_ns': capture + skew_us * 1000, 'source_seq': index + 1}}
        elapsed = index / 30
        # Exercise transport ordering: normalized data can arrive before raw metadata.
        check.normalized(doc, capture, elapsed)
        check.raw('depth', depth, header, header + 5_000_000, elapsed)
        check.raw('rgb', raw, header, header + 5_000_000, elapsed)
        if images: check.image(capture, capture, capture + skew_us * 1000, elapsed)


def report(check, parameters=True):
    values = {**expected_parameters(check.camera), 'use_sim_time': False} if parameters is True else parameters
    return check.report(2., values)


def test_good_observations_pass_without_claiming_physical_clock_accuracy():
    check = CameraCheck(camera(), verify_images=True, exercise_auto=True)
    observed(check, dynamic=True)
    result = report(check)
    assert result['status'] == 'PASS', result
    assert result['counts']['verified_pairs'] == 60
    assert result['counts']['verified_images'] == 60
    assert result['metrics']['rgb_depth_midpoint_skew_ms']['max'] == .1
    assert result['physical_clock_accuracy_verified'] is False


@pytest.mark.parametrize('change,reason', [
    ({'exposure': 6000}, '实际曝光超过要求上限'),
    ({'skew_us': 2000}, '曝光中点差超过'),
    ({'mode': 0}, '自动曝光状态'),
    ({'mapping_error': 20_000}, '原始元数据换算不一致'),
    ({'domain': 'hardware_clock'}, 'GLOBAL_TIME'),
    ({'images': False}, '图像校验覆盖率'),
    ({'count': 10}, '接收帧率'),
])
def test_unacceptable_observations_fail(change, reason):
    check = CameraCheck(camera(), verify_images=True)
    observed(check, **change)
    result = report(check)
    assert result['status'] == 'FAIL'
    assert any(reason in name for name in result['failures']), result


def test_missing_frames_never_pass():
    assert report(CameraCheck(camera()), None)['status'] == 'FAIL'


def test_missing_parameter_readback_is_not_a_pass():
    check = CameraCheck(camera()); observed(check)
    assert report(check, None)['status'] == 'UNKNOWN'


def test_unchanged_light_is_inconclusive_only_when_response_test_requested():
    check = CameraCheck(camera(), exercise_auto=True); observed(check)
    assert report(check)['status'] == 'UNKNOWN'
    check.exercise_auto = False
    assert report(check)['status'] == 'PASS'


def test_config_and_parameter_mismatch_is_visible():
    check = CameraCheck(camera()); observed(check)
    params = {**expected_parameters(check.camera), 'use_sim_time': False,
              'depth_module.global_time_enabled': False}
    result = report(check, params)
    assert result['status'] == 'FAIL'
    assert '驱动参数不符:depth_module.global_time_enabled' in result['failures']


def test_wraparound_and_invalid_global_timestamp():
    raw = {'clock_domain': 'global_time', 'frame_timestamp': '1780000000000.000001',
           'hw_timestamp': 100, 'sensor_timestamp': 2**32 - 1900}
    assert mapped_midpoint_ns(raw) == 1780000000000000001 - 2_000_000
    with pytest.raises(ValueError): mapped_midpoint_ns({**raw, 'frame_timestamp': 'invalid'})


def test_frame_gaps_and_time_epochs_are_reported():
    check = CameraCheck(camera()); observed(check)
    observed(check, count=1, parameter_epoch=1)
    result = report(check)
    assert result['status'] == 'FAIL'
    assert '检测到时钟epoch变化' in result['failures']
    assert any('帧号重复或倒退' in name for name in result['failures'])


def test_html_json_and_csv_report_are_standalone_and_escape_text(tmp_path):
    check = CameraCheck(camera()); observed(check)
    result = assemble([check], 2., {'front': {**expected_parameters(check.camera), 'use_sim_time': False}},
                      verify_images=False, data_origin='合成数据自测')
    result['cameras'][0]['serial_no'] = '<script>unsafe</script>'
    output = write_report(result, tmp_path)
    assert json.loads((output / 'report.json').read_text())['status'] == 'PASS'
    page = (output / 'report.html').read_text()
    assert '&lt;script&gt;' in page and '<script>unsafe</script>' not in page
    assert '合成数据示例' in page
    assert len((output / 'samples.csv').read_text().splitlines()) == 61


def test_command_receiver_keeps_two_cameras_and_parameter_services_separate(monkeypatch):
    import importlib.util
    from pathlib import Path
    from types import SimpleNamespace as Obj
    import rclpy
    path = Path(__file__).resolve().parents[2] / 'humanoid_camera/scripts/check_cameras.py'
    spec = importlib.util.spec_from_file_location('manual_camera_command', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    cameras = validate_cameras([{'id': name, 'device_type': 'd405', 'serial_no': name}
                               for name in ('front', 'rear')])
    subscriptions, state = {}, {'elapsed': 0., 'index': 0, 'destroyed': False}
    base = 1_780_000_000_000_000_000
    def header(ns): return Obj(stamp=Obj(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000))
    class Client:
        def service_is_ready(self): return True
        def call_async(self, request):
            expected = {**expected_parameters(cameras[0]), 'use_sim_time': False}
            values = [Obj(type=1, bool_value=expected[name]) if isinstance(expected[name], bool)
                      else Obj(type=2, integer_value=expected[name]) for name in request.names]
            return Obj(done=lambda: True, result=lambda: Obj(values=values))
    class Node:
        def create_client(self, kind, topic): return Client()
        def create_subscription(self, kind, topic, callback, qos):
            subscriptions[topic] = callback
            return Obj(topic_name=topic)
        def get_publishers_info_by_topic(self, topic): return []
        def get_clock(self): return Obj(now=lambda: Obj(nanoseconds=base + state['index'] * 33_333_333 + 5_000_000))
        def destroy_node(self): state['destroyed'] = True
    def spin_once(node, timeout_sec):
        state['index'] += 1; state['elapsed'] += 1 / 30
        i = state['index']; frame = base + i * 33_333_333; midpoint = frame - 2_000_000
        for camera in cameras:
            prefix = '/' + camera['id'] + '/camera'
            raw = {'frame_number': i, 'frame_timestamp': f'{frame // 1_000_000}.{frame % 1_000_000:06d}',
                   'clock_domain': 'global_time', 'hw_timestamp': i * 33333,
                   'sensor_timestamp': i * 33333 - 2000, 'actual_exposure': 4000,
                   'gain_level': 16, 'auto_exposure': 1}
            for suffix in ('color', 'depth'):
                subscriptions[prefix + '/' + suffix + '/metadata'](Obj(header=header(frame), json_data=json.dumps(raw)))
            normalized = {'source_id': camera['id'], 'clock_epoch': 0, 'capture_time_ns': midpoint,
                          'driver_header_timestamp_ns': frame,
                          'rgb': {'source_seq': i, 'capture_time_ns': midpoint},
                          'depth': {'source_seq': i, 'capture_time_ns': midpoint}}
            subscriptions[camera['metadata_topic']](Obj(header=header(midpoint), json_data=json.dumps(normalized)))
            subscriptions[camera['rgbd_topic']](Obj(header=header(midpoint), rgb=Obj(header=header(midpoint)), depth=Obj(header=header(midpoint))))
    monkeypatch.setattr(module.time, 'monotonic', lambda: state['elapsed'])
    monkeypatch.setattr(rclpy, 'init', lambda **kwargs: None)
    monkeypatch.setattr(rclpy, 'try_shutdown', lambda: None)
    monkeypatch.setattr(rclpy, 'create_node', lambda name: Node())
    monkeypatch.setattr(rclpy, 'spin_once', spin_once)
    args = Obj(domain_id=223, duration=2., warmup=0., verify_images=True, exercise_auto=False, max_age_ms=500.)
    result = module.run(cameras, args)
    assert result['status'] == 'PASS', result
    assert [c['id'] for c in result['cameras']] == ['front', 'rear']
    assert all(c['counts']['verified_pairs'] >= 60 for c in result['cameras'])
    assert state['destroyed']
