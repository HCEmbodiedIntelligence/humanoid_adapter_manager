"""Run the normal dashboard's MCAP and replay flow with a mock execution layer."""
import asyncio
from collections import Counter
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import urllib.request

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('aiohttp')
pytest.importorskip('mcap')
from humanoid_manager.web.config import ConfigStore


def test_mock_feedback_mcap_and_browser_replay(tmp_path):
    package = Path(__file__).resolve().parents[1]
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    base = f'http://127.0.0.1:{port}'
    topics = {topic: 'sensor_msgs/msg/JointState' for topic in (
        '/hc_teleop/joint_cmd', '/hc_teleop/joint_states',
        '/hc_teleop/gripper_commands', '/hc_teleop/gripper_states')}
    topics.update({f'/camera_{name}/camera/color/image_raw': 'sensor_msgs/msg/Image'
                   for name in ('left', 'right', 'hand')})
    topics['/diagnostics'] = 'diagnostic_msgs/msg/DiagnosticArray'
    state = tmp_path / 'web'
    ConfigStore(state / 'configurator.yaml').save({
        'server': {'host': '127.0.0.1', 'port': port},
        'adapter_manager': {'cli': str(package / 'scripts/humanoid_pluginctl.py'),
                            'plugin_root': str(state / 'plugins'), 'state_root': str(state / 'configuration')},
        'ros': {'domain_id': 199, 'recording': {'directory': str(state / 'recordings')},
                'subscriptions': [{'topic': topic, 'type': kind, 'outputs': ['record'], 'max_hz': 0}
                                  for topic, kind in topics.items()]},
    })
    env = {**os.environ, 'ROS_DOMAIN_ID': '199', 'ROS_LOCALHOST_ONLY': '1',
           'HUMANOID_WEB_PYTHON': sys.executable, 'PYTHONUNBUFFERED': '1'}
    def api(path, data=None):
        request = urllib.request.Request(base + path,
            data=json.dumps(data).encode() if data is not None else None,
            headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(request, timeout=10) as response:
            return json.load(response)
    processes, logs = [], []
    try:
        for name, command in (
            ('feedback', [sys.executable, str(package / 'scripts/mock_robot_feedback.py'), '--domain-id', '199']),
            ('source', [sys.executable, str(package / 'test/mock_feedback_recording_source.py')]),
            ('web', [str(package / 'start_configurator.sh'), '--state-root', str(state), '--domain-id', '199']),
        ):
            log = (tmp_path / f'{name}.log').open('w+')
            logs.append(log)
            processes.append(subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        deadline, check = time.monotonic() + 15, None
        while time.monotonic() < deadline:
            assert all(p.poll() is None for p in processes), 'A test process exited; inspect logs'
            try:
                check = api('/api/recording/precheck')
                if check['ready']:
                    break
            except OSError:
                pass
            time.sleep(.2)
        else:
            pytest.fail(f'MCAP precheck did not become ready: {check}')
        started = api('/api/recording/start', {'filename': 'mock_feedback_test'})
        time.sleep(6)
        stopped = api('/api/recording/stop', {})['status']
        assert stopped['dropped'] == 0 and not stopped['error'], stopped
        path = Path(started['path'])
        audit = api(f'/api/datasets/{path.name}/audit', {})
        assert audit['ok'], audit
        overview = api(f'/api/datasets/{path.name}')
        assert overview['message_count'] > 1000
        frames = api(f'/api/datasets/{path.name}/frames?start=0&end=1&limit=10')
        assert frames['frames']

        from mcap.reader import make_reader
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        counts, values = Counter(), {}
        with path.open('rb') as stream:
            reader = make_reader(stream, validate_crcs=True)
            summary = reader.get_summary()
            assert summary.statistics.message_count > 1000
            for schema, channel, message in reader.iter_messages():
                if channel.topic not in topics:
                    continue
                assert channel.message_encoding == 'cdr'
                assert schema.name == topics[channel.topic]
                decoded = deserialize_message(message.data, get_message(schema.name))
                counts[channel.topic] += 1
                if schema.name == 'sensor_msgs/msg/Image':
                    assert decoded.width == 640 and decoded.height == 480
                    assert decoded.header.frame_id == 'synthetic_camera_test'
                    index = ('left', 'right', 'hand').index(channel.topic.split('/')[1].removeprefix('camera_'))
                    assert bytes(decoded.data) == bytes([index + 1, 40, 80]) * (640 * 480)
                elif schema.name == 'sensor_msgs/msg/JointState':
                    assert len(decoded.name) == len(decoded.position) == (2 if 'gripper' in channel.topic else 14)
                    values.setdefault(channel.topic, set()).add(round(decoded.position[0], 6))
        assert set(counts) == set(topics)
        assert all(counts[t] >= 100 for t in topics if t.endswith('/image_raw'))
        assert all(len(samples) > 100 for samples in values.values())

        async def browser_replay():
            import aiohttp
            async with aiohttp.ClientSession() as client:
                async with client.ws_connect(base + '/ws') as ws:
                    async with client.post(base + '/api/replay/play', json={'filename': path.name}) as response:
                        assert response.status == 200
                    deadline = time.monotonic() + 5
                    while time.monotonic() < deadline:
                        event = await ws.receive_json(timeout=5)
                        if event.get('kind') == 'replay_message':
                            assert event['filename'] == path.name
                            return True
            return False
        assert asyncio.run(browser_replay())
        api('/api/replay/stop', {})
        report = {'mcap': str(path), 'counts': dict(counts), 'total': sum(counts.values()),
                  'audit_ok': True, 'browser_replay_ok': True, 'images': 'synthetic, not real cameras'}
        (tmp_path / 'result.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report))
    finally:
        # Close the web recorder before its sources, then reap every process group.
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=45)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for log in logs:
            log.close()
