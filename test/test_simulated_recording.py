"""Send real ROS messages through the HTTP recorder, then decode the resulting MCAP."""
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
pytest.importorskip('mcap')
from mcap.reader import make_reader
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def test_simulated_recording_roundtrip_and_interruption(tmp_path):
    package = Path(__file__).resolve().parents[1]
    with socket.socket() as reservation:
        reservation.bind(('127.0.0.1', 0))
        port = reservation.getsockname()[1]
    base = f'http://127.0.0.1:{port}'

    def api(path, body=None):
        request = urllib.request.Request(base + path,
            data=json.dumps(body).encode() if body is not None else None,
            headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(request, timeout=10) as response:
            return json.load(response)

    # --domain-id must override an inherited domain for every child process too.
    env = {**os.environ, 'ROS_DOMAIN_ID': '198', 'ROS_LOCALHOST_ONLY': '1',
           'PYTHONUNBUFFERED': '1'}
    # Use the test interpreter for the web process too; avoid dependency installation.
    env['HUMANOID_WEB_PYTHON'] = sys.executable
    with (tmp_path / 'demo.log').open('w+') as log:
        child = subprocess.Popen([
            sys.executable, str(package / 'scripts/simulate_recording.py'),
            '--web', '--domain-id', '224', '--port', str(port),
            '--state-root', str(tmp_path / 'web'), '--pause-after', '16', '--pause-for', '4',
        ], env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        began = time.monotonic()
        try:
            deadline, check = began + 12, None
            while time.monotonic() < deadline:
                if child.poll() is not None:
                    log.seek(0)
                    pytest.fail(log.read())
                try:
                    check = api('/api/recording/precheck')
                    if check['ready']:
                        break
                except OSError:
                    pass
                time.sleep(.1)
            else:
                log.seek(0)
                pytest.fail(f'Recording precheck did not become ready: {check}\n' + log.read())

            started = api('/api/recording/start', {'filename': 'synthetic-normal'})
            time.sleep(6)
            stopped = api('/api/recording/stop', {})
            assert stopped['status']['dropped'] == 0
            assert not stopped['status']['error']
            filename = Path(started['path']).name
            audit = api(f'/api/datasets/{filename}/audit', {})
            assert audit['ok'], audit
            overview = api(f'/api/datasets/{filename}')
            assert overview['message_count'] > 1000
            messages, edges, annotations = {}, [], []
            with Path(started['path']).open('rb') as stream:
                for schema, channel, message in make_reader(stream, validate_crcs=True).iter_messages():
                    if channel.topic == '/humanoid/annotations':
                        annotations.append(json.loads(message.data))
                        continue
                    if channel.message_encoding != 'cdr':
                        continue
                    decoded = deserialize_message(message.data, get_message(schema.name))
                    messages.setdefault(channel.topic, []).append(decoded)
                    if channel.topic.endswith('/buttons'):
                        document = json.loads(decoded.data)
                        assert document['synthetic']
                        edges.extend(document['edges'])
                    else:
                        assert decoded.header.frame_id == 'synthetic_recording_test'
                        assert decoded.header.stamp.sec > 0
                        assert len(decoded.name) == len(decoded.position) == len(decoded.velocity)
                        if 'gripper' in channel.topic:
                            assert decoded.name == ['left_gripper', 'right_gripper']
                            assert all(0 <= value <= .044 for value in decoded.position)
                        else:
                            assert len(decoded.name) == 14
            assert set(messages) == {'/recording_test/' + name for name in (
                'joint_states', 'joint_commands', 'gripper_states', 'gripper_commands', 'buttons')}
            for topic, values in messages.items():
                assert len(values) > (400 if '/joint_' in topic else 70), (topic, len(values))
                if not topic.endswith('/buttons'):
                    assert len({value.position[0] for value in values}) > 50
            assert {edge['action'] for edge in edges} == {'pressed', 'released'}
            assert any(event['source'] == 'right/secondary' for event in annotations)
            frames = api(f'/api/datasets/{filename}/frames?start=0&end=2&limit=10')
            assert frames

            gap_start = api('/api/recording/start', {'filename': 'synthetic-gap'})
            saw_gap, saw_recovery = False, False
            while time.monotonic() - began < 27:
                health = api('/api/status')['recording_executor']['topic_health']
                saw_gap |= any(item['state'] == 'no_data' for item in health.values())
                if saw_gap and health and all(item['state'] == 'ok' for item in health.values()):
                    saw_recovery = True
                    break
                time.sleep(.2)
            api('/api/recording/stop', {})
            assert saw_gap and saw_recovery, health
            gap_audit = api(f'/api/datasets/{Path(gap_start["path"]).name}/audit', {})
            assert gap_audit['counts'].get('recording_health', 0) >= 1, gap_audit
            print(json.dumps({'normal_counts': {key: len(value) for key, value in messages.items()},
                              'normal_audit_ok': audit['ok'], 'button_edges': len(edges),
                              'interruption_detected': saw_gap, 'recovery_detected': saw_recovery,
                              'gap_audit_counts': gap_audit['counts']}))
        finally:
            if child.poll() is None:
                # Let the demo close its own web child and flush recording workers.
                child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=25)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
