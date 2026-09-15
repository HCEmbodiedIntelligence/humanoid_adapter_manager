"""Vendor launch contexts and shutdown are exercised without ROS hardware."""
import os
import signal
import subprocess
import time

import pytest


@pytest.mark.parametrize('vendor_failure', [False, True])
def test_delayed_vendor_arguments_and_owned_shutdown(tmp_path, vendor_failure):
    worker = tmp_path / 'worker.py'
    worker.write_text('''
import os, signal, sys
from pathlib import Path
root = Path(os.environ['VENDOR_TEST_ROOT'])
def stop(*args):
    (root / 'stopped').touch()
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
(root / 'pid').write_text(str(os.getpid()))
while True: signal.pause()
''')
    vendor = tmp_path / 'vendor.launch.py'
    vendor.write_text('''
import json, os, sys
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
def fail(context):
    raise RuntimeError('vendor test failure')
def after_initializer(context):
    root = Path(os.environ['VENDOR_TEST_ROOT'])
    values = {key: LaunchConfiguration(key).perform(context)
              for key in ('arm_prefix', 'robot_controller', 'controllers_file')}
    (root / 'arguments.json').write_text(json.dumps(values))
    actions = [ExecuteProcess(cmd=[sys.executable, str(root / 'worker.py')])]
    if os.environ['VENDOR_TEST_FAILURE'] == '1':
        actions.append(TimerAction(period=0.5, actions=[OpaqueFunction(function=fail)]))
    return actions
def generate_launch_description():
    initializer = ExecuteProcess(cmd=[sys.executable, '-c', 'import time; time.sleep(.1)'])
    return LaunchDescription([
        DeclareLaunchArgument('arm_prefix', default_value='default_vendor'),
        DeclareLaunchArgument('robot_controller', default_value='wrong_default'),
        DeclareLaunchArgument('controllers_file', default_value='wrong.yaml'),
        RegisterEventHandler(OnProcessExit(target_action=initializer,
            on_exit=[OpaqueFunction(function=after_initializer)])),
        initializer,
    ])
''')
    parent = tmp_path / 'parent.launch.py'
    parent.write_text('''
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from humanoid_manager.startup import vendor_launch_actions
def check_platform(context):
    root = Path(os.environ['VENDOR_TEST_ROOT'])
    (root / 'platform').write_text(LaunchConfiguration('arm_prefix').perform(context))
    return []
def generate_launch_description():
    root = Path(os.environ['VENDOR_TEST_ROOT'])
    command = ['ros2', 'launch', str(root / 'vendor.launch.py'),
               'arm_prefix:=robot_01', 'robot_controller:=forward_position_controller',
               'controllers_file:=split controllers.yaml']
    return LaunchDescription([
        DeclareLaunchArgument('arm_prefix', default_value='platform'),
        *vendor_launch_actions(command),
        TimerAction(period=.2, actions=[OpaqueFunction(function=check_platform)]),
    ])
''')
    log_path = tmp_path / 'launch.log'
    with log_path.open('w') as log:
        process = subprocess.Popen(
            ['ros2', 'launch', str(parent)], stdout=log, stderr=log,
            start_new_session=True,
            env={**os.environ, 'VENDOR_TEST_ROOT': str(tmp_path),
                 'VENDOR_TEST_FAILURE': str(int(vendor_failure)),
                 'ROS_LOG_DIR': str(tmp_path / 'logs')})
        try:
            deadline = time.monotonic() + 15
            while not (tmp_path / 'pid').exists():
                assert process.poll() is None, log_path.read_text()
                assert time.monotonic() < deadline, log_path.read_text()
                time.sleep(.05)
            if vendor_failure:
                process.wait(timeout=15)
                assert '底层 launch 已退出' in log_path.read_text()
            else:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=15)
                assert process.returncode == 0, log_path.read_text()
            import json
            assert json.loads((tmp_path / 'arguments.json').read_text()) == {
                'arm_prefix': 'robot_01', 'robot_controller': 'forward_position_controller',
                'controllers_file': 'split controllers.yaml'}
            assert (tmp_path / 'platform').read_text() == 'platform'
            assert (tmp_path / 'stopped').exists(), log_path.read_text()
            with pytest.raises(ProcessLookupError):
                os.kill(int((tmp_path / 'pid').read_text()), 0)
        finally:
            # Clean up the entire test-owned group, including orphaned children.
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=5)
