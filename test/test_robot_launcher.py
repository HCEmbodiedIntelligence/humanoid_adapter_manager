"""Lifecycle tests use harmless owned Python processes, never hardware."""
import asyncio
import copy
import json
import importlib.util
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, Mock, patch

import pytest
pytest.importorskip('aiohttp')
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from humanoid_manager.configuration import ConfigurationConflict
from humanoid_manager.deployment import DeploymentError
from humanoid_manager.runtime_state import acquire_robot_run_lock
from humanoid_manager.startup import StartupPlans, default_plan, validate_plan
from humanoid_manager.web.robot_launcher import RobotLauncher, _group_running, register_launcher_routes


def test_startup_selection_is_explicit_and_persistent(tmp_path):
    plans = StartupPlans(tmp_path)
    assert plans.read()['selected_robot'] == ''
    first = default_plan('first')
    first['start_teleop'] = True
    saved = plans.save(first, 'initial')
    assert StartupPlans(tmp_path).read() == saved
    with pytest.raises(ConfigurationConflict):
        plans.save(default_plan('second'), 'initial')
    saved = plans.save(default_plan('second'), saved['etag'])
    assert saved['profiles']['first']['start_teleop'] is True
    assert saved['selected_robot'] == 'second'
    plans.path.write_text('{broken')
    with pytest.raises(DeploymentError, match='startup.json'):
        plans.read()


@pytest.mark.parametrize('bringup', [
    {'package': 'vendor', 'launch_file': '../escape.launch.py', 'arguments': {}},
    {'package': '', 'launch_file': 'robot.launch.py', 'arguments': {}},
    {'package': 'vendor', 'launch_file': 'robot.launch.py', 'arguments': {'cmd': '$(exec unsafe)'}},
    {'package': 'vendor', 'launch_file': 'robot.launch.py', 'arguments': {'use_mock': True}},
])
def test_invalid_vendor_options_are_rejected(bringup):
    plan = default_plan('robot')
    plan['bringup'] = bringup
    with pytest.raises(DeploymentError):
        validate_plan(plan)


def test_exclusive_robot_run_lease(tmp_path):
    lease = acquire_robot_run_lock(tmp_path)
    try:
        with pytest.raises(DeploymentError, match='重复启动'):
            acquire_robot_run_lock(tmp_path)
    finally:
        lease.close()
    acquire_robot_run_lock(tmp_path).close()


def test_port_bind_failure_never_autostarts_hardware(tmp_path, monkeypatch):
    pytest.importorskip('mcap')
    path = Path(__file__).resolve().parents[1] / 'scripts/configurator_launcher.py'
    spec = importlib.util.spec_from_file_location('configurator_launcher_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(sys, 'argv', [str(path), '--run-robot', '--offline', '--robot-id', 'mock_arm',
        '--state-root', str(tmp_path / 'state'), '--plugin-root', str(tmp_path / 'plugins')])
    begin = Mock()
    monkeypatch.setattr(RobotLauncher, 'begin_autostart', begin)
    async def occupied(_site):
        raise OSError('test port already in use')
    monkeypatch.setattr(web.TCPSite, 'start', occupied)
    with pytest.raises(OSError, match='already in use'):
        module.main()
    begin.assert_not_called()


class RobotLauncherTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='launcher-test-')
        self.root = Path(self.temporary.name)
        self.robot = {'robot_id': 'mock_arm', 'latest': 'rev1', 'etag': 'etag1', 'diff': [],
                      'deployed': None, 'saved': {'resources': {'hc_teleop_config': {}}}}
        async def call(operation, **arguments):
            if operation == 'apply':
                self.robot['deployed'] = {'revision': arguments['revision']}
            return copy.deepcopy(self.robot)
        self.adapter = SimpleNamespace(call=AsyncMock(side_effect=call), config={
            'cli': str(Path(__file__).resolve().parents[1] / 'scripts/humanoid_pluginctl.py'),
            'plugin_root': str(self.root / 'plugins')})
        self.runtime = SimpleNamespace(state_root=self.root, config={'ros': {'domain_id': 230}},
            ros=SimpleNamespace(status=lambda: {'state': 'running', 'graph_age': 0}),
            require_robot_stopped=Mock(), recorder=None, capture=None, player=None)
        self.launcher = RobotLauncher(self.runtime, self.adapter, enabled=True)
        self.command = None
        real_spawn = asyncio.create_subprocess_exec
        async def spawn(*command, **kwargs):
            self.command = command
            return await real_spawn(sys.executable, '-c', 'import time; time.sleep(120)', **kwargs)
        self.spawn_patch = patch('humanoid_manager.web.robot_launcher.asyncio.create_subprocess_exec', side_effect=spawn)
        self.spawn = self.spawn_patch.start()
        app = web.Application()
        register_launcher_routes(app, self.launcher)
        async def alive(request):
            return web.Response(text='web alive')
        app.router.add_get('/alive', alive)
        self.client = TestClient(TestServer(app))
        await self.client.start_server()

    async def asyncTearDown(self):
        await self.launcher.stop()
        await self.client.close()
        self.spawn_patch.stop()
        self.temporary.cleanup()

    async def post(self, action, data=None, expected=200):
        response = await self.client.post('/api/launcher/' + action, json=data or {})
        content = await response.text()
        self.assertEqual(response.status, expected, content)
        return json.loads(content) if expected == 200 else content

    async def test_start_stop_restart_and_saved_version(self):
        state = await self.post('start', {'robot_id': 'mock_arm', 'revision': 'rev1', 'start_teleop': True})
        self.assertEqual(state['runtime']['phase'], 'running')
        process = self.launcher.processes[0][1]
        self.assertIn('start_teleop:=true', self.command)
        await self.post('start', {'robot_id': 'mock_arm'}, expected=409)
        self.assertEqual(self.spawn.call_count, 1)
        self.robot['latest'] = 'rev2'
        self.assertIsNone(process.returncode)
        self.assertEqual(self.launcher.revision, 'rev1')
        state = await self.post('restart', {'robot_id': 'mock_arm', 'revision': 'rev2'})
        self.assertIsNotNone(process.returncode)
        self.assertEqual(state['runtime']['revision'], 'rev2')
        self.assertEqual(self.spawn.call_count, 2)
        await self.post('stop')
        self.assertEqual(await (await self.client.get('/alive')).text(), 'web alive')
        self.assertEqual(self.launcher.status()['owned_processes'], 0)
        self.assertEqual(StartupPlans(self.root).read()['selected_robot'], 'mock_arm')

    async def test_invalid_restart_does_not_stop_running_robot(self):
        await self.post('start', {'robot_id': 'mock_arm'})
        process = self.launcher.processes[0][1]
        await self.post('restart', {'robot_id': 'mock_arm', 'start_teleop': 'false'}, expected=400)
        await self.post('restart', {'robot_id': 'mock_arm', 'revision': 'stale'}, expected=409)
        self.runtime.recorder = SimpleNamespace(is_recording=lambda: True)
        await self.post('restart', {'robot_id': 'mock_arm'}, expected=409)
        self.assertIsNone(process.returncode)
        self.assertEqual(self.spawn.call_count, 1)

    async def test_crash_keeps_web_and_does_not_auto_restart(self):
        await self.post('start', {'robot_id': 'mock_arm'})
        self.launcher.processes[0][1].terminate()
        for _ in range(100):
            if self.launcher.phase == 'failed':
                break
            await asyncio.sleep(.01)
        self.assertEqual(self.launcher.phase, 'failed')
        self.assertEqual(self.spawn.call_count, 1)
        self.assertEqual((await self.client.get('/alive')).status, 200)
        self.assertEqual((await self.client.get('/api/launcher/log')).status, 200)

    async def test_fresh_install_and_readonly_web_do_not_start_robot(self):
        self.launcher.begin_autostart()
        await self.launcher.auto_task
        self.spawn.assert_not_called()
        self.launcher.enabled = False
        await self.post('start', {'robot_id': 'mock_arm'}, expected=409)
        self.spawn.assert_not_called()

    async def test_autostart_reuses_last_explicit_robot_and_options(self):
        plan = default_plan('mock_arm')
        plan['start_cameras'] = False
        self.launcher.plans.save(plan, 'initial')
        self.launcher.begin_autostart()
        await self.launcher.auto_task
        self.assertEqual(self.launcher.robot_id, 'mock_arm')
        self.assertIn('start_cameras:=false', self.command)

    async def test_log_creation_failure_is_recoverable(self):
        (self.root / 'runtime_logs').write_text('not a directory')
        with self.assertRaises(FileExistsError):
            await self.launcher.start(robot_id='mock_arm')
        self.assertEqual(self.launcher.phase, 'failed')
        self.assertFalse(self.launcher.busy)
        self.spawn.assert_not_called()

    async def test_foreign_ros_nodes_prevent_start_without_touching_processes(self):
        self.runtime.require_robot_stopped.side_effect = web.HTTPConflict(text='外部节点仍在运行')
        await self.post('start', {'robot_id': 'mock_arm'}, expected=409)
        await self.post('stop')
        self.spawn.assert_not_called()

    async def test_stop_escalates_for_orphaned_child_ignoring_sigint(self):
        self.spawn_patch.stop()
        marker = self.root / 'child.ready'
        child = ("import signal,time,pathlib; signal.signal(signal.SIGINT,signal.SIG_IGN); "
                 f"pathlib.Path({str(marker)!r}).touch(); time.sleep(120)")
        parent = f"import subprocess,time; subprocess.Popen([{sys.executable!r}, '-c', {child!r}]); time.sleep(120)"
        process = await asyncio.create_subprocess_exec(sys.executable, '-c', parent,
            start_new_session=True, stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL)
        self.launcher.processes = [('无害测试进程', process)]
        self.launcher.phase = 'running'
        for _ in range(100):
            if marker.exists():
                break
            await asyncio.sleep(.01)
        self.assertTrue(marker.exists())
        process.kill()  # Abrupt parent death; child remains in the owned group.
        await process.wait()
        self.assertTrue(_group_running(process.pid))
        await self.launcher.stop()
        self.assertFalse(_group_running(process.pid))
        self.assertEqual(self.launcher.phase, 'stopped')
