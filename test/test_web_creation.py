"""Real HTTP/CLI creation, with optional browser coverage and no ROS connection."""
import asyncio
import json
import os
from pathlib import Path
import re
import shutil
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

import pytest
pytest.importorskip('aiohttp')
pytest.importorskip('mcap')
from aiohttp.test_utils import TestClient, TestServer

from humanoid_manager.configuration import ConfigurationManager
from humanoid_manager.deployment import deploy_archive, pack_directory
from humanoid_manager.web.config import ConfigStore
from humanoid_manager.web import server as web_server
from humanoid_manager.web.server import create_app
from test_deployment_plugins import _hardware_tree, _model_tree, _gripper_tree


class WebCreationTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='manager-create-test-')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.manager = ConfigurationManager(self.root / 'plugins', self.root / 'configuration')
        bundles = os.environ.get('HUMANOID_TEST_PLUGIN_BUNDLES')
        for name, factory in [('driver', _hardware_tree), ('model', _model_tree), ('gripper', _gripper_tree)]:
            if bundles:
                self.manager.import_bundle(Path(bundles) / f'{name}.zip')
            else:
                archive = pack_directory(factory(self.root / name), self.root / f'{name}.zip')
                deploy_archive(archive, self.manager.plugin_root)
        catalog = self.manager.catalog()
        self.arguments = {
            'robot_id': 'openarmx_01', 'name': 'OpenArmX 双臂',
            'driver_id': next(iter(catalog['hardware_drivers'])),
            'model_id': next(iter(catalog['robot_models'])),
            'gripper_id': next(iter(catalog['gripper_drivers'])),
        }
        store = self.store = ConfigStore(self.root / 'manager.yaml')
        store.save({'adapter_manager': {
            'cli': str(Path(__file__).resolve().parents[1] / 'scripts/humanoid_pluginctl.py'),
            'plugin_root': str(self.manager.plugin_root), 'state_root': str(self.manager.state_root)},
            'ros': {'enabled': False, 'recording': {'directory': str(self.root / 'recordings')}}})
        self.client = TestClient(TestServer(create_app(store)))
        await self.client.start_server()
        self.addAsyncCleanup(self.client.close)

    async def test_page_scripts_and_styles_are_served(self):
        page = await self.client.get('/dashboard/')
        self.assertEqual(page.status, 200)
        assets = re.findall(r'(?:src|href)="(/static/[^"]+)"', await page.text())
        self.assertIn('/static/configurator.js', assets)
        for path in assets:
            response = await self.client.get(path)
            self.assertEqual(response.status, 200, path)
            self.assertTrue(await response.read(), path)

    async def test_symlink_install_serves_scripts_from_resolved_module(self):
        source = Path(web_server.__file__).resolve()
        installed = self.root / 'symlink-install/web'
        (installed / 'static').mkdir(parents=True)
        (installed / 'server.py').symlink_to(source)
        (installed / 'static/configurator.js').symlink_to(source.parent / 'static/configurator.js')
        with patch.object(web_server, '__file__', str(installed / 'server.py')):
            client = TestClient(TestServer(create_app(self.store)))
        await client.start_server()
        self.addAsyncCleanup(client.close)
        response = await client.get('/static/configurator.js')
        self.assertEqual(response.status, 200, await response.text())
        self.assertIn('createRobotPayload', await response.text())

    async def test_actual_http_cli_creation_copy_and_duplicate(self):
        arguments = {**self.arguments, 'robot_id': ' \topenarmx_01\n'}
        response = await self.client.post('/api/adapters/robots', json=arguments)
        self.assertEqual(response.status, 201, await response.text())
        robot = await response.json()
        self.assertEqual(robot['robot_id'], 'openarmx_01')
        detail = await self.client.get('/api/adapters/robots/openarmx_01')
        self.assertEqual((await detail.json())['latest'], robot['latest'])
        duplicate = await self.client.post('/api/adapters/robots', json=self.arguments)
        self.assertEqual(duplicate.status, 409, await duplicate.text())
        copied = await self.client.post('/api/adapters/robots', json={
            'robot_id': 'openarmx_02', 'name': '复制的配置', 'source_workspace': 'openarmx_01'})
        self.assertEqual(copied.status, 201, await copied.text())
        self.assertEqual(self.manager.get('openarmx_01')['latest'], robot['latest'])
        self.assertEqual(len(self.manager.catalog()['workspaces']), 2)

    async def test_actual_http_errors_identify_fields_without_creating_configuration(self):
        for field in ('robot_id', 'driver_id', 'model_id'):
            response = await self.client.post('/api/adapters/robots', json={**self.arguments, field: ''})
            self.assertEqual(response.status, 400, await response.text())
            self.assertIn(field, await response.text())
        for field in ('robot_id', 'name'):
            arguments = dict(self.arguments)
            arguments.pop(field)
            response = await self.client.post('/api/adapters/robots', json=arguments)
            self.assertEqual(response.status, 400, await response.text())
            self.assertIn(field, await response.text())
        response = await self.client.post('/api/adapters/robots', json={**self.arguments, 'model_id': 'missing_model'})
        self.assertEqual(response.status, 400, await response.text())
        self.assertIn('模型插件 missing_model 不存在', await response.text())
        self.assertEqual(self.manager.catalog()['workspaces'], [])

    async def test_browser_form_submits_selected_plugins_and_persists(self):
        node = shutil.which('node')
        if not node:
            self.skipTest('Browser regression requires Node and Playwright')
        script = Path(__file__).with_name('web_creation_browser.cjs')
        process = await asyncio.create_subprocess_exec(
            node, str(script), str(self.client.make_url('/')), json.dumps(self.arguments),
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
        try:
            output, _ = await asyncio.wait_for(process.communicate(), timeout=75)
        except (asyncio.TimeoutError, asyncio.CancelledError):
            process.kill()
            await process.wait()
            raise
        message = output.decode(errors='replace')
        if process.returncode == 77:
            self.skipTest(message)
        self.assertEqual(process.returncode, 0, message)
        self.assertEqual(self.manager.get('openarmx_01')['name'], 'OpenArmX 双臂')

    async def test_browser_lifecycle_buttons_keep_web_alive(self):
        node = shutil.which('node')
        if not node:
            self.skipTest('Browser regression requires Node and Playwright')
        response = await self.client.post('/api/adapters/robots', json=self.arguments)
        self.assertEqual(response.status, 201, await response.text())
        runtime = self.client.server.app['runtime']
        runtime.launcher.enabled = True
        runtime.require_robot_stopped = Mock()
        real_spawn = asyncio.create_subprocess_exec
        async def safe_spawn(*command, **kwargs):
            if command[:2] == ('ros2', 'launch'):
                return await real_spawn(sys.executable, '-c', 'import time; time.sleep(120)', **kwargs)
            return await real_spawn(*command, **kwargs)
        # Fake plugin files are NEVER loaded: only the child launch invocation
        # is replaced with a harmless process. The web API/CLI remain real.
        with patch('humanoid_manager.web.robot_launcher.asyncio.create_subprocess_exec', side_effect=safe_spawn):
            process = await asyncio.create_subprocess_exec(node, str(Path(__file__).with_name('web_launcher_browser.cjs')),
                str(self.client.make_url('/')), stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
            try:
                output, _ = await asyncio.wait_for(process.communicate(), timeout=75)
            except (asyncio.TimeoutError, asyncio.CancelledError):
                process.kill()
                await process.wait()
                raise
            finally:
                await runtime.launcher.stop()
        message = output.decode(errors='replace')
        if process.returncode == 77:
            self.skipTest(message)
        self.assertEqual(process.returncode, 0, message)
