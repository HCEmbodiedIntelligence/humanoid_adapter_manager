import pytest
pytest.importorskip("aiohttp")
pytest.importorskip("mcap")
import copy
import json
from pathlib import Path
import tempfile
import time
import unittest

from aiohttp.test_utils import TestClient, TestServer
from mcap.reader import make_reader

from humanoid_manager.web.server import create_app
from humanoid_manager.web.config import ConfigStore
from humanoid_manager.web.protocol import envelope
from humanoid_manager.web.topic_recorder import RECORDING_METADATA_NAME


class ConfiguratorTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.store = ConfigStore(self.root / 'manager.yaml')
        self.store.save({'adapter_manager': {'cli': str(Path(__file__).resolve().parents[1]/'scripts/humanoid_pluginctl.py'),
            'plugin_root': str(self.root/'plugins'), 'state_root': str(self.root/'configuration')},
            'ros': {'enabled':False, 'recording': {'directory':str(self.root/'recordings')}}})
        self.app = create_app(self.store)
        self.client = TestClient(TestServer(self.app))
        await self.client.start_server()
        self.runtime = self.app['runtime']

    async def asyncTearDown(self):
        await self.client.close()
        self.temp.cleanup()

    async def state(self):
        return await (await self.client.get('/api/settings')).json()

    async def post(self, operation, data):
        response = await self.client.post('/api/settings/' + operation, json=data)
        self.assertEqual(response.status, 200, await response.text())
        return await response.json()

    async def test_save_during_real_mcap_recording_preserves_writer_and_running_config(self):
        state = await self.state()
        original = copy.deepcopy(self.runtime.config)
        self.runtime.ros.events['configuration'] = (time.monotonic(), {
            'robot_id':'lab', 'name':'实验室', 'revision':'r-0123456789abcdef',
            'fingerprint':'content-identity', 'state':'observed', 'missing_nodes':[]})
        response = await self.client.post('/api/recording/start', json={'filename': 'capture', 'force': True})
        self.assertEqual(response.status, 200, await response.text())
        status_before = self.runtime.recorder.status()
        for i in range(3):
            self.runtime.recorder.record(envelope('test', 'integration', {'index': i}))
        draft = state['draft']
        draft['ros']['recording']['directory'] = str(self.root / 'next-recordings')
        state = await self.post('draft', {'config': draft, 'etag': state['etag']})
        state = await self.post('save', {'etag': state['etag']})
        status_after = self.runtime.recorder.status()
        self.assertTrue(status_after['recording'])
        self.assertEqual(status_before['writer_pid'], status_after['writer_pid'])
        self.assertEqual(self.runtime.config['ros']['recording'], original['ros']['recording'])
        blocked = await self.client.post('/api/settings/apply', json={'etag': state['etag']})
        self.assertEqual(blocked.status, 409)
        self.assertTrue(self.runtime.recorder.is_recording())
        response = await self.client.post('/api/recording/stop')
        self.assertEqual(response.status, 200)
        with Path(status_before['path']).open('rb') as stream:
            reader = make_reader(stream)
            metadata = next(item.metadata for item in reader.iter_metadata() if item.name == RECORDING_METADATA_NAME)
            self.assertEqual(metadata['configuration_revision'], 'r-0123456789abcdef')
            self.assertEqual(metadata['robot_id'], 'lab')
            self.assertEqual(len(list(reader.iter_messages())), 3)
        state = await self.post('apply', {'etag': state['etag']})
        self.assertFalse(state['pending'])
        self.assertEqual(str(self.runtime.recorder.directory), str(self.root / 'next-recordings'))

    async def test_conflicts_and_history_are_visible_without_applying(self):
        first = await self.state()
        config = first['draft']
        config['ros']['node_name'] = 'renamed_node'
        draft = await self.post('draft', {'config': config, 'etag': first['etag']})
        collision = await self.client.post('/api/settings/draft', json={'config': config, 'etag': first['etag']})
        self.assertEqual(collision.status, 409)
        saved = await self.post('save', {'etag': draft['etag']})
        self.assertTrue(saved['pending'])
        self.assertEqual(len(saved['history']), 2)
        original = saved['history'][-1]['revision']
        restored = await self.post('restore', {'etag': saved['etag'], 'revision': original})
        self.assertNotEqual(restored['draft']['ros']['node_name'], 'renamed_node')
        self.assertEqual(restored['saved']['ros']['node_name'], 'renamed_node')
        self.assertNotEqual(self.runtime.config['ros']['node_name'], 'renamed_node')

    async def test_domain_change_requires_restart_and_external_edit_conflicts(self):
        state = await self.state()
        proposed = state['draft']
        proposed['ros']['domain_id'] = 201
        state = await self.post('draft', {'config': proposed, 'etag': state['etag']})
        state = await self.post('save', {'etag': state['etag']})
        state = await self.post('apply', {'etag': state['etag']})
        self.assertTrue(state['server_restart_required'])
        self.assertNotEqual(self.runtime.config['ros']['domain_id'], 201)
        other = ConfigStore(self.store.path)
        value = other.load()
        value['ros']['node_name'] = 'external_node'
        other.save(value)
        response = await self.client.post('/api/settings/save', json={'etag': state['etag']})
        self.assertEqual(response.status, 409)

    async def test_platform_metadata_expires_and_mutations_require_same_origin(self):
        self.runtime.ros.events['configuration'] = (time.monotonic()-10, {'robot_id': 'stale', 'revision':'old'})
        self.assertNotIn('robot_id', self.runtime.recording_metadata())
        response = await self.client.post('/api/ros/publish', json={}, headers={'Origin':'https://unrelated.example'})
        self.assertEqual(response.status, 403)

    async def test_recording_path_cannot_escape_and_duplicate_names_preserve_files(self):
        response = await self.client.post('/api/recording/start', json={'filename':'../escape', 'force':True})
        self.assertEqual(response.status, 400)
        self.assertFalse((self.root / 'escape.mcap').exists())
        first = await (await self.client.post('/api/recording/start', json={'filename':'same','force':True})).json()
        await self.client.post('/api/recording/stop')
        second = await (await self.client.post('/api/recording/start', json={'filename':'same','force':True})).json()
        self.assertNotEqual(first['path'], second['path'])
        self.assertTrue(Path(first['path']).exists())

    async def test_long_dataset_scan_does_not_block_recording_stop(self):
        import asyncio
        import threading
        entered, release = threading.Event(), threading.Event()
        def scan(_filename):
            entered.set()
            release.wait(3)
            return {'ok':True}
        self.runtime.datasets.audit=scan
        response=await self.client.post('/api/recording/start',json={'filename':'parallel','force':True})
        self.assertEqual(response.status,200,await response.text())
        scanning=asyncio.create_task(self.client.post('/api/datasets/other.mcap/audit',json={}))
        try:
            self.assertTrue(await asyncio.to_thread(entered.wait,2))
            stopped=await asyncio.wait_for(self.client.post('/api/recording/stop'),timeout=1)
            self.assertEqual(stopped.status,200)
            self.assertFalse(self.runtime.recorder.is_recording())
            self.assertFalse(scanning.done())
        finally:
            release.set()
            await scanning

    async def test_shutdown_closes_open_browser_websocket_without_waiting_for_http_timeout(self):
        import asyncio
        socket = await self.client.ws_connect('/ws')
        try:
            await asyncio.wait_for(self.client.server.close(),timeout=2)
        finally:
            await socket.close()

    async def test_capture_policy_preflight_lifecycle_and_quality_endpoint(self):
        import asyncio
        response=await self.client.get('/api/capture')
        state=await response.json()
        self.assertFalse(state['preflight']['ready'])
        rejected=await self.client.post('/api/capture/start',json={})
        self.assertEqual(rejected.status,400)
        cfg=state['config']
        cfg['alignment']['strict']=False
        cfg['clock']['domain']='host_monotonic'
        cfg['sources']={'joints':{'kind':'state','transport':'envelope','fields':{'position':{'semantics':'continuous','units':'rad','coordinate_frame':'joint'}}}}
        saved=await self.client.post('/api/capture/config',json={'config':cfg,'etag':state['etag']})
        self.assertEqual(saved.status,200,await saved.text())
        started=await self.client.post('/api/capture/start',json={})
        self.assertEqual(started.status,200,await started.text())
        for seq in range(5):
            now=time.monotonic_ns()
            result=await self.client.post('/api/capture/sample',json={'metadata':{'source_id':'joints','source_seq':seq,
                'source_timestamp_ns':now,'receive_time_ns':now,'clock_model_id':'host_receive_v1','clock_id':'host_monotonic',
                'clock_epoch':0,'timestamp_quality':'host_receive','payload':{'position':[float(seq)]}}})
            self.assertEqual(result.status,200,await result.text())
            await asyncio.sleep(.01)
        settings=await self.state()
        busy=await self.client.post('/api/settings/apply',json={'etag':settings['etag']})
        self.assertEqual(busy.status,409)
        marked=await self.client.post('/api/capture/mark',json={'scope':'segment'})
        self.assertEqual(marked.status,200,await marked.text())
        stopped=await self.client.post('/api/capture/stop',json={})
        self.assertEqual(stopped.status,200,await stopped.text())
        sessions=await (await self.client.get('/api/sessions')).json()
        ident=sessions['sessions'][0]['session_id']
        row=await (await self.client.get(f'/api/sessions/{ident}/row?k=0')).json()
        self.assertFalse(row['exportable'])
        self.assertEqual(row['persistence_state'],'committed')
        escape=await self.client.get(f'/api/sessions/{ident}/file?path=../capture-config.json')
        self.assertEqual(escape.status,404)
