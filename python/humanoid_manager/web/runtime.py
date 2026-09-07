"""Independent ROS monitoring and MCAP recording owned by the manager."""
import asyncio
import copy
from pathlib import Path
import time

from aiohttp import web

from .observer import RosObserver
from .ros_recording_executor import RosRecordingExecutor
from .settings_workspace import fingerprint
from .topic_recorder import TopicRecorder
from .datasets import Datasets, DatasetPlayer
from .quality import DataQuality
from ..recording.hub import CaptureHub


class PlatformRuntime:
    def __init__(self,config,state_root):
        self.config=copy.deepcopy(config)
        self.state_root=Path(state_root)
        self.ros=None
        self.recorder=None
        self.recording_ros=None
        self.player=None
        self.websockets=set()
        self.loop=None
        self.started_at=time.time()
        self.quality=DataQuality(self)
        self.health_task=None
        self.capture=None

    async def start(self):
        self.loop=asyncio.get_running_loop()
        self.capture=CaptureHub(self.state_root,self.config['ros'])
        self.recorder=TopicRecorder(self.config['ros']['recording'],self.state_root)
        self.recording_ros=RosRecordingExecutor(self.config['ros'],self.recorder)
        self.recorder.quality_rules=self.config['data_quality']['gap_rules']
        self.datasets=Datasets(self.recorder)
        self.player=DatasetPlayer(self.datasets,self.emit)
        self.ros=RosObserver(self.config['ros']['enabled'],self.config['ros']['domain_id'],self.config['ros']['subscriptions'],self.observe,
            self.config['ros']['node_name'],self.config['data_quality']['button_topic'])
        self.ros.start()
        self.recording_ros.start()
        self.health_task=asyncio.create_task(self.watch_health())

    async def watch_health(self):
        while True:
            await asyncio.sleep(1)
            try:
                self.quality.check()
            except OSError as error:
                self.emit({'kind':'data_quality_error','payload':str(error)})

    def observe(self,event):
        if event.get('topic') == self.config['data_quality']['button_topic']:
            if self.capture and self.capture.session:
                self.capture.session.record_button(event)
            self.loop.call_soon_threadsafe(self.quality.buttons,event)
        self.emit(event)

    async def stop(self):
        if self.capture:
            await asyncio.to_thread(self.capture.stop)
        if self.health_task:
            self.health_task.cancel()
            try:
                await self.health_task
            except asyncio.CancelledError:
                pass
        if self.player:
            await asyncio.to_thread(self.player.stop)
        if self.recorder:
            await asyncio.to_thread(self.recorder.stop)
        if self.recording_ros:
            await asyncio.to_thread(self.recording_ros.stop)
        if self.ros:
            await asyncio.to_thread(self.ros.stop)

    async def restart(self,config):
        if self.capture and self.capture.busy():
            raise web.HTTPConflict(text='对齐采集或数据处理正在运行，请完成后再应用')
        if self.recorder and self.recorder.is_recording():
            raise web.HTTPConflict(text='正在录制，请停止后再应用')
        await self.stop()
        self.config=copy.deepcopy(config)
        await self.start()

    def emit(self,event):
        if self.loop and not self.loop.is_closed():
            self.loop.call_soon_threadsafe(lambda:asyncio.create_task(self.broadcast(event)))

    async def broadcast(self,event):
        for ws in tuple(self.websockets):
            try:
                await ws.send_json(event)
            except (ConnectionError,RuntimeError):
                self.websockets.discard(ws)

    def recording_metadata(self):
        platform=self.ros.platform_status()
        current=platform['configuration']
        metadata={'configuration_state':'unknown'}
        if current and current['fresh']:
            identity=current['data']
            metadata.update(robot_id=identity.get('robot_id',''),profile_id=identity.get('robot_id',''),
                profile_display_name=identity.get('name',''),robot_name=identity.get('name',''),
                configuration_revision=identity.get('revision',''),configuration_fingerprint=identity.get('fingerprint',''),
                configuration_state=identity.get('state','unknown'))
        receiver=platform['teleop']
        if receiver and receiver['fresh']:
            data=receiver['data'].get('configuration',{})
            metadata['receiver_config_sha256']=data.get('sha256','')
            if not metadata.get('robot_id') and data.get('robot_id'):
                metadata.update(robot_id=data['robot_id'],profile_id=data['robot_id'])
        metadata.update(recording_config_fingerprint=fingerprint(self.config),ros_domain_id=str(self.config['ros']['domain_id']))
        return metadata

    def status(self):
        ros=self.ros.status()
        recording=self.recording_ros.status()
        ros['topic_health']={**ros.get('topic_health',{}),**recording.get('topic_health',{})}
        return {'status':'ok','uptime_seconds':round(time.time()-self.started_at,1),
                'ros':ros,'recording_executor':recording,'recording':self.recorder.status(),
                'capture':self.capture.state()['status'],'capture_job':self.capture.job,
                'replay':self.player.status(),'data_quality':self.quality.status(),
                'platform':self.ros.platform_status(),'config_fingerprint':fingerprint(self.config)}

    def precheck(self):
        health=self.recording_ros.get_topic_health()
        rules=[x for x in self.config['ros']['subscriptions'] if x['enabled'] and 'record' in x['outputs']]
        topics=[health.get(x['topic'],{'topic':x['topic'],'state':'no_data','hz':0,'min_hz':1,'reason':'尚未收到数据'}) for x in rules]
        issues=[{**x,'reason':x.get('message',x.get('reason','话题未就绪'))} for x in topics if x['state']!='ok']
        return {'ok':True,'ready':bool(topics) and not issues,'topic_count':len(topics),'topics':topics,'issues':issues,
                'reason':'未勾选录制话题' if not topics else ('部分话题没有数据或频率不足' if issues else '')}

    async def start_recording(self,data):
        if self.capture and self.capture.busy():
            raise web.HTTPConflict(text='对齐采集或数据处理正在运行')
        if self.player.status()['is_active']:
            raise web.HTTPConflict(text='请停止回放后再开始录制')
        if self.recorder.is_recording():
            raise web.HTTPConflict(text='已经在录制，请先停止当前录制')
        check=self.precheck()
        if check['topic_count']==0:
            raise web.HTTPBadRequest(text='未选择任何录制话题')
        if not check['ready'] and not data.get('force',False):
            return web.json_response({**check,'ok':False,'can_force':True},status=400)
        filename=str(data.get('filename','')).strip()
        if filename and (Path(filename).name!=filename or '\\' in filename or filename.startswith('.') or '\x00' in filename):
            raise web.HTTPBadRequest(text='文件名不能包含目录或以点开头')
        self.recording_ros.prepare_recording()
        self.quality.reset()
        try:
            path=await asyncio.to_thread(self.recorder.start,filename,self.recording_metadata())
        except (OSError,RuntimeError) as error:
            raise web.HTTPServiceUnavailable(text=f'无法启动录制：{error}') from error
        return web.json_response({'ok':True,'recording':True,'path':path})
