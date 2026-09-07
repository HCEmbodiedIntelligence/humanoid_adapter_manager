"""Application service for SDK/envelope sources, sessions and background exports."""
from dataclasses import asdict
import importlib.metadata
import json
from pathlib import Path
import re
import threading
import time
import uuid

from .adapters import preflight
from .catalog import read_json, session_path, summary
from .clocks import Clocks
from .config import DEFAULT, validate, config_hash
from .session import Session
from .storage import atomic_json


class CaptureHub:
    def __init__(self,state_root,ros_config):
        self.state_root=Path(state_root)
        self.config_path=self.state_root/'capture-config.json'
        self.cfg=validate(read_json(self.config_path,DEFAULT))
        self.ros_config=ros_config
        self.clocks=Clocks()
        self.clocks.add({'id':'host_receive_v1','clock_id':'host_monotonic','epoch':0,
            'device_origin_ns':0,'common_origin_ns':0,'uncertainty_ns':None,'evidence':''})
        self.reports={}
        self.session=None
        self.ros_bridge=None
        self.adapters={}
        self.adapter_threads=[]
        self.adapter_stop=threading.Event()
        self.job={'state':'idle'}
        self.job_thread=None
        self.directory.mkdir(parents=True,exist_ok=True)
        # A prior process's active files are not committed just because they exist.
        for path in self.directory.glob('*/manifest.json'):
            manifest=read_json(path,{})
            if manifest.get('state') in {'recording','warming_up','draining'}:
                manifest.update(state='interrupted',recovery='仅已提交清单内的文件可直接使用；活动文件需检查恢复')
                atomic_json(path,manifest)

    @property
    def directory(self):
        path=Path(self.cfg['storage']['directory']).expanduser()
        return path if path.is_absolute() else self.state_root/path

    def busy(self):
        return bool(self.session and not self.session.closed.is_set()) or bool(self.job_thread and self.job_thread.is_alive())

    def state(self):
        return {'config':self.cfg,'etag':config_hash(self.cfg),'directory':str(self.directory),
            'status':self.session.status() if self.session else {'state':'idle','active':False},
            'preflight':preflight(self.cfg,self.reports,self.clocks),'job':self.job,
            'available_adapters':[],'camera_architecture':'external_ros_node',
            'ros_input_error':self.ros_bridge.error if self.ros_bridge else None}

    def save_config(self,value,etag):
        if self.busy():
            raise ValueError('正在采集或处理数据，请结束后修改采集策略')
        if etag!=config_hash(self.cfg):
            raise ValueError('采集配置已变化，请重新读取')
        cfg=validate(value)
        self.close_adapters()
        self.cfg=cfg
        self.reports={}
        atomic_json(self.config_path,cfg)
        self.directory.mkdir(parents=True,exist_ok=True)
        return self.state()

    def register_clock(self,value):
        result=self.clocks.add(value)
        if self.session and not self.session.closed.is_set():
            self.session.storage.required('clock_model',result)
        return result

    def register_report(self,ident,report):
        if self.busy():
            raise ValueError('运行中更新能力需要先停止并重新验证配置')
        if ident not in self.cfg['sources'] or self.cfg['sources'][ident]['transport']!='envelope':
            raise ValueError('能力报告接口仅用于已配置的 envelope 驱动来源')
        if not isinstance(report,dict):
            raise ValueError('能力报告必须为对象')
        encoded=json.dumps(report,allow_nan=False)
        if len(encoded.encode())>1024*1024:
            raise ValueError('能力报告超过 1 MiB，请引用独立校准资源')
        reports={**self.reports,ident:{**json.loads(encoded),'registered_monotonic_ns':time.monotonic_ns(),'config_hash':config_hash(self.cfg)}}
        try:
            result=preflight(self.cfg,reports,self.clocks)
        except (TypeError,AttributeError) as error:
            raise ValueError('能力报告字段格式无效: '+str(error)) from error
        self.reports=reports
        return result

    def prepare_adapters(self):
        if self.busy():
            raise ValueError('采集期间不能重建输入订阅')
        self.close_adapters()
        if any(s['transport'].startswith('ros_') for s in self.cfg['sources'].values()):
            if not self.ros_config['enabled']:
                raise ValueError('请启用 ROS 连接，独立相机节点与管理器须使用同一 domain_id')
            from .ros_bridge import RosSources
            self.ros_bridge=RosSources(self,self.ros_config)
            self.ros_bridge.start()
        return preflight(self.cfg,self.reports,self.clocks)

    def start(self,metadata=None):
        if self.busy():
            raise ValueError('采集或数据处理仍在进行')
        if any(s['transport'].startswith('ros_') for s in self.cfg['sources'].values()) and not self.ros_bridge:
            self.prepare_adapters()
        self.session=Session(self.directory,self.cfg,self.clocks,self.reports,metadata)
        return self.session.start()

    def close_adapters(self):
        # This closes subscriptions only. Camera hardware belongs to its own node.
        if self.ros_bridge:
            self.ros_bridge.stop()
            self.ros_bridge=None

    def stop(self):
        try:
            return self.session.stop() if self.session and not self.session.closed.is_set() else self.state()['status']
        finally:
            self.close_adapters()
            if self.ros_bridge:
                self.ros_bridge.stop()
                self.ros_bridge=None

    def sessions(self):
        result=[]
        for path in sorted(self.directory.iterdir(),reverse=True):
            if path.is_dir() and not path.is_symlink() and (path/'session.json').exists():
                value=summary(path)
                result.append({key:value.get(key) for key in ('session_id','manifest','row_count','valid_rows','persistence_gap_count')})
        return result

    def start_job(self,operation,ident,data):
        if self.busy():
            raise ValueError('请等待当前采集或数据处理结束')
        path=session_path(self.directory,ident)
        self.job={'state':'starting','operation':operation,'session_id':ident}
        def progress(value):
            self.job={**self.job,**value}
        def run():
            try:
                if operation=='export':
                    from .exporter import export
                    name=data.get('name') or 'dataset_'+uuid.uuid4().hex[:8]
                    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,80}',name):
                        raise ValueError('导出名称只允许字母、数字、下划线或横线')
                    output=self.state_root/'datasets'/name
                    output.parent.mkdir(parents=True,exist_ok=True)
                    result=export(path,output,data.get('version','online'),progress)
                elif operation=='realign':
                    from .offline import realign
                    result=realign(path,progress)
                else:
                    raise ValueError('未知数据操作')
                self.job={**self.job,'state':'completed','result':result}
            except Exception as error:
                self.job={**self.job,'state':'failed','error':str(error)}
        self.job_thread=threading.Thread(target=run,name='session-'+operation,daemon=True)
        self.job_thread.start()
        return self.job
