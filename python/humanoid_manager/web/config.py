"""Standalone dashboard and recording configuration. No legacy application imports."""
import copy
import os
from pathlib import Path
import re
import tempfile

import yaml


class ConfigError(ValueError):
    pass


DEFAULT_CONFIG = {
    'server': {'host':'127.0.0.1','port':7876},
    'adapter_manager': {'enabled':True,'cli':'','plugin_root':'','state_root':''},
    'data_quality': {'button_topic':'/hc_teleop_recv/buttons', 'button_enabled':False,
                     'controller':'right', 'button':'secondary', 'scope':'segment', 'gap_rules':[]},
    'ros': {'enabled':True,'domain_id':14,'node_name':'humanoid_configurator',
            'subscriptions':[], 'recording':{'directory':'recordings'}},
}


def merge(base, overrides):
    result=copy.deepcopy(base)
    for key,value in overrides.items():
        if isinstance(value,dict) and isinstance(result.get(key),dict):
            result[key]=merge(result[key],value)
        else:
            result[key]=copy.deepcopy(value)
    return result


def validate_config(config):
    if not isinstance(config,dict):
        raise ConfigError('配置根节点必须为对象')
    value=merge(DEFAULT_CONFIG,config)
    for section in ('server','adapter_manager','ros'):
        if not isinstance(value[section],dict):
            raise ConfigError(f'{section} 必须为对象')
    manager=value['adapter_manager']
    for key in ('cli','plugin_root','state_root'):
        if not isinstance(manager.get(key),str) or not Path(manager[key]).is_absolute():
            raise ConfigError(f'adapter_manager.{key} 必须是绝对路径')
    manager['enabled']=True
    port=value['server']['port']
    if type(port) is not int or not 1<=port<=65535:
        raise ConfigError('网页端口必须在 1–65535 之间')
    if not isinstance(value['server']['host'],str) or not value['server']['host'].strip():
        raise ConfigError('网页监听地址不能为空')
    ros=value['ros']
    quality=value['data_quality']
    if not isinstance(quality,dict) or type(quality.get('button_enabled')) is not bool:
        raise ConfigError('按钮标记开关必须为布尔值')
    if quality.get('controller') not in {'left','right'} or quality.get('button') not in {'primary','secondary','menu','primary_axis_click','secondary_axis_click','grip_button','trigger_button'}:
        raise ConfigError('无效的标记手柄或按钮')
    if quality.get('scope') not in {'segment','recording','frame'}:
        raise ConfigError('无效的标记范围')
    if not isinstance(quality.get('button_topic'),str) or not re.fullmatch(r'/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*',quality['button_topic']):
        raise ConfigError('按钮话题必须为绝对 ROS 话题名')
    if not isinstance(quality['gap_rules'],list):
        raise ConfigError('数据间隔规则必须为列表')
    from .datasets import finite_number
    for rule in quality['gap_rules']:
        if not isinstance(rule,dict) or not isinstance(rule.get('topic'),str) or not rule['topic'].startswith('/'):
            raise ConfigError('数据间隔规则话题无效')
        finite_number(rule.get('max_gap_seconds'), '最大消息间隔', .001, 3600)
    if type(ros['enabled']) is not bool:
        raise ConfigError('ROS 开关必须为布尔值')
    if type(ros['domain_id']) is not int or not 0<=ros['domain_id']<=232:
        raise ConfigError('ROS Domain ID 必须在 0–232 之间')
    if not isinstance(ros['node_name'],str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*',ros['node_name']):
        raise ConfigError('ROS 节点名称无效')
    recording=ros['recording']
    if not isinstance(recording,dict) or not isinstance(recording.get('directory'),str) or not recording['directory'].strip():
        raise ConfigError('录制目录不能为空')
    from ..configuration import validate_recording, DeploymentError
    try:
        validate_recording({'directory':recording['directory'],'subscriptions':ros['subscriptions']})
    except DeploymentError as error:
        raise ConfigError(str(error)) from error
    for item in ros['subscriptions']:
        item.setdefault('enabled',True)
        item.setdefault('outputs',['record'])
        item.setdefault('max_hz',0.0)
        item.setdefault('event_max_hz',0.0)
    for topic,kind in {'/hc_teleop/joint_states':'sensor_msgs/msg/JointState',
                      '/hc_teleop/joint_cmd':'sensor_msgs/msg/JointState',
                      quality['button_topic']:'std_msgs/msg/String'}.items():
        item=next((x for x in ros['subscriptions'] if x['topic']==topic),None)
        if item is None:
            item={'topic':topic,'type':kind,'enabled':True,'outputs':['websocket'],'max_hz':0.0,'event_max_hz':20.0}
            ros['subscriptions'].append(item)
        if item['type']!=kind:
            raise ConfigError(f'{topic} 消息类型应为 {kind}')
        item['enabled']=True
        if 'websocket' not in item['outputs']:
            item['outputs'].append('websocket')
        if topic == quality['button_topic'] and quality['button_enabled']:
            if 'record' not in item['outputs']:
                item['outputs'].append('record')
            item['max_hz']=0
    for key in ('vr','safety','camera','robot_profiles'):
        value.pop(key,None)
    ros.pop('command_mux',None)
    return value


class ConfigStore:
    def __init__(self,path):
        self.path=Path(path).expanduser().resolve()
        self.value={}

    def load(self):
        if not self.path.exists():
            raise ConfigError(f'配置文件不存在: {self.path}')
        self.value=validate_config(yaml.safe_load(self.path.read_text(encoding='utf-8')))
        return copy.deepcopy(self.value)

    def save(self,value):
        value=validate_config(value)
        self.path.parent.mkdir(parents=True,exist_ok=True)
        fd,name=tempfile.mkstemp(dir=self.path.parent,prefix='.settings-')
        try:
            with os.fdopen(fd,'w',encoding='utf-8') as stream:
                yaml.safe_dump(value,stream,allow_unicode=True,sort_keys=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(name,self.path)
        finally:
            if os.path.exists(name):
                os.unlink(name)
        self.value=value
        return copy.deepcopy(value)
