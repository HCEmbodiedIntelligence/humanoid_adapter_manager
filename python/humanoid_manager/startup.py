"""Persistent, explicitly selected robot startup plans (never auto-pick a robot)."""
import copy
import fcntl
import json
from pathlib import Path
import re
import uuid

from .configuration import ConfigurationConflict, _id, atomic_json
from .deployment import DeploymentError


def default_plan(robot_id=''):
    return {'robot_id': robot_id, 'start_teleop': False, 'start_cameras': True,
            'bringup': {'package': '', 'launch_file': '', 'arguments': {}}}


def validate_plan(value):
    if not isinstance(value, dict) or set(value) != set(default_plan()):
        raise DeploymentError('启动设置字段不完整')
    value = copy.deepcopy(value)
    if value['robot_id'] != '':
        _id(value['robot_id'])
    for key in ('start_teleop', 'start_cameras'):
        if type(value[key]) is not bool:
            raise DeploymentError(f'{key} 必须是布尔值')
    bringup = value['bringup']
    if not isinstance(bringup, dict) or set(bringup) != {'package', 'launch_file', 'arguments'}:
        raise DeploymentError('底层启动项需包含 package、launch_file、arguments')
    package, filename, arguments = (bringup[key] for key in ('package', 'launch_file', 'arguments'))
    if not isinstance(package, str) or not isinstance(filename, str):
        raise DeploymentError('底层 ROS 包和 launch 文件需为字符串')
    if bool(package) != bool(filename):
        raise DeploymentError('底层 ROS 包与 launch 文件必须同时填写或同时留空')
    if package and not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', package):
        raise DeploymentError('底层 ROS 包名无效')
    if filename and not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.-]*\.launch\.(py|xml|yaml)', filename):
        raise DeploymentError('请选择 ROS 包内的 .launch.py / .launch.xml / .launch.yaml 文件名，不要填写路径')
    if not isinstance(arguments, dict) or len(arguments) > 64:
        raise DeploymentError('底层 launch 参数必须是 JSON 对象，最多 64 项')
    for key, item in arguments.items():
        if (not isinstance(key, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', key)
                or not isinstance(item, str) or len(item) > 4096 or '\x00' in item):
            raise DeploymentError('底层 launch 参数名无效，或参数值不是字符串')
        # launch substitutions can execute code even without a shell. Only
        # literal values are accepted here; trusted launch files own execution.
        if '$(' in item:
            raise DeploymentError('底层 launch 参数只接受字面值，不允许 $(...) 替换表达式')
    if not package and arguments:
        raise DeploymentError('填写底层 launch 参数前请先选择 ROS 包和 launch 文件')
    if not value['robot_id'] and (package or value['start_teleop']):
        raise DeploymentError('请先选择启动机器人')
    return value


def bringup_command(plan):
    bringup = validate_plan(plan)['bringup']
    if not bringup['package']:
        return None
    from ros2launch.api import get_share_file_path_from_package
    try:
        path = get_share_file_path_from_package(
            package_name=bringup['package'], file_name=bringup['launch_file'])
    except Exception as error:
        raise DeploymentError(f'找不到底层启动文件，请先安装或编译对应 ROS 包：{error}') from error
    if not path or not Path(path).is_file():
        raise DeploymentError('底层 launch 文件不存在')
    return ['ros2', 'launch', str(path), *[
        f'{key}:={value}' for key, value in bringup['arguments'].items()]]


class StartupPlans:
    def __init__(self, state_root):
        self.root = Path(state_root)
        self.path = self.root / 'startup.json'

    def read(self):
        try:
            value = json.loads(self.path.read_text())
        except FileNotFoundError:
            return {'selected_robot': '', 'profiles': {}, 'etag': 'initial'}
        except (OSError, ValueError) as error:
            raise DeploymentError(f'无法读取启动设置 startup.json：{error}；未启动任何机器人') from error
        if (not isinstance(value, dict) or set(value) != {'selected_robot', 'profiles', 'etag'}
                or not isinstance(value['profiles'], dict) or not isinstance(value['etag'], str)
                or not isinstance(value['selected_robot'], str)):
            raise DeploymentError('启动设置文件损坏，请检查 startup.json；未启动任何机器人')
        for key, plan in value['profiles'].items():
            if validate_plan(plan)['robot_id'] != key or not key:
                raise DeploymentError('启动设置中的机器人 ID 不一致')
        if value['selected_robot'] and value['selected_robot'] not in value['profiles']:
            raise DeploymentError('启动机器人没有对应的启动设置')
        return value

    def save(self, plan, etag):
        plan = validate_plan(plan)
        self.root.mkdir(parents=True, exist_ok=True)
        with (self.root / '.startup.lock').open('a+') as stream:
            fcntl.flock(stream, fcntl.LOCK_EX)
            value = self.read()
            if value['etag'] != etag:
                raise ConfigurationConflict('启动设置已被其他窗口修改，请重新读取')
            if plan['robot_id']:
                value['profiles'][plan['robot_id']] = plan
            value.update(selected_robot=plan['robot_id'], etag=uuid.uuid4().hex)
            atomic_json(self.path, value)
        return value
