"""Declarative driver startup: no robot names or transport-specific branches."""
import os
from pathlib import Path
import re

from .deployment import DeploymentError


def validate_startup(entries):
    if not isinstance(entries, list) or len(entries) > 32:
        raise DeploymentError('plugin startup must be a list of at most 32 steps')
    for step in entries:
        if (not isinstance(step, dict) or not isinstance(step.get('kind'), str)
                or step['kind'] not in {'node', 'launch'}):
            raise DeploymentError('plugin startup kind must be node or launch')
        is_node = step['kind'] == 'node'
        target = 'executable' if is_node else 'launch_file'
        required = {'kind', 'package', target, 'arguments'}
        optional = {'wait_for_exit'} if is_node else set()
        if not required <= step.keys() or step.keys() - required - optional:
            raise DeploymentError('plugin startup step has missing or unknown fields')
        if not isinstance(step['package'], str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', step['package']):
            raise DeploymentError('plugin startup package is invalid')
        pattern = r'[A-Za-z0-9_][A-Za-z0-9_.-]*' if is_node else r'[A-Za-z0-9_][A-Za-z0-9_.-]*\.launch\.(py|xml|yaml)'
        if not isinstance(step[target], str) or not re.fullmatch(pattern, step[target]):
            raise DeploymentError(f'plugin startup {target} must be a filename, not a path')
        arguments = step['arguments']
        if is_node:
            if not isinstance(arguments, list) or len(arguments) > 128:
                raise DeploymentError('plugin startup node arguments must be a list')
            if type(step.get('wait_for_exit', False)) is not bool:
                raise DeploymentError('plugin startup wait_for_exit must be a boolean')
            values = arguments
        else:
            if not isinstance(arguments, dict) or len(arguments) > 64:
                raise DeploymentError('plugin startup launch arguments must be a mapping')
            if any(not isinstance(key, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', key) for key in arguments):
                raise DeploymentError('plugin startup launch argument name is invalid')
            values = arguments.values()
        if any(not isinstance(value, str) or len(value) > 4096 or '\x00' in value or '$(' in value for value in values):
            raise DeploymentError('plugin startup arguments must be literal strings without $(...)')
        if not is_node and any(value == '' for value in arguments.values()):
            raise DeploymentError('plugin startup launch arguments cannot be empty; omit an argument to use its default')
    return entries


def startup_command(step, environment):
    """Resolve in the selected plugin's prefix before the inherited ROS environment."""
    package = step['package']
    prefixes = environment.get('AMENT_PREFIX_PATH', os.environ.get('AMENT_PREFIX_PATH', ''))
    for prefix in filter(None, prefixes.split(os.pathsep)):
        prefix = Path(prefix)
        if (prefix / 'share/ament_index/resource_index/packages' / package).is_file():
            if step['kind'] == 'node':
                path = prefix / 'lib' / package / step['executable']
                if path.is_file() and os.access(path, os.X_OK):
                    return [str(path), *step['arguments']]
            else:
                path = prefix / 'share' / package / 'launch' / step['launch_file']
                if path.is_file():
                    return ['ros2', 'launch', str(path), *[
                        f'{key}:={value}' for key, value in step['arguments'].items()]]
            break  # A broken overlay must not silently select another installation.
    target = step.get('executable', step.get('launch_file'))
    raise DeploymentError(f'插件启动依赖不存在或不可执行：{package}/{target}；请安装并加载对应 ROS 包')


def startup_actions(steps, continuation):
    """Preflight every step, then gate runtimes on successful one-shot initialization."""
    from launch.actions import ExecuteProcess, Shutdown
    from launch.logging import get_logger

    prepared = []
    for step, environment in steps:
        validate_startup([step])
        prepared.append((step, environment, startup_command(step, environment)))

    def build(index):
        if index == len(prepared):
            return continuation
        step, environment, command = prepared[index]
        wait = step.get('wait_for_exit', False)

        def exited(event, context):
            if context.is_shutdown:
                return []
            if wait and event.returncode == 0:
                return build(index + 1)
            label = step['package'] + '/' + step.get('executable', step.get('launch_file'))
            reason = f'插件启动进程退出：{label}，退出码 {event.returncode}'
            get_logger('plugin_startup').error(reason)
            return [Shutdown(reason=reason)]

        process = ExecuteProcess(cmd=command, additional_env=environment,
                                 output='screen', on_exit=exited)
        return [process] if wait else [process, *build(index + 1)]

    return build(0)
