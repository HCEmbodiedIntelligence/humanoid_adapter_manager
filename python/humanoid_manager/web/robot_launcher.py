"""Supervise only processes explicitly owned by the unified entry point."""
import asyncio
import json
import os
from pathlib import Path
import signal
import tempfile
import time

from aiohttp import web

from ..deployment import DeploymentError
from ..startup import StartupPlans, bringup_command, default_plan, validate_plan


def _group_running(group_id):
    """Linux robot hosts: include orphaned descendants, but not inert zombies."""
    try:
        os.killpg(group_id, 0)
    except ProcessLookupError:
        return False
    # A crashed launch parent can leave nodes alive in its session. Waiting
    # only for that parent's returncode is insufficient before a restart.
    for entry in Path('/proc').iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
            if int(fields[2]) == group_id and fields[0] not in {'Z', 'X'}:
                return True
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError:
            return True  # Cannot prove the process group is stopped.
    return False


class RobotLauncher:
    def __init__(self, runtime, client, *, enabled=False, bringup=None, initial_robot=None):
        self.runtime, self.client, self.enabled = runtime, client, enabled
        self.bringup = bringup
        self.initial_robot = initial_robot or {}
        self.plans = StartupPlans(runtime.state_root)
        self.lock = asyncio.Lock()
        self.processes = []
        self.monitor = None
        self.phase = 'stopped'
        self.robot_id = ''
        self.revision = ''
        self.error = ''
        self.log_path = ''
        self.log_stream = None

    @property
    def busy(self):
        return self.phase in {'waiting', 'starting', 'running', 'stopping'}

    def status(self):
        return {'enabled': self.enabled, 'phase': self.phase, 'robot_id': self.robot_id,
                'revision': self.revision, 'error': self.error, 'log_path': self.log_path,
                'owned_processes': len(self.processes)}

    def state(self):
        return {**self.plans.read(), 'runtime': self.status(),
                'defaults': {**self.initial_robot, 'bringup': self.bringup}}

    async def start(self, robot_id=None, revision=None, start_teleop=None, start_cameras=None):
        async with self.lock:
            if not self.enabled:
                raise web.HTTPConflict(text='当前为仅网页模式，请退出后运行 ros2 launch robot_bringup registered_robot.launch.py')
            if self.busy:
                raise web.HTTPConflict(text='机器人正在启动或运行，不会重复启动')
            self.runtime.require_robot_stopped()
            plans = self.plans.read()
            robot_id = robot_id if robot_id is not None else (
                self.initial_robot.get('robot_id') or plans['selected_robot'])
            if not robot_id:
                raise web.HTTPBadRequest(text='请在左侧选择机器人配置，然后点击“开启机器人”')
            plan = dict(plans['profiles'].get(robot_id, default_plan(robot_id)))
            for key in ('start_teleop', 'start_cameras'):
                if key in self.initial_robot:
                    plan[key] = self.initial_robot[key]
            for key, value in (('start_teleop', start_teleop), ('start_cameras', start_cameras)):
                if value is not None:
                    plan[key] = value
            if self.bringup is not None:
                plan['bringup'] = self.bringup
            plan = validate_plan(plan)
            robot = await self.client.call('get', robot_id=plan['robot_id'])
            if self.bringup is None:
                # The saved robot owns its driver setup. Do not also launch an
                # old command-line integration for the same vendor launch.
                configured = robot['saved'].get('plugin_settings', {}).get('driver', {}).get('startup', [])
                vendor = plan['bringup']
                if any(step.get('kind') == 'launch' and step.get('package') == vendor['package']
                       and step.get('launch_file') == vendor['launch_file'] for step in configured):
                    plan['bringup'] = default_plan()['bringup']
            if robot['diff'] or (revision is not None and revision != robot['latest']):
                raise web.HTTPConflict(text='配置有未保存修改或已被其他窗口更新，请先保存并刷新配置')
            if plan['start_teleop'] and 'hc_teleop_config' not in robot['saved']['resources']:
                raise web.HTTPBadRequest(text='该模型没有遥操作配置，请关闭启动遥操作或更换模型')
            bringup_command(plan)
            if not robot['deployed'] or robot['deployed']['revision'] != robot['latest']:
                # Restart activates exactly the latest validated saved version;
                # the web process stays alive throughout stop/apply/start.
                await self.client.call('apply', robot_id=robot_id, revision=robot['latest'], etag=robot['etag'])
            self.plans.save(plan, plans['etag'])
            # Use the same manager checkout as the web CLI, including after a
            # source-only update; installed launch remains a fallback.
            source = Path(self.client.config['cli']).resolve().parents[1]
            launch = source / 'launch/managed_robot.launch.py'
            if not launch.is_file():
                from ament_index_python.packages import get_package_share_directory
                launch = Path(get_package_share_directory('humanoid_manager')) / 'launch/managed_robot.launch.py'
            command = ['ros2', 'launch', str(launch), f"robot_id:={plan['robot_id']}",
                       f"plugin_root:={self.client.config['plugin_root']}",
                       f"start_teleop:={str(plan['start_teleop']).lower()}",
                       f"start_cameras:={str(plan['start_cameras']).lower()}",
                       'bringup_json:=' + json.dumps(plan['bringup'], ensure_ascii=False)]
            self.robot_id, self.revision = plan['robot_id'], robot['latest']
            self.phase, self.error = 'starting', ''
            try:
                logs = self.runtime.state_root / 'runtime_logs'
                logs.mkdir(parents=True, exist_ok=True)
                fd, self.log_path = tempfile.mkstemp(prefix=f'{self.robot_id}-', suffix='.log', dir=logs)
                self.log_stream = os.fdopen(fd, 'ab', buffering=0)
                env = {**os.environ, 'ROS_DOMAIN_ID': str(self.runtime.config['ros']['domain_id']),
                       'HUMANOID_MANAGER_PID': str(os.getpid())}
                # A single launch owns vendor and platform processes. Its run
                # lock is acquired before the vendor's hardware launch starts.
                for label, argv in [('机器人', command)]:
                    process = await asyncio.create_subprocess_exec(
                        *argv, env=env, stdout=self.log_stream, stderr=asyncio.subprocess.STDOUT,
                        start_new_session=True)
                    self.processes.append((label, process))
                self.phase = 'running'
                self.monitor = asyncio.create_task(self._watch())
            except BaseException as error:
                await self._terminate()
                self.phase, self.error = 'failed', str(error)
                raise
            return self.state()

    async def restart(self, **arguments):
        if not self.enabled:
            raise web.HTTPConflict(text='请使用 registered_robot.launch.py 启动后再操作机器人')
        if self.runtime.recorder and self.runtime.recorder.is_recording():
            raise web.HTTPConflict(text='请先停止录制，再重启机器人')
        if self.runtime.capture and self.runtime.capture.busy():
            raise web.HTTPConflict(text='请先结束采集或数据处理，再重启机器人')
        if self.runtime.player and self.runtime.player.status().get('is_active'):
            raise web.HTTPConflict(text='请先停止回放，再重启机器人')
        robot_id = arguments.get('robot_id') or self.plans.read()['selected_robot']
        robot = await self.client.call('get', robot_id=robot_id)
        if robot['diff'] or (arguments.get('revision') is not None and arguments['revision'] != robot['latest']):
            raise web.HTTPConflict(text='请先保存并刷新机器人配置，再重启')
        await self.stop()
        # ROS discovery may briefly retain nodes after the owned process exits.
        for _ in range(150):
            try:
                self.runtime.require_robot_stopped()
                break
            except web.HTTPConflict:
                await asyncio.sleep(.1)
        return await self.start(**arguments)

    async def _watch(self):
        tasks = {asyncio.create_task(process.wait()): label for label, process in self.processes}
        try:
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            task = next(iter(done))
            async with self.lock:
                self.phase = 'stopping'
                await self._terminate()
                self.phase = 'failed'
                self.error = f'{tasks[task]}进程已退出（{task.result()}）；其余自管进程已停止，不会自动重启。请查看启动日志。'
        finally:
            for task in tasks:
                if not task.done():
                    task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

    async def _terminate(self):
        for sig, timeout in ((signal.SIGINT, 6), (signal.SIGTERM, 3), (signal.SIGKILL, 3)):
            deadline = time.monotonic() + timeout
            for _, process in self.processes:
                # Each child has a dedicated session; never kill by executable
                # name and never signal externally started robot processes.
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    pass
            pending = [process.wait() for _, process in self.processes if process.returncode is None]
            if pending:
                try:
                    await asyncio.wait_for(asyncio.gather(*pending), timeout)
                except asyncio.TimeoutError:
                    continue
            while any(_group_running(process.pid) for _, process in self.processes):
                if time.monotonic() >= deadline:
                    break
                await asyncio.sleep(.05)
            else:
                break
        else:
            raise RuntimeError('受控进程组尚未完全退出；保留监管状态，不能重复启动机器人')
        self.processes.clear()
        if self.log_stream:
            self.log_stream.close()
            self.log_stream = None

    async def stop(self):
        if self.monitor:
            self.monitor.cancel()
            await asyncio.gather(self.monitor, return_exceptions=True)
            self.monitor = None
        async with self.lock:
            self.phase = 'stopping'
            await self._terminate()
            self.phase, self.error = 'stopped', ''
            return self.status()

    def log_tail(self):
        """Keep startup diagnostics alongside recent output, with bounded reads."""
        if not self.log_path:
            return ''
        with Path(self.log_path).open('rb') as stream:
            stream.seek(0, os.SEEK_END)
            size = stream.tell()
            head_bytes, tail_bytes = 64000, 16000
            stream.seek(0)
            if size <= head_bytes + tail_bytes:
                return stream.read(size).decode(errors='replace')
            head = stream.read(head_bytes).decode(errors='replace')
            stream.seek(size - tail_bytes)
            tail = stream.read(tail_bytes).decode(errors='replace')
        skipped = size - head_bytes - tail_bytes
        return (f'{head}\n\n……中间省略 {skipped} 字节，以下为最新日志……\n'
                f'完整日志文件：{self.log_path}\n\n{tail}')


def register_launcher_routes(app, launcher):
    operation_lock = asyncio.Lock()

    async def state(_request):
        try:
            return web.json_response(launcher.state())
        except DeploymentError as error:
            raise web.HTTPBadRequest(text=str(error)) from error

    async def start(request):
        data = await request.json() if request.can_read_body else {}
        if not isinstance(data, dict) or set(data) - {'robot_id', 'revision', 'start_teleop', 'start_cameras'}:
            raise web.HTTPBadRequest(text='机器人启动参数无效')
        for key in ('robot_id', 'revision'):
            if key in data and (not isinstance(data[key], str) or not data[key]):
                raise web.HTTPBadRequest(text=f'{key} 必须是非空字符串')
        for key in ('start_teleop', 'start_cameras'):
            if key in data and type(data[key]) is not bool:
                raise web.HTTPBadRequest(text=f'{key} 必须是布尔值')
        if operation_lock.locked():
            raise web.HTTPConflict(text='正在处理机器人操作，请稍候')
        try:
            async with operation_lock:
                operation = launcher.restart if request.path.endswith('/restart') else launcher.start
                return web.json_response(await operation(**data))
        except DeploymentError as error:
            raise web.HTTPBadRequest(text=str(error)) from error

    async def stop(_request):
        if operation_lock.locked():
            raise web.HTTPConflict(text='正在处理机器人操作，请稍候')
        async with operation_lock:
            return web.json_response(await launcher.stop())

    async def log(_request):
        return web.json_response({'text': launcher.log_tail()})

    app.router.add_get('/api/launcher', state)
    app.router.add_post('/api/launcher/start', start)
    app.router.add_post('/api/launcher/restart', start)
    app.router.add_post('/api/launcher/stop', stop)
    app.router.add_get('/api/launcher/log', log)
