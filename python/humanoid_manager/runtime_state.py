"""File leases and content identities shared by deployment and bringup."""
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import yaml

from .deployment import DeploymentError, resolve_robot_deployment


def acquire_deployment_lock(root: Path, *, shared: bool = False):
    root = Path(root).resolve()
    root.mkdir(parents=True, exist_ok=True)
    stream = (root / ".deployment.lock").open("a+")
    try:
        fcntl.flock(stream, (fcntl.LOCK_SH if shared else fcntl.LOCK_EX) | fcntl.LOCK_NB)
    except BlockingIOError as error:
        stream.close()
        raise DeploymentError("机器人正在运行或部署中，请停止相关进程后再应用配置") from error
    return stream


def _exclusive_process_lock(root: Path, filename: str, message: str):
    root = Path(root).resolve()
    root.mkdir(parents=True, exist_ok=True)
    stream = (root / filename).open('a+')
    try:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        stream.seek(0)
        holder = stream.read(32).strip()
        stream.close()
        detail = f'（PID {holder}）' if holder.isdecimal() else ''
        raise DeploymentError(message + detail) from error
    stream.seek(0)
    stream.truncate()
    stream.write(str(os.getpid()))
    stream.flush()
    return stream


def acquire_robot_run_lock(root: Path):
    """One publisher stack per plugin root, independent of ROS discovery lag."""
    return _exclusive_process_lock(root, '.robot-run.lock',
        '该插件目录已有机器人启动进程，请先停止，不能重复启动')


def acquire_manager_run_lock(root: Path):
    """Changing the web state directory or port must not create a second owner."""
    return _exclusive_process_lock(root, '.manager-run.lock',
        '该插件目录已有机器人管理进程，请关闭原入口，不能重复启动')


def bind_to_parent(expected_parent_pid: int):
    """Linux: request graceful shutdown even if the owner is killed abruptly."""
    import ctypes
    import signal
    if expected_parent_pid <= 1 or os.getppid() != expected_parent_pid:
        raise DeploymentError('启动进程已退出，取消启动机器人')
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(1, signal.SIGINT, 0, 0, 0) != 0:  # PR_SET_PDEATHSIG
        error = ctypes.get_errno()
        raise DeploymentError(f'无法绑定启动进程生命周期：{os.strerror(error)}')
    # The parent can die between the first check and prctl. Do not launch any
    # hardware if its death happened before the kernel started watching it.
    if os.getppid() != expected_parent_pid:
        raise DeploymentError('启动进程已退出，取消启动机器人')


@contextmanager
def deployment_lock(root: Path, *, shared: bool = False):
    stream = acquire_deployment_lock(root, shared=shared)
    try:
        yield
    finally:
        stream.close()


def configuration_identity(root: Path, robot_id: str) -> dict:
    deployment = resolve_robot_deployment(root, robot_id)
    robot = yaml.safe_load(deployment.manifest_path.read_text())
    paths = [deployment.manifest_path.parent]
    paths += [Path(root) / directory / robot["plugins"][kind] for kind, directory in (
        ("hardware_driver", "hardware_drivers"), ("robot_model", "robot_models"))]
    from .deployment import gripper_references
    paths.extend(Path(root) / 'gripper_drivers' / plugin_id
                 for _, plugin_id in sorted(gripper_references(robot).items()))
    digest = hashlib.sha256()
    for path in paths:
        digest.update((path / "checksums.sha256").read_bytes())
    fingerprint = digest.hexdigest()
    marker = Path(root) / ".configuration-revisions" / f"{robot_id}.json"
    try:
        saved = json.loads(marker.read_text())
    except (OSError, ValueError):
        saved = {}
    return {
        "robot_id": robot_id, "name": deployment.name, "fingerprint": fingerprint,
        "revision": saved.get("revision", "") if saved.get("fingerprint") == fingerprint else "",
        "plugin_root": str(Path(root).resolve()),
    }
