"""File leases and content identities shared by deployment and bringup."""
from contextlib import contextmanager
import fcntl
import hashlib
import json
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
