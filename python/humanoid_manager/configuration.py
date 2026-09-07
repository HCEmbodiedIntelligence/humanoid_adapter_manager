"""Editable robot workspaces backed by validated, immutable bundle snapshots.

The active deployment is never an editing surface. All robot-specific copies use
private driver/model IDs, so changing one robot cannot modify another robot.
"""
from __future__ import annotations

from contextlib import contextmanager
import copy
from datetime import datetime, timezone
import fcntl
import json
import math
import os
from pathlib import Path
import re
import shutil
import tempfile
import uuid
import xml.etree.ElementTree as ET
import zipfile

import yaml

from .deployment import (
    DeploymentError, MAX_CONFIG_BYTES, _resolve_member, _safe_extract,
    deploy_archive, list_deployed, pack_directory, resolve_robot_deployment,
    validate_archive, validate_tree, write_checksums,
)
from .runtime_state import configuration_identity, deployment_lock


class ConfigurationConflict(DeploymentError):
    pass


def atomic_json(path: Path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=".write-", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def _id(value):
    if not isinstance(value, str) or not re.fullmatch(r"[a-z0-9][a-z0-9_.-]{0,63}", value):
        raise DeploymentError("ID 需为 1–64 个小写字母、数字、点、下划线或短横线")
    return value


def _yaml(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def _write_yaml(path, document):
    Path(path).write_text(yaml.safe_dump(document, allow_unicode=True, sort_keys=False), encoding="utf-8")


def _resource_document(path):
    if path.suffix.lower() == ".toml":
        import toml
        return toml.loads(path.read_text(encoding="utf-8"))
    return _yaml(path)


def differences(old, new, path=""):
    if isinstance(old, dict) and isinstance(new, dict):
        result = []
        for key in sorted(set(old) | set(new)):
            result.extend(differences(old.get(key), new.get(key), f"{path}/{key}"))
        return result
    if old == new:
        return []
    return [{"path": path, "before": old, "after": new}]


def validate_values(value, path=""):
    """Reject unsafe scalar values in addition to cross-resource validation."""
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise DeploymentError(f"{path}: 配置字段名必须是字符串")
            validate_values(item, f"{path}/{key}")
            if key.endswith(("_hz", "_ms", "_s", "_rad_s", "_rad_s2", "_rad_s3", "_m_s", "_m_s2", "_m_s3")) and not isinstance(item, (dict, list)):
                if isinstance(item, bool) or not isinstance(item, (int, float)) or item < 0:
                    raise DeploymentError(f"{path}/{key}: 必须是非负数")
            if key in {"control_frequency_hz", "diagnostic_frequency_hz"} and not 0 < item <= 1000:
                raise DeploymentError(f"{key}: 频率必须在 0–1000 Hz 之间（不含 0）")
    elif isinstance(value, list):
        for item in value:
            validate_values(item, path)
    elif isinstance(value, float) and not math.isfinite(value):
        raise DeploymentError(f"{path}: 必须是有限数值")


def validate_recording(value):
    if not isinstance(value, dict) or not isinstance(value.get("directory"), str) or not value["directory"].strip():
        raise DeploymentError("录制目录不能为空")
    if not isinstance(value.get("subscriptions"), list):
        raise DeploymentError("录制规则必须是列表")
    seen = set()
    for rule in value["subscriptions"]:
        if not isinstance(rule, dict):
            raise DeploymentError("录制规则格式错误")
        topic = rule.get("topic", "")
        if not isinstance(topic, str) or not re.fullmatch(r"/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*", topic) or topic in seen:
            raise DeploymentError(f"录制话题无效或重复: {topic}")
        seen.add(topic)
        if not isinstance(rule.get("type"), str) or not re.fullmatch(r"\w+/msg/\w+", rule["type"]):
            raise DeploymentError(f"{topic}: 无效消息类型")
        if type(rule.get("enabled", True)) is not bool:
            raise DeploymentError(f"{topic}: enabled 必须为布尔值")
        for key in ("max_hz", "event_max_hz", "min_hz", "target_hz"):
            hz = rule.get(key, 0)
            if isinstance(hz, bool) or not isinstance(hz, (int, float)) or not math.isfinite(hz) or hz < 0:
                raise DeploymentError(f"{topic}: {key} 必须是非负有限数")
        if not isinstance(rule.get("outputs", ["record"]), list) or any(x not in {"record", "websocket", "udp"} for x in rule.get("outputs", [])):
            raise DeploymentError(f"{topic}: 无效输出方式")


def validate_cameras(value):
    """Validate robot-owned camera definitions without touching camera hardware."""
    if not isinstance(value, list) or len(value) > 16:
        raise DeploymentError("相机配置必须是列表，且最多 16 台")
    result, identifiers, endpoints, serials = [], set(), set(), set()
    ros_name = re.compile(r"[A-Za-z_][A-Za-z0-9_]{0,63}")
    ros_topic = re.compile(r"/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*")
    for raw in value:
        if not isinstance(raw, dict):
            raise DeploymentError("每台相机配置必须是对象")
        camera = copy.deepcopy(raw)
        ident = camera.get("id", "")
        if not isinstance(ident, str) or not re.fullmatch(r"[a-z][a-z0-9_]{0,63}", ident):
            raise DeploymentError("相机 ID 需以小写字母开头，只允许小写字母、数字和下划线")
        if ident in identifiers:
            raise DeploymentError(f"相机 ID 重复: {ident}")
        identifiers.add(ident)
        camera.setdefault("enabled", True)
        camera.setdefault("backend", "realsense")
        camera.setdefault("required", True)
        camera.setdefault("pointcloud", False)
        camera.setdefault("device_type", "d405" if camera["backend"] == "realsense" else "custom_rgbd")
        camera.setdefault("serial_no", "")
        camera.setdefault("sync_rgb_depth", True)
        camera.setdefault("timestamp_alignment", camera.get("normalize_timestamps", True))
        camera.pop("normalize_timestamps", None)
        camera.setdefault("max_actual_exposure_us", 5000)
        camera.setdefault("rgbd_max_midpoint_skew_ms", 1.0)
        camera.setdefault("camera_max_error_ms", 1.0)
        for key in ("enabled", "required", "pointcloud", "sync_rgb_depth", "timestamp_alignment"):
            if type(camera[key]) is not bool:
                raise DeploymentError(f"{ident}.{key} 必须为布尔值")
        if not isinstance(camera["device_type"], str) or not re.fullmatch(r"[A-Za-z0-9_.-]{1,64}", camera["device_type"]):
            raise DeploymentError(f"{ident}.device_type 无效")
        if not isinstance(camera["serial_no"], str) or len(camera["serial_no"]) > 128 or any(c.isspace() for c in camera["serial_no"]):
            raise DeploymentError(f"{ident}.serial_no 无效")
        if camera["enabled"] and camera["serial_no"]:
            if camera["serial_no"] in serials:
                raise DeploymentError(f"相机序列号重复: {camera['serial_no']}")
            serials.add(camera["serial_no"])
        exposure = camera["max_actual_exposure_us"]
        if type(exposure) is not int or not 1 <= exposure <= 5000:
            raise DeploymentError(f"{ident}.max_actual_exposure_us 必须是 1–5000 的整数")
        for key in ("rgbd_max_midpoint_skew_ms", "camera_max_error_ms"):
            number = camera[key]
            if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number) or not 0 < number <= 1000:
                raise DeploymentError(f"{ident}.{key} 必须是 0–1000 的有限正数")
        if camera["backend"] not in {"realsense", "ros_topics"}:
            raise DeploymentError(f"{ident}: backend 只支持 realsense 或 ros_topics")
        if camera["backend"] == "ros_topics":
            camera.setdefault("fps", 30)
            if type(camera["fps"]) is not int or not 1 <= camera["fps"] <= 240:
                raise DeploymentError(f"{ident}.fps 必须是 1–240 的整数")
            camera.setdefault("rgbd_topic", f"/{ident}/normalized/rgbd")
            camera.setdefault("metadata_topic", f"/{ident}/normalized/metadata")
            camera.setdefault("pointcloud_topic", f"/{ident}/normalized/points")
            camera.setdefault("pointcloud_metadata_topic", f"/{ident}/normalized/points_metadata")
            for key in ("rgbd_topic", "metadata_topic"):
                if not isinstance(camera[key], str) or not ros_topic.fullmatch(camera[key]):
                    raise DeploymentError(f"{ident}.{key} 必须是绝对 ROS 话题")
            if camera["pointcloud"]:
                for key in ("pointcloud_topic", "pointcloud_metadata_topic"):
                    if not isinstance(camera[key], str) or not ros_topic.fullmatch(camera[key]):
                        raise DeploymentError(f"{ident}.{key} 必须是绝对 ROS 话题")
            result.append(camera)
            continue
        defaults = {
            "namespace": ident, "camera_name": "camera", "width": 640, "height": 480, "fps": 30,
            "color_format": "RGB8", "depth_format": "Z16", "align_depth": False,
            "depth_auto_exposure": True, "depth_exposure_us": 4500, "depth_gain": 64,
            "depth_auto_exposure_limit_us": 4500, "depth_auto_gain_limit": 64,
            "color_auto_exposure": False, "color_exposure_us": 4500,
            "color_gain": 64, "parameters": {},
        }
        for key, default in defaults.items():
            camera.setdefault(key, default)
        for key in ("namespace", "camera_name"):
            if not isinstance(camera[key], str) or not ros_name.fullmatch(camera[key]):
                raise DeploymentError(f"{ident}.{key} 不是有效 ROS 名称")
        endpoint = (camera["namespace"], camera["camera_name"])
        if endpoint in endpoints:
            raise DeploymentError(f"相机 ROS 命名空间与节点名重复: /{endpoint[0]}/{endpoint[1]}")
        endpoints.add(endpoint)
        for key in ("align_depth", "depth_auto_exposure", "color_auto_exposure"):
            if type(camera[key]) is not bool:
                raise DeploymentError(f"{ident}.{key} 必须为布尔值")
        for key, low, high in (("width", 1, 8192), ("height", 1, 8192), ("fps", 1, 240),
                               ("depth_exposure_us", 1, 5000), ("depth_gain", 0, 10000),
                               ("depth_auto_exposure_limit_us", 1, 5000), ("depth_auto_gain_limit", 1, 10000),
                               ("color_exposure_us", 1, 5000), ("color_gain", 0, 10000)):
            number = camera[key]
            if type(number) is not int or not low <= number <= high:
                raise DeploymentError(f"{ident}.{key} 必须是 {low}–{high} 的整数")
        depth_setting = camera["depth_auto_exposure_limit_us"] if camera["depth_auto_exposure"] else camera["depth_exposure_us"]
        if depth_setting > camera["max_actual_exposure_us"]:
            raise DeploymentError(f"{ident}: 深度曝光设置不能超过曝光要求上限")
        if camera["device_type"].lower() != "d405" and not camera["color_auto_exposure"] and camera["color_exposure_us"] > camera["max_actual_exposure_us"]:
            raise DeploymentError(f"{ident}: RGB 手动曝光不能超过曝光要求上限")
        for key in ("color_format", "depth_format"):
            if not isinstance(camera[key], str) or not re.fullmatch(r"[A-Za-z0-9_]{1,32}", camera[key]):
                raise DeploymentError(f"{ident}.{key} 无效")
        if not isinstance(camera["parameters"], dict):
            raise DeploymentError(f"{ident}.parameters 必须为对象")
        validate_values(camera["parameters"], f"/cameras/{ident}/parameters")
        result.append(camera)
    active_realsense = [x for x in result if x["backend"] == "realsense" and x["enabled"]]
    if len(active_realsense) > 1 and any(not x["serial_no"] for x in active_realsense):
        raise DeploymentError("同时启用多台 RealSense 时，每台都必须填写唯一序列号")
    return result


def validate_initial_poses(value, resources):
    """Validate named MoveJ poses against the editable model and channel resources."""
    if not isinstance(value, list) or len(value) > 32:
        raise DeploymentError("初始姿态必须是列表，且最多 32 个")
    if not isinstance(resources, dict):
        raise DeploymentError("初始姿态缺少机器人资源")
    try:
        motion = resources["motion_params"]["humanoid_motion_control"]["ros__parameters"]
        channels = resources["channel_config"]["channels"]
    except (KeyError, TypeError) as error:
        raise DeploymentError("初始姿态无法读取运动分组或通道配置") from error
    if not isinstance(channels, list):
        raise DeploymentError("初始姿态无法读取运动通道")
    channel_map = {}
    for channel in channels:
        if isinstance(channel, dict) and isinstance(channel.get("name"), str):
            channel_map[channel["name"]] = channel

    result, identifiers = [], set()
    allowed_pose_keys = {
        "id", "name", "targets", "velocity_scale", "acceleration_scale",
        "jerk_scale", "timeout_sec",
    }
    for raw in value:
        if not isinstance(raw, dict) or set(raw) - allowed_pose_keys:
            raise DeploymentError("每个初始姿态必须是有效对象且不能包含未知字段")
        pose = copy.deepcopy(raw)
        ident = pose.get("id", "")
        if not isinstance(ident, str) or not re.fullmatch(r"[a-z][a-z0-9_]{0,63}", ident):
            raise DeploymentError("初始姿态 ID 需以小写字母开头，只允许小写字母、数字和下划线")
        if ident in identifiers:
            raise DeploymentError(f"初始姿态 ID 重复: {ident}")
        identifiers.add(ident)
        name = pose.get("name", "")
        if not isinstance(name, str) or not name.strip() or len(name) > 100:
            raise DeploymentError(f"{ident}: 初始姿态名称需为 1–100 个字符")
        pose["name"] = name.strip()
        defaults = {
            "velocity_scale": 0.15,
            "acceleration_scale": 0.15,
            "jerk_scale": 0.15,
            "timeout_sec": 60.0,
        }
        for key, default in defaults.items():
            pose.setdefault(key, default)
            number = pose[key]
            upper = 600.0 if key == "timeout_sec" else 1.0
            if (isinstance(number, bool) or not isinstance(number, (int, float)) or
                    not math.isfinite(number) or not 0 < number <= upper):
                unit = "0–600 秒（不含 0）" if key == "timeout_sec" else "0–1（不含 0）"
                raise DeploymentError(f"{ident}.{key} 必须在 {unit}范围内")
            pose[key] = float(number)

        targets = pose.get("targets")
        if not isinstance(targets, list) or not targets or len(targets) > 16:
            raise DeploymentError(f"{ident}: 至少需要一个初始姿态目标，最多 16 个")
        normalized_targets, used_channels, used_joints = [], set(), set()
        for raw_target in targets:
            if not isinstance(raw_target, dict) or set(raw_target) != {"channel", "positions_rad"}:
                raise DeploymentError(f"{ident}: 姿态目标只能包含 channel 和 positions_rad")
            channel_name = raw_target.get("channel", "")
            channel = channel_map.get(channel_name)
            if channel is None or channel.get("kind") != "move_j" or not channel.get("group"):
                raise DeploymentError(f"{ident}: 通道 {channel_name} 不是带关节组的 MoveJ 通道")
            if channel_name in used_channels:
                raise DeploymentError(f"{ident}: MoveJ 通道重复: {channel_name}")
            used_channels.add(channel_name)
            group = channel["group"]
            joints = motion.get(f"groups.{group}")
            lower = motion.get(f"group_lower_limits.{group}")
            upper = motion.get(f"group_upper_limits.{group}")
            if not (isinstance(joints, list) and isinstance(lower, list) and isinstance(upper, list) and
                    len(joints) == len(lower) == len(upper) and joints):
                raise DeploymentError(f"{ident}: 通道 {channel_name} 的关节组或限位不完整")
            overlap = used_joints.intersection(joints)
            if overlap:
                raise DeploymentError(f"{ident}: 多个目标包含相同关节: {', '.join(sorted(overlap))}")
            used_joints.update(joints)
            positions = raw_target.get("positions_rad")
            if not isinstance(positions, list) or len(positions) != len(joints):
                raise DeploymentError(f"{ident}: {group} 需要 {len(joints)} 个关节位置")
            normalized_positions = []
            for joint, position, low, high in zip(joints, positions, lower, upper):
                if (isinstance(position, bool) or not isinstance(position, (int, float)) or
                        not math.isfinite(position)):
                    raise DeploymentError(f"{ident}.{joint}: 初始位置必须是有限数值")
                if position < low or position > high:
                    raise DeploymentError(f"{ident}.{joint}: {position} rad 超出限位 [{low}, {high}]")
                normalized_positions.append(float(position))
            normalized_targets.append({"channel": channel_name, "positions_rad": normalized_positions})
        pose["targets"] = normalized_targets
        result.append(pose)
    return result


def resolve_initial_pose(document, pose_id):
    """Resolve a validated named pose to concrete MoveJ action endpoints."""
    poses = validate_initial_poses(document.get("initial_poses", []), document.get("resources"))
    pose = next((item for item in poses if item["id"] == pose_id), None)
    if pose is None:
        raise DeploymentError("初始姿态不存在")
    resources = document["resources"]
    motion = resources["motion_params"]["humanoid_motion_control"]["ros__parameters"]
    channels = {item["name"]: item for item in resources["channel_config"]["channels"]}
    goals = []
    for target in pose["targets"]:
        channel = channels[target["channel"]]
        group = channel["group"]
        goals.append({
            "channel": channel["name"],
            "endpoint": channel["endpoint"],
            "group": group,
            "joint_names": list(motion[f"groups.{group}"]),
            "positions_rad": list(target["positions_rad"]),
            "velocity_scale": pose["velocity_scale"],
            "acceleration_scale": pose["acceleration_scale"],
            "jerk_scale": pose["jerk_scale"],
            "timeout_sec": pose["timeout_sec"],
        })
    return {**pose, "goals": goals}


def normalize_document(document):
    value = copy.deepcopy(document)
    value.setdefault("cameras", [])
    value.setdefault("initial_poses", [])
    value.setdefault("gripper_driver", None)
    return value


def validate_gripper_selection(value):
    if value is None:
        return None
    if not isinstance(value, dict) or set(value) != {"plugin_id", "name"}:
        raise DeploymentError("夹爪驱动选择必须包含 plugin_id 和 name")
    plugin_id = _id(value.get("plugin_id"))
    name = value.get("name")
    if not isinstance(name, str) or not name.strip() or len(name) > 100:
        raise DeploymentError("夹爪驱动名称需为 1–100 个字符")
    return {"plugin_id": plugin_id, "name": name.strip()}


class ConfigurationManager:
    def __init__(self, plugin_root, state_root):
        self.plugin_root = Path(plugin_root).resolve()
        self.state_root = Path(state_root).resolve()

    @contextmanager
    def lock(self):
        self.state_root.mkdir(parents=True, exist_ok=True)
        with (self.state_root / ".lock").open("a+") as stream:
            fcntl.flock(stream, fcntl.LOCK_EX)
            yield

    def _workspace(self, robot_id):
        return self.state_root / "robots" / _id(robot_id)

    def _read(self, robot_id):
        try:
            return json.loads((self._workspace(robot_id) / "index.json").read_text())
        except FileNotFoundError as error:
            raise DeploymentError("机器人配置不存在") from error

    def _revision(self, robot_id, revision):
        if not isinstance(revision, str) or not re.fullmatch(r"r-[a-f0-9]{16}", revision):
            raise DeploymentError("无效的配置版本")
        path = self._workspace(robot_id) / "versions" / revision
        if not (path / "document.json").is_file():
            raise DeploymentError("配置版本不存在")
        return path

    def _check_etag(self, index, etag):
        if etag != index["etag"]:
            raise ConfigurationConflict("配置已被其他窗口修改，请刷新后重新编辑")

    def catalog(self):
        catalog = list_deployed(self.plugin_root)
        for kind, entries in catalog.items():
            for ident, entry in entries.items():
                try:
                    manifest = _yaml(Path(entry["path"]) / "manifest.yaml")
                    entry.update(name=manifest.get("name", ident), plugins=manifest.get("plugins", {}))
                    if kind == "gripper_drivers":
                        validated = validate_tree(Path(entry["path"]))
                        entry["plugin_class"] = validated["plugin_class"]
                        entry["template"] = _resource_document(
                            validated["_resources"]["gripper_params"]
                        )
                except (OSError, yaml.YAMLError, AttributeError, DeploymentError):
                    entry["error"] = "无法读取插件清单"
        catalog["workspaces"] = []
        base = self.state_root / "robots"
        for path in sorted(base.glob("*/index.json")):
            item = json.loads(path.read_text())
            catalog["workspaces"].append({k: item[k] for k in ("robot_id", "name", "latest", "etag")})
        catalog["plugin_root"] = str(self.plugin_root)
        return catalog

    def _components(self, root, robot_id):
        robot_path = root / "robots" / robot_id
        composition = _yaml(robot_path / "manifest.yaml")
        plugins = composition["plugins"]
        gripper_id = plugins.get("gripper_driver")
        return (
            root / "hardware_drivers" / plugins["hardware_driver"],
            root / "robot_models" / plugins["robot_model"],
            root / "gripper_drivers" / gripper_id if gripper_id else None,
            robot_path,
        )

    def _document(self, root, robot_id, recording=None, cameras=None, initial_poses=None):
        driver, model, gripper, robot = self._components(root, robot_id)
        resources = {}
        for path in (driver, model, gripper):
            if path is None:
                continue
            manifest = _yaml(path / "manifest.yaml")
            for key, relative in manifest["resources"].items():
                target = _resolve_member(path, relative, key)
                resources[key] = target.read_text(encoding="utf-8") if key == "urdf" else _resource_document(target)
        if cameras is None:
            camera_path = robot / "cameras.yaml"
            cameras = (_yaml(camera_path) or {}).get("cameras", []) if camera_path.is_file() else []
        if initial_poses is None:
            pose_path = robot / "initial_poses.yaml"
            initial_poses = (_yaml(pose_path) or {}).get("initial_poses", []) if pose_path.is_file() else []
        gripper_selection = None
        if gripper is not None:
            gripper_manifest = _yaml(gripper / "manifest.yaml")
            gripper_selection = {
                "plugin_id": gripper_manifest["plugin_id"],
                "name": gripper_manifest.get("name", gripper_manifest["plugin_id"]),
            }
        return {"name": _yaml(robot / "manifest.yaml")["name"], "resources": resources,
                "cameras": validate_cameras(cameras),
                "initial_poses": validate_initial_poses(initial_poses, resources),
                "gripper_driver": gripper_selection,
                "recording": recording or {"directory": "../runtime/topic_recordings", "subscriptions": [
                    {"topic": "/hc_teleop/joint_states", "type": "sensor_msgs/msg/JointState", "enabled": True, "outputs": ["record", "websocket"], "max_hz": 0, "event_max_hz": 20},
                    {"topic": "/diagnostics", "type": "diagnostic_msgs/msg/DiagnosticArray", "enabled": True, "outputs": ["record", "websocket"], "max_hz": 0, "event_max_hz": 5},
                ]}}

    def _persist_revision(self, robot_id, root, document):
        revision = "r-" + uuid.uuid4().hex[:16]
        destination = self._workspace(robot_id) / "versions" / revision
        destination.mkdir(parents=True)
        os.replace(root, destination / "root")
        atomic_json(destination / "document.json", document)
        atomic_json(destination / "metadata.json", {
            "revision": revision, "created_at": datetime.now(timezone.utc).isoformat(),
            "fingerprint": configuration_identity(destination / "root", robot_id)["fingerprint"],
        })
        return revision

    def create(
        self, robot_id, name, source_robot="", driver_id="", model_id="",
        gripper_id="", source_workspace="",
    ):
        robot_id = _id(robot_id)
        if not isinstance(name, str) or not name.strip():
            raise DeploymentError("机器人名称不能为空")
        with self.lock():
            workspace = self._workspace(robot_id)
            if workspace.exists():
                raise ConfigurationConflict("配置 ID 已存在，请使用其他 ID")
            source_root, recording, cameras, initial_poses = self.plugin_root, None, None, None
            if source_workspace:
                source = self._read(source_workspace)
                source_root = self._revision(source_workspace, source["latest"]) / "root"
                source_robot = source_workspace
                source_document = normalize_document(json.loads((source_root.parent / "document.json").read_text()))
                recording, cameras = source_document["recording"], source_document["cameras"]
                initial_poses = source_document["initial_poses"]
            if source_robot:
                resolve_robot_deployment(source_root, _id(source_robot))
                source_driver, source_model, source_gripper, _ = self._components(
                    source_root, source_robot
                )
            else:
                source_driver = source_root / "hardware_drivers" / _id(driver_id)
                source_model = source_root / "robot_models" / _id(model_id)
                source_gripper = (
                    source_root / "gripper_drivers" / _id(gripper_id)
                    if gripper_id else None
                )
            component_specs = [
                (source_driver, "hardware_drivers", "driver"),
                (source_model, "robot_models", "model"),
            ]
            if source_gripper is not None:
                component_specs.append((source_gripper, "gripper_drivers", "gripper"))
            for path, _, _ in component_specs:
                validate_tree(path)
            with tempfile.TemporaryDirectory(dir=self.state_root, prefix=".create-") as temporary:
                root = Path(temporary) / "root"
                for source, kind, suffix in component_specs:
                    target = root / kind / f"{robot_id}.{suffix}"
                    shutil.copytree(source, target)
                    manifest = _yaml(target / "manifest.yaml")
                    manifest["plugin_id"] = f"{robot_id}.{suffix}"
                    if kind == "robot_models" and "hc_teleop_config" in manifest["resources"]:
                        receiver_path = _resolve_member(target, manifest["resources"]["hc_teleop_config"], "hc_teleop_config")
                        receiver = _yaml(receiver_path)
                        receiver.setdefault("adapter", {})["robot_id"] = robot_id
                        _write_yaml(receiver_path, receiver)
                    _write_yaml(target / "manifest.yaml", manifest)
                    write_checksums(target)
                robot = root / "robots" / robot_id
                robot.mkdir(parents=True)
                plugins = {
                    "hardware_driver": f"{robot_id}.driver",
                    "robot_model": f"{robot_id}.model",
                }
                if source_gripper is not None:
                    plugins["gripper_driver"] = f"{robot_id}.gripper"
                _write_yaml(robot / "manifest.yaml", {
                    "schema_version": 1,
                    "artifact_type": "robot_composition",
                    "robot_id": robot_id,
                    "name": name.strip(),
                    "plugins": plugins,
                })
                write_checksums(robot)
                resolve_robot_deployment(root, robot_id)
                document = self._document(root, robot_id, recording, cameras, initial_poses)
                revision = self._persist_revision(robot_id, root, document)
                atomic_json(workspace / "index.json", {"robot_id": robot_id, "name": name.strip(), "latest": revision, "etag": uuid.uuid4().hex, "draft": document})
        return self.get(robot_id)

    def get(self, robot_id):
        index = self._read(robot_id)
        saved = normalize_document(json.loads((self._revision(robot_id, index["latest"]) / "document.json").read_text()))
        draft = normalize_document(index["draft"])
        result = {**index, "draft": draft, "saved": saved, "diff": differences(saved, draft), "history": []}
        for path in (self._workspace(robot_id) / "versions").glob("*/metadata.json"):
            result["history"].append(json.loads(path.read_text()))
        result["history"].sort(key=lambda x: x["created_at"], reverse=True)
        try:
            result["deployed"] = configuration_identity(self.plugin_root, robot_id)
        except (DeploymentError, OSError):
            result["deployed"] = None
        try:
            urdf = ET.fromstring(index["draft"]["resources"]["urdf"])
            result["model_info"] = {"name": urdf.get("name", ""), "links": [x.get("name") for x in urdf.findall("link")], "joints": [{**x.attrib, "parent": x.find("parent").get("link") if x.find("parent") is not None else "", "child": x.find("child").get("link") if x.find("child") is not None else ""} for x in urdf.findall("joint")]}
        except (ET.ParseError, KeyError, TypeError):
            result["model_info"] = {"error": "URDF 尚未通过校验", "links": [], "joints": []}
        return result

    def draft(self, robot_id, document, etag):
        if not isinstance(document, dict) or set(document) != {"name", "resources", "recording", "cameras", "initial_poses", "gripper_driver"}:
            raise DeploymentError("配置需包含 name、resources、cameras、initial_poses、gripper_driver 和 recording")
        selection = validate_gripper_selection(document["gripper_driver"])
        has_gripper_resource = (
            isinstance(document.get("resources"), dict)
            and "gripper_params" in document["resources"]
        )
        if (selection is None) != (not has_gripper_resource):
            raise DeploymentError("夹爪驱动选择与 gripper_params 资源必须同时存在或同时移除")
        try:
            encoded = json.dumps(document, ensure_ascii=False, allow_nan=False).encode()
        except (ValueError, TypeError) as error:
            raise DeploymentError("配置包含无效数值") from error
        if len(encoded) > MAX_CONFIG_BYTES:
            raise DeploymentError("配置超过 16 MB")
        with self.lock():
            index = self._read(robot_id)
            self._check_etag(index, etag)
            optional = {"hc_teleop_config", "gripper_params"}
            if not isinstance(document["resources"], dict) or set(document["resources"])-optional != set(index["draft"]["resources"])-optional:
                raise DeploymentError("资源类型应与模型插件一致，请通过导入模型插件增减资源")
            value = copy.deepcopy(document)
            value["gripper_driver"] = selection
            index.update(draft=value, etag=uuid.uuid4().hex)
            atomic_json(self._workspace(robot_id) / "index.json", index)
        return self.get(robot_id)

    def _stage(self, robot_id, index, root):
        document = normalize_document(index["draft"])
        if not isinstance(document["name"], str) or not document["name"].strip():
            raise DeploymentError("机器人名称不能为空")
        validate_values(document)
        validate_recording(document["recording"])
        cameras = validate_cameras(document["cameras"])
        initial_poses = validate_initial_poses(document["initial_poses"], document["resources"])
        gripper_selection = validate_gripper_selection(document["gripper_driver"])
        shutil.copytree(self._revision(robot_id, index["latest"]) / "root", root)
        robot = root / "robots" / robot_id
        composition = _yaml(robot / "manifest.yaml")
        current_gripper_id = composition["plugins"].get("gripper_driver")
        current_gripper = (
            root / "gripper_drivers" / current_gripper_id if current_gripper_id else None
        )
        saved = normalize_document(json.loads(
            (self._revision(robot_id, index["latest"]) / "document.json").read_text()
        ))
        if gripper_selection is None:
            composition["plugins"].pop("gripper_driver", None)
            if current_gripper is not None and current_gripper.exists():
                shutil.rmtree(current_gripper)
        else:
            private_id = f"{robot_id}.gripper"
            keep_current = (
                current_gripper is not None
                and current_gripper.is_dir()
                and gripper_selection == saved.get("gripper_driver")
            )
            if not keep_current:
                source = self.plugin_root / "gripper_drivers" / gripper_selection["plugin_id"]
                validated = validate_tree(source)
                if validated.get("_deployment_type") != "gripper_driver":
                    raise DeploymentError("所选插件不是夹爪驱动")
                target = root / "gripper_drivers" / private_id
                if current_gripper is not None and current_gripper.exists():
                    shutil.rmtree(current_gripper)
                if target.exists():
                    shutil.rmtree(target)
                shutil.copytree(source, target)
                manifest = _yaml(target / "manifest.yaml")
                manifest["plugin_id"] = private_id
                _write_yaml(target / "manifest.yaml", manifest)
                write_checksums(target)
            elif current_gripper_id != private_id:
                target = root / "gripper_drivers" / private_id
                if target.exists():
                    shutil.rmtree(target)
                os.replace(current_gripper, target)
                manifest = _yaml(target / "manifest.yaml")
                manifest["plugin_id"] = private_id
                _write_yaml(target / "manifest.yaml", manifest)
                write_checksums(target)
            composition["plugins"]["gripper_driver"] = private_id
        _write_yaml(robot / "manifest.yaml", composition)

        driver, model, gripper, robot = self._components(root, robot_id)
        for path in (driver, model, gripper):
            if path is None:
                continue
            manifest = _yaml(path / "manifest.yaml")
            if path == model:
                if "hc_teleop_config" not in document["resources"]:
                    manifest["resources"].pop("hc_teleop_config", None)
                if "hc_teleop_config" in document["resources"]:
                    if "hc_teleop_config" not in manifest["resources"]:
                        manifest["resources"]["hc_teleop_config"] = "resources/managed_hc_teleop.yaml"
                        target = model / "resources/managed_hc_teleop.yaml"
                        target.parent.mkdir(parents=True, exist_ok=True)
                        target.write_text("{}", encoding="utf-8")
                _write_yaml(path / "manifest.yaml", manifest)
            for key, relative in manifest["resources"].items():
                target = _resolve_member(path, relative, key)
                value = document["resources"][key]
                if key == "urdf":
                    if not isinstance(value, str):
                        raise DeploymentError("URDF 内容必须为 XML 文本")
                    target.write_text(value, encoding="utf-8")
                else:
                    if not isinstance(value, dict):
                        raise DeploymentError(f"{key}: 配置必须为对象")
                    if target.suffix.lower() == ".toml":
                        import toml
                        target.write_text(toml.dumps(value), encoding="utf-8")
                    else:
                        _write_yaml(target, value)
            write_checksums(path)
            validate_tree(path)
        manifest = _yaml(robot / "manifest.yaml")
        manifest["name"] = document["name"]
        _write_yaml(robot / "manifest.yaml", manifest)
        _write_yaml(robot / "cameras.yaml", {"schema_version": 1, "cameras": cameras})
        _write_yaml(robot / "initial_poses.yaml", {"schema_version": 1, "initial_poses": initial_poses})
        write_checksums(robot)
        resolve_robot_deployment(root, robot_id)

    def validate(self, robot_id, etag, save=False):
        with self.lock():
            index = self._read(robot_id)
            self._check_etag(index, etag)
            with tempfile.TemporaryDirectory(dir=self.state_root, prefix=".validate-") as temporary:
                root = Path(temporary) / "root"
                self._stage(robot_id, index, root)
                if save:
                    revision = self._persist_revision(robot_id, root, index["draft"])
                    index.update(latest=revision, name=index["draft"]["name"], etag=uuid.uuid4().hex)
                    atomic_json(self._workspace(robot_id) / "index.json", index)
        return self.get(robot_id) if save else {"ok": True, "message": "配置及驱动/模型/通道关联校验通过"}

    def restore(self, robot_id, revision, etag):
        document = normalize_document(json.loads((self._revision(robot_id, revision) / "document.json").read_text()))
        return self.draft(robot_id, document, etag)

    def apply(self, robot_id, revision, etag):
        with self.lock(), deployment_lock(self.plugin_root):
            index = self._read(robot_id)
            self._check_etag(index, etag)
            if revision != index["latest"]:
                raise ConfigurationConflict("只能应用最新已保存版本；恢复历史版本后请先保存")
            root = self._revision(robot_id, revision) / "root"
            resolve_robot_deployment(root, robot_id)
            sources = [source for source in self._components(root, robot_id) if source is not None]
            composition = _yaml(root / "robots" / robot_id / "manifest.yaml")
            stale_gripper = self.plugin_root / "gripper_drivers" / f"{robot_id}.gripper"
            remove_stale_gripper = (
                "gripper_driver" not in composition["plugins"] and stale_gripper.is_dir()
            )
            with tempfile.TemporaryDirectory(dir=self.plugin_root, prefix=".configuration-") as temporary:
                stage = Path(temporary)
                archives = []
                for i, source in enumerate(sources):
                    archive = pack_directory(source, stage / f"{i}.zip")
                    validate_archive(archive, check_linkage=True)
                    archives.append(archive)
                destinations = [self.plugin_root / source.relative_to(root) for source in sources]
                existed = []
                for i, destination in enumerate(destinations):
                    existed.append(destination.exists())
                    if destination.exists():
                        shutil.copytree(destination, stage / f"backup-{i}")
                if remove_stale_gripper:
                    shutil.copytree(stale_gripper, stage / "backup-stale-gripper")
                try:
                    for archive in archives:
                        # The complete staged robot tree was already cross-validated above.
                        # Suppress per-component dependent checks while the exclusive deployment
                        # lock protects the short, temporarily mixed replacement sequence.
                        deploy_archive(
                            archive, self.plugin_root, check_dependents=False
                        )
                    if remove_stale_gripper:
                        shutil.rmtree(stale_gripper)
                    resolve_robot_deployment(self.plugin_root, robot_id)
                    identity = configuration_identity(self.plugin_root, robot_id)
                    atomic_json(self.plugin_root / ".configuration-revisions" / f"{robot_id}.json", {**identity, "revision": revision})
                except Exception:
                    for i, destination in enumerate(destinations):
                        if destination.exists():
                            shutil.rmtree(destination)
                        if existed[i]:
                            shutil.copytree(stage / f"backup-{i}", destination)
                    if remove_stale_gripper and not stale_gripper.exists():
                        shutil.copytree(stage / "backup-stale-gripper", stale_gripper)
                    raise
        return {"ok": True, "deployed": configuration_identity(self.plugin_root, robot_id), "restart_required": True}

    def export(self, robot_id, revision, output):
        version = self._revision(robot_id, revision)
        output = Path(output)
        with tempfile.TemporaryDirectory(dir=self.state_root, prefix=".export-") as temporary:
            temporary = Path(temporary)
            with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED) as archive:
                archive.writestr("workspace.json", (version / "document.json").read_text())
                driver, model, gripper, composition = self._components(
                    version / "root", robot_id
                )
                components = [("driver", driver), ("model", model)]
                if gripper is not None:
                    components.append(("gripper", gripper))
                components.append(("composition", composition))
                for name, path in components:
                    packed = pack_directory(path, temporary / f"{name}.zip")
                    archive.write(packed, f"{name}.zip")
        return {"path": str(output)}

    def import_bundle(self, archive):
        with deployment_lock(self.plugin_root):
            manifest = validate_archive(Path(archive), check_linkage=True)
            destination = deploy_archive(Path(archive), self.plugin_root)
        return {"ok": True, "path": str(destination), "artifact_type": manifest["artifact_type"]}

    def import_workspace(self, archive, robot_id, name):
        self.state_root.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=self.state_root, prefix=".import-") as temporary:
            folder = Path(temporary)
            _safe_extract(Path(archive), folder / "input")
            document = normalize_document(json.loads((folder / "input/workspace.json").read_text()))
            validate_recording(document["recording"])
            validate_cameras(document["cameras"])
            validate_initial_poses(document["initial_poses"], document["resources"])
            validate_gripper_selection(document["gripper_driver"])
            root = folder / "root"
            filenames = ["driver", "model"]
            if (folder / "input/gripper.zip").is_file():
                filenames.append("gripper")
            filenames.append("composition")
            for filename in filenames:
                deploy_archive(folder / "input" / f"{filename}.zip", root)
            robots = list_deployed(root)["robots"]
            if len(robots) != 1:
                raise DeploymentError("配置包需包含一个机器人组合")
            importer = ConfigurationManager(root, self.state_root)
            result = importer.create(robot_id, name, source_robot=next(iter(robots)))
            # Exported resource contents come from the validated plugin ZIPs;
            # only the independent recording plan is taken from workspace.json.
            result["draft"]["recording"] = document["recording"]
            result["draft"]["cameras"] = document["cameras"]
            result["draft"]["initial_poses"] = document["initial_poses"]
            result = self.draft(robot_id, result["draft"], result["etag"])
            return self.validate(robot_id, result["etag"], save=True)

    def dispatch(self, request):
        op = request.get("operation")
        args = request.get("arguments", {})
        operations = {"catalog": self.catalog, "create": self.create, "get": self.get,
            "draft": self.draft, "validate": self.validate, "restore": self.restore,
            "apply": self.apply, "export": self.export, "import_bundle": self.import_bundle,
            "import_workspace": self.import_workspace}
        if op not in operations or not isinstance(args, dict):
            raise DeploymentError("不支持的配置操作")
        return operations[op](**args)
