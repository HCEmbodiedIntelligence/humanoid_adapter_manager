"""HTTP facade for the isolated humanoid configuration CLI."""
from __future__ import annotations
import asyncio
import json
import os
from pathlib import Path
import sys
import tempfile

from aiohttp import web

from ..configuration import resolve_gripper_test, resolve_initial_pose, resolve_joint_jog
from ..deployment import DeploymentError
from .gripper_command import GripperCommandError, execute_gripper_test
from .motion_command import MotionCommandError, execute_joint_jog, execute_move_j_pose


class AdapterClient:
    def __init__(self, config, config_dir):
        self.config = config
        self.config_dir = config_dir

    async def call(self, operation, **arguments):
        if not self.config.get("enabled", False):
            raise web.HTTPServiceUnavailable(text="机器人配置管理尚未启用，请使用 start_configurator.sh 启动")
        cli = self.config.get("cli", "")
        if not cli or not Path(cli).is_file():
            raise web.HTTPServiceUnavailable(text="找不到 humanoid_pluginctl.py，请配置 adapter_manager.cli")
        command = [sys.executable, cli, "--root", self.config["plugin_root"],
                   "--state-root", self.config["state_root"], "web"]
        env = dict(os.environ)
        source = Path(cli).resolve().parents[1]
        extra = [str(source / "python"), str(source.parent / "hc_teleop_recv")]
        env["PYTHONPATH"] = os.pathsep.join(extra + [env.get("PYTHONPATH", "")])
        process = await asyncio.create_subprocess_exec(*command, stdin=asyncio.subprocess.PIPE,
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE, env=env)
        try:
            stdout, stderr = await asyncio.wait_for(process.communicate(json.dumps(
                {"operation": operation, "arguments": arguments}, allow_nan=False).encode()), timeout=180)
        except (asyncio.TimeoutError, asyncio.CancelledError):
            process.kill()
            await process.wait()
            raise
        if process.returncode:
            try:
                error = json.loads(stderr)
            except ValueError:
                error = {"message": stderr.decode(errors="replace")[-2000:]}
            exception = web.HTTPConflict if error.get("kind") == "ConfigurationConflict" else web.HTTPBadRequest
            raise exception(text=error.get("message", "配置操作失败"))
        return json.loads(stdout)


def register_adapter_routes(app, store, runtime):
    client = AdapterClient(runtime.config.get("adapter_manager", {}), store.path.parent)
    runtime.adapter_client = client
    operation_lock = asyncio.Lock()
    motion_lock = asyncio.Lock()

    def require_stopped():
        if getattr(runtime, 'capture', None) and runtime.capture.busy():
            raise web.HTTPConflict(text='对齐采集或数据处理正在运行，请完成后再应用配置')
        if runtime.recorder and runtime.recorder.is_recording():
            raise web.HTTPConflict(text="正在录制，请停止后再应用或部署机器人配置")
        if runtime.player and runtime.player.status().get("is_active"):
            raise web.HTTPConflict(text="正在回放，请停止后再应用机器人配置")
        status = runtime.ros.status() if runtime.ros else {}
        if status.get("state") != "running" or status.get("graph_age", 100) > 5:
            raise web.HTTPConflict(text="尚未取得新鲜 ROS 节点状态，无法确认机器人已停止；可先保存配置")
        names = {item["name"] for item in status.get("discovered_nodes", [])}
        if names & {"humanoid_driver_runtime", "humanoid_gripper_runtime", "humanoid_motion_control", "humanoid_configuration_status", "hc_teleop_recv", "timestamp_adapter"}:
            raise web.HTTPConflict(text="机器人节点仍在运行，请停止对应机器人的启动进程后再应用配置")

    async def catalog(_request):
        return web.json_response(await client.call("catalog"))

    async def detail(request):
        return web.json_response(await client.call("get", robot_id=request.match_info["robot_id"]))

    async def create(request):
        data = await request.json()
        allowed = {"robot_id", "name", "source_robot", "driver_id", "model_id", "gripper_id", "source_workspace"}
        if not isinstance(data, dict) or set(data) - allowed:
            raise web.HTTPBadRequest(text="无效的机器人创建参数")
        async with operation_lock:
            return web.json_response(await client.call("create", **data), status=201)

    async def action(request):
        data = await request.json()
        op = request.match_info["operation"]
        allowed = {"draft": {"document", "etag"}, "validate": {"etag", "save"},
                   "restore": {"etag", "revision"}, "apply": {"etag", "revision"}}
        if op not in allowed or not isinstance(data, dict) or set(data) - allowed[op]:
            raise web.HTTPBadRequest(text="无效的配置操作")
        async with operation_lock:
            if op == "apply":
                require_stopped()
            return web.json_response(await client.call(op, robot_id=request.match_info["robot_id"], **data))

    async def upload(request):
        async with operation_lock:
            with tempfile.TemporaryDirectory(prefix="humanoid-web-upload-") as temporary:
                archive = Path(temporary) / "upload.zip"
                fields = {}
                total = 0
                reader = await request.multipart()
                async for part in reader:
                    if part.name == "archive":
                        with archive.open("wb") as stream:
                            while chunk := await part.read_chunk():
                                total += len(chunk)
                                if total > 100 * 1024 * 1024:
                                    raise web.HTTPRequestEntityTooLarge(max_size=100 * 1024 * 1024, actual_size=total)
                                stream.write(chunk)
                    elif part.name in {"kind", "robot_id", "name", "expected_plugin_type"}:
                        fields[part.name] = await part.text()
                if not archive.exists():
                    raise web.HTTPBadRequest(text="请选择 ZIP 配置包")
                if fields.get("kind") == "workspace":
                    result = await client.call("import_workspace", archive=str(archive), robot_id=fields.get("robot_id", ""), name=fields.get("name", ""))
                else:
                    require_stopped()
                    result = await client.call(
                        "import_bundle", archive=str(archive),
                        expected_plugin_type=fields.get("expected_plugin_type", ""),
                    )
                return web.json_response(result)

    async def export(request):
        # Stream inside the lifetime of the temporary directory.
        with tempfile.TemporaryDirectory(prefix="humanoid-web-export-") as temporary:
            output = Path(temporary) / "configuration.zip"
            await client.call("export", robot_id=request.match_info["robot_id"],
                revision=request.query.get("revision", ""), output=str(output))
            response = web.StreamResponse(headers={"Content-Type": "application/zip",
                "Content-Disposition": 'attachment; filename="robot-configuration.zip"'})
            response.content_length = output.stat().st_size
            await response.prepare(request)
            with output.open("rb") as stream:
                while chunk := stream.read(256 * 1024):
                    await response.write(chunk)
            await response.write_eof()
            return response

    async def execute_pose(request):
        robot_id = request.match_info["robot_id"]
        pose_id = request.match_info["pose_id"]
        async with motion_lock:
            detail = await client.call("get", robot_id=robot_id)
            deployed = detail.get("deployed")
            if not deployed or not deployed.get("revision") or deployed.get("revision") != detail.get("latest"):
                raise web.HTTPConflict(text="请先校验、保存并应用包含该初始姿态的机器人版本")
            platform = runtime.ros.platform_status() if runtime.ros else {}
            current = platform.get("configuration")
            if not current or not current.get("fresh"):
                raise web.HTTPConflict(text="没有检测到当前运行机器人的新鲜配置状态")
            identity = current.get("data", {})
            if identity.get("robot_id") != robot_id or identity.get("revision") != detail.get("latest"):
                raise web.HTTPConflict(text="网页所选机器人版本与当前运行版本不一致，请启动已部署版本")
            if identity.get("state") != "observed" or identity.get("missing_nodes"):
                raise web.HTTPConflict(text="机器人驱动或运动服务尚未全部就绪")
            teleop = platform.get("teleop")
            if teleop and teleop.get("fresh") and teleop.get("data", {}).get("enabled"):
                raise web.HTTPConflict(text="请先在遥操作端停止使能，再执行初始姿态")
            if runtime.player and runtime.player.status().get("is_active"):
                raise web.HTTPConflict(text="正在回放数据，请停止回放后再执行初始姿态")
            try:
                pose = resolve_initial_pose(detail["saved"], pose_id)
                results = await asyncio.to_thread(
                    execute_move_j_pose, pose["goals"], runtime.config["ros"]["domain_id"]
                )
            except DeploymentError as error:
                raise web.HTTPBadRequest(text=str(error)) from error
            except MotionCommandError as error:
                raise web.HTTPConflict(text=str(error)) from error
            return web.json_response({
                "ok": True,
                "robot_id": robot_id,
                "revision": detail["latest"],
                "pose_id": pose["id"],
                "pose_name": pose["name"],
                "results": results,
            })

    def require_running_robot(detail, robot_id, required_nodes):
        deployed = detail.get("deployed")
        if not deployed or deployed.get("revision") != detail.get("latest"):
            raise web.HTTPConflict(text="请先保存并应用当前机器人版本")
        platform = runtime.ros.platform_status() if runtime.ros else {}
        current = platform.get("configuration")
        if not current or not current.get("fresh"):
            raise web.HTTPConflict(text="没有检测到当前运行机器人的新鲜配置状态")
        identity = current.get("data", {})
        if identity.get("robot_id") != robot_id or identity.get("revision") != detail.get("latest"):
            raise web.HTTPConflict(text="网页配置与当前运行机器人版本不一致")
        if identity.get("state") != "observed" or identity.get("missing_nodes"):
            raise web.HTTPConflict(text="机器人运行组件尚未全部就绪")
        graph = runtime.ros.status() if runtime.ros else {}
        nodes = {item.get("name") for item in graph.get("discovered_nodes", [])}
        missing = sorted(set(required_nodes) - nodes)
        if graph.get("state") != "running" or graph.get("graph_age", 100) > 5:
            raise web.HTTPConflict(text="ROS 2 节点图不可用或状态已过期")
        if missing:
            raise web.HTTPConflict(text="测试所需节点未运行: " + ", ".join(missing))
        teleop = platform.get("teleop")
        if teleop and teleop.get("fresh") and teleop.get("data", {}).get("enabled"):
            raise web.HTTPConflict(text="请先停止遥操作使能，再执行手动测试")
        if runtime.player and runtime.player.status().get("is_active"):
            raise web.HTTPConflict(text="正在回放数据，请停止回放后再执行手动测试")

    async def test_gripper(request):
        robot_id = request.match_info["robot_id"]
        gripper_name = request.match_info["gripper_name"]
        body = await request.json()
        if not isinstance(body, dict) or set(body) != {"target"}:
            raise web.HTTPBadRequest(text="夹爪测试参数无效")
        async with motion_lock:
            detail = await client.call("get", robot_id=robot_id)
            require_running_robot(detail, robot_id, {"humanoid_gripper_runtime"})
            try:
                command = resolve_gripper_test(detail["saved"], gripper_name, body["target"])
                result = await asyncio.to_thread(
                    execute_gripper_test, command, runtime.config["ros"]["domain_id"]
                )
            except DeploymentError as error:
                raise web.HTTPBadRequest(text=str(error)) from error
            except GripperCommandError as error:
                raise web.HTTPConflict(text=str(error)) from error
            return web.json_response({"ok": True, "target": body["target"], **result})

    async def jog_joint(request):
        robot_id = request.match_info["robot_id"]
        joint_name = request.match_info["joint_name"]
        body = await request.json()
        if not isinstance(body, dict) or set(body) != {"delta_rad"}:
            raise web.HTTPBadRequest(text="关节点动参数无效")
        async with motion_lock:
            detail = await client.call("get", robot_id=robot_id)
            require_running_robot(
                detail, robot_id, {"humanoid_driver_runtime", "humanoid_motion_control"}
            )
            try:
                jog = resolve_joint_jog(detail["saved"], joint_name, body["delta_rad"])
                result = await asyncio.to_thread(
                    execute_joint_jog, jog, runtime.config["ros"]["domain_id"]
                )
            except DeploymentError as error:
                raise web.HTTPBadRequest(text=str(error)) from error
            except MotionCommandError as error:
                raise web.HTTPConflict(text=str(error)) from error
            return web.json_response({"ok": True, **result})

    app.router.add_get("/api/adapters", catalog)
    app.router.add_post("/api/adapters/robots", create)
    app.router.add_post("/api/adapters/import", upload)
    app.router.add_get("/api/adapters/robots/{robot_id}", detail)
    app.router.add_get("/api/adapters/robots/{robot_id}/export", export)
    app.router.add_post("/api/adapters/robots/{robot_id}/poses/{pose_id}/execute", execute_pose)
    app.router.add_post(
        "/api/adapters/robots/{robot_id}/grippers/{gripper_name}/test", test_gripper
    )
    app.router.add_post(
        "/api/adapters/robots/{robot_id}/joints/{joint_name}/jog", jog_joint
    )
    app.router.add_post("/api/adapters/robots/{robot_id}/{operation}", action)
    return client
