"""Configure through the real HTTP API, then run only the packaged Mock driver."""
import asyncio
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import tempfile
import time
import unittest
from unittest.mock import patch

import pytest
pytest.importorskip("aiohttp")
pytest.importorskip("mcap")
pytest.importorskip("rclpy")
from aiohttp import FormData
from aiohttp.test_utils import TestClient, TestServer
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import yaml

from humanoid_manager.deployment import pack_directory, resolve_robot_deployment
from humanoid_manager.web.config import ConfigStore
from humanoid_manager.web.server import create_app
from test_deployment_plugins import _hardware_tree, _model_tree, _write_yaml


def mock_plugins(folder):
    """Use installed generic test resources and an actual plugin shared library."""
    driver = _hardware_tree(folder / "driver")
    manifest = yaml.safe_load((driver / "manifest.yaml").read_text())
    driver_class = "humanoid_driver_runtime/MockRobotDriver"
    manifest["plugin_class"] = driver_class
    _write_yaml(driver / "manifest.yaml", manifest)
    (driver / manifest["plugin_xml"]).write_text(
        '<library path="fake_driver"><class name="' + driver_class + '" '
        'type="humanoid_driver_runtime::MockRobotDriver" '
        'base_class_type="humanoid_driver_interface::RobotDriverPlugin"/></library>')
    driver_share = Path(get_package_share_directory("humanoid_driver_runtime"))
    shutil.copyfile(driver_share / "config/mock_driver.yaml", driver / manifest["resources"]["driver_params"])
    shutil.copyfile(Path(get_package_prefix("humanoid_driver_runtime")) / "lib/libhumanoid_mock_driver.so",
                    driver / manifest["library"])

    model = _model_tree(folder / "model")
    resources = model / "resources"
    motion_share = Path(get_package_share_directory("humanoid_motion_server"))
    for source, destination in (("motion_control.yaml", "motion.yaml"),
                                ("channels.yaml", "channels.yaml"), ("tools.yaml", "tools.yaml")):
        shutil.copyfile(motion_share / "config" / source, resources / destination)
    sdk = yaml.safe_load((motion_share / "config/robo_manip.test_humanoid.yaml").read_text())
    sdk.pop("execution", None)
    sdk["model_path"] = "robot.urdf"
    _write_yaml(resources / "sdk.yaml", sdk)
    shutil.copyfile(motion_share / "urdf/test_humanoid.urdf", resources / "robot.urdf")
    receiver = yaml.safe_load((Path(get_package_share_directory("hc_teleop_recv")) /
                              "config/single_arm.example.yaml").read_text())
    receiver["input"].update(mode="vrdata", publish_vrdata=False)
    receiver["channels"][0].update(target_pose_topic="/teleop/servo_p",
                                  fk_pose_topic="/teleop/left_arm/fk_pose", tool_frame="left_tool0")
    _write_yaml(resources / "hc_teleop.yaml", receiver)
    manifest = yaml.safe_load((model / "manifest.yaml").read_text())
    manifest["resources"]["hc_teleop_config"] = "resources/hc_teleop.yaml"
    _write_yaml(model / "manifest.yaml", manifest)
    return [("hardware_driver", pack_directory(driver, folder / "driver.zip")),
            ("robot_model", pack_directory(model, folder / "model.zip"))]


class WebRobotWorkflowTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="manager-web-workflow-")
        self.root = Path(self.temporary.name)
        self.process = None
        self.robot_log = None
        self.client = None
        self.domain_id = 229
        transport = self.root / "dds.xml"
        transport.write_text('''<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors><transport_descriptor><transport_id>loopback_udp</transport_id>
    <type>UDPv4</type><interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>
  </transport_descriptor></transport_descriptors>
  <participant profile_name="loopback" is_default_profile="true"><rtps>
    <userTransports><transport_id>loopback_udp</transport_id></userTransports>
    <useBuiltinTransports>false</useBuiltinTransports>
  </rtps></participant>
</profiles>''')
        self.environment = patch.dict(os.environ, {"ROS_LOCALHOST_ONLY": "1",
            "ROS_DOMAIN_ID": str(self.domain_id), "FASTRTPS_DEFAULT_PROFILES_FILE": str(transport),
            "PYTHONDONTWRITEBYTECODE": "1"})
        self.environment.start()
        self.addCleanup(self.environment.stop)
        store = ConfigStore(self.root / "manager.yaml")
        store.save({"adapter_manager": {
            "cli": str(Path(__file__).resolve().parents[1] / "scripts/humanoid_pluginctl.py"),
            "plugin_root": str(self.root / "plugins"), "state_root": str(self.root / "configuration")},
            "ros": {"enabled": True, "domain_id": self.domain_id,
                    "recording": {"directory": str(self.root / "recordings")}}})
        self.app = create_app(store)
        self.client = TestClient(TestServer(self.app))
        await self.client.start_server()
        self.runtime = self.app["runtime"]
        await self.wait_for(lambda: self.runtime.ros.status()["state"] == "running", 10)

    async def asyncTearDown(self):
        if self.process and self.process.returncode is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                await asyncio.wait_for(self.process.wait(), 10)
            except asyncio.TimeoutError:
                os.killpg(self.process.pid, signal.SIGKILL)
                await self.process.wait()
        if self.robot_log:
            self.robot_log.close()
        if self.client:
            await self.client.close()
        self.temporary.cleanup()

    async def wait_for(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            if self.process and self.process.returncode is not None:
                break
            await asyncio.sleep(.1)
        log = self.root / "robot.log"
        self.fail("等待状态超时：" + str(self.runtime.ros.status()) + "\n" +
                  (log.read_text()[-6000:] if log.exists() else ""))

    async def post(self, path, document, expected=200):
        response = await self.client.post(path, json=document)
        text = await response.text()
        self.assertEqual(response.status, expected, text)
        return json.loads(text) if expected < 300 else text

    async def test_unified_web_start_save_restart_stop_with_real_mock_nodes(self):
        self.runtime.launcher.enabled = True
        # A tiny installed vendor launch proves integration and lifecycle
        # ownership without starting CAN, ros2_control, or physical drivers.
        vendor_prefix = self.root / 'vendor_install'
        marker = vendor_prefix / 'share/ament_index/resource_index/packages/mock_vendor'
        marker.parent.mkdir(parents=True)
        marker.write_text('')
        vendor_launch = vendor_prefix / 'share/mock_vendor/launch/robot.launch.py'
        vendor_launch.parent.mkdir(parents=True)
        vendor_pids = self.root / 'vendor_pids.txt'
        script = f"import os,time; open({str(vendor_pids)!r}, 'a').write(str(os.getpid())+'\\n'); time.sleep(120)"
        vendor_launch.write_text('from launch import LaunchDescription\n'
            'from launch.actions import DeclareLaunchArgument, ExecuteProcess\n'
            'def generate_launch_description():\n'
            "    return LaunchDescription([DeclareLaunchArgument('robot_id', default_value='vendor_default'),\n"
            f"        ExecuteProcess(cmd=['/usr/bin/python3', '-c', {script!r}])])\n")
        vendor_environment = patch.dict(os.environ, {'AMENT_PREFIX_PATH': str(vendor_prefix) + ':' + os.environ['AMENT_PREFIX_PATH']})
        vendor_environment.start()
        self.addCleanup(vendor_environment.stop)
        self.runtime.launcher.bringup = {'package': 'mock_vendor', 'launch_file': 'robot.launch.py',
                                        'arguments': {'robot_id': 'vendor_scope_only'}}
        for kind, archive in mock_plugins(self.root):
            data = FormData()
            data.add_field('kind', 'plugin')
            data.add_field('expected_plugin_type', kind)
            data.add_field('archive', archive.read_bytes(), filename=archive.name, content_type='application/zip')
            response = await self.client.post('/api/adapters/import', data=data)
            self.assertEqual(response.status, 200, await response.text())
        robot = await self.post('/api/adapters/robots', {'robot_id': 'managed_mock', 'name': '运行控制测试',
            'driver_id': 'fake_driver', 'model_id': 'test_model'}, expected=201)
        # Browser JSON turns integral floats into integers, including driver
        # scales/offsets and rates. Save that exact shape before the real launch.
        def browser_numbers(value):
            if isinstance(value, float) and value.is_integer():
                return int(value)
            if isinstance(value, list):
                return [browser_numbers(item) for item in value]
            if isinstance(value, dict):
                return {key: browser_numbers(item) for key, item in value.items()}
            return value
        document = browser_numbers(robot['draft'])
        motion = document['resources']['motion_params']['humanoid_motion_control']['ros__parameters']
        for key, limits in motion.items():
            if key.startswith('group_upper_limits.'):
                motion[key] = [0 if value == .1 else value for value in limits]
        robot = await self.post('/api/adapters/robots/managed_mock/save', {'document': document, 'etag': robot['etag']})
        await self.post('/api/launcher/start', {'robot_id': robot['robot_id'], 'revision': robot['latest'],
            'start_teleop': True, 'start_cameras': False})
        process = self.runtime.launcher.processes[0][1]
        def observed(revision):
            state = self.runtime.ros.platform_status().get('configuration')
            return (state and state['fresh'] and state['data'].get('state') == 'observed'
                    and state['data'].get('revision') == revision)
        try:
            await self.wait_for(lambda: observed(robot['latest']), 20)
            self.assertEqual(len(vendor_pids.read_text().splitlines()), 1)
            await self.post('/api/launcher/start', {'robot_id': robot['robot_id']}, expected=409)
            robot = await (await self.client.get('/api/adapters/robots/managed_mock')).json()
            document = copy.deepcopy(robot['draft'])
            document['resources']['driver_params']['humanoid_driver_runtime']['ros__parameters']['command_watchdog_ms'] = 220.0
            old_revision = robot['latest']
            robot = await self.post('/api/adapters/robots/managed_mock/save', {'document': document, 'etag': robot['etag']})
            self.assertNotEqual(robot['latest'], old_revision)
            self.assertIsNone(process.returncode)
            self.assertTrue(observed(old_revision))
            await self.post('/api/adapters/robots/managed_mock/apply', {'revision': robot['latest'], 'etag': robot['etag']}, expected=409)
            # Restart runs concurrently with this GET: the web stays responsive.
            restart = asyncio.create_task(self.post('/api/launcher/restart', {
                'robot_id': robot['robot_id'], 'revision': robot['latest']}))
            await asyncio.sleep(.1)
            self.assertEqual((await self.client.get('/dashboard/')).status, 200)
            await restart
            self.assertIsNotNone(process.returncode)
            self.assertNotEqual(self.runtime.launcher.processes[0][1].pid, process.pid)
            await self.wait_for(lambda: observed(robot['latest']), 20)
            self.assertEqual(len(vendor_pids.read_text().splitlines()), 2)
            deployment = resolve_robot_deployment(self.root / 'plugins', robot['robot_id'])
            parameters = yaml.safe_load(deployment.resources['driver_params'].read_text())
            self.assertEqual(parameters['humanoid_driver_runtime']['ros__parameters']['command_watchdog_ms'], 220.0)
            await self.post('/api/launcher/stop', {})
            self.assertEqual((await self.client.get('/dashboard/')).status, 200)
            self.assertEqual(self.runtime.launcher.status()['phase'], 'stopped')
            self.assertEqual(self.runtime.launcher.status()['owned_processes'], 0)
            for pid in vendor_pids.read_text().splitlines():
                with self.assertRaises(ProcessLookupError):
                    os.kill(int(pid), 0)
        except BaseException:
            print(self.runtime.launcher.log_tail())
            raise

    async def test_individual_import_create_edit_apply_and_run_saved_configuration(self):
        page = await self.client.get("/dashboard/")
        self.assertEqual(page.status, 200)
        catalog = await (await self.client.get("/api/adapters")).json()
        self.assertEqual(catalog["workspaces"], [])
        for kind, archive in mock_plugins(self.root):
            data = FormData()
            data.add_field("kind", "plugin")
            data.add_field("expected_plugin_type", kind)
            data.add_field("archive", archive.read_bytes(), filename=archive.name, content_type="application/zip")
            response = await self.client.post("/api/adapters/import", data=data)
            self.assertEqual(response.status, 200, await response.text())
        robot = await self.post("/api/adapters/robots", {"robot_id": "web_arm", "name": "网页配置测试",
            "driver_id": "fake_driver", "model_id": "test_model"}, expected=201)
        original = robot["latest"]
        document = copy.deepcopy(robot["draft"])
        document["resources"]["driver_params"]["humanoid_driver_runtime"]["ros__parameters"]["command_watchdog_ms"] = 180.0
        document["resources"]["hc_teleop_config"]["channels"][0]["position_scale"] = .35
        robot = await self.post("/api/adapters/robots/web_arm/save", {"document": document, "etag": robot["etag"]})
        self.assertNotEqual(robot["latest"], original)
        invalid = copy.deepcopy(robot["draft"])
        invalid["resources"]["motion_params"]["humanoid_motion_control"]["ros__parameters"]["groups.left_arm"] = ["missing_joint"]
        await self.post("/api/adapters/robots/web_arm/save", {"document": invalid, "etag": robot["etag"]}, expected=400)
        await self.post("/api/adapters/robots/web_arm/apply", {"revision": robot["latest"], "etag": robot["etag"]})
        deployment = resolve_robot_deployment(self.root / "plugins", "web_arm")
        applied = yaml.safe_load(deployment.resources["driver_params"].read_text())
        self.assertEqual(applied["humanoid_driver_runtime"]["ros__parameters"]["command_watchdog_ms"], 180.0)
        self.robot_log = (self.root / "robot.log").open("w")
        self.process = await asyncio.create_subprocess_exec("ros2", "launch", "humanoid_manager",
            "managed_robot.launch.py", "robot_id:=web_arm", f"plugin_root:={self.root / 'plugins'}",
            "start_cameras:=false", stdout=self.robot_log, stderr=self.robot_log, start_new_session=True)

        def observed():
            state = self.runtime.ros.platform_status().get("configuration")
            return state and state["fresh"] and state["data"].get("state") == "observed"
        await self.wait_for(observed, 20)
        state = self.runtime.ros.platform_status()["configuration"]["data"]
        self.assertEqual(state["robot_id"], "web_arm")
        self.assertEqual(state["revision"], robot["latest"])
        expected_receiver_hash = hashlib.sha256(deployment.resources["hc_teleop_config"].read_bytes()).hexdigest()
        def receiver_loaded_saved_configuration():
            teleop = self.runtime.ros.platform_status().get("teleop")
            return (teleop and teleop["fresh"] and teleop["data"].get("configuration", {}).get("sha256")
                    == expected_receiver_hash)
        await self.wait_for(receiver_loaded_saved_configuration, 5)
        # This HTTP action moves the isolated Mock driver, never a physical robot.
        result = await self.post("/api/adapters/robots/web_arm/joints/left_shoulder_pitch/jog", {"delta_rad": .017453292519943295})
        self.assertTrue(result["ok"])
        await self.post("/api/adapters/robots/web_arm/apply", {"revision": robot["latest"], "etag": robot["etag"]}, expected=409)
