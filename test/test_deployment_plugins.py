from __future__ import annotations

from pathlib import Path
import platform
import shutil
import zipfile

import pytest
import yaml

from humanoid_adapter_manager.deployment import (
    DeploymentError,
    deploy_archive,
    list_deployed,
    pack_directory,
    resolve_robot_deployment,
    validate_archive,
)


PLUGIN_CLASS = "fake_driver/FakeRobotDriver"


def _architecture() -> str:
    value = platform.machine().lower()
    return {"amd64": "x86_64", "arm64": "aarch64"}.get(value, value)


def _write_yaml(path: Path, document: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")


def _hardware_tree(
    root: Path, *, joint_names: list[str] | None = None
) -> Path:
    joint_names = joint_names or ["joint1", "joint2"]
    package_share = root / "prefix/share/fake_driver"
    marker = root / "prefix/share/ament_index/resource_index/packages/fake_driver"
    library = root / "prefix/lib/libfake_driver.so"
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.write_text("", encoding="utf-8")
    package_share.mkdir(parents=True, exist_ok=True)
    (package_share / "package.xml").write_text(
        """<?xml version="1.0"?>
<package format="3"><name>fake_driver</name><version>1.0.0</version>
<description>test</description><maintainer email="test@example.com">test</maintainer>
<license>Proprietary</license></package>
""",
        encoding="utf-8",
    )
    plugins = package_share / "plugins/fake_driver_plugins.xml"
    plugins.parent.mkdir(parents=True, exist_ok=True)
    plugins.write_text(
        f"""<library path="fake_driver">
  <class name="{PLUGIN_CLASS}" type="fake_driver::FakeRobotDriver"
    base_class_type="humanoid_driver_interface::RobotDriverPlugin"/>
</library>
""",
        encoding="utf-8",
    )
    driver_config = package_share / "config/driver.yaml"
    _write_yaml(
        driver_config,
        {
            "humanoid_driver_runtime": {
                "ros__parameters": {
                    "plugin_class": PLUGIN_CLASS,
                    "joint_names": joint_names,
                    "vendor_joint_names": [name.upper() for name in joint_names],
                    "vendor_joint_groups": ["arm"] * len(joint_names),
                }
            }
        },
    )
    library.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile("/bin/true", library)
    _write_yaml(
        root / "manifest.yaml",
        {
            "schema_version": 1,
            "artifact_type": "plugin",
            "plugin_type": "hardware_driver",
            "plugin_id": "fake_driver",
            "name": "Fake test driver",
            "compatibility": {
                "ros_distro": "humble",
                "architecture": _architecture(),
                "driver_interface_abi": 1,
            },
            "package_name": "fake_driver",
            "ament_prefix": "prefix",
            "plugin_xml": "prefix/share/fake_driver/plugins/fake_driver_plugins.xml",
            "library": "prefix/lib/libfake_driver.so",
            "plugin_class": PLUGIN_CLASS,
            "resources": {
                "driver_params": "prefix/share/fake_driver/config/driver.yaml",
            },
        },
    )
    return root


def _model_tree(
    root: Path,
    *,
    sdk_joint_order: list[str] | None = None,
    configure_execution: bool = False,
) -> Path:
    resources = root / "resources"
    _write_yaml(
        resources / "motion.yaml",
        {
            "humanoid_motion_control": {
                "ros__parameters": {
                    "joint_group_names": ["arm"],
                    "groups.arm": ["joint1", "joint2"],
                    "group_lower_limits.arm": [-1.0, -1.0],
                    "group_upper_limits.arm": [1.0, 1.0],
                }
            }
        },
    )
    sdk = {
        "model_path": "robot.urdf",
        "joint_groups": {"arm": sdk_joint_order or ["joint1", "joint2"]},
    }
    if configure_execution:
        sdk["execution"] = {"driver": "mock"}
    _write_yaml(resources / "sdk.yaml", sdk)
    _write_yaml(
        resources / "channels.yaml",
        {
            "channels": [
                {
                    "name": "arm_move_j",
                    "kind": "move_j",
                    "endpoint": "/motion/arm/move_j",
                    "priority": 50,
                    "group": "arm",
                }
            ]
        },
    )
    _write_yaml(resources / "tools.yaml", {"tools": []})
    (resources / "robot.urdf").write_text(
        """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base"/><link name="link1"/><link name="link2"/>
  <joint name="joint1" type="revolute"><parent link="base"/><child link="link1"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
  <joint name="joint2" type="revolute"><parent link="link1"/><child link="link2"/>
    <axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
</robot>
""",
        encoding="utf-8",
    )
    _write_yaml(
        root / "manifest.yaml",
        {
            "schema_version": 1,
            "artifact_type": "plugin",
            "plugin_type": "robot_model",
            "plugin_id": "test_model",
            "name": "Test robot model",
            "resources": {
                "motion_params": "resources/motion.yaml",
                "sdk_config": "resources/sdk.yaml",
                "channel_config": "resources/channels.yaml",
                "tool_config": "resources/tools.yaml",
                "urdf": "resources/robot.urdf",
            },
        },
    )
    return root


def _composition_tree(root: Path, *, name: str = "Test robot") -> Path:
    _write_yaml(
        root / "manifest.yaml",
        {
            "schema_version": 1,
            "artifact_type": "robot_composition",
            "robot_id": "test_robot",
            "name": name,
            "plugins": {
                "hardware_driver": "fake_driver",
                "robot_model": "test_model",
            },
        },
    )
    return root


def test_deploy_independent_plugins_compose_and_resolve(tmp_path: Path) -> None:
    plugin_root = tmp_path / "deployed"
    hardware_archive = tmp_path / "hardware.zip"
    model_archive = tmp_path / "model.zip"
    composition_archive = tmp_path / "composition.zip"
    replacement_archive = tmp_path / "replacement.zip"
    pack_directory(_hardware_tree(tmp_path / "hardware"), hardware_archive)
    pack_directory(_model_tree(tmp_path / "model"), model_archive)
    pack_directory(_composition_tree(tmp_path / "composition"), composition_archive)
    pack_directory(
        _composition_tree(tmp_path / "replacement", name="Replacement robot"),
        replacement_archive,
    )

    assert validate_archive(hardware_archive)["plugin_id"] == "fake_driver"
    assert validate_archive(model_archive)["plugin_id"] == "test_model"

    with pytest.raises(DeploymentError, match="not deployed"):
        deploy_archive(composition_archive, plugin_root)

    hardware_path = deploy_archive(hardware_archive, plugin_root, check_linkage=False)
    with pytest.raises(DeploymentError, match="not deployed"):
        deploy_archive(composition_archive, plugin_root)
    model_path = deploy_archive(model_archive, plugin_root)
    robot_path = deploy_archive(composition_archive, plugin_root)

    assert hardware_path == plugin_root / "hardware_drivers/fake_driver"
    assert model_path == plugin_root / "robot_models/test_model"
    assert robot_path == plugin_root / "robots/test_robot"

    deployment = resolve_robot_deployment(plugin_root, "test_robot")
    assert deployment.driver_class == PLUGIN_CLASS
    assert deployment.name == "Test robot"
    assert deployment.resources["driver_params"].is_file()
    assert deployment.resources["urdf"].is_file()
    assert deployment.driver_plugin_xml_paths[0].is_file()
    assert str(deployment.driver_ament_prefixes[0]) in deployment.environment()[
        "AMENT_PREFIX_PATH"
    ]
    assert str(deployment.library_paths[0]) in deployment.environment()["LD_LIBRARY_PATH"]
    assert "LD_LIBRARY_PATH" not in deployment.resource_environment({})

    assert deploy_archive(replacement_archive, plugin_root) == robot_path
    assert resolve_robot_deployment(plugin_root, "test_robot").name == "Replacement robot"
    assert list_deployed(plugin_root) == {
        "hardware_drivers": {
            "fake_driver": {"path": str(hardware_path.resolve())},
        },
        "robot_models": {
            "test_model": {"path": str(model_path.resolve())},
        },
        "robots": {
            "test_robot": {"path": str(robot_path.resolve())},
        },
    }
    assert list((plugin_root / ".staging").iterdir()) == []


def test_model_cross_configuration_mismatch_is_rejected(tmp_path: Path) -> None:
    with pytest.raises(DeploymentError, match="joint order differ"):
        pack_directory(
            _model_tree(tmp_path / "bad-model", sdk_joint_order=["joint2", "joint1"]),
            tmp_path / "bad-model.zip",
        )


def test_model_cannot_select_an_execution_driver(tmp_path: Path) -> None:
    with pytest.raises(DeploymentError, match="must not configure execution"):
        pack_directory(
            _model_tree(tmp_path / "executing-model", configure_execution=True),
            tmp_path / "executing-model.zip",
        )


def test_composition_rejects_incompatible_driver_and_model(tmp_path: Path) -> None:
    plugin_root = tmp_path / "deployed"
    hardware_archive = tmp_path / "hardware.zip"
    model_archive = tmp_path / "model.zip"
    composition_archive = tmp_path / "composition.zip"
    pack_directory(
        _hardware_tree(tmp_path / "hardware", joint_names=["joint1", "joint3"]),
        hardware_archive,
    )
    pack_directory(_model_tree(tmp_path / "model"), model_archive)
    pack_directory(_composition_tree(tmp_path / "composition"), composition_archive)
    deploy_archive(hardware_archive, plugin_root, check_linkage=False)
    deploy_archive(model_archive, plugin_root)
    with pytest.raises(DeploymentError, match="different logical joints"):
        deploy_archive(composition_archive, plugin_root)


def test_hardware_bundle_cannot_embed_third_party_libraries(tmp_path: Path) -> None:
    tree = _hardware_tree(tmp_path / "hardware-with-private-dependency")
    shutil.copyfile("/bin/true", tree / "prefix/lib/libprivate_dependency.so")
    with pytest.raises(DeploymentError, match="third-party dependencies"):
        pack_directory(tree, tmp_path / "hardware.zip")


def test_archive_path_traversal_is_rejected(tmp_path: Path) -> None:
    archive = tmp_path / "traversal.zip"
    with zipfile.ZipFile(archive, "w") as output:
        output.writestr("../outside", "unsafe")
    with pytest.raises(DeploymentError, match="unsafe path"):
        validate_archive(archive)
