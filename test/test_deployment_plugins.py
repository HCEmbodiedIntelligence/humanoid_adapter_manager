from __future__ import annotations

from pathlib import Path
import platform
import shutil
import zipfile

import pytest
import yaml

from humanoid_manager.deployment import (
    DeploymentError,
    deploy_archive,
    list_deployed,
    pack_directory,
    resolve_robot_deployment,
    validate_archive,
)


PLUGIN_CLASS = "fake_driver/FakeRobotDriver"
GRIPPER_CLASS = "fake_gripper/FakeGripperDriver"


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


def _gripper_tree(root: Path) -> Path:
    package_share = root / "prefix/share/fake_gripper"
    marker = root / "prefix/share/ament_index/resource_index/packages/fake_gripper"
    library = root / "prefix/lib/libfake_gripper.so"
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.write_text("", encoding="utf-8")
    package_share.mkdir(parents=True, exist_ok=True)
    (package_share / "package.xml").write_text(
        """<?xml version="1.0"?>
<package format="3"><name>fake_gripper</name><version>1.0.0</version>
<description>test</description><maintainer email="test@example.com">test</maintainer>
<license>Proprietary</license></package>
""",
        encoding="utf-8",
    )
    plugins = package_share / "plugins/fake_gripper_plugins.xml"
    plugins.parent.mkdir(parents=True, exist_ok=True)
    plugins.write_text(
        f"""<library path="fake_gripper">
  <class name="{GRIPPER_CLASS}" type="fake_gripper::FakeGripperDriver"
    base_class_type="humanoid_driver_interface::GripperDriverPlugin"/>
</library>
""",
        encoding="utf-8",
    )
    gripper_config = package_share / "config/gripper.yaml"
    _write_yaml(gripper_config, {
        "humanoid_gripper_runtime": {"ros__parameters": {
            "plugin_class": GRIPPER_CLASS,
            "gripper_names": ["left_gripper", "right_gripper"],
            "vendor_gripper_names": ["left_finger", "right_finger"],
            "position_units": ["m", "m"],
            "vendor_to_logical_scales": [1.0, -1.0],
            "vendor_to_logical_offsets": [0.0, 0.04],
            "platform_gripper_state_topic": "/hc_teleop/gripper_states",
            "platform_gripper_command_topic": "/hc_teleop/gripper_commands",
            "plugin_parameters": ["transport=test"],
        }}
    })
    library.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile("/bin/true", library)
    _write_yaml(root / "manifest.yaml", {
        "schema_version": 1,
        "artifact_type": "plugin",
        "plugin_type": "gripper_driver",
        "plugin_id": "fake_gripper",
        "name": "Fake test gripper",
        "compatibility": {
            "ros_distro": "humble",
            "architecture": _architecture(),
            "driver_interface_abi": 1,
        },
        "package_name": "fake_gripper",
        "ament_prefix": "prefix",
        "plugin_xml": "prefix/share/fake_gripper/plugins/fake_gripper_plugins.xml",
        "library": "prefix/lib/libfake_gripper.so",
        "plugin_class": GRIPPER_CLASS,
        "resources": {"gripper_params": "prefix/share/fake_gripper/config/gripper.yaml"},
    })
    return root


def _ros_topic_gripper_tree(root: Path) -> Path:
    root = _gripper_tree(root)
    manifest_path = root / "manifest.yaml"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    manifest["plugin_class"] = "humanoid_gripper/RosTopicGripperDriver"
    _write_yaml(manifest_path, manifest)
    plugin_path = root / manifest["plugin_xml"]
    plugin_path.write_text(
        plugin_path.read_text(encoding="utf-8").replace(
            GRIPPER_CLASS, manifest["plugin_class"]
        ),
        encoding="utf-8",
    )
    config_path = root / manifest["resources"]["gripper_params"]
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    parameters = config["humanoid_gripper_runtime"]["ros__parameters"]
    parameters["plugin_class"] = manifest["plugin_class"]
    parameters["plugin_parameters"] = [
        "left_gripper.command_topic=/left/command",
        "left_gripper.command_type=float64_multi_array",
        "left_gripper.feedback_topic=/joint_states",
        "left_gripper.feedback_type=joint_state",
        "left_gripper.min_position=0",
        "left_gripper.max_position=0.04",
        "right_gripper.command_topic=/right/command",
        "right_gripper.command_type=float64_multi_array",
        "right_gripper.feedback_topic=/joint_states",
        "right_gripper.feedback_type=joint_state",
        "right_gripper.min_position=0",
        "right_gripper.max_position=0.04",
        "feedback_timeout_s=0.5",
    ]
    import importlib.util
    packager = Path(__file__).resolve().parents[2] / 'humanoid_gripper/tools/create_deployment_bundle.py'
    spec = importlib.util.spec_from_file_location('gripper_packager', packager)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    manifest.update(module.plugin_metadata(config))
    _write_yaml(manifest_path, manifest)
    _write_yaml(config_path, config)
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


def _composition_tree(
    root: Path, *, name: str = "Test robot", gripper_id: str = ""
) -> Path:
    plugins = {
        "hardware_driver": "fake_driver",
        "robot_model": "test_model",
    }
    if gripper_id:
        plugins["gripper_driver"] = gripper_id
    _write_yaml(
        root / "manifest.yaml",
        {
            "schema_version": 1,
            "artifact_type": "robot_composition",
            "robot_id": "test_robot",
            "name": name,
            "plugins": plugins,
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
        "gripper_drivers": {},
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


def test_gripper_plugin_composes_and_resolves_independently(tmp_path: Path) -> None:
    plugin_root = tmp_path / "deployed"
    for name, tree in (
        ("hardware", _hardware_tree(tmp_path / "hardware")),
        ("model", _model_tree(tmp_path / "model")),
        ("gripper", _gripper_tree(tmp_path / "gripper")),
        ("composition", _composition_tree(
            tmp_path / "composition", gripper_id="fake_gripper"
        )),
    ):
        archive = pack_directory(tree, tmp_path / f"{name}.zip")
        deploy_archive(archive, plugin_root, check_linkage=False)

    deployment = resolve_robot_deployment(plugin_root, "test_robot")
    assert deployment.gripper_class == GRIPPER_CLASS
    assert deployment.resources["gripper_params"].is_file()
    assert deployment.gripper_plugin_xml_paths[0].is_file()
    assert str(deployment.gripper_ament_prefixes[0]) in deployment.gripper_environment()[
        "AMENT_PREFIX_PATH"
    ]
    assert str(deployment.gripper_library_paths[0]) in deployment.gripper_environment()[
        "LD_LIBRARY_PATH"
    ]


def test_ros_topic_gripper_parameters_are_validated_before_deployment(tmp_path: Path) -> None:
    tree = _ros_topic_gripper_tree(tmp_path / "gripper")
    pack_directory(tree, tmp_path / "gripper.zip")
    manifest = yaml.safe_load((tree / "manifest.yaml").read_text(encoding="utf-8"))
    config_path = tree / manifest["resources"]["gripper_params"]
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    parameters = config["humanoid_gripper_runtime"]["ros__parameters"]
    parameters["plugin_parameters"] = [
        entry for entry in parameters["plugin_parameters"]
        if not entry.startswith("left_gripper.max_position=")
    ]
    _write_yaml(config_path, config)
    with pytest.raises(DeploymentError, match="left_gripper.max_position"):
        pack_directory(tree, tmp_path / "invalid-gripper.zip")


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


def _hc_model_tree(root: Path) -> Path:
    root = _model_tree(root)
    manifest = yaml.safe_load((root / "manifest.yaml").read_text())
    manifest["resources"]["hc_teleop_config"] = "resources/hc_teleop.yaml"
    _write_yaml(root / "manifest.yaml", manifest)
    _write_yaml(root / "resources/channels.yaml", {"channels": [{
        "name": "teleop_arm", "kind": "servo_p", "endpoint": "/teleop/arm/servo_p",
        "priority": 50, "group": "arm", "base_frame": "base", "tip_frame": "link2",
        "fk_pose_topic": "/teleop/arm/fk_pose",
    }]})
    _write_yaml(root / "resources/hc_teleop.yaml", {"schema_version": 1, "channels": [{
        "id": "arm", "controller": "right", "target_pose_topic": "/teleop/arm/servo_p",
        "fk_pose_topic": "/teleop/arm/fk_pose", "base_frame": "base", "tool_frame": "link2",
        "axis_mapping": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    }]})
    return root


def test_hc_receiver_resource_packs_deploys_and_resolves(tmp_path):
    trees = [_hardware_tree(tmp_path / "hardware"), _hc_model_tree(tmp_path / "model"),
             _composition_tree(tmp_path / "composition")]
    for index, tree in enumerate(trees):
        archive = tmp_path / f"{index}.zip"
        pack_directory(tree, archive)
        deploy_archive(archive, tmp_path / "deployed", check_linkage=False)
    resolved = resolve_robot_deployment(tmp_path / "deployed", "test_robot")
    assert resolved.resources["hc_teleop_config"].is_absolute()
    assert resolved.resources["hc_teleop_config"].is_file()


@pytest.mark.parametrize("key,value", [
    ("base_frame", "link1"), ("tool_frame", "link1"),
    ("fk_pose_topic", "/wrong/fk"), ("target_pose_topic", "/wrong/servo_p"),
    ("axis_mapping", [[1, 0, 0], [0, 1, 0], [0, 0, -1]]),
])
def test_hc_receiver_rejects_incompatible_motion_contract(tmp_path, key, value):
    root = _hc_model_tree(tmp_path / "model")
    path = root / "resources/hc_teleop.yaml"
    config = yaml.safe_load(path.read_text())
    config["channels"][0][key] = value
    _write_yaml(path, config)
    with pytest.raises(DeploymentError, match="invalid hc_teleop_config"):
        pack_directory(root, tmp_path / "bad-model.zip")


def test_legacy_receiver_resource_is_rejected(tmp_path):
    root = _hc_model_tree(tmp_path / "model")
    path = root / "manifest.yaml"
    manifest = yaml.safe_load(path.read_text())
    manifest["resources"]["teleop_config"] = "resources/old.toml"
    (root / "resources/old.toml").write_text("# legacy receiver config\n")
    _write_yaml(path, manifest)
    with pytest.raises(DeploymentError, match="unknown keys"):
        pack_directory(root, tmp_path / "two-receivers.zip")
