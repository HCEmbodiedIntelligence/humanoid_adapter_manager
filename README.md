# humanoid_adapter_manager

`humanoid_adapter_manager` owns the control-plane operations for no-build robot deployments. It
validates, packs, deploys, lists, and resolves independent hardware-driver plugins, robot-model
plugins, and their small robot-composition descriptors. It does not run motion control, load
vendor drivers, or start robot processes.

The CLI is intentionally short-lived:

```bash
ros2 run humanoid_adapter_manager humanoid_pluginctl.py validate bundle.zip
ros2 run humanoid_adapter_manager humanoid_pluginctl.py deploy bundle.zip
ros2 run humanoid_adapter_manager humanoid_pluginctl.py list
ros2 run humanoid_adapter_manager humanoid_pluginctl.py resolve robot_id
```

Whole-robot process orchestration belongs to `robot_bringup`. A composition contains references
only; driver parameters remain with the driver plugin and URDF/motion resources remain with the
model plugin. Runtime consumers receive resolved absolute paths and read-only environment
overrides; plugin deployment is not part of their control loop. See
[`docs/deploying_plugins.md`](docs/deploying_plugins.md) for the bundle format.
