# 免编译插件部署契约

## 边界

目标机只安装平台核心包。机器人适配内容拆成三个相互独立的插件类型：

1. `hardware_driver`：驱动 `.so`、pluginlib 元数据及该驱动的参数；
2. `gripper_driver`：可选的夹爪 `.so`、pluginlib 元数据、话题/SDK 参数与夹爪映射；
3. `robot_model`：运动学 URDF、运动组/限制、通道和工具定义。

`robot_composition` 是只引用插件 ID 的整机组合清单，不是功能插件，不包含代码或资源。
`humanoid_manager` 只负责校验、部署和解析；`robot_bringup` 才负责启动进程。

目标机不编译或安装厂商 driver/description 源码包。模型插件不得配置 SDK execution driver；硬件
I/O 始终由 `humanoid_driver_runtime` 独占。这里没有仿真启动逻辑。

默认部署目录：

```text
/var/lib/humanoid-plugins/
├── hardware_drivers/<plugin-id>/
├── gripper_drivers/<plugin-id>/
├── robot_models/<plugin-id>/
├── robots/<robot-id>/
└── .staging/
```

相同 ID 再次部署会原子替换；部署期间不会热加载，也不会自动启动真机。
首次部署前由系统管理员创建目录并交给运行 manager/Web 后端的服务账户，例如：

```bash
sudo install -d -m 0755 -o "$USER" -g "$(id -gn)" /var/lib/humanoid-plugins
```

## hardware_driver 插件

```text
manifest.yaml
checksums.sha256
prefix/
├── lib/libmy_robot_driver.so
└── share/
    ├── ament_index/resource_index/packages/my_robot_driver
    └── my_robot_driver/
        ├── package.xml
        ├── config/driver.yaml
        └── plugins/driver_plugins.xml
```

```yaml
schema_version: 1
artifact_type: plugin
plugin_type: hardware_driver
plugin_id: my_robot_driver
name: My robot driver
compatibility:
  ros_distro: humble
  architecture: x86_64
  driver_interface_abi: 1
package_name: my_robot_driver
ament_prefix: prefix
plugin_xml: prefix/share/my_robot_driver/plugins/driver_plugins.xml
library: prefix/lib/libmy_robot_driver.so
plugin_class: my_robot_driver/MyRobotDriver
resources:
  driver_params: prefix/share/my_robot_driver/config/driver.yaml
```

部署器校验平台、ABI、ELF、pluginlib 基类、驱动参数、包索引、校验和，并默认执行 `ldd -r`。
`prefix/lib` 只允许存在 manifest 声明的插件 `.so`；第三方库必须按正常系统布局安装。

驱动进程会获得只用于发现该未安装插件的 `AMENT_PREFIX_PATH` 和 `LD_LIBRARY_PATH`。motion 进程
不会继承插件库路径。

## gripper_driver 插件

夹爪插件使用 `humanoid_driver_interface::GripperDriverPlugin`，由独立的
`humanoid_gripper_runtime_node` 加载。包结构和二进制校验与 `hardware_driver` 相同，manifest
中的 `plugin_type` 为 `gripper_driver`，资源键为 `gripper_params`。平台侧固定使用具名
`sensor_msgs/msg/JointState`：

```yaml
humanoid_gripper_runtime:
  ros__parameters:
    plugin_class: humanoid_gripper/RosTopicGripperDriver
    platform_gripper_state_topic: /hc_teleop/gripper_states
    platform_gripper_command_topic: /hc_teleop/gripper_commands
    control_frequency_hz: 50.0
    gripper_names: [left_gripper, right_gripper]
    vendor_gripper_names: [left_finger_joint, right_finger_joint]
    position_units: [m, m]
```

上例中的运行参数、映射数组和插件参数会由部署器完整校验。
`humanoid_gripper/RosTopicGripperDriver` 可把已有夹爪驱动的 `JointState`、`Float64`
或 `Float64MultiArray` 话题转换到平台接口。CAN、串口、SDK、Action 或灵巧手可在
`humanoid_gripper` 包中继续添加新的插件类，无需修改机械臂驱动。

## robot_model 插件

模型插件不含可执行代码，也不选择硬件驱动：

```text
manifest.yaml
checksums.sha256
resources/
├── robot.urdf
├── motion.yaml
├── sdk.yaml
├── channels.yaml
└── tools.yaml
```

```yaml
schema_version: 1
artifact_type: plugin
plugin_type: robot_model
plugin_id: my_robot_model
name: My robot model
resources:
  motion_params: resources/motion.yaml
  sdk_config: resources/sdk.yaml
  channel_config: resources/channels.yaml
  tool_config: resources/tools.yaml
  urdf: resources/robot.urdf
```

如果 URDF 使用 `package://`，可以额外声明并携带最小 `ament_prefix`；不引用 mesh 时不需要。
部署器检查 URDF 树、关节组、上下限、SDK joint order、tool frame 和 channel frame。

模型 SDK YAML 中禁止出现 `execution`。motion server 只使用 SDK 做运动计算，命令仍通过平台
joint-command 通道交给 driver runtime。

### HC 遥操作前端配置

需要 HC PICO 遥操作时，在模型 resources 中声明：

```yaml
hc_teleop_config: resources/hc_teleop.yaml
```

配置随模型资源打包。`hc_teleop_recv` 只接收手柄、读取 motion server 的实测 FK，
并生成 `PoseStamped` ServoP 目标；运动学和关节命令仍由 motion server 负责。
管理器复用接收器的校验器，检查每个通道的目标 endpoint、FK topic、base_frame 和
tool_frame 与模型 channels.yaml 中的 ServoP 定义一致，并检查旋转矩阵和数值范围。
配置格式见 [`hc_teleop_recv/README.md`](../../hc_teleop_recv/README.md)。

`teleop_config` 不再是有效的模型资源；遥操作统一使用 `hc_teleop_config` 和
`hc_teleop_recv`。`resolve` 返回配置的绝对路径，启动器根据资源键启动接收端。
切换机型需要停止旧进程，再按新的 robot_id 启动；部署不会自动热切换正在运行的接收器。

## robot_composition 组合清单

组合清单只完成 ID 绑定：

```yaml
schema_version: 1
artifact_type: robot_composition
robot_id: my_robot
name: My complete robot
plugins:
  hardware_driver: my_robot_driver
  robot_model: my_robot_model
  gripper_driver: my_gripper_driver  # 可选
```

部署清单时，manager 会确认组件均已部署，并分别检查手臂关节契约以及夹爪运行时与
`hc_teleop_recv` 的具名话题契约。`resolve my_robot` 返回两个运行时各自的 `.so`、插件 XML、
参数文件以及模型和 motion 资源；两类动态库环境不会互相合并。

## 打包、部署、启动

开发机：

```bash
ros2 run humanoid_manager humanoid_pluginctl.py pack STAGED_DIR output.zip
ros2 run humanoid_manager humanoid_pluginctl.py validate output.zip
```

目标机先部署机械臂驱动、模型和可选夹爪插件：

```bash
ros2 run humanoid_manager humanoid_pluginctl.py deploy driver.zip
ros2 run humanoid_manager humanoid_pluginctl.py deploy model.zip
ros2 run humanoid_manager humanoid_pluginctl.py deploy gripper.zip
```

随后在网页“机器人配置”中选择机械臂驱动、模型和可选夹爪插件，填写机器人 ID 和名称。
管理器会创建独立配置副本，并在保存、应用时生成和部署内部组合清单；用户不需要生成、编辑
或导入 composition ZIP。CLI 输出 JSON，部署前需要停止正在使用目标插件的进程。

## OpenArmX 参考产物

开发机完成构建并 source 工作区后：

```bash
python3 src/openarmx_driver/tools/create_deployment_bundle.py \
  "$(ros2 pkg prefix openarmx_driver)" deploy_artifacts/openarmx-driver.zip

python3 src/openarmx_driver/tools/create_model_bundle.py \
  deploy_artifacts/openarmx-v10-model.zip

python3 src/humanoid_gripper/tools/create_deployment_bundle.py \
  "$(ros2 pkg prefix humanoid_gripper)" deploy_artifacts/openarmx-gripper.zip \
  --config openarmx_v10_bimanual.yaml \
  --plugin-id openarmx_v10_bimanual_gripper

```

导入 OpenArmX 手臂驱动、模型和夹爪三个产物后，直接在机器人页面新建配置。目标机不安装
`openarmx_driver`、`openarmx_description` 或 `humanoid_gripper` 源码包。
