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

## 插件自身的启动依赖

`hardware_driver` 和 `gripper_driver` 的 `manifest.yaml` 均可声明可选的 `startup` 列表。
所选插件决定需要启动什么；通用 launch 不识别机器人型号、夹爪型号或具体控制器名称。
省略该字段或使用 `startup: []` 时行为与旧插件一致，适合插件内直接连接 SDK、CAN 或串口的设备。

例如，某夹爪通过 ros2_control 控制，可在其清单中声明：

```yaml
startup:
  - kind: node
    package: humanoid_manager
    executable: ensure_controllers.py
    arguments:
      - tool_controller
      - --controller-manager
      - /my_robot/controller_manager
      - --timeout
      - '60'
    wait_for_exit: true
```

`ensure_controllers.py` 是通用的 ros2_control 初始化工具：等待服务，只加载缺失的控制器，
配置 `unconfigured` 控制器，激活 `inactive` 控制器，并确认最终状态。
已经 `active` 的控制器直接复用，不会因重启 HC 重复配置失败，也不停止其他控制器。
控制器类型、关节和接口仍须由底层 controller_manager 的 YAML 提供。
只有使用该工具的插件才需要目标机安装 `controller_manager`；其他类型设备没有此依赖。
此工具初始化后退出；已激活控制器的生命周期归底层 controller_manager 管理。

其他厂商 ROS 驱动可声明常驻节点或 launch：

```yaml
startup:
  - kind: launch
    package: some_vendor_bringup
    launch_file: device.launch.py
    arguments:
      port: /dev/ttyUSB0
  - kind: node
    package: some_vendor_driver
    executable: wait_until_ready
    arguments: ['--timeout', '30']
    wait_for_exit: true
```

- `node` 在包的 `lib/<package>/` 中查找可执行文件，`arguments` 为字符串列表。
  默认常驻；设置 `wait_for_exit: true` 表示初始化步骤，成功退出后才继续下一步。
- `launch` 在包的 `share/<package>/launch/` 中查找文件，`arguments` 为非空字符串值的映射。
  使用默认值的参数可省略，避免 ROS CLI 拒绝空的 `name:=`。
  launch 作为被监管的子进程运行，接着执行下一步；需要等待设备就绪时由后续初始化步骤检查。
- 所有步骤先校验依赖，再按机械臂驱动、夹爪驱动的顺序执行。全部初始化成功后才启动
  HC 驱动运行时、运动服务等节点；初始化失败或常驻进程退出则结束本次机器人启动。
- 初始化程序须设置自身的等待超时。Ctrl+C / 网页关闭机器人会关闭本次启动的子进程；
  外部独立启动的厂商服务继续由原终端管理。底层初始化应支持重复执行。
- 进程继承所属插件的 ament 和库路径；包可以随插件提供，也可预装在目标机。
  导入 ZIP 时校验字段与字面参数，启动前检查包和程序是否存在，不执行 shell 字符串。
- `start_driver:=false` / `start_gripper:=false` 同时跳过对应插件的启动步骤。
  保存、复制、导出配置会保留清单；切换或移除夹爪插件会使用新插件的清单。

不要把同一个厂商服务同时配置在 `startup` 和外部 `vendor_*` 启动项中。
`startup` 随驱动插件发布；修改设备的命名空间等启动参数时，更新插件配置并重新打包导入。
在网页中选择更新后的源插件、保存并重启，已有运行配置不会因导入 ZIP 自动变化。
旧版管理器不认识该字段，导入新插件前需先更新管理器。

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

## 多设备实例与厂商接口

组合现已支持多个独立夹爪插件实例、可编辑的启动步骤和共享实例变量。
参数规则及开合测试能力由插件声明。详见 [设备插件、虚接口与实例](device_instances.md)。
