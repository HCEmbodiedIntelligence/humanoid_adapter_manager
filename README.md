# humanoid_manager

机器人配置、运行状态与数据管理的独立网页入口。原包 `humanoid_adapter_manager` 已改名为 `humanoid_manager`，源码目录和 Python 导入名也已同步更新。

本程序配合 `hc_teleop_recv` 使用，不需要启动 `HC-teleop-robotic` 或 `teleop_vr_recv`。驱动和运动计算仍由已有的 `humanoid_driver_runtime`、`humanoid_motion_server` 执行。

## 启动网页

```bash
cd /home/czy/teleop_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select hc_teleop_recv humanoid_manager robot_bringup --symlink-install
./src/humanoid_manager/start_configurator.sh
```

打开 **http://127.0.0.1:7876/dashboard/**。首次启动会通过 `setup_web.sh` 安装网页依赖到包内 `.web-venv`，不修改系统 Python。已有解释器可以通过 `HUMANOID_WEB_PYTHON=/path/to/python` 指定。没有 `uv` 时安装脚本使用系统 `venv` 与 `pip`。

默认网页状态存放于 `~/.local/share/humanoid-manager`。插件目录优先使用已有 `/var/lib/humanoid-plugins`，否则使用 `~/.local/share/humanoid-plugins`。ROS Domain ID 默认为 14。

可以分别指定部署资源、网页状态和 ROS 域：

```bash
./src/humanoid_manager/start_configurator.sh \
  --plugin-root /absolute/path/to/plugins \
  --state-root /absolute/path/to/manager-state \
  --domain-id 14 --host 127.0.0.1 --port 7876
```

`--offline` 可在没有 ROS 连接时编辑、校验和保存配置。网页应用部署要求取得新鲜 ROS 状态，以确认机器人节点已经停止。

## 网页操作

- **机器人配置**：导入驱动、模型、组合插件或完整配置包；新建、复制机器人；编辑驱动参数、关节映射、URDF、关节分组、运动通道、工具、初始姿态、接收端与录制方案。包括独立底盘页与多个夹爪配置，默认不启用硬件接口。没有接收端资源时，可在“机械臂遥操作”页根据运动通道添加 `hc_teleop_recv` 配置。
- **遥操作初始姿态**：在“初始姿态”页为一个或多个 MoveJ 通道设置关节角、速度、加速度、加加速度和超时。一个姿态可以同时包含双臂。姿态必须通过模型限位校验并保存、应用到当前运行版本后才能点击执行；网页执行前再次确认，接收端仍处于遥操作使能时拒绝发送。运动完成以 `humanoid_motion_server` 的真实关节反馈结果为准。
- **草稿与版本**：保存草稿、关联校验、保存版本、比较修改、恢复历史、导入导出。每个机器人使用独立驱动/模型参数副本。多个窗口同时修改会提示冲突。
- **运行状态**：显示运行中的机器人、配置版本、驱动诊断、接收端状态、关节反馈与指令。超时状态显示过期；ROS 节点存在与驱动已连接分别显示。
- **对齐采集**：按 v1.2 使用共同 ROS 时间、固定 FPS 网格、原始 RGB MP4、无损深度/点云及数值 MCAP，支持手动 episode、回放标记、离线重对齐和 LeRobot v3 导出。相机由独立 `humanoid_camera` 包启动官方驱动并转换曝光中点。详见 [采集接口](docs/capture_interfaces.md)。
- **机器人相机方案**：每个机器人版本可配置任意数量的 D405、D435、其他 RealSense 或已标准化 ROS 话题相机；保存序列号、命名空间、分辨率、帧率、点云与录制必需性，并可同步到连续采集策略。
- **ROS 原始录制**：发现话题，选择录制类型、限频、保存目录与文件名，显示实际接收频率、写入数量、丢弃数量和文件大小。按钮话题自动加入录制且不限频。保存配置不会重启录制；应用配置要求录制/回放停止。
- **回放与数据编辑**：MCAP 时间轴、播放/暂停/跳转、0.1–4 倍速、关节曲线、位姿/按钮/事件预览、时间裁剪、有效/无效标记、备注、另存 MCAP、文件导入下载删除。
- **数据异常处理**：记录话题中断、频率不足、队列丢弃、写入错误；扫描 CRC、解码失败、NaN/Inf、时间戳倒退与指定最大间隔；异常中断文件可生成修复副本。原始文件保持不变。

录制方案分为“机器人中已保存的方案”和“网页当前录制配置”：载入机器人方案到表单，再保存并应用后用于下一次录制。

## 启动已部署机器人

网页中的“应用”更新部署文件。停止原启动进程后应用，再使用新入口启动：

```bash
source /home/czy/teleop_ws/install/setup.bash
ROS_DOMAIN_ID=14 ros2 launch humanoid_manager managed_robot.launch.py \
  plugin_root:=$HOME/.local/share/humanoid-plugins \
  robot_id:=你的机器人ID
```

此入口启动驱动、运动服务和 `hc_teleop_recv`，持有部署读锁并发布 `/humanoid/configuration_state`。其报告的配置指纹/版本会写入录制元数据。可以通过 `start_driver:=false`、`start_motion:=false` 或 `start_teleop:=false` 分别关闭组件。启用遥操作时必须有 `hc_teleop_config`，不会启动旧接收端。

尚未导入插件时，在网页“导入配置包”中导入驱动和模型 ZIP，然后在“机器人配置”中选择两者创建机器人。机器人组合由管理器内部生成，不需要准备或编辑 composition ZIP。CLI 仍可用于导入底层插件：

```bash
ros2 run humanoid_manager humanoid_pluginctl.py --root "$HOME/.local/share/humanoid-plugins" deploy driver.zip
ros2 run humanoid_manager humanoid_pluginctl.py --root "$HOME/.local/share/humanoid-plugins" deploy model.zip
```

## 按钮标记与数据保存

接收端发布 `/hc_teleop_recv/buttons`（`std_msgs/msg/String` JSON），包含手柄按住/按下/释放掩码、模拟量、序号、时间戳和去重后的按下/释放事件。

在“ROS 原始录制 → 按钮监听与数据标记”中选择手柄、按钮与范围，启用自动标记，保存并应用。默认监听右手次按钮（B），自动标记默认关闭；不启用标记也会录制按钮数据，便于后续处理。对齐采集同样使用此按钮配置，标记保存在对应 session；其点标记匹配一个目标周期内的最近目标行。

| 范围 | 行为 |
| --- | --- |
| 当前片段 | 第一次按下开始无效片段，第二次按下结束；录制结束时未闭合的片段延续到文件末尾 |
| 整次录制 | 标记当前整份录制无效 |
| 当前时间点 | 标记按钮前后 0.5 秒内最近的已录制采样时刻；导出排除该时刻的消息，附近无采样则保留为点备注 |

网页也提供手动标记按钮。原始按钮事件、在线标记和录制异常写入 MCAP。离线编辑使用配套 `.mcap.edit.json`；另存导出将编辑信息和来源写入 MCAP 元数据。详见 [数据管理说明](docs/data_management.md)。

## 验证

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH="src/humanoid_manager/python:src/hc_teleop_recv:$PYTHONPATH" \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  src/humanoid_manager/.web-venv/bin/python -m pytest -q \
  src/humanoid_manager/test src/hc_teleop_recv/test
```

测试环境需要安装 `pytest`。没有网页依赖的系统 Python 会跳过网页/MCAP 测试；完整验证应使用安装了 `requirements-web.txt` 的解释器。已覆盖配置回滚与锁、编辑冲突、录制不中断、裁剪导出、异常扫描、文件修复、回放控制和真实 ROS 按钮/配置服务；Mock 集成验证不连接物理机器人。

插件包结构与校验规则见 [部署说明](docs/deploying_plugins.md)。
