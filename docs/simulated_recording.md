# 真实相机与模拟机器人数据录制

此文档是 `/recording_test/*` 自动波形演示。若需要接收真实平台控制命令、
替代故障机器人的执行层，请使用 [无执行层的真实相机录制](mock_robot_recording.md)。

在机器人电脑上保留真实相机，机器人关节和夹爪数据由脚本模拟。
先退出之前带 `vendor_*` 参数的整机 launch，再分别运行相机、网页和模拟发布器。
每个终端先加载工作区，并设置同一个 ROS 域：

```bash
cd /home/hc_op/workspace/teleop_ws
source install/setup.bash
export ROS_DOMAIN_ID=0
```

终端 1：加载此前已部署的真实相机配置。这条启动入口不启动机械臂、夹爪硬件或 CAN。

```bash
ros2 launch humanoid_camera multi_camera.launch.py \
  camera_config:="$PWD/deployed_plugins/robots/openarmx_v10_bimanual/cameras.yaml"
```

终端 2：启动日常网页，使用已有插件目录和相机/录制设置。

```bash
./src/humanoid_manager/start_configurator.sh \
  --plugin-root "$PWD/deployed_plugins" --domain-id 0
```

终端 3：只发布模拟机器人数据。

```bash
python3 src/humanoid_manager/scripts/simulate_recording.py --domain-id 0
```

打开 **http://127.0.0.1:7876/dashboard/#topics**。在“ROS 原始录制”中勾选当前真实相机话题，
以及下表中的 `/recording_test/` 话题；取消本次没有发布器的真实机器人话题。
保存、应用后，等所选话题频率正常，再开始录制。相机话题名沿用之前测试通过的配置。
若已经单独启动相机或只启动网页，可直接复用对应进程。

按钮自动标记需要把“按钮监听与数据标记”的话题设为 `/recording_test/buttons`，
选择右手次按钮（B），按需启用。不要在本次测试期间启动原来的整机控制 launch。
模拟消息只发往 `/recording_test/`，相机图像由真实相机节点发布。

如测试“对齐采集”，在现有采集配置中保留真实相机来源，
把状态来源改为 `/recording_test/joint_states`、动作来源改为 `/recording_test/joint_commands`，
采用 `ros_joint_state` 传输；实际 RGB-D 同步与导出仍需在这台相机电脑上验收。

## 仅模拟数据的独立页面

在已安装 ROS 2 和管理器网页依赖的工作区运行：

```bash
cd /home/hc_op/workspace/teleop_ws
source install/setup.bash
python3 src/humanoid_manager/scripts/simulate_recording.py --web --domain-id 0
```

打开 **http://127.0.0.1:7877/dashboard/#topics**，等待话题频率正常，点击“开始录制”，
录制约 15 秒后点击“停止录制”。到“回放与数据编辑”打开生成的 MCAP，检查消息内容、
曲线和按钮标记，再点击“检查数据异常”。测试页面默认只监听本机。

脚本启动模拟发布器和独立网页，使用 `/recording_test/` 话题，不启动机器人、CAN、
夹爪硬件、相机或遥操作。网页已勾选全部测试话题，按原始频率录制。
`--domain-id` 同时设置发布器和网页子进程的 ROS 域，覆盖继承的 `ROS_DOMAIN_ID`；
单独启动的相机和日常网页仍需使用相同的域。
每次默认创建新的持久目录 `~/.local/share/humanoid-manager-recording-demos/demo-*`；
终端会显示实际录制目录。停止脚本后 MCAP 文件仍保留，可以导入日常管理页面查看。
`--state-root /绝对路径/新目录` 可指定保存位置，已有配置文件不会被覆盖。

| 话题 | 类型 | 内容与频率 |
| --- | --- | --- |
| `/recording_test/joint_states` | `sensor_msgs/msg/JointState` | 双臂 14 关节，100 Hz |
| `/recording_test/joint_commands` | `sensor_msgs/msg/JointState` | 模拟目标，100 Hz |
| `/recording_test/gripper_states` | `sensor_msgs/msg/JointState` | 左右夹爪位置，20 Hz |
| `/recording_test/gripper_commands` | `sensor_msgs/msg/JointState` | 模拟开合目标，20 Hz |
| `/recording_test/buttons` | `std_msgs/msg/String` | HC 手柄格式 JSON，20 Hz |

关节使用周期变化的模拟值；夹爪范围为 0–0.044 m。反馈相对目标具有固定模拟延迟。
JointState 含当前 ROS Header 时间戳，frame_id 为 `synthetic_recording_test`；
按钮 JSON 带 `synthetic: true`，不发布真实机器人的配置身份或就绪状态。
右手 B 键每 5 秒按下一次，测试网页已启用“片段”按钮标记：一次按下开始无效片段，
下一次按下结束。默认导出会排除这些无效片段，完整原始消息仍保留在最初的 MCAP 中。

测试期间保持脚本运行；先在网页停止录制，再按 Ctrl+C 退出网页和发布器。
本脚本测试 ROS 原始 MCAP 数据链路；相机图像、RGB-D 对齐和 LeRobot 导出应单独验收。

## 只发布话题 / 中断测试

已有网页时省略 `--web`，网页 ROS 域同样设为 0，勾选上表话题并保存、应用：

```bash
python3 src/humanoid_manager/scripts/simulate_recording.py --domain-id 0
```

要自动测试话题中断和恢复，在启动后 30 秒暂停全部模拟话题 5 秒：

```bash
python3 src/humanoid_manager/scripts/simulate_recording.py \
  --web --domain-id 0 --pause-after 30 --pause-for 5
```

在暂停前开始录制，观察话题中断提示，恢复发布后停止录制。
“检查数据异常”应报告录制时检测到的话题中断。`--duration 60` 可限制总运行时间，
`--hz 200` 可调整关节发布频率，`--port 7878` 可更换测试网页端口。

## 软件验证

`test/test_simulated_recording.py` 在隔离 ROS 域、本机临时端口启动真实发布器和网页，
通过 HTTP 开始/停止录制。验证 MCAP CRC、ROS CDR 解码、五个话题的变化数据、
Header、按钮边沿及片段标记，并暂停发布 4 秒验证中断和恢复。
这些结果用于验证软件链路，不代表真机通信或硬件动作已通过测试。
