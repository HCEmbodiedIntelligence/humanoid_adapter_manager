# 无执行层的真实相机录制

机器人本体不可用时，以 `mock_robot_feedback.py` 替代平台机械臂、夹爪执行层。
仍使用现有 `humanoid_manager` 网页的 ROS 原始 MCAP 录制和“回放与数据编辑”。
相机由原来的真实相机配置启动；脚本不生成图像，也不自行生成控制命令。
第一阶段不启动 VR、motion、真实 driver 或 gripper runtime。

```text
手动控制（以后替换为遥操作）
  ├─ /hc_teleop/joint_cmd ──────────→ 模拟反馈脚本 ─→ /hc_teleop/joint_states
  └─ /hc_teleop/gripper_commands ──→ 模拟反馈脚本 ─→ /hc_teleop/gripper_states
                控制、反馈 ────────────┐
真实相机 → 原来的图像话题 ─────────────┼→ 现有网页 MCAP 录制 → 回放与数据编辑
脚本 → /diagnostics（标明 simulation）─┘
```

## 接口和延迟

四个控制/反馈话题的类型都是 `sensor_msgs/msg/JointState`，与平台 runtime 相同。

| 部分 | 控制输入 | 反馈输出 | 关节名与默认频率 |
| --- | --- | --- | --- |
| 机械臂 | `/hc_teleop/joint_cmd` | `/hc_teleop/joint_states` | `openarmx_left_joint1`–`7`、`openarmx_right_joint1`–`7`；100 Hz |
| 夹爪 | `/hc_teleop/gripper_commands` | `/hc_teleop/gripper_states` | `left_gripper`、`right_gripper`；50 Hz |

初始臂关节为 0 rad，夹爪为 0.04 m。按关节名接收命令，可更新左臂、右臂或部分关节，
未指定关节保持原值。空命令、重复/未知关节、长度不匹配、NaN/Infinity 整条拒收，
不会部分应用。可选 velocity/effort 字段也检查长度和有限值；模拟只使用位置目标，反馈速度和力为 0。

“上一帧”指反馈控制周期：收到命令 C 后，第一次反馈 tick 仍发布旧状态；第二次 tick
发布 C。如果两个 tick 间收到多个命令，按到达顺序更新，后来的同关节目标覆盖先前目标。
停止发送后持续保持最后状态，不需要再发送下一条命令才能体现 C。
反馈 header 使用当前 ROS 时间，`frame_id=mock_robot_feedback` 标明模拟数据来源。

脚本只订阅控制话题，绝不向控制话题回发反馈。因此 MCAP 的 action 是外部实际发送的命令，
state 是延迟后的反馈。它不模拟动力学、限位、碰撞、watchdog 或真实硬件故障。
不需要 `/left_forward_position_controller/commands` 等厂商底层话题，也不需要 `hc_openarmx` 仓库。

## 启动

先退出占用这些反馈话题的真实 driver/gripper runtime，避免两个节点同时发布反馈。
在每个终端加载同一个工作区，并使用相同 domain。跨机器时也需要相同 domain 和可达的 DDS 网络。

```bash
cd /home/czy/teleop_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=199
```

首次加入脚本后执行一次 `colcon build --packages-select humanoid_manager --symlink-install`，再加载环境。
其他电脑使用它自己的 `teleop_ws` 路径。

推荐使用以下入口一次启动模拟反馈和真实相机，将机器人 ID 和插件目录替换为日常使用值：

```bash
ros2 launch humanoid_manager mock_robot_recording.launch.py \
  robot_id:=实际机器人ID \
  plugin_root:=/实际插件目录 \
  domain_id:=199
```

此入口与真实 managed launch 一样解析已应用的机器人配置，从同一目录读取 `cameras.yaml`，
原样交给 `humanoid_camera/multi_camera.launch.py`。没有另设分辨率、FPS、曝光、增益或序列号默认值。
原生相机驱动、时间戳适配器、D435 自动增益节点和配置中的自定义启动步骤仍由相机包处理。
启动日志显示相机配置文件路径、SHA-256 和已应用 revision，便于确认运行的版本。
相机配置为空或没有启用设备时启动失败，不会悄悄使用默认相机。
模拟节点的关节名、平台话题和臂频率也读取所选机器人配置。当前支持平台共享夹爪话题。

修改相机配置后，先停止这个入口，在网页保存并**应用机器人配置**，再重启。
仅保存草稿不会改变运行中的设备。此入口沿用真实启动的配置锁，防止运行时部署覆盖相机配置。
相机固件支持情况仍以原生驱动启动日志和实际参数回读为准；本机没有相机，未做硬件回读验收。

使用上面的联合入口后，直接运行下方“终端 3”的日常网页即可，不要再重复启动终端 1/2。
若相机已独立运行，可以改用下面的分开启动方式。

终端 1（分开启动方式）：运行替代执行层的脚本。

```bash
python3 src/humanoid_manager/scripts/mock_robot_feedback.py --domain-id 199
```

也支持 `ros2 run humanoid_manager mock_robot_feedback.py --domain-id 199`。
`--duration 30` 可自动退出；`--arm-hz`、`--gripper-hz` 可调整周期。
支持标准 ROS `--ros-args -r 原话题:=新话题` 重映射。

终端 2：只启动真实相机。使用机器人电脑已经部署、验证过的 `cameras.yaml` 绝对路径，
保留序列号、图像话题、分辨率和 FPS。将下面占位路径替换为实际文件。

```bash
ros2 launch humanoid_camera multi_camera.launch.py \
  camera_config:=/实际插件目录/robots/实际机器人ID/cameras.yaml
```

若相机已在运行则复用现有进程。不要启动带真实执行层的整机 launch。
真实相机必须实际连在运行相机节点的电脑上；空的 `cameras: []` 配置不会产生图像。

终端 3：打开原来的管理器网页，沿用日常录制配置。默认端口 7876。

```bash
./src/humanoid_manager/start_configurator.sh --domain-id 199
```

如日常入口指定了 `--state-root` 或 `--plugin-root`，沿用原值。
不要传 `--run-robot`，本步骤只启动网页、观察器和录制服务。

## 发送控制命令

终端 4：先用命令行测试左臂某个关节。右臂保持初始值。

```bash
ros2 topic pub --once /hc_teleop/joint_cmd sensor_msgs/msg/JointState \
  '{name: [openarmx_left_joint1], position: [0.2]}'

ros2 topic pub --once /hc_teleop/gripper_commands sensor_msgs/msg/JointState \
  '{name: [left_gripper, right_gripper], position: [0.02, 0.04]}'

ros2 topic echo /hc_teleop/joint_states
```

`rqt_publisher` 使用同样的话题、类型和字段即可。需要连续控制数据时把 `--once` 换成
`--rate 100`（机械臂）或 `--rate 50`（夹爪）；之后可修改目标观察录制曲线。
命令本身的 header 是否填写由命令发布者决定，脚本不会篡改录下的控制消息。

## MCAP 录制与回放

在网页“ROS 原始录制”中选择四个控制/反馈话题、`/diagnostics` 和现有真实相机图像话题。
若这台机器仍沿用三相机命名，它们是：

- `/camera_left/camera/color/image_raw`
- `/camera_right/camera/color/image_raw`
- `/camera_hand/camera/color/image_raw`

以实际 ROS graph 为准。保持原来的相机类型 `sensor_msgs/msg/Image`，最大录制频率 0 表示不降频。
保存并应用配置，再开始录制。未启动 VR 时关闭按钮标记，并取消录制 `/hc_teleop_recv/buttons`、
`/hc_teleop_recv/status` 等没有发布者的话题。未运行完整配置状态节点时也取消对应录制项。
新配置不会因为关闭着的按钮标记而自动勾选按钮录制；已有手动勾选不会被自动删除。

控制消息可能是按需发送的，低频或暂时未发送时会触发现有就绪提示。
可先连续发布测试命令，或者在确认反馈和相机正常后使用页面已有的“仍然录制”选项。
只有收到过消息的话题才会在 MCAP 中建立通道；不要把“没发命令”误判为录制器丢消息。

录制至少 5 秒，改变几次控制目标，先点击“停止录制”，再打开“回放与数据编辑”。
检查话题消息数、反馈/命令曲线、三路图像通道和“检查数据异常”结果。
MCAP 路径由页面录制目录决定，默认独立管理器配置位于
`~/.local/share/humanoid-manager`；以页面显示的实际路径为准。
回放是原有的浏览器只读预览，不会重新向 ROS 控制话题发送动作。
图像原始字节保存在 MCAP 中；当前回放预览对大图像保留元信息，不等于三路视频画面播放。

先停止录制，再退出模拟脚本和本次启动的相机、网页进程。

## 验证边界

`test_mock_feedback.py` 检查延迟、保持、独立关节更新和非法输入。
`test_mock_feedback_recording.py` 启动真实 ROS 节点与现有网页，通过 HTTP 录制 MCAP、
CRC/CDR 解码并通过 WebSocket 检查回放事件。测试用三路 640×480 合成图像验证数据传输，
不会调用相机硬件，不能替代真实 RealSense 验收。

本阶段不接 VR，也不实现新的 LeRobot 录制入口；先确认真实相机 MCAP 链路，再验证现有
对齐采集/LeRobot 导出所需的状态和动作字段映射。
