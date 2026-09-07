> 历史 v1.1 实现说明。当前设备路径和用户确认的简化相机流程请以 [v1.2 接口](capture_interfaces.md) 为准；当前没有曝光/增益回读、预热门控或逐帧曝光验收。

# 连续数据采集与时间对齐 v1.1

实现位于 `humanoid_manager/recording`，网页入口为 **对齐采集**。与机器人型号无关，不依赖旧遥操作应用。采集目录不是已经完成的 LeRobot 数据集；先连续采集，再标注 episode，最后导出。

## 使用顺序

1. 安装更新后的 `requirements-web.txt`（`./setup_web.sh`），启动网页。
2. 在“对齐采集”添加状态、动作、RGB-D 来源；配置 FPS、时间窗口及保存目录。保存策略。
3. SDK 相机点击“连接并配置相机”；外部驱动通过 [接口说明](capture_interfaces.md) 注册时钟与能力报告。严格检查会列出缺失证据。
4. 开始连续采集。必需来源到齐后，以主相机首组合格 RGB 曝光中点设 T0。无相机则使用 session 单调时钟。预热数据也保存。
5. 可手动停止，也可设置“自动结束时长”3600 秒。目标范围为 `[T0,T_end)`，收尾期间等待最后 deadline，保存期间不重置场景。
6. 选择采集记录，查看目标帧、原始 RGB、逐字段时间误差及文件提交状态。添加 episode 的 `[起始帧,结束帧)` 与任务说明，保存标注后导出。

底盘和夹爪的网页配置位于“机器人配置”的对应页签。夹爪硬件插件、逻辑名、厂商话题、单位、方向和限位保存在 `gripper_params`；使能按键、输入轴、开合位置和速度等遥操作映射保存在 `hc_teleop_config`。接口默认关闭，可配置单夹爪、双夹爪或更多独立夹爪。实际驱动接通后才产生对应状态；网页保存不会启动机器人。

## 已实现的规则

| 范围 | 行为 |
| --- | --- |
| 时间网格 | 整数纳秒直接计算 `T0 + round(k*1e9/F)`；30 FPS 一小时目标 108000 行；无效行不改变编号 |
| 原始数值 | 新采样立即投递，不使用 dataset.fps 限流；序号与丢失/拒绝计数保留；NaN/Inf 保留原 IEEE 浮点位模式并标记无效 |
| 时钟 | `Clocks` 保存有版本和有效区间的偏移/漂移模型；未知不确定度为 null；禁止修改已注册版本；禁止跨 clock_epoch 或轨迹断点插值 |
| RGB-D | 整组不可变采样；RGB、深度独立曝光元数据；曝光开始/结束正确转换成中点；设备 frame_number 不能冒充已验证共同周期 |
| 严格限值 | 两路实际曝光加误差界分别 ≤5000 μs；RGB-D 相对偏差、每路图像到网格、必需图像之间偏差分别校验；三个 1 ms 均注明暂定工程目标 |
| 图像匹配 | 共同周期映射优先（配置 trigger_origin，或驱动提供已验证网格映射）；否则完整组最近时间，距离相同选早组、再选 pair_seq；按 deadline 前到达筛选；支持因果模式 |
| 去重 | 原始 RGB/深度 source_seq 分别去重，不重复编码 latest；相邻有效行默认禁止复用；失败原因保留 |
| 状态 | 连续量线性插值、恰好命中直接取值、30 ms 默认最大缺口、禁止外推；离散量前值保持和最大年龄；角度需已知速度界避免未知圈数；四元数最短路径 SLERP |
| 动作 | 来自实际发送链路的保持型绝对命令；effective_time 优先，否则 send_time 并标明近似；不能提前使用未来命令；失效/模式/协议超时参与质量判断 |
| 原始保存 | 单独 MCAP writer；每相机独立编码器；RGB MP4 默认60秒轮转；深度按原 dtype 字节无损写入 ZSTD MCAP；编码器实际回报 PTS/显示帧号 |
| 持久化 | 对齐与提交是两个状态；RGB 容器关闭及配置的 fsync 完成才提交；原始 MCAP 正常 finish/close 后提交；入队不是落盘 |
| 过载 | 队列同时限制条目数和字节；投递不等容量；饱和明确计数/记录缺口；数值或深度无法持久化、磁盘故障结束失败 session；不发送机器人指令 |
| 后台落后 | 在线追赶行数有上限；写出未生成目标范围；离线从原始索引生成独立新版本，不改在线结果或原始数据 |
| 标记 | 保存全部收到的按钮消息；自动标记使用现有“按钮监听与数据标记”配置；无效片段、整次录制、点标记进入 session MCAP；未闭合片段在结束时关闭 |
| 导出 | 检查整个 episode，拒绝必需字段无效、原始缺失、未提交或人工无效的区间；顺序解码选帧并重新编码为 dataset.fps；每路视频帧数与表格行数核对 |

`active_at_target` 的动作定义保存在导出元数据中。增量动作、cycle_linked、需要抗混叠滤波的专用字段在没有转换器时**拒绝配置**，不按保持型/普通线性插值冒充实现。

角度规则设置 `semantics: angle`、`period`（默认 2π）、`max_speed`（单位/秒）；只有相邻样本时间内可能位移小于半圈且符合速度界才解包插值。四元数按 XYZW/调用方字段声明保存，输出归一化。

## 文件与证据

```text
sessions/<session_id>/
  session.json                配置快照、哈希、版本、能力回读、时钟模型、主机 boot ID
  manifest.json               状态、T0/T_end、已关闭/提交的文件
  raw.mcap                    原始数值、无损深度、在线对齐、帧索引、按钮和质量事件
  index.sqlite3               磁盘查询索引；避免加载整小时数据
  videos/<camera>/<segment>.mp4
  quality.json                计数、分布、最大值、内存/队列峰值
  edits.json                  手动 episode/无效范围/备注，带编辑冲突检查
  alignments/offline_<id>/     离线重新关联的独立 MCAP、索引、版本及提交说明
```

MCAP `log_time` 是写入墙钟时间；有真实发布时间才设置相应 `publish_time`，否则以记录时间作为缺省发布时间。统一单调采样时间显式保存在 payload，绝不为了对齐修改原始采样时间。原始 MP4 的 PTS 按相机声明频率和新帧到达顺序组织，真实曝光时间在逐帧索引中；PTS 不作为曝光同步证据。

原始深度消息编码为 `humanoid-depth`：4 字节小端 JSON 长度、UTF-8 元数据、C 顺序原始像素字节。元数据保存 dtype 的字节序、shape、depth_scale、invalid_value、标定与配对信息。完整 MCAP 使用无损 ZSTD 分块压缩。

队列总预算核算包含入口、数值/深度 writer、每相机 RGB 编码队列、元数据缓存与复制工作区。复制工作区串行非阻塞取得，忙时有明确拒绝计数。SDK 自有缓冲、编码器内部内存、Python/Web 运行时另有开销；质量报告记录整个进程的 RSS 峰值。必须在实际分辨率和相机数量下验证总内存、深度吞吐与控制频率。

分位数使用最多4096条的有界蓄水池估计，**最大值与计数是精确统计**。逐行、逐帧记录仍保存全部证据，不能仅凭 p99 忽略曝光上限违规。强制退出时活动文件保持未提交；重新启动会将该 session 显示为 interrupted。已提交 RGB 块可下载，不承诺未关闭 MP4 或 MCAP 自动恢复。

## LeRobot 与深度

格式固定为 **LeRobot 0.4.4 / v3.0**，MCAP 依赖 **1.4.0**，schema `humanoid-session/1.1.0`，对齐实现 `humanoid-alignment/1.1.0`。输出包括 v3 Parquet、RGB MP4、episode 视频/数据偏移元数据、任务表、统计及逐行 `source_mapping.jsonl`。

默认导出逐帧重编码，支持 episode 跨多个原始 MP4 块。当前没有启用免重编码复用路径，不承诺任意裁剪或重采样可以复用原视频。

精确深度使用附加 `depth.mcap` 和 `depth_index.sqlite3`，由专用读取器返回原 dtype、数值及元数据；没有把深度伪装成 LeRobot 普通 RGB 视频：

```python
from humanoid_manager.recording.exporter import DepthReader
metadata, depth = DepthReader('/path/to/export').get('front', episode_index=0, frame_index=0)
distance = depth * metadata['depth']['depth_scale']
```

训练端需要使用此适配器合并深度；原生 RGB 读取器不会自动读取该扩展。

## 已执行的软件验证

- `pytest`：102项通过，覆盖管理器、对齐保存/导出、网页生命周期、接收端及底盘/夹爪 ROS Mock 消息。
- 加速模拟3600秒数值时间轴：360000个100 Hz样本、108000行；插值最大误差约9.1e-13；时间缓存峰值20113字节，结束保留24个样本。这只验证数值逻辑与缓存，不是相机/磁盘/控制链路的一小时验收。
- LeRobot 0.4.4原生读取器实际读回两个episode的5行数据及各段首尾RGB画面、状态、动作和时间戳；深度另由精确扩展读取器核对。
- 桌面1440px、手机390px页面无横向溢出和脚本错误；无H.264支持的浏览器通过原始PTS索引的JPEG预览显示目标帧。

可复现命令：

```bash
# manager依赖环境：只加速测试数值对齐，不启动硬件
.web-venv/bin/python scripts/benchmark_alignment.py --seconds 3600
# 已安装 LeRobot 0.4.4 的独立训练环境：检查导出首尾
/path/to/lerobot/python scripts/validate_session_export.py /path/to/export
```

详细验证记录：[alignment_validation.json](alignment_validation.json)。

## 设备验收边界

仓库测试使用明确标识的合成时钟/能力报告，验证软件规则、RGB 编码、深度无损读回、跨块导出和 HTTP 操作。没有把这些合成证据写成真实设备能力。

当前新增的是 SDK 插件接口与完整元数据消息接口，仓库未包含针对某一型号已经验证过的严格 RGB-D SDK 驱动。普通 ROS JointState 输入可用于诊断，保留 ROS 原始 header，但只有主机接收时间近似，严格模式拒绝。动作话题是否为限幅后的实际发送值必须由驱动确认，不能直接把观测关节当作动作。

硬件接入后仍必须执行：亮/暗场曝光与深度有效率、实际 RGB-D 中点差、共同事件/曝光有效信号验证、重连/改 FPS 后回读、温漂/时钟漂移、1小时吞吐/内存及录制开关前后的控制时延基线。未知误差界、未经验证的滚动快门/复合测量时间模型、跨帧融合，均不能宣称严格合格。

规范参考：[MCAP 消息语义](https://mcap.dev/spec#message-op0x05)、[LeRobot v3 格式](https://huggingface.co/docs/lerobot/main/en/lerobot-dataset-v3#format-design)、[固定版本格式定义](https://github.com/huggingface/lerobot/blob/v0.4.4/src/lerobot/datasets/utils.py)、[固定版本读取器](https://github.com/huggingface/lerobot/blob/v0.4.4/src/lerobot/datasets/lerobot_dataset.py)。
