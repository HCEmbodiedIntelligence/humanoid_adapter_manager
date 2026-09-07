# v1.2 ROS 采集接口

相机和录制器是两个独立进程。`humanoid_camera` 启动官方 RealSense ROS 驱动，并转换曝光中点；`humanoid_manager` 不操作 SDK，不在采集回调中设置相机参数。相机不做曝光/增益回读或逐帧曝光验收。

## ROS 来源

| transport | 输入 | 元数据 |
|---|---|---|
| ros_joint_state | sensor_msgs/JointState | 直接保留 Header，配置读数/发送/生效语义 |
| ros_rgbd | realsense2_camera_msgs/RGBD | Metadata 话题，含 pair_seq 与各路真实原始帧号 |
| ros_pointcloud | sensor_msgs/PointCloud2 | Metadata 话题，含源 RGB/depth 帧号与 pair_seq |
| envelope | HTTP 标准化样本 | 同一 JSON 输入契约 |
| ros_images | 标准 Image 两路 | 仅用于直接读取原始驱动 Header 的诊断关联 |

默认订阅 `/front/normalized/rgbd`、`/front/normalized/metadata`；点云订阅 `/front/normalized/points`、`/front/normalized/points_metadata`。图像/metadata 按 Header 关联，ROS 输入组装缓存同时限制条目与字节；完整消息随后进入录制队列。相机转接时用的是 SDK GLOBAL_TIME 和各路曝光中点，细节见相机包的实施说明。

关节直接使用原有 ROS Header，不注册额外机器人时钟映射。实际命令来源必须声明 action_stage=sent，以及 Header 是 send 还是 effective。状态和动作的单位、坐标系、字段名称保留在配置与原始记录中。

输入元数据最少包含：

```text
input_contract = humanoid-ros-capture/1.2
source_id, source_seq, pair_seq（图像/点云）
capture_clock_id = ros
capture_time_ns = header_timestamp_ns = 对应消息 Header
source_timestamp_ns, clock_id, clock_epoch, clock_model_id, timestamp_quality
receive_time_ns, receive_clock_id, receive_steady_ns
```

两路图像分别保留 capture_time_ns。点云的 capture_time_ns 等于 source_depth_seq 对应深度的时间；不按相近到达时间重新挑选来源。原始 metadata 和映射锚点直接保存，不检查曝光/增益是否超限。

## 对齐与保存

数据匹配使用 ROS 时间；deadline 使用本机单调时钟。ROS 时间跳变会结束当前录制时间段并报告原因，重新开始会建立新 session/epoch，不能跨跳变插值。时钟质量未知不填零。

当前配置的 `camera_validation=timestamps` 表示按输入采样时间做关联与时间差检查，不进行曝光认证；默认来源仍是已经转换曝光中点的 ros_rgbd。`strict_qualified` 保留为物理严格认证状态，当前模式不会将其置 true；`export_qualified` 表示当前配置下的数据完整性与对齐是否合格。导出另检查所有必需图像/深度/点云的提交状态及人工无效标记。

原始 RGB 保存到 MP4，ROS 驱动输出的深度数值、数值采样和点云无损保存到 MCAP。点云使用 `humanoid-pointcloud` 消息编码：4 字节小端 JSON 长度、JSON 布局与来源、原始 PointCloud2.data。深度使用官方 ROS 驱动发布后的数值及 scale（默认 0.001 m），不是把深度着色成视频。

LeRobot 0.4.4 / v3 导出保持训练 RGB 帧数与数据行一致。精确深度和点云为伴随 MCAP，可用 `DepthReader` / `PointCloudReader` 按 episode/frame 读取；不冒充 LeRobot 通用 RGB 视频字段。

## 网页 API

| 请求 | 用途 |
|---|---|
| GET /api/capture | 配置、etag、来源状态、录制及后台任务 |
| POST /api/capture/config | 保存配置 |
| POST /api/capture/prepare | 连接 ROS 输入，不配置相机硬件 |
| POST /api/capture/start / stop | 开始连续采集 / 完成最后 deadline 并关闭文件 |
| POST /api/capture/sample | 标准化 envelope 样本，容量不足返回 429 |
| POST /api/capture/mark | 无效片段、帧或整段标记 |
| GET /api/sessions | 连续录制列表 |
| GET /api/sessions/{id}/row | 目标帧、来源、对齐与持久化状态 |
| POST /api/sessions/{id}/edits | 划分 episode、编辑无效区间 |
| POST /api/sessions/{id}/realign | 用原始数据生成新的对齐版本 |
| POST /api/sessions/{id}/export | 按标注区间导出 |
| GET /api/sessions/{id}/preview | 提交后的 RGB 帧预览 |

完整采集配置见 `config/alignment-strategy.example.json`。历史 v1.1 的显式 host_monotonic 输入兼容逻辑仅用于旧数据和回归测试，当前相机路径不走进程内 SDK 插件。
