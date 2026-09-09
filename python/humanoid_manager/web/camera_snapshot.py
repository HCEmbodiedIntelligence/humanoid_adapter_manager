"""One-shot, read-only camera preview. No camera SDK, recording or motion commands."""
import base64
import io
import time
import uuid

from ..deployment import DeploymentError
from ..plugin_metadata import resolved_document


class CameraSnapshotError(RuntimeError):
    pass


def resolve_camera_snapshot(document, camera_id):
    from humanoid_camera.configuration import validate_cameras
    cameras = validate_cameras(document.get('cameras', []))
    camera = next((item for item in cameras if item['id'] == camera_id), None)
    if camera is None:
        raise DeploymentError('已保存配置中没有这台相机')
    if not camera['enabled']:
        raise DeploymentError('这台相机已禁用，请启用并保存配置')
    camera = resolved_document(camera, camera)
    raw = camera['backend'] == 'realsense'
    return {'camera_id': camera_id, 'device_type': camera.get('device_type', camera['backend']),
            'serial_no': camera.get('serial_no', ''),
            'topic': camera['rgb_topic'] if raw else camera['rgbd_topic'],
            'message_type': 'image' if raw else 'rgbd'}


def encode_snapshot(message):
    """Decode common 8-bit color formats, honoring padded ROS image rows."""
    from PIL import Image
    formats = {'rgb8': ('RGB', 'RGB', 3), 'bgr8': ('RGB', 'BGR', 3),
               'rgba8': ('RGBA', 'RGBA', 4), 'bgra8': ('RGBA', 'BGRA', 4),
               'mono8': ('L', 'L', 1)}
    if message.encoding not in formats:
        raise CameraSnapshotError('拍照不支持图像编码 ' + message.encoding + '，请配置 RGB8 或 BGR8 彩色流')
    mode, raw_mode, channels = formats[message.encoding]
    width, height, step = message.width, message.height, message.step
    if (not 1 <= width <= 8192 or not 1 <= height <= 8192 or step < width * channels
            or step * height > 64 * 1024 * 1024 or len(message.data) != step * height):
        raise CameraSnapshotError('相机图像尺寸或数据长度无效')
    image = Image.frombytes(mode, (width, height), bytes(message.data), 'raw', raw_mode, step, 1)
    image = image.convert('RGB')
    image.thumbnail((1280, 960))
    output = io.BytesIO()
    image.save(output, format='JPEG', quality=90)
    return {'image_data_url': 'data:image/jpeg;base64,' + base64.b64encode(output.getvalue()).decode('ascii'),
            'width': width, 'height': height, 'preview_width': image.width, 'preview_height': image.height,
            'frame_id': message.header.frame_id,
            'stamp_ns': message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec}


def capture_camera_snapshot(camera, domain_id, timeout_sec=10.0):
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rclpy.qos_event import SubscriptionEventCallbacks, UnsupportedEventTypeError
        if camera['message_type'] == 'image':
            from sensor_msgs.msg import Image as Message
            message_type = 'sensor_msgs/msg/Image'
        else:
            from realsense2_camera_msgs.msg import RGBD as Message
            message_type = 'realsense2_camera_msgs/msg/RGBD'
    except ImportError as error:
        raise CameraSnapshotError('当前环境缺少相机 ROS 消息支持') from error
    context, node, executor = Context(), None, None
    received, rejected = [], []
    lost_messages = 0
    try:
        rclpy.init(args=[], context=context, domain_id=int(domain_id))
        node = rclpy.create_node('humanoid_camera_snapshot_' + uuid.uuid4().hex[:10], context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        def receive(message):
            frame = message if camera['message_type'] == 'image' else message.rgb
            stamp = frame.header.stamp.sec * 1_000_000_000 + frame.header.stamp.nanosec
            age = node.get_clock().now().nanoseconds - stamp
            if stamp <= 0 or not -1_000_000_000 <= age <= 2_000_000_000:
                rejected[:] = ['图像时间戳未初始化或已过期，请检查相机时钟']
                return
            received[:] = [frame]

        def message_lost(event):
            nonlocal lost_messages
            lost_messages += max(0, event.total_count_change)

        subscription, reliability = None, None
        endpoints, publishers = [], []
        next_discovery = 0.
        deadline = time.monotonic() + timeout_sec
        while not received and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_discovery:
                endpoints = node.get_publishers_info_by_topic(camera['topic'])
                publishers = [info for info in endpoints if info.topic_type == message_type]
                # A reliable writer also matches best-effort readers, but those
                # readers cannot recover a lost fragment of a large image. Use
                # retransmission when offered; preserve sensor-data compatibility.
                selected = (ReliabilityPolicy.RELIABLE if publishers and all(
                    info.qos_profile.reliability == ReliabilityPolicy.RELIABLE for info in publishers)
                    else ReliabilityPolicy.BEST_EFFORT)
                if subscription is None or selected != reliability:
                    if subscription is not None:
                        node.destroy_subscription(subscription)
                    qos = QoSProfile(depth=1, reliability=selected, durability=DurabilityPolicy.VOLATILE)
                    try:
                        subscription = node.create_subscription(Message, camera['topic'], receive, qos,
                            event_callbacks=SubscriptionEventCallbacks(message_lost=message_lost))
                    except UnsupportedEventTypeError:
                        subscription = node.create_subscription(Message, camera['topic'], receive, qos)
                    reliability = selected
                next_discovery = now + .25
            executor.spin_once(timeout_sec=min(.1, max(0., deadline - time.monotonic())))
        if not received:
            if rejected:
                reason = '已收到图像，但' + rejected[-1]
            elif lost_messages:
                reason = (f'未收到完整图像，ROS 报告丢失 {lost_messages} 条消息；'
                          '请检查 DDS 传输、缓冲区以及图像分辨率和帧率')
            elif publishers:
                reason = (f'已发现 {len(publishers)} 个图像发布端，但 {timeout_sec:g} 秒内未收到图像；'
                          '请检查相机驱动是否持续出帧及 DDS 传输')
            elif endpoints:
                types = ', '.join(sorted({info.topic_type for info in endpoints}))
                reason = f'话题消息类型为 {types}，拍照需要 {message_type}，请检查配置的话题'
            else:
                reason = '未发现图像发布端，请启动对应相机并检查话题及 ROS domain_id'
            qos_name = reliability.name if reliability is not None else '尚未订阅'
            raise CameraSnapshotError(f"{camera['camera_id']}: {reason}"
                                      f"（{camera['topic']}；domain_id={domain_id}；QoS={qos_name}）")
        return {**camera, **encode_snapshot(received[0]), 'received_at': time.time()}
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
