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


def capture_camera_snapshot(camera, domain_id, timeout_sec=5.0):
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        if camera['message_type'] == 'image':
            from sensor_msgs.msg import Image as Message
        else:
            from realsense2_camera_msgs.msg import RGBD as Message
    except ImportError as error:
        raise CameraSnapshotError('当前环境缺少相机 ROS 消息支持') from error
    context, node, executor = Context(), None, None
    received, rejected = [], []
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

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        node.create_subscription(Message, camera['topic'], receive, qos)
        deadline = time.monotonic() + timeout_sec
        while not received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=min(.1, max(0., deadline - time.monotonic())))
        if not received:
            reason = rejected[-1] if rejected else '未收到图像，请启动对应相机并检查序列号、话题和 ROS domain_id'
            raise CameraSnapshotError(f"{camera['camera_id']}: {reason}（{camera['topic']}）")
        return {**camera, **encode_snapshot(received[0]), 'received_at': time.time()}
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
