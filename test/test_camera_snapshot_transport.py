"""Receive full camera-sized ROS messages; no camera or robot hardware."""
from concurrent.futures import ThreadPoolExecutor
import time
import uuid

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.context import Context
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from humanoid_manager.web.camera_snapshot import capture_camera_snapshot


@pytest.mark.parametrize('reliability', [ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT])
def test_snapshot_receives_large_images_with_the_offered_reliability(reliability):
    context = Context()
    rclpy.init(args=[], context=context, domain_id=226)
    node = rclpy.create_node('synthetic_camera_source', context=context)
    topic = '/snapshot_test_' + uuid.uuid4().hex + '/rgb'
    publisher = node.create_publisher(Image, topic, QoSProfile(depth=10,
        reliability=reliability, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    message = Image(width=640, height=480, encoding='rgb8', step=1920,
                    data=bytes([200, 100, 30]) * (640 * 480))
    try:
        with ThreadPoolExecutor(max_workers=1) as worker:
            capture = worker.submit(capture_camera_snapshot,
                {'camera_id': 'head', 'topic': topic, 'message_type': 'image'}, 226, 5.)
            deadline = time.monotonic() + 5.
            while not capture.done() and time.monotonic() < deadline:
                readers = node.get_subscriptions_info_by_topic(topic)
                # Wait for the chosen policy so that reliable-only coverage
                # cannot accidentally pass through the initial sensor-data reader.
                if any(info.qos_profile.reliability == reliability for info in readers):
                    message.header.stamp = node.get_clock().now().to_msg()
                    publisher.publish(message)
                time.sleep(.02)
            result = capture.result(timeout=1.)
        assert (result['width'], result['height']) == (640, 480)
        assert result['image_data_url'].startswith('data:image/jpeg;base64,')
        deadline = time.monotonic() + 2.
        while publisher.get_subscription_count() and time.monotonic() < deadline:
            time.sleep(.02)
        assert publisher.get_subscription_count() == 0
    finally:
        node.destroy_node()
        context.shutdown()
