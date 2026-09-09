"""Exercise actual one-shot helpers with deterministic synthetic ROS feedback."""
import time
from types import SimpleNamespace

import pytest
pytest.importorskip('rclpy')
from sensor_msgs.msg import Image, JointState
from humanoid_manager.web.gripper_command import execute_gripper_test
from humanoid_manager.web.camera_snapshot import CameraSnapshotError, capture_camera_snapshot


class FakeContext:
    def __init__(self): self.active = True
    def ok(self): return self.active
    def shutdown(self): self.active = False


@pytest.fixture
def ros(monkeypatch):
    import rclpy
    import rclpy.context
    import rclpy.executors
    state = SimpleNamespace(callback=None, messages=[], feedback=None, destroyed=False, executor_shutdown=False,
                            subscription_topic=None, publisher_topic=None)
    class Publisher:
        def get_subscription_count(self): return 1
        def publish(self, message): state.messages.append(message)
    class Node:
        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=time.time_ns(),
                to_msg=lambda: JointState().header.stamp))
        def create_publisher(self, kind, topic, qos):
            state.publisher_topic = topic
            return Publisher()
        def create_subscription(self, kind, topic, callback, qos):
            state.subscription_topic = topic
            state.callback = callback
            return object()
        def destroy_node(self): state.destroyed = True
        def destroy_subscription(self, subscription): pass
        def destroy_publisher(self, publisher): pass
    class Executor:
        def __init__(self, **kwargs): pass
        def add_node(self, node): pass
        def remove_node(self, node): pass
        def shutdown(self): state.executor_shutdown = True
        def spin_once(self, **kwargs):
            if state.feedback: state.callback(state.feedback())
    monkeypatch.setattr(rclpy, 'init', lambda **kwargs: None)
    monkeypatch.setattr(rclpy.context, 'Context', FakeContext)
    monkeypatch.setattr(rclpy, 'create_node', lambda *args, **kwargs: Node())
    monkeypatch.setattr(rclpy.executors, 'SingleThreadedExecutor', Executor)
    return state


@pytest.mark.parametrize('position', [.07, .01])
def test_open_and_close_accept_instance_routing_and_observe_measured_result(ros, position):
    command = {'name': 'tool_b', 'position': position, 'max_effort': 5., 'timeout_sec': .2,
               'command_topic': '/tools/command', 'state_topic': '/tools/state',
               'runtime_node': 'humanoid_gripper_runtime_vendor_b'}
    ros.feedback = lambda: JointState(name=['tool_a', 'tool_b'],
        position=[.99, ros.messages[-1].position[0] if ros.messages else .04])
    result = execute_gripper_test(command, 0)
    assert result['initial_position'] == .04
    assert result['final_position'] == position
    assert result['command_messages'] >= 1
    assert all(message.name == ['tool_b'] for message in ros.messages)
    assert list(ros.messages[-1].position) == [position]
    assert ros.destroyed


def test_photo_receives_selected_topic_and_cleans_up_without_publishing(ros):
    def feedback():
        message = Image(width=1, height=1, encoding='rgb8', step=3, data=bytes([255, 0, 0]))
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(time.time_ns(), 1_000_000_000)
        return message
    ros.feedback = feedback
    result = capture_camera_snapshot({'camera_id': 'rear', 'topic': '/rear/rgb', 'message_type': 'image'}, 0)
    assert result['image_data_url'].startswith('data:image/jpeg;base64,')
    assert ros.subscription_topic == '/rear/rgb'
    assert ros.messages == [] and ros.publisher_topic is None
    assert ros.destroyed and ros.executor_shutdown


@pytest.mark.parametrize('stale', [False, True])
def test_photo_timeout_or_stale_frame_does_not_return_old_photo(ros, stale):
    if stale:
        ros.feedback = lambda: Image(width=1, height=1, encoding='rgb8', step=3, data=bytes([0, 0, 0]))
    with pytest.raises(CameraSnapshotError, match='时间戳|未收到图像'):
        capture_camera_snapshot({'camera_id': 'rear', 'topic': '/rear/rgb', 'message_type': 'image'}, 0, .01)
    assert ros.destroyed and ros.executor_shutdown
