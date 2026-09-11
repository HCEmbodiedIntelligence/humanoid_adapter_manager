import pytest

from humanoid_manager.mock_feedback import ARM_NAMES, DelayedFeedback


def test_one_frame_delay_partial_commands_and_hold():
    feedback = DelayedFeedback(ARM_NAMES, [0.] * 14)
    assert feedback.tick() == [0.] * 14
    feedback.accept([ARM_NAMES[1], ARM_NAMES[0]], [.2, .1])
    assert feedback.tick() == [0.] * 14
    assert feedback.tick() == [.1, .2] + [0.] * 12
    feedback.accept([ARM_NAMES[7]], [.3])
    assert feedback.tick()[7] == 0.
    expected = [.1, .2] + [0.] * 5 + [.3] + [0.] * 6
    for _ in range(5):
        assert feedback.tick() == expected
    feedback.accept([ARM_NAMES[0]], [.4])
    feedback.accept([ARM_NAMES[0]], [.5])
    assert feedback.tick() == expected
    assert feedback.tick()[0] == .5


@pytest.mark.parametrize('names,positions,velocity', [
    ([], [], []), ([ARM_NAMES[0]], [], []),
    ([ARM_NAMES[0]] * 2, [.1, .2], []),
    ([ARM_NAMES[0], 'unknown'], [.1, .2], []),
    ([ARM_NAMES[0]], [float('nan')], []),
    ([ARM_NAMES[0]], [float('inf')], []),
    ([ARM_NAMES[0]], [.1], [0., 0.]),
    ([ARM_NAMES[0]], [.1], [float('inf')]),
])
def test_reject_malformed_command_atomically(names, positions, velocity):
    feedback = DelayedFeedback(ARM_NAMES, [0.] * 14)
    with pytest.raises(ValueError):
        feedback.accept(names, positions, velocity)
    assert feedback.tick() == feedback.tick() == [0.] * 14


def test_disabled_button_marking_does_not_force_recording(tmp_path):
    from humanoid_manager.web.config import validate_config
    config = {'adapter_manager': {key: str(tmp_path / key) for key in ('cli', 'plugin_root', 'state_root')}}
    result = validate_config(config)
    button = next(x for x in result['ros']['subscriptions'] if x['topic'] == '/hc_teleop_recv/buttons')
    assert 'record' not in button['outputs']
    config['data_quality'] = {'button_enabled': True}
    result = validate_config(config)
    button = next(x for x in result['ros']['subscriptions'] if x['topic'] == '/hc_teleop_recv/buttons')
    assert 'record' in button['outputs']
    # An explicit user's selection remains authoritative even without marking.
    result['data_quality']['button_enabled'] = False
    assert 'record' in next(x for x in validate_config(result)['ros']['subscriptions']
                            if x['topic'] == '/hc_teleop_recv/buttons')['outputs']


def test_image_audit_preserves_metadata_without_expanding_pixel_bytes():
    from types import SimpleNamespace
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Image
    from humanoid_manager.web.datasets import decode
    image = Image(width=640, height=480, encoding='rgb8', step=1920,
                  data=bytes([20, 40, 80]) * (640 * 480))
    image.header.frame_id = 'camera_frame'
    raw = serialize_message(image)
    result = decode(SimpleNamespace(name='sensor_msgs/msg/Image'),
                    SimpleNamespace(message_encoding='cdr'), SimpleNamespace(data=raw))
    assert result['data'] == {'binary_bytes': 640 * 480 * 3}
    assert result['width'] == 640 and result['height'] == 480
    assert result['header']['frame_id'] == 'camera_frame'
    # CDR padding bytes are not necessarily deterministic across serializations.
    assert bytes(image.data) == bytes([20, 40, 80]) * (640 * 480)
