"""Synthetic camera and command source used only by the MCAP integration test."""
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image, JointState

from humanoid_manager.mock_feedback import ARM_NAMES, GRIPPER_NAMES


def main():
    rclpy.init()
    node = rclpy.create_node('mock_feedback_test_source')
    arms = node.create_publisher(JointState, '/hc_teleop/joint_cmd', 10)
    grippers = node.create_publisher(JointState, '/hc_teleop/gripper_commands', 10)
    cameras = [node.create_publisher(Image, f'/camera_{name}/camera/color/image_raw', 10)
               for name in ('left', 'right', 'hand')]
    frames = [Image(width=640, height=480, encoding='rgb8', step=1920,
                    data=bytes([i + 1, 40, 80]) * (640 * 480)) for i in range(3)]
    began = time.monotonic()
    def command():
        phase = time.monotonic() - began
        arm = JointState(name=list(ARM_NAMES), position=[.2 * math.sin(phase)] * 14)
        arm.header.stamp = node.get_clock().now().to_msg()
        arms.publish(arm)
        grip = JointState(name=list(GRIPPER_NAMES), position=[.02 + .01 * math.sin(phase)] * 2)
        grip.header.stamp = arm.header.stamp
        grippers.publish(grip)
    def images():
        for pub, frame in zip(cameras, frames):
            frame.header.stamp = node.get_clock().now().to_msg()
            frame.header.frame_id = 'synthetic_camera_test'
            pub.publish(frame)
    node.create_timer(.01, command)
    node.create_timer(1 / 30, images)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
